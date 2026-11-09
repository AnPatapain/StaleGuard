"""Independent PyBullet S_cmd/C_cmd/S_rec on the frozen 19-delay grid.

No policy/predictor is run. Matching No Recovery records are reused; recovery
starts are always independently simulated. Existing benchmark files are read-only.
"""
from pathlib import Path
import sys
if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import fcntl
import hashlib
import json
import time
import numpy as np
from underpass_inspection.storage import (ROOT,read_run,load_inputs,iter_committed,metadata,check_provenance,
    initialize_run,committed_hasher,commit_record,write_progress,write_json,file_hash,check_approval_snapshot)
from underpass_inspection.simulation import execute
from underpass_inspection.calibration import release_memory
from underpass_inspection.baselines.staleguard import TTL_GRID


def key(row):return row['seed'],row['mode'],round(row['start_s']*50)


def physical(job):
    s,snapshot,mode,delay=job
    release_memory()
    row,arrays=execute(s,mode=mode,start_s=delay,capture_snapshot=True)
    check_approval_snapshot(arrays['approval_snapshot'],snapshot)
    assert row['physics_samples']==row['horizon_step']*20+1
    assert row['geometry_unchanged'] and row['pid_unchanged_at_initiation'] and row['physical_state_unchanged_at_initiation']
    row.update(origin='independent_pybullet',approval_snapshot_verified=True)
    return row


def source_data(folder):
    run=read_run(folder)
    if run['status']!='complete':raise ValueError('Use a completed benchmark')
    cases=load_inputs(folder,run);reused={};immediate={}
    for row in iter_committed(folder,run):
        policy=row['event']['policy']
        if policy=='no_recovery' and row['delay_s'] in TTL_GRID:
            assert row['mode']=='dive' and row['start_s']==row['delay_s']
            if key(row) in reused:raise ValueError('Duplicate source command')
            reused[key(row)]=row
        elif policy=='always_recover' and row['delay_s']==0.:
            immediate[row['seed']]=row
    return run,cases,reused,immediate


def validate(out,run,source):
    expected={(s['seed'],mode,round(d*50)) for s in run['protocol']['scenarios'] for mode in ('dive','sideways') for d in TTL_GRID}
    seen=set();counts={};original={r['seed']:r for r in source['protocol']['scenarios']}
    for row in iter_committed(out,run):
        k=key(row)
        if k in seen or k not in expected:raise ValueError('Duplicate/unexpected physical execution')
        seen.add(k);counts[row['origin']]=counts.get(row['origin'],0)+1
        s=original[row['seed']];total=round((s['approval_s']+s['observation_s'])*50)
        assert row['horizon_step']==total and row['physics_samples']==total*20+1
        assert row['safe']==(row['first_violation'] is None)==(row['min_clearance_m']>=s['clearance_m'])
        assert row['safe_completion']==(row['safe'] and row['inspection_completed'])
        assert row['geometry_unchanged'] and row['pid_unchanged_at_initiation'] and row['physical_state_unchanged_at_initiation']
    assert seen==expected and len(seen)==run['completed_flights']
    return dict(passed=True,profiles=3,scenarios=len(original),physical_records=len(seen),origins=counts,
        full_observation_horizon=True,physics_monitoring_hz=1000,duplicates=0,source_files_unchanged=True)


def smoke(args):
    source,cases,reused,immediate=source_data(args.source);s,snapshot,_=cases[0]
    jobs=[(s,snapshot,'dive',0.),(s,snapshot,'sideways',0.)]
    serial=[physical(job) for job in jobs]
    with ProcessPoolExecutor(max_workers=2,mp_context=get_context('spawn')) as pool:parallel=list(pool.map(physical,jobs))
    for a,b in zip(serial,parallel):
        for field in ('physics_sha256','safe','safe_completion','min_clearance_m','first_violation'):assert a[field]==b[field]
    assert serial[0]['physics_sha256']==reused[s.seed,'dive',0]['physics_sha256']
    assert serial[1]['physics_sha256']==immediate[s.seed]['physics_sha256']
    args.out.mkdir(parents=True,exist_ok=True)
    write_json(args.out/'smoke_validation.json',dict(passed=True,seed=s.seed,actual_validation_flights=4,
        serial_parallel_trajectory_hashes_identical=True,archived_d0_command_and_recovery_hashes_identical=True,
        full_horizon_1khz=True))
    print('Smoke passed: four serial/parallel flights, two archived physical hashes matched')


def sweep(args):
    source,cases,reused,immediate=source_data(args.source)
    if len(cases)!=400 or len(reused)!=4000 or len(immediate)!=400:
        raise ValueError('Expected the complete 400-scenario v2 cohort and exactly 4000 matching command records')
    out=args.out;path=out/'run.json';started=time.perf_counter()
    if path.exists():
        if not args.resume:raise FileExistsError('Use --resume')
        run=read_run(out,repair=True);check_provenance(run['provenance'])
        if run['driver_sha256']!=file_hash(__file__):raise ValueError('Physical sweep driver changed')
        if run['source_journal_sha256']!=source['journal_sha256']:raise ValueError('Source benchmark changed')
    else:
        config=dict(format='underpass_independent_feasibility_v1',provenance=metadata(),driver_sha256=file_hash(__file__),
            source_path=str(args.source.resolve()),source_config_sha256=source['config_sha256'],source_journal_sha256=source['journal_sha256'],
            source_prediction_sha256=source['prediction_sha256'],inputs_sha256=source['inputs_sha256'],
            protocol=dict(scenarios=source['protocol']['scenarios'],grid_s=list(TTL_GRID),modes=['dive','sideways'],
                monitoring_hz=1000,control_hz=50,observation_s=8.,physical_workers=args.workers,
                command_reuse='Only existing No Recovery at identical candidate starts; no prediction labels',
                recovery='Independent predefined Eastward sideways starts at all 19 delays'))
        run=initialize_run(out,config)
    digest=committed_hasher(out,run);done=set()
    for row in iter_committed(out,run):
        k=key(row)
        if k in done:raise ValueError('Duplicate physical record')
        done.add(k)
    for k,row in reused.items():
        if k not in done:
            copied=dict(row,origin='reused_no_recovery');commit_record(out,run,copied,digest);done.add(k)
    jobs=[(s,snapshot,mode,d) for s,snapshot,_ in cases for mode in ('dive','sideways') for d in TTL_GRID
          if (s.seed,mode,round(d*50)) not in done]
    run['status']='running';run['resumptions']+=int(args.resume);write_progress(out,run)
    added=0
    try:
        with ProcessPoolExecutor(max_workers=args.workers,mp_context=get_context('spawn')) as pool:
            pending=deque();iterator=iter(jobs)
            def submit():
                job=next(iterator,None)
                if job is not None:pending.append(pool.submit(physical,job))
            for _ in range(2*args.workers):submit()
            while pending:
                row=pending.popleft().result();k=key(row)
                if row['mode']=='sideways' and row['start_s']==0.:
                    assert row['physics_sha256']==immediate[row['seed']]['physics_sha256']
                assert k not in done
                commit_record(out,run,row,digest);done.add(k);added+=1;submit()
                if added%100==0:print(f'{len(done)}/15200 records; {added} new flights this invocation; {time.perf_counter()-started:.0f} s',flush=True)
                if args.max_flights and added>=args.max_flights:break
    finally:
        run=read_run(out,repair=True);run['status']='complete' if run['completed_flights']==15200 else 'partial';write_progress(out,run)
        print('Saved',run['completed_flights'],'physical records;',added,'new flights this invocation',flush=True)
    if run['status']=='complete':write_json(out/'validation.json',validate(out,run,source))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=ROOT/'results/evaluation/baseline_benchmark_v2')
    p.add_argument('--out',type=Path,default=ROOT/'results/evaluation/supplementary_feasibility_v1')
    p.add_argument('--workers',type=int,default=4);p.add_argument('--resume',action='store_true')
    p.add_argument('--smoke',action='store_true');p.add_argument('--max-flights',type=int,default=0)
    a=p.parse_args()
    if not 1<=a.workers<=4:p.error('Use one to four isolated physical workers')
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        smoke(a) if a.smoke else sweep(a)


if __name__=='__main__':main()
