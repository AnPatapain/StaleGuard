"""Seven policies, frozen approval forecasts and independent actual paired flights."""
from pathlib import Path
import sys
if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import time
import numpy as np
from model.scg import load_scg_platform_params
from model.lightweight import LightweightPredictiveModel
from underpass_inspection.simulation import check_memory
from underpass_inspection.baselines.staleguard import TTL_GRID
from underpass_inspection.baselines import fbs, fixed_early_recovery
from underpass_inspection.calibration import release_memory
from underpass_inspection.scenario import HZ,POS
from underpass_inspection.simulation import execute
from underpass_inspection.baselines.policies import POLICIES,APPROVAL_POLICIES
from underpass_inspection.baselines.staleguard import predict_delays,segment_box_clearance
from underpass_inspection.storage import (ROOT,canonical,load_calibrated_scenarios,
    metadata,check_provenance,write_json,append_json,file_hash,read_committed,flight_key,
    load_ground_truth,verify_physical_reference)
from underpass_inspection.baseline_benchmark.metrics import write_outputs

DISPATCH_DELAYS=tuple(round(i*.02,2) for i in range(51))
SMOKE_DELAYS=(0.,.2,.38,.68,.84,.86,.9,.92,1.)


def manifest_path(path):
    return path/'calibrated_scenarios.json' if path.is_dir() else path


def run(args):
    cases=load_calibrated_scenarios(args.calibration)
    if args.seeds:
        cases=[case for case in cases if case[0].seed in args.seeds]
        if {s.seed for s,_,_ in cases}!=set(args.seeds):
            raise ValueError('Requested seed is not accepted in the frozen manifest')
    if not cases:raise ValueError('No accepted scenarios')
    data_path=manifest_path(args.calibration);dataset=json.loads(data_path.read_text())
    truth,truth_run=load_ground_truth(args.ground_truth)
    archived={s['seed']:s for s in truth_run['protocol']['scenarios']}
    for s,_,_ in cases:
        if archived.get(s.seed)!=canonical(asdict(s)):
            raise ValueError('Archived truth does not describe the same frozen scenario')
    representatives=args.representative_seeds or [cases[0][0].seed]
    if not set(representatives)<={s.seed for s,_,_ in cases}:
        raise ValueError('Representative seed absent from selected scenarios')
    protocol=canonical(dict(stage='physical_smoke' if args.smoke else 'development_benchmark',
        scenarios=[asdict(s) for s,_,_ in cases],
        dataset_sha256=file_hash(data_path),snapshot_sha256={str(s.seed):file_hash(p) for s,_,p in cases},
        calibration_statistics=dataset['statistics'],ground_truth_run_sha256=file_hash(args.ground_truth/'run.json'),
        ground_truth_journal_sha256=file_hash(args.ground_truth/'executions.jsonl'),
        policies=list(args.policies),delays=list(args.delays),ttl_grid_s=list(TTL_GRID),
        control_hz=50,physics_hz=1000,model_hz=100,observation_s='unchanged per frozen scenario',
        primary_recovery='Existing Eastward sideways maneuver only; no climb selection',
        fixed_early_ttl_s=fixed_early_recovery.FIXED_EARLY_TTL_S,
        fbs=dict(alpha=fbs.ALPHA,epsilon=fbs.EPSILON,max_ttl_steps=fbs.MAX_TTL_STEPS,
            feature='required clearance minus minimum actual 3D body clearance; EMA through approval inclusive'),
        readiness='Measured warmed runtime replayed on the control clock; unavailable approval forecasts fail open',
        ordering='Approval first; readiness; dispatch; pending timers; maneuver start; physical ticks. Dispatch wins timer ties.',
        use_time_recheck='One immediate-command rollout from actual dispatch state; act at dispatch + floor(runtime*50)+1',
        representative_seeds=representatives,representative_delay_s=.92,
        unknown_delay='Only the physical driver knows scheduled dispatch. Policies receive the event when it occurs.'))
    out=args.out;run_path=out/'run.json';journal=out/'executions.jsonl'
    if run_path.exists():
        if not args.resume:raise FileExistsError('Use --resume or a fresh --out')
        saved=json.loads(run_path.read_text())
        if saved['protocol']!=protocol:raise ValueError('Frozen benchmark protocol differs')
        check_provenance(saved['provenance']);rows=read_committed(out,saved,repair=True)
        for row in rows:
            if row['event']['policy'] in APPROVAL_POLICIES:
                forecast=saved['predictions'][str(row['seed'])]
                if any(row['event'][k]!=forecast[k] for k in ('computation_s','expiry_steps')):
                    raise ValueError('Committed flight differs from its frozen approval forecast')
    else:
        if args.resume:raise FileNotFoundError(run_path)
        if out.exists() and any(out.iterdir()):raise FileExistsError('Use an empty benchmark output')
        out.mkdir(parents=True,exist_ok=True);journal.touch();rows=[]
        saved=dict(format='underpass_seven_policy_benchmark_v1',status='partial',protocol=protocol,
            provenance=metadata(),predictions={},created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            journal_bytes=0,journal_sha256=hashlib.sha256(b'').hexdigest(),resumptions=0)
        write_json(run_path,saved)
    expected={(s.seed,d,p) for s,_,_ in cases for d in args.delays for p in args.policies}
    done={flight_key(r) for r in rows}
    if len(done)!=len(rows) or not done<=expected:raise ValueError('Unexpected or duplicate journal flight')
    saved['resumptions']+=int(args.resume)
    platform=None
    if done!=expected and set(args.policies)&(set(APPROVAL_POLICIES)|{'use_time_recheck'}):
        check_memory()
        began=time.perf_counter();platform=load_scg_platform_params()
        model=LightweightPredictiveModel(*platform)
        try:
            ctrl,obs,info,_=model.start(cases[0][1])
            model.step(ctrl.select_action(obs,info),obs,info)
            box=cases[0][0].obstacles[0]
            segment_box_clearance(obs[POS],obs[POS],np.array(box.lower),np.array(box.upper))
        finally:model.close({})
        saved.setdefault('startup_warmups',[]).append(dict(seconds=time.perf_counter()-began,
            description='Platform load; shared PID/RK4 and 3D checker kernel warmup; no full forecast'))
        saved['platform_parameters']=canonical({name:{k:v.tolist() if isinstance(v,np.ndarray) else v
            for k,v in asdict(value).items()} for name,value in zip(('quadrotor','pid'),platform)})
        write_json(run_path,saved)
    trajectories={};rep=out/'representative_trajectories.npz'
    if rep.exists():
        with np.load(rep) as archive:trajectories={k:archive[k].copy() for k in archive.files}
    added=0
    try:
        for s,snapshot,_ in cases:
            if all((s.seed,d,p) in done for d in args.delays for p in args.policies):continue
            received=saved['predictions'].get(str(s.seed))
            if received is None and set(args.policies)&set(APPROVAL_POLICIES):
                release_memory()
                received=predict_delays(snapshot,platform,TTL_GRID)
                saved['predictions'][str(s.seed)]=received;write_json(run_path,saved)
                print(f'Approval forecast seed={s.seed}: expiry_steps={received["expiry_steps"]} runtime={received["computation_s"]:.4f}s',flush=True)
            for delay in args.delays:
                for policy in args.policies:
                    key=s.seed,delay,policy
                    if key in done:continue
                    release_memory()
                    row,arrays=execute(s,policy=policy,delay_s=delay,received=received,
                        platform=platform,expected_snapshot=snapshot)
                    assert row['physics_samples']==row['horizon_step']*20+1
                    assert row['geometry_unchanged'] and row['approval_snapshot_verified']
                    if row['initiation_step'] is not None:
                        assert row['pid_unchanged_at_initiation'] and row['physical_state_unchanged_at_initiation']
                        assert row['mode'] in ('dive','sideways')
                        assert row['final_position_m'][1]<row['maneuver_anchor_m'][1]
                    row['archived_physical_reference_verified']=verify_physical_reference(row,truth)
                    if s.seed in representatives and delay==protocol['representative_delay_s']:
                        prefix=f'seed{s.seed}_{round(delay*HZ):03d}_{policy}'
                        for kind,array in arrays.items():
                            if kind!='approval_snapshot':trajectories[prefix+'__'+kind]=array
                        np.savez_compressed(rep,**trajectories)
                    del arrays
                    append_json(journal,row);rows.append(row);done.add(key);added+=1
                    raw=journal.read_bytes();saved.update(completed_flights=len(done),journal_bytes=len(raw),
                        journal_sha256=hashlib.sha256(raw).hexdigest())
                    write_json(run_path,saved)
                    if added%10==0 or len(done)==len(expected):
                        print(f'{len(done)}/{len(expected)} seed={s.seed} D={delay:.2f} {policy} safe={row["safe"]} inspect={row["completed"]} recovery={row["event"]["fallback_step"]}',flush=True)
                    if args.max_flights and added>=args.max_flights:return
    finally:
        raw=journal.read_bytes();saved.update(status='complete' if done==expected else 'partial',
            completed_flights=len(done),journal_bytes=len(raw),journal_sha256=hashlib.sha256(raw).hexdigest(),
            updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
        write_json(run_path,saved);write_outputs(out,saved,rows,truth)
        print(f'Saved {len(done)}/{len(expected)} physical flights; new this invocation {added}',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration',type=Path,default=ROOT/'results/development/calibration')
    p.add_argument('--ground-truth',type=Path,default=ROOT/'results/development/deadline_sweep')
    p.add_argument('--out',type=Path)
    p.add_argument('--seeds',nargs='+',type=int)
    p.add_argument('--delays',nargs='+',type=float)
    p.add_argument('--policies',nargs='+',choices=POLICIES,default=POLICIES)
    p.add_argument('--representative-seeds',nargs='+',type=int)
    p.add_argument('--smoke',action='store_true')
    p.add_argument('--resume',action='store_true')
    p.add_argument('--max-flights',type=int,default=0)
    p.add_argument('--metrics-only',action='store_true')
    a=p.parse_args()
    if a.smoke and a.seeds is None:a.seeds=[1,6,14]
    if a.delays is None:a.delays=SMOKE_DELAYS if a.smoke else DISPATCH_DELAYS
    if a.out is None:a.out=ROOT/('results/development/baseline_smoke' if a.smoke else 'results/development/baseline_benchmark')
    if (a.max_flights<0 or not a.policies or not a.delays or len(set(a.policies))!=len(a.policies)
            or len(set(a.delays))!=len(a.delays) or any(not math.isfinite(d) or d<0 or d>=8 or
            not math.isclose(d*HZ,round(d*HZ),abs_tol=1e-9) for d in a.delays)):
        p.error('Use distinct policies and nonnegative control-grid delays below the frozen horizon')
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.metrics_only:
            saved=json.loads((a.out/'run.json').read_text());rows=read_committed(a.out,saved)
            truth,_=load_ground_truth(a.ground_truth)
            print(json.dumps(write_outputs(a.out,saved,rows,truth)['by_policy'],indent=2))
        else:run(a)


if __name__=='__main__':main()
