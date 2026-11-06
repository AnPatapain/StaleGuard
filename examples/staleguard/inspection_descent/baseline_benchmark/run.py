"""Frozen scenarios, one approval forecast/seed, actual paired flights, three files."""
from pathlib import Path
import sys
if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import inspection_descent
import argparse
from dataclasses import asdict
import ctypes
import fcntl
import gc
import hashlib
import json
import math
import os
from inspection_descent.scenario import CTRL_FREQ_HZ, DELAY_GRID, configuration
from inspection_descent.storage import (ROOT,load_calibrated_scenarios,metadata,check_provenance,
    check_memory,write_json,append_json,file_hash,historical_benchmark,normalize_historical)
from inspection_descent.simulation import execute
from inspection_descent.baselines.staleguard import predict_at_reception,LightweightPredictiveModel
from inspection_descent.baseline_benchmark.metrics import POLICIES,read_executions,summarize
from model.scg import load_scg_platform_params

DELAYS=tuple(i/10 for i in range(11))
APPROVAL_POLICIES={'staleguard','staleguard_early_recovery'}


def run(args):
    cases=load_calibrated_scenarios(args.calibration)
    data_path=args.calibration/'calibrated_scenarios.json' if args.calibration.is_dir() else args.calibration
    dataset=json.loads(data_path.read_text())
    if args.seeds:
        cases=[c for c in cases if c[0] in args.seeds]
        if {c[0] for c in cases}!=set(args.seeds):
            raise ValueError('Requested seed is not a feasible calibrated scenario')
    if not cases:
        raise ValueError('No calibrated scenarios')
    protocol=dict(scenarios=[dict(seed=seed,**asdict(s)) for seed,s,_,_ in cases],
        snapshot_sha256={str(seed):file_hash(path) for seed,_,_,path in cases},
        dataset_sha256=file_hash(data_path),config=configuration(),control_hz=CTRL_FREQ_HZ,
        delays=list(args.delays),policies=list(args.policies),ttl_grid_s=list(DELAY_GRID),
        calibration_statistics=dataset['statistics'])
    out=args.out or data_path.parent.parent/'baseline_benchmark'
    out.mkdir(parents=True,exist_ok=True)
    run_path=out/'run.json'; journal=out/'executions.jsonl'
    if run_path.exists():
        saved=json.loads(run_path.read_text())
        if saved.get('origin')!='physical_execution' or saved['protocol']!=protocol:
            raise ValueError('Historical/incompatible output: use a fresh --out')
        check_provenance(saved['provenance'])
        raw=journal.read_bytes(); size=saved.get('journal_bytes',0)
        if len(raw)<size or hashlib.sha256(raw[:size]).hexdigest()!=saved['journal_sha256']:
            raise ValueError('Committed journal changed')
        rows=read_executions(journal,repair_tail=True)
        for row in rows:
            if row['event']['policy'] in APPROVAL_POLICIES:
                forecast=saved['predictions'][str(row['seed'])]
                if row['event']['computation_s']!=forecast['computation_s']:
                    raise ValueError('Frozen forecast timing changed')
    else:
        if any(out.iterdir()):
            raise FileExistsError('Choose an empty --out')
        saved=dict(origin='physical_execution',status='partial',protocol=protocol,provenance=metadata(),predictions={},
                   journal_bytes=0,journal_sha256=hashlib.sha256(b'').hexdigest())
        write_json(run_path,saved); journal.touch(); rows=[]
        write_json(out/'metrics.json',summarize(journal,protocol))
    done={(r['seed'],r['delay_s'],r['event']['policy']) for r in rows}
    expected={(seed,d,p) for seed,_,_,_ in cases for d in args.delays for p in args.policies}
    if not done<=expected:
        raise ValueError('Unexpected journal flight')
    platform=None
    if done!=expected and (APPROVAL_POLICIES|{'use_time_recheck','ppr'})&set(args.policies):
        check_memory(); platform=load_scg_platform_params()
        model=LightweightPredictiveModel(*platform)
        try:
            ctrl,obs,info,_=model.start(cases[0][2])
            model.step(ctrl.select_action(obs,info),obs,info)
        finally:
            model.close({})
    added=0
    try:
        for seed,scenario,snapshot,_ in cases:
            if all((seed,d,p) in done for d in args.delays for p in args.policies):
                continue
            received=saved['predictions'].get(str(seed))
            if received is None and APPROVAL_POLICIES&set(args.policies):
                check_memory()
                prediction,elapsed=predict_at_reception(snapshot,*platform)
                received=dict(prediction=prediction,computation_s=elapsed)
                saved['predictions'][str(seed)]=received; write_json(run_path,saved)
            for delay in args.delays:
                for policy in args.policies:
                    key=seed,delay,policy
                    if key in done:
                        continue
                    row,_,_=execute(scenario,delay,policy=policy,snapshot=snapshot,
                        seed=seed,received=received,platform=platform)
                    append_json(journal,row); done.add(key); added+=1
                    print(f'seed={seed} D={delay:.2f} {policy}: safe={row["safe"]} completed={row["completed"]} [{len(done)}/{len(expected)}]',flush=True)
                    if added%20==0:
                        gc.collect();ctypes.CDLL(None).malloc_trim(0)
                    if args.max_flights and added>=args.max_flights:
                        return
    finally:
        raw=journal.read_bytes()
        saved.update(status='complete' if done==expected else 'partial',journal_bytes=len(raw),
                     journal_sha256=hashlib.sha256(raw).hexdigest())
        write_json(run_path,saved);write_json(out/'metrics.json',summarize(journal,protocol))


def import_historical(out):
    """Explicit normalized derivative; never append new physical flights to it."""
    source,rows=historical_benchmark();out=Path(out)
    if out.exists() and any(out.iterdir()):
        saved=json.loads((out/'run.json').read_text())
        if saved.get('origin')!='historical_schema_conversion' or saved['source_sha256']!=file_hash(source):
            raise FileExistsError(out)
        if saved['journal_sha256']!=file_hash(out/'executions.jsonl'):
            raise ValueError('Converted historical journal changed')
        return
    cases=load_calibrated_scenarios(ROOT/'results/evaluation/calibration')
    dataset=json.loads((ROOT/'results/evaluation/calibration/calibrated_scenarios.json').read_text())
    protocol=dict(scenarios=[dict(seed=seed,**asdict(s)) for seed,s,_,_ in cases],
        snapshot_sha256={str(seed):file_hash(path) for seed,_,_,path in cases},
        control_hz=50,delays=list(DELAYS),policies=list(POLICIES[:6]),ttl_grid_s=list(DELAY_GRID),
        config=configuration(),calibration_statistics=dataset['statistics'])
    out.mkdir(parents=True,exist_ok=True); journal=out/'executions.jsonl'
    temporary=journal.with_suffix('.jsonl.tmp')
    with temporary.open('w') as stream:
        for row in rows:
            stream.write(json.dumps(normalize_historical(row),allow_nan=False)+'\n')
        stream.flush();os.fsync(stream.fileno())
    temporary.replace(journal)
    saved=dict(origin='historical_schema_conversion',protocol=protocol,status='complete',
        source=str(source.relative_to(ROOT)),source_sha256=file_hash(source),provenance=metadata(),
        historical_provenance=json.loads((source.parent/'preserved_files.json').read_text()),
        predictions={str(seed):json.loads((ROOT/f'independent_evaluation_20_319/scenario_checkpoints/{seed}/reception.json').read_text())
                     for seed,_,_,_ in cases},journal_bytes=journal.stat().st_size,journal_sha256=file_hash(journal),
        note='No new simulations; execution_kind retains native versus verified reference reuse. Missing measurements stay null.')
    write_json(out/'run.json',saved);write_json(out/'metrics.json',summarize(journal,protocol))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',type=Path,default=ROOT/'results/development/calibration')
    parser.add_argument('--out',type=Path)
    parser.add_argument('--seeds',nargs='+',type=int)
    parser.add_argument('--delays',nargs='+',type=float,default=DELAYS)
    parser.add_argument('--policies',nargs='+',choices=POLICIES,default=POLICIES[:6])
    parser.add_argument('--max-flights',type=int,default=0)
    parser.add_argument('--import-historical',action='store_true')
    args=parser.parse_args()
    if (len(set(args.delays))!=len(args.delays) or len(set(args.policies))!=len(args.policies)
            or args.max_flights<0 or any(not math.isfinite(d) or d<0 or
                not math.isclose(d*50,round(d*50),abs_tol=1e-9) for d in args.delays)):
        parser.error('Use distinct policies and nonnegative control-grid delays')
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if args.import_historical:
            import_historical(args.out or ROOT/'results/evaluation/baseline_benchmark')
        else:
            run(args)


if __name__=='__main__':
    main()
