"""Approval prediction versus the original 8.5-second PyBullet safety truth."""
from pathlib import Path
import sys
from staleguard import inspection_descent
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
from staleguard.inspection_descent.scenario import DELAY_GRID, classify_delay_results
from staleguard.inspection_descent.simulation import run_ground_truth
from staleguard.model.scg import load_scg_platform_params
from staleguard.inspection_descent.baselines.staleguard import (predict_at_reception,ttl_expired_at_dispatch,
                                                    LightweightPredictiveModel)
from staleguard.inspection_descent.storage import (ROOT,HISTORY,DEV_BUNDLE,file_hash,write_json,append_json,
    metadata,check_provenance,check_memory,load_pickle,check_scientific_sources,check_approval_snapshot,load_calibrated_scenarios)
from staleguard.inspection_descent.ttl_pred_accuracy.metrics import summarize,read_records

DELAYS=tuple(i/10 for i in range(11))



def classify_scenario(s, delays):
    # 1. Check nominal trajectory.
    nominal = run_ground_truth(s, None)
    if nominal["hit_nominal"] is not None:
        return classify_delay_results(
            delays,
            results=None,
            nominal_safe=False,
        )
    # 2. Simulate each dispatch delay.
    results = []
    for delay in delays:
        result = run_ground_truth(s, delay)
        if result["hit_while_waiting"] is not None:
            status = "WAIT_UNSAFE"
        elif result["hit_after_command"] is not None:
            status = "UNSAFE"
        else:
            status = "SAFE"
        results.append(status)

    # 3. Shared classification.
    return classify_delay_results(delays, results)

def historical_truth(cases):
    """Only the compatible calibrated development accuracy measurements qualify.

    Operational benchmark flights have a different horizon and are not reused.
    """
    folder=ROOT/'results/development/temporal_safety_correction'
    inputs=load_pickle(DEV_BUNDLE)
    calibration=ROOT/'results/development/calibration/generated_scenarios.json'
    if inputs['calibration_sha256']!=file_hash(calibration):
        raise ValueError('Historical accuracy input calibration changed')
    preserved=json.loads((HISTORY/'main_result/preserved_files.json').read_text())
    try:
        check_scientific_sources({n:preserved[n] for n in
            ('experiment.py','geometry.py','ground_truth.py','physics_monitor.py','staleguard.py',
             'model/common.py','model/lightweight.py','model/scg.py','model/pybullet.py')})
    except ValueError:
        return {},dict(status='incompatible scientific sources; fresh ground truth required')
    original=json.loads(calibration.read_text());truth=json.loads((folder/'ground_truth_results.json').read_text())
    comparison=json.loads((folder/'accuracy_comparison.json').read_text())
    checked={r['seed']:r for r in comparison}
    reused={}
    for seed,s,snapshot,_ in cases:
        if str(seed) not in original or original[str(seed)]['command']!=asdict(s):
            continue
        check_approval_snapshot(snapshot,inputs['snapshots'][seed])
        if truth[str(seed)]['actual']!=checked[seed]['ground_truth']:
            raise ValueError('Historical accuracy classifications disagree')
        reused[seed]=truth[str(seed)]['actual']
    return reused,dict(status='validated historical development accuracy',
        source_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in
            (DEV_BUNDLE,calibration,folder/'ground_truth_results.json',folder/'accuracy_comparison.json')},
        horizon='8.5 s from flight start; original 1000 Hz truth, now simulation.run_ground_truth')


def run(args):
    cases=load_calibrated_scenarios(args.calibration)
    if args.seeds:
        cases=[c for c in cases if c[0] in args.seeds]
        if {c[0] for c in cases}!=set(args.seeds):
            raise ValueError('Unknown/infeasible selected seed')
    dataset=args.calibration/'calibrated_scenarios.json' if args.calibration.is_dir() else args.calibration
    actual,evidence=historical_truth(cases)
    historical=set(actual)
    protocol=dict(seeds=[c[0] for c in cases],delays=list(DELAYS),prediction_grid_s=list(DELAY_GRID),
        dataset_sha256=file_hash(dataset),snapshot_sha256={str(seed):file_hash(path) for seed,_,_,path in cases},
        ground_truth=evidence,calibration_statistics=json.loads(dataset.read_text())['statistics'])
    out=args.out or dataset.parent.parent/'ttl_pred_accuracy';out.mkdir(parents=True,exist_ok=True)
    journal=out/'executions.jsonl';run_path=out/'run.json'
    if run_path.exists():
        saved=json.loads(run_path.read_text());check_provenance(saved['provenance'])
        if saved['protocol']!=protocol:
            raise ValueError('Accuracy protocol/inputs changed')
        raw=journal.read_bytes();size=saved['journal_bytes']
        if len(raw)<size or hashlib.sha256(raw[:size]).hexdigest()!=saved['journal_sha256']:
            raise ValueError('Accuracy journal changed')
        rows=read_records(journal,repair_tail=True)
    else:
        if any(out.iterdir()):
            raise FileExistsError('Choose an empty --out')
        journal.touch();rows=[]
        saved=dict(status='partial',protocol=protocol,provenance=metadata(),predictions={},actual={},
            journal_bytes=0,journal_sha256=hashlib.sha256(b'').hexdigest())
        write_json(run_path,saved)
        write_json(out/'metrics.json',summarize(journal,protocol))
    done={(r['seed'],r['delay_s']) for r in rows}; expected={(c[0],d) for c in cases for d in DELAYS}
    if not done<=expected:
        raise ValueError('Unexpected accuracy comparison')
    platform=None
    if done!=expected:
        check_memory();platform=load_scg_platform_params();model=LightweightPredictiveModel(*platform)
        try:
            ctrl,obs,info,_=model.start(cases[0][2]);model.step(ctrl.select_action(obs,info),obs,info)
        finally:
            model.close({})
    try:
        for seed,scenario,snapshot,_ in cases:
            if all((seed,d) in done for d in DELAYS):
                continue
            if str(seed) not in saved['predictions']:
                check_memory();prediction,elapsed=predict_at_reception(snapshot,*platform)
                saved['predictions'][str(seed)]=dict(prediction=prediction,computation_s=elapsed)
                write_json(run_path,saved)
            forecast=saved['predictions'][str(seed)]
            if str(seed) not in saved['actual']:
                if seed not in actual:
                    check_memory()
                    actual[seed]=classify_scenario(scenario,DELAY_GRID)
                saved['actual'][str(seed)]=actual[seed];write_json(run_path,saved)
            truth=saved['actual'][str(seed)];pred=forecast['prediction']
            for index,delay in enumerate(DELAYS):
                if (seed,delay) in done:
                    continue
                # The original mission excludes invalid nominal profiles from binary accuracy.
                safe=None if truth['results'] is None else truth['results'][index]=='SAFE'
                raw=None if pred['results'] is None else pred['results'][index]=='SAFE'
                row=dict(seed=seed,delay_s=delay,safe=safe,completed=None,safe_completion=None,
                    predicted_safe_raw=raw,predicted_safe_ttl=not ttl_expired_at_dispatch(pred,round(delay*50)),
                    prediction_grid_point=True,prediction=pred,actual=truth,
                    computation_s=forecast['computation_s'],ground_truth_reused=seed in actual and seed in historical,
                    first_violation_s=None)
                append_json(journal,row);done.add((seed,delay))
            print(f'seed={seed}: {pred["classification"]} / {truth["classification"]} [{len(done)}/{len(expected)}]',flush=True)
    finally:
        saved.update(status='complete' if done==expected else 'partial',journal_bytes=journal.stat().st_size,
                     journal_sha256=file_hash(journal))
        write_json(run_path,saved);write_json(out/'metrics.json',summarize(journal,protocol))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',type=Path,default=ROOT/'results/development/calibration')
    parser.add_argument('--out',type=Path)
    parser.add_argument('--seeds',nargs='+',type=int)
    args=parser.parse_args()
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        run(args)


if __name__=='__main__':
    main()
