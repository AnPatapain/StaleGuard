"""Compare frozen StaleGuard forecasts with actual No-Recovery geofence safety."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

import geofence_turn  # CPU-only, one numerical thread, before importing the model.
import argparse
import fcntl
from geofence_turn.scenario import Config, HZ
from geofence_turn.simulation import start_work, check_work, check_memory, execute
from geofence_turn.storage import (ROOT, load_development, load_ground_truth,
    validate_flight, check_approval_snapshot, open_accuracy, save_accuracy, write_json, append_json)
from geofence_turn.baselines.staleguard import (predict_delays, staleguard_status,
    PREDICTION_DELAYS, LightweightPredictiveModel)
from geofence_turn.ttl_pred_accuracy.metrics import summarize, report_markdown

DELAYS = (0.,.02,.04,.06,.08,.10,.12,.14,.16,.18,.20,.30,.40,.50,.60,.70,.80,.90,1.)


def comparison(scenario, actual, forecast):
    if forecast['grid_s'] != list(PREDICTION_DELAYS):
        raise ValueError('Original prediction grid changed')
    by_delay = {row['delay_s']:row for row in forecast['results']}
    if set(by_delay) != set(PREDICTION_DELAYS) or len(forecast['results']) != len(PREDICTION_DELAYS):
        raise ValueError('Incomplete or duplicate directly predicted grid points')
    delay = actual['delay_s']
    raw = by_delay.get(delay)
    # Availability is deliberately assumed, without changing the measured cache.
    ready, expired = staleguard_status(dict(forecast,computation_s=0.),round(delay*HZ))
    assert ready
    first = actual['first_violation_s']
    return dict(seed=scenario.seed,delay_s=delay,safe=actual['safe'],completed=actual['completed'],
        safe_completion=actual['safe_completion'],min_clearance_m=actual['min_clearance_m'],
        first_violation_s=None if first is None else first-scenario.approval_s,
        first_violation_flight_s=first,approval_s=scenario.approval_s,
        completion_s=actual['completion_s'],predicted_safe_raw=None if raw is None else raw['safe'],
        predicted_safe_ttl=not expired,prediction_grid_point=raw is not None,
        predicted_min_clearance_m=None if raw is None else raw['min_clearance_m'],
        predicted_first_unsafe_delay_s=None if forecast['expiry_steps'] is None else forecast['expiry_steps']/HZ,
        prediction_computation_s=forecast['computation_s'])


def run_accuracy(args, work, config):
    """Load missions and truth, forecast once per mission, compare, and summarize."""
    cases = load_development(args.calibration,config)
    if not cases:
        raise ValueError('No calibrated scenarios')
    check_memory(config)
    actual, evidence = load_ground_truth(args.calibration,cases,config,args.ground_truth)
    out = Path(args.out)
    journal = out/'executions.jsonl'
    run, rows = open_accuracy(out,args.calibration,cases,evidence,DELAYS,
                             PREDICTION_DELAYS,config,args.resume)
    scenarios = {s.seed:s for s,_,_ in cases}
    for row in run['ground_truth']['generated']:
        key = row['seed'],row['delay_s']
        if key in actual or key[0] not in scenarios or key[1] not in DELAYS:
            raise ValueError('Duplicate or unexpected cached ground-truth flight')
        validate_flight(row,scenarios[key[0]],config)
        actual[key] = row
    for row in rows:
        key = row['seed'],row['delay_s']
        forecast = run['predictions'].get(str(row['seed']))
        if key not in actual or key[1] not in DELAYS or forecast is None or row != comparison(scenarios[key[0]],actual[key],forecast):
            raise ValueError('Saved comparison differs from ground truth or its frozen forecast')
    completed = {(row['seed'],row['delay_s']) for row in rows}
    run.update(status='running')
    run.pop('error',None)
    save_accuracy(out,run,completed)
    try:
        if len(run['predictions']) < len(cases):
            from model.scg import load_scg_platform_params
            check_memory(config)
            platform = load_scg_platform_params()
            model = LightweightPredictiveModel(*platform)
            try:  # Same one-control-step kernel warmup; not an extra forecast.
                controller,observation,info,_ = model.start(cases[0][1])
                model.step(controller.select_action(observation,info),observation,info)
            finally:
                model.close({})
            check_memory(config)
        for scenario,snapshot,_ in cases:
            forecast = run['predictions'].get(str(scenario.seed))
            if forecast is None:
                check_work(work,config)
                forecast = predict_delays(snapshot,platform)  # Original grid, unchanged.
                run['predictions'][str(scenario.seed)] = forecast
                write_json(out/'run.json',run)  # Persist before appending comparisons.
            for delay in DELAYS:
                key = scenario.seed,delay
                if key in completed:
                    continue
                if key not in actual:
                    result,replayed,_ = execute(scenario,delay,policy='no_recovery',
                                               config=config,work=work)
                    check_approval_snapshot(replayed,snapshot)
                    validate_flight(result,scenario,config)
                    actual[key] = result
                    run['ground_truth']['generated'].append(result)
                    run['ground_truth']['new_pybullet_flights'] += 1
                    write_json(out/'run.json',run)  # Keep new truth across interruption.
                append_json(journal,comparison(scenario,actual[key],forecast))
                completed.add(key)
            save_accuracy(out,run,completed)
            print(f'seed={scenario.seed}: {forecast["computation_s"]*1000:.2f} ms; '
                  f'{len(completed)}/{len(cases)*len(DELAYS)} comparisons',flush=True)
        result = summarize(journal,run['protocol'])
        if result['missing_comparisons'] or len(run['predictions']) != len(cases):
            raise ValueError('Accuracy experiment is incomplete')
        run.update(status='complete',report_markdown=report_markdown(result))
    except BaseException as error:
        run.update(status='interrupted',error=dict(type=type(error).__name__,message=str(error)))
        raise
    finally:
        save_accuracy(out,run,completed)
        write_json(out/'metrics.json',summarize(journal,run['protocol']))
    print(f'Complete: {len(completed)} comparisons; {len(cases)} cached forecasts; '
          f'{run["ground_truth"]["new_pybullet_flights"]} new No-Recovery flights.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration','--development',dest='calibration',type=Path,
                        default=ROOT/'results/development/calibration')
    parser.add_argument('--ground-truth',type=Path,action='append',default=[],
                        help='Optional saved No-Recovery journal; repeat to combine compatible sources')
    parser.add_argument('--out',type=Path,help='Default: ttl_pred_accuracy beside the calibration directory')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--max-trials',type=int,default=40)
    parser.add_argument('--max-seconds',type=float,default=300)
    args = parser.parse_args()
    if args.out is None:
        folder = args.calibration if args.calibration.is_dir() else args.calibration.parent
        args.out = folder.parent/'ttl_pred_accuracy'
    work,config = start_work(args.max_trials,args.max_seconds),Config()
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        run_accuracy(args,work,config)


if __name__ == '__main__':
    main()
