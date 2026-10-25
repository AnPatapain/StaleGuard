"""Locked 20-scenario safety-accuracy experiment; reuse saved No-Recovery flights."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

import geofence_turn  # CPU-only, one numerical thread, before importing the model.
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import time
from geofence_turn.storage import (ROOT, load_development, metadata, check_provenance,
                                     file_hash, write_json, append_json)
from geofence_turn.scenario import Config, SCHEMA, HZ, PHYSICS_HZ, scenario_dict
from geofence_turn.simulation import start_work, check_work, check_memory
from geofence_turn.baselines.staleguard import (predict_delays, staleguard_status,
                                               PREDICTION_DELAYS, LightweightPredictiveModel)
from geofence_turn.ttl_pred_accuracy.metrics import read_records, summarize, report_markdown

SEEDS = (0,1,2,3,4,6,7,8,9,10,11,12,14,15,16,17,18,19,20,21)
DELAYS = (0.,.02,.04,.06,.08,.10,.12,.14,.16,.18,.20,.30,.40,.50,.60,.70,.80,.90,1.)
GROUND_TRUTH = ROOT/'support/results/delay_tolerance_20/flights.jsonl'


def validate_ground_truth(cases, config):
    """Require the original command, snapshot, physics and full-flight monitor."""
    folder = GROUND_TRUTH.parent
    saved = json.loads((folder/'metadata.json').read_text())
    original_protocol = json.loads((folder/'protocol.json').read_text())
    finished = json.loads((folder/'finished.json').read_text())
    if saved['schema'] != SCHEMA or saved['config'] != asdict(config):
        raise ValueError('Ground-truth mission or safety configuration differs')
    current = metadata(config)
    for field in ('python','numpy','platform','numerical_threads','cpu_only','simulation_workers'):
        if saved[field] != current[field]:
            raise ValueError(f'Ground-truth numerical environment differs: {field}')
    # I/O organization changed after these flights. Scientific modules must still
    # match byte-for-byte; current full source provenance is mandatory on resume.
    scientific = ('__init__.py','scenario.py','simulation.py','baselines/staleguard.py',
                  'baselines/use_time_recheck.py','baselines/fixed_ttl.py')
    checked = {}
    for name in scientific:
        if file_hash(ROOT/name) != saved['source_hashes'][name]:
            raise ValueError(f'Ground-truth scientific source differs: {name}')
        checked[str(ROOT/name)] = file_hash(ROOT/name)
    shared = json.loads((ROOT/'support/logs/provenance.json').read_text())['shared_source_sha256']
    for name, checksum in {**shared,**original_protocol['shared_model_sha256']}.items():
        path = ROOT.parent/name
        if file_hash(path) != checksum:
            raise ValueError(f'Ground-truth shared model/environment source differs: {name}')
        checked[str(path.resolve())] = checksum
    if (original_protocol['policy'] != 'no_recovery' or original_protocol['delays_s'] != list(DELAYS)
            or original_protocol['delay_steps'] != [round(delay*HZ) for delay in DELAYS]
            or original_protocol['control_hz'] != HZ or original_protocol['observation_hz'] != PHYSICS_HZ):
        raise ValueError('Ground-truth dispatch or observation protocol differs')
    checksum = file_hash(GROUND_TRUTH)
    if finished['status'] != 'SUCCESS' or finished['delay_sweep_flights'] != 380 or finished['journal_sha256'] != checksum:
        raise ValueError('Ground-truth journal differs from its completed experiment')
    original = load_development(folder/'calibrated_scenarios.json',config)
    if [s.seed for s,_,_ in cases] != list(SEEDS) or [s.seed for s,_,_ in original] != list(SEEDS):
        raise ValueError('Exactly the locked 20 calibrated scenarios are required')
    for (s,_,path),(old,_,old_path) in zip(cases,original):
        if s != old or file_hash(path) != file_hash(old_path):
            raise ValueError(f'Ground-truth scenario/command/snapshot differs: {s.seed}')
    rows = [json.loads(line) for line in GROUND_TRUTH.read_text().splitlines()]
    keyed = {}
    scenarios = {s.seed:s for s,_,_ in cases}
    for row in rows:
        key = row['seed'],row['delay_s']
        if key in keyed or key[0] not in scenarios or key[1] not in DELAYS:
            raise ValueError(f'Duplicate or unexpected ground-truth flight: {key}')
        scenario = scenarios[key[0]]
        total_steps = round(scenario.approval_s*HZ)+math.ceil(scenario.deadline_s*HZ-1e-10)
        dispatch = round((scenario.approval_s+row['delay_s'])*HZ)
        event = row['event']
        violation = row['first_violation_s']
        if (event['policy'] != 'no_recovery' or event['fallback_step'] is not None
                or event['application_step'] != dispatch or event['scheduled_dispatch_step'] != dispatch
                or row['horizon_s'] != total_steps/HZ
                or row['physics_samples'] != total_steps*(PHYSICS_HZ//HZ)+1
                or row['expected_physics_samples'] != row['physics_samples']):
            raise ValueError(f'Ground truth did not monitor the whole specified flight: {key}')
        if (type(row['safe']) is not bool or type(row['completed']) is not bool
                or row['safe'] != (violation is None)
                or row['safe'] != (row['min_clearance_m'] >= config.d_min_m)
                or row['safe_completion'] != (row['safe'] and row['completed'])
                or (violation is not None and not 0 <= violation <= row['horizon_s'])):
            raise ValueError(f'Ground-truth safety/completion fields disagree: {key}')
        predispatch = violation if violation is not None and violation <= dispatch/HZ else None
        if row['pre_dispatch_violation_s'] != predispatch:
            raise ValueError(f'Ground-truth waiting-period safety differs: {key}')
        if row['delay_s'] == 0 and not row['safe_completion']:
            raise ValueError(f'Calibrated D=0 mission did not complete safely: {key}')
        keyed[key] = row
    if set(keyed) != {(seed,delay) for seed in SEEDS for delay in DELAYS}:
        raise ValueError('Ground truth is incomplete; do not silently mix datasets')
    evidence = dict(status='passed',source=str(GROUND_TRUTH),sha256=checksum,flights=len(rows),
        new_pybullet_flights=0,scientific_source_sha256=checked,
        input_sha256={str(path):file_hash(path) for path in
            (folder/'metadata.json',folder/'protocol.json',folder/'finished.json',
             folder/'calibrated_scenarios.json',ROOT/'support/logs/provenance.json')},
        checks=['exact 20 seeds and 19 delays, once each','identical frozen commands, scenarios and snapshot bytes',
                'identical scientific source, shared model, PID and UAV body asset',
                'matching Python, NumPy, platform, configuration and rates',
                '1000 Hz safety throughout waiting and the entire deadline horizon',
                'unchanged 50 Hz dispatch, no recovery, and all D=0 safe completions'],
        io_source_note='Historical calibration/benchmark storage code changed; scientific modules and inputs are independently required to match. Resume requires all current sources to match.')
    return keyed,evidence


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
    cases = load_development(args.development,config)
    actual,evidence = validate_ground_truth(cases,config)
    source = Path(args.development)
    if source.is_dir():
        source = source/'calibrated_scenarios.json'
    protocol = dict(seeds=list(SEEDS),delays_s=list(DELAYS),prediction_grid_s=list(PREDICTION_DELAYS),
        scenarios=[scenario_dict(s) for s,_,_ in cases],control_hz=HZ,observation_hz=PHYSICS_HZ,
        snapshot_sha256={str(s.seed):file_hash(path) for s,_,path in cases},dataset_sha256=file_hash(source),
        config=asdict(config),prediction_target='full-flight geofence safety, not completion',
        readiness='assumed available; reject iff waited_steps >= expiry_steps',
        violation_time='first_violation_s is relative to approval; first_violation_flight_s retains source absolute time',
        timing='existing predictor call-to-result timer; one sample per scenario; one numerical-kernel control-step warmup')
    protocol = json.loads(json.dumps(protocol))  # Canonical JSON lists on fresh run and resume.
    out = Path(args.out)
    journal,path = out/'executions.jsonl',out/'run.json'
    if args.resume:
        run = json.loads(path.read_text())
        if run.get('format') != 'compact_ttl_accuracy_v1':
            raise ValueError('Incompatible accuracy artifact format')
        check_provenance(run['provenance'],config)
        if run['protocol'] != protocol or run['ground_truth'] != evidence:
            raise ValueError('Resume protocol, configuration, snapshots or ground truth changed')
        data = journal.read_bytes()
        committed = run['journal_bytes']
        if len(data) < committed or hashlib.sha256(data[:committed]).hexdigest() != run['journal_sha256']:
            raise ValueError('Previously committed comparisons changed or are missing')
        rows = read_records(journal,repair_tail=True)
    else:
        if out.exists() and any(out.iterdir()):
            raise FileExistsError('Output contains results; use --resume or a fresh --out directory')
        out.mkdir(parents=True,exist_ok=True)
        journal.touch()
        rows = []
        run = dict(format='compact_ttl_accuracy_v1',provenance=metadata(config),protocol=protocol,
            ground_truth=evidence,predictions={},sessions=[],status='running',journal_bytes=0,
            journal_sha256=hashlib.sha256(b'').hexdigest())
    scenarios = {s.seed:s for s,_,_ in cases}
    if not set(run['predictions']) <= {str(seed) for seed in SEEDS}:
        raise ValueError('Unexpected cached scenario')
    for row in rows:
        key = row['seed'],row['delay_s']
        forecast = run['predictions'].get(str(row['seed']))
        if key not in actual or forecast is None or row != comparison(scenarios[row['seed']],actual[key],forecast):
            raise ValueError('Saved comparison differs from ground truth or its frozen forecast')
    completed = {(row['seed'],row['delay_s']) for row in rows}
    session = dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),prediction_calls=0,warmup_model_steps=0)
    run['sessions'].append(session)
    run.update(status='running')
    run.pop('error',None)

    def checkpoint():
        data = journal.read_bytes()
        run.update(journal_bytes=len(data),journal_sha256=hashlib.sha256(data).hexdigest(),
                   completed_comparisons=len(completed),cached_forecasts=len(run['predictions']))
        session.update(elapsed_s=time.monotonic()-work['start'])
        write_json(path,run)

    checkpoint()
    try:
        if len(run['predictions']) < len(SEEDS):
            from model.scg import load_scg_platform_params
            check_memory(config)
            platform = load_scg_platform_params()
            model = LightweightPredictiveModel(*platform)
            try:
                controller,observation,info,_ = model.start(cases[0][1])
                model.step(controller.select_action(observation,info),observation,info)
                session['warmup_model_steps'] = 1
            finally:
                model.close({})
            check_memory(config)
        for scenario,snapshot,_ in cases:
            forecast = run['predictions'].get(str(scenario.seed))
            if forecast is None:
                check_work(work,config)
                forecast = predict_delays(snapshot,platform)  # Original default grid, unchanged.
                session['prediction_calls'] += 1
                run['predictions'][str(scenario.seed)] = forecast
                checkpoint()
                print(f'seed={scenario.seed}: {forecast["computation_s"]*1000:.2f} ms; expiry_steps={forecast["expiry_steps"]}',flush=True)
            for delay in DELAYS:
                key = scenario.seed,delay
                if key not in completed:
                    append_json(journal,comparison(scenario,actual[key],forecast))
                    completed.add(key)
                    checkpoint()
        result = summarize(journal,protocol)
        if result['total_comparisons'] != 380 or result['raw_prediction']['total_tests'] != 220 or result['ttl_dispatch_decision']['total_tests'] != 380 or len(run['predictions']) != 20:
            raise ValueError('Locked accuracy experiment is incomplete')
        run.update(status='complete',report_markdown=report_markdown(result))
        run['validation'] = dict(unique_comparisons=380,scenarios=20,dispatch_delays=19,cached_forecasts=20,
            raw_tests=220,ttl_tests=380,new_ground_truth_flights=0,confusion_invariants='passed',
            algorithms_and_grid_unchanged=True,ground_truth_unchanged=True)
        session['final_rss_mb'],session['available_mb'] = check_memory(config)
    except BaseException as error:
        run.update(status='interrupted',error=dict(type=type(error).__name__,message=str(error)))
        raise
    finally:
        checkpoint()
        write_json(out/'metrics.json',summarize(journal,protocol))
    print(f'Complete: {len(completed)} comparisons; 20 cached approval forecasts; zero new ground-truth flights.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',default=str(ROOT/'results/development/ttl_pred_accuracy'))
    parser.add_argument('--development',default=str(ROOT/'results/development/calibration'))
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--max-trials',type=int,default=40)
    parser.add_argument('--max-seconds',type=float,default=300)
    args = parser.parse_args()
    work,config = start_work(args.max_trials,args.max_seconds),Config()
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        run_accuracy(args,work,config)


if __name__ == '__main__':
    main()
