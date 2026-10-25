"""Calibrate fixed Geofence Turn missions using actual D=0 PyBullet flights."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import geofence_turn  # Set CPU/thread limits before importing numerical libraries.
import argparse
from dataclasses import replace
import fcntl
import gc
import json
import math
import numpy as np
from geofence_turn.scenario import Config, HZ, RAMP_S, generate_scenario, scenario_dict
from geofence_turn.simulation import execute
from geofence_turn.storage import (ROOT, DEFAULT_ENVELOPE, open_calibration,
                                   save_progress, save_snapshot)


def find_nominal_bound(scenario, config=Config()):
    """Fly straight east; round the first actual violation up to the approval grid."""
    arrival = (scenario.fence_x-scenario.initial_x)/scenario.speed + RAMP_S/2
    probe_end = arrival + scenario.command.arc_s + config.checkpoint_north_travel_s
    probe = replace(scenario,deadline_s=probe_end-scenario.approval_s)
    result, _, _ = execute(probe,delay=None,policy='no_recovery',config=config)
    if result['first_violation_s'] is None:
        raise RuntimeError('Nominal probe did not cross; no measured calibration upper bound')
    return math.ceil(result['first_violation_s']/config.approval_grid_s)*config.approval_grid_s


def approval_candidates(scenario, upper, config=Config()):
    """Latest to earliest on the unchanged 0.20 s grid, ending at 0.60 s."""
    first = round(upper*HZ)
    last = round(config.approval_min_s*HZ)
    stride = round(config.approval_grid_s*HZ)
    for step in range(first,last-1,-stride):
        yield replace(scenario,approval_s=step/HZ)


def verify_calibration(selected, snapshot, config=Config()):
    """Repeat the selected flight and require the same scientific snapshot."""
    result, replayed, _ = execute(selected,delay=0,policy='no_recovery',config=config)
    assert result['safe_completion'],result
    np.testing.assert_array_equal(snapshot.state,replayed.state)
    for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
        np.testing.assert_array_equal(getattr(snapshot.pid_state,name),getattr(replayed.pid_state,name))
    np.testing.assert_array_equal(snapshot.remaining_reference,replayed.remaining_reference)
    assert snapshot.command == replayed.command == selected.command
    return result


def calibrate_scenarios(seeds, target, output, resume=False, config=Config(), envelope_path=DEFAULT_ENVELOPE):
    """Generate each mission, find its latest safe approval, verify, and save."""
    seeds = list(seeds)
    if not 1 <= target <= len(seeds) or len(set(seeds)) != len(seeds):
        raise ValueError('Use distinct ordered seeds and 1 <= target <= seed count')
    output = Path(output)
    envelope = json.loads(Path(envelope_path).read_text())
    data = open_calibration(output,seeds,target,envelope_path,config,resume)
    if data['calibration']['status'] in ('complete','exhausted'):
        return data
    save_progress(output,data)
    completed = {row['seed'] for row in data['calibration']['attempts']}

    for seed in seeds:
        if len(data['scenarios']) >= target:
            break
        if seed in completed:
            continue
        gc.collect()  # Release closed simulation objects between scenarios.
        try:
            scenario = generate_scenario(seed,envelope,config)
        except ValueError as error:
            if not str(error).startswith('checkpoint_geometry:'):
                raise
            data['calibration']['attempts'].append(dict(seed=seed,accepted=False,
                candidate_count=0,rejection_reason=str(error)))
            save_progress(output,data)
            print(f'seed={seed} rejected: {error}',flush=True)
            continue

        upper = find_nominal_bound(scenario,config)
        selected = None
        candidate_count = 0
        for candidate in approval_candidates(scenario,upper,config):
            result, snapshot, _ = execute(candidate,delay=0,policy='no_recovery',config=config)
            candidate_count += 1
            if result['safe_completion']:
                selected = candidate
                break

        if selected is not None:
            verification = verify_calibration(selected,snapshot,config)
            snapshot_path = output/f'snapshots/seed_{seed}.pkl'
            checksum = save_snapshot(snapshot_path,snapshot)
            data['scenarios'].append(dict(scenario=scenario_dict(selected),
                approval_snapshot=str(snapshot_path.relative_to(output)),
                snapshot_sha256=checksum,d0_verification=verification))

        data['calibration']['attempts'].append(dict(seed=seed,accepted=selected is not None,
            candidate_count=candidate_count,search_upper_s=upper,grid_s=config.approval_grid_s,
            rejection_reason=None if selected else 'no_feasible_approval_on_grid'))
        save_progress(output,data)
        print(f'seed={seed} approval={selected.approval_s if selected else None} '
              f'accepted={len(data["scenarios"])}/{target}',flush=True)

    data['calibration']['status'] = 'complete' if len(data['scenarios']) >= target else 'exhausted'
    save_progress(output,data)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start-seed',type=int,default=0)
    parser.add_argument('--attempts',type=int,default=10)
    parser.add_argument('--target',type=int,default=5)
    parser.add_argument('--out',type=Path,default=ROOT/'results/development/calibration')
    parser.add_argument('--resume',action='store_true')
    args = parser.parse_args()
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        data = calibrate_scenarios(range(args.start_seed,args.start_seed+args.attempts),
                                   args.target,args.out,resume=args.resume)
    print(f'{data["calibration"]["status"]}: {data["statistics"]}',flush=True)
    if data['calibration']['status'] != 'complete':
        raise SystemExit('Candidate pool exhausted before the requested target')


if __name__ == '__main__':
    main()
