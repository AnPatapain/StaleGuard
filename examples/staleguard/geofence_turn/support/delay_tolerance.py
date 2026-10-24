"""Task 1: unchanged calibration followed by a bounded No-Recovery delay sweep."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import geofence_turn  # Set the existing CPU/thread limits before NumPy.
import argparse
from dataclasses import asdict
import fcntl
import json
import math
import os
import subprocess
import time
import numpy as np
from geofence_turn import simulation
from geofence_turn.calibration import (ROOT, DEFAULT_DEVELOPMENT, DEFAULT_ENVELOPE, append_json,
    file_hash, load_development, prepare_output, write_json)
from geofence_turn.scenario import Config, HZ, PHYSICS_HZ, SCHEMA, scenario_dict

DELAY_STEPS = tuple(range(11)) + tuple(range(15, 51, 5))


def read(path):
    return json.loads(Path(path).read_text())


def initialize(out):
    config = Config()
    cases = load_development(DEFAULT_DEVELOPMENT, config)
    assert len(cases) == 5
    if out.exists():
        prepare_output(out, config, resume=True)
        protocol = read(out/'protocol.json')
        assert protocol['delay_steps'] == list(DELAY_STEPS)
        assert protocol['existing_snapshot_sha256'] == {str(s.seed):file_hash(p) for s,_,p in cases}
    else:
        prepare_output(out, config)
        write_json(out/'protocol.json', dict(
            purpose='Diagnose delay tolerance, without changing the calibration rule.',
            existing_seeds=[s.seed for s,_,_ in cases], additional_accepted_target=15,
            first_new_seed=max(s.seed for s,_,_ in cases)+1,
            seed_order='Consecutive seeds; keep every rejection; stop at 15 additional accepted cases.',
            delay_steps=list(DELAY_STEPS), delays_s=[step/HZ for step in DELAY_STEPS],
            policy='no_recovery', control_hz=HZ, observation_hz=PHYSICS_HZ,
            existing_snapshot_sha256={str(s.seed):file_hash(p) for s,_,p in cases},
            turning_envelope_sha256=file_hash(DEFAULT_ENVELOPE),
            shared_model_sha256={str(p.relative_to(ROOT.parent)):file_hash(p)
                                 for p in (ROOT.parent/'inspection_descent/model').glob('*.py')},
            storage='One flight journal; no per-flight trajectory files.'))
    return config, cases


def calibrate_additional(out, config, existing):
    """Use the original CLI, with its original selection and resource guards.

    Generate only as many seeds as accepted cases still needed. Thus every
    generated seed is attempted, even when geometry rejects some of them.
    Each subprocess retains the existing <=20-seed, <=200-trial limits.
    """
    cases = list(existing)
    next_seed = max(s.seed for s,_,_ in cases)+1
    sources = [DEFAULT_DEVELOPMENT]
    attempts = 5
    batch = 1
    while len(cases) < 20:
        remaining = 20-len(cases)
        directory = out/f'calibration_{batch:02d}'
        command = [sys.executable, str(ROOT/'calibration.py'), '--out', str(directory),
                   '--start-seed', str(next_seed), '--attempts', str(remaining),
                   '--target', str(remaining), '--max-trials', '200', '--max-seconds', '900']
        compact = directory/'calibrated_scenarios.json'
        done = (read(compact).get('calibration',{}).get('status') == 'complete'
                if compact.exists() and read(compact).get('format') == 'compact_calibration_v1'
                else (directory/'finished.json').exists())
        if not done:
            if directory.exists():
                command.append('--resume')
            subprocess.run(command, check=True)
        selected = load_development(directory, config)
        if compact.exists() and read(compact).get('format') == 'compact_calibration_v1':
            summary = read(compact)['statistics']
            generated_count = summary['generated']
        else:
            summary = read(directory/'summary.json')
            generated_count = len(read(directory/'generated.json'))
        assert summary['attempted'] == remaining
        assert generated_count == remaining
        cases.extend(selected)
        sources.append(directory)
        attempts += summary['attempted']
        next_seed += summary['attempted']
        batch += 1
        if batch > 20:
            raise RuntimeError('Twenty bounded calibration batches exhausted; retain partial evidence')
    assert len(cases) == len({s.seed for s,_,_ in cases}) == 20
    entries = [dict(scenario=scenario_dict(s), approval_snapshot=os.path.relpath(p,out),
                    snapshot_sha256=file_hash(p)) for s,_,p in cases]
    write_json(out/'calibrated_scenarios.json', dict(schema=SCHEMA, config=asdict(config),
        provenance=dict(path='metadata.json',sha256=file_hash(out/'metadata.json')),
        calibration_sources=[dict(path=os.path.relpath(p,out),
                                 dataset_sha256=file_hash(p/('calibrated_scenarios.json' if (p/'calibrated_scenarios.json').exists() else 'summary.json'))) for p in sources],
        generated_and_attempted=attempts, accepted=20, scenarios=entries))
    print(f'Combined calibration: {len(cases)}/{attempts} accepted; original five reused.', flush=True)


def fly_with_checkpoint_observations(s, delay, config, work):
    """Read extra checkpoint observations without changing control or physics."""
    original_monitor = simulation.monitor_physics_steps
    lower = np.asarray(s.command.checkpoint_lower)
    upper = np.asarray(s.command.checkpoint_upper)
    observed = dict(position_seen=False, valid_velocity_seen=False, north_altitude_samples=0,
                    min_x_at_north_altitude=None, max_x_at_north_altitude=None,
                    longest_position_dwell_s=0., longest_qualified_dwell_s=0.)
    entered_position = entered_qualified = None

    def monitor(env, observer):
        def observe(position, tick):
            nonlocal entered_position, entered_qualified
            observer(position, tick)  # The original 1 kHz safety/completion check.
            elapsed = tick/PHYSICS_HZ-s.approval_s
            if not 0 <= elapsed <= s.deadline_s:
                return
            position = np.asarray(position)
            in_position = bool(np.all(position >= lower) and np.all(position <= upper))
            if np.all(position[1:] >= lower[1:]) and np.all(position[1:] <= upper[1:]):
                observed['north_altitude_samples'] += 1
                x = float(position[0])
                old_min, old_max = observed['min_x_at_north_altitude'], observed['max_x_at_north_altitude']
                observed['min_x_at_north_altitude'] = x if old_min is None else min(x,old_min)
                observed['max_x_at_north_altitude'] = x if old_max is None else max(x,old_max)
            velocity, _ = simulation.p.getBaseVelocity(env.DRONE_ID, physicsClientId=env.PYB_CLIENT)
            qualified = in_position and np.linalg.norm(np.asarray(velocity)-[0.,s.speed,0.]) <= config.velocity_tolerance_mps
            observed['position_seen'] |= in_position
            observed['valid_velocity_seen'] |= bool(qualified)
            if in_position:
                entered_position = tick if entered_position is None else entered_position
                observed['longest_position_dwell_s'] = max(observed['longest_position_dwell_s'],(tick-entered_position)/PHYSICS_HZ)
            else:
                entered_position = None
            if qualified:
                entered_qualified = tick if entered_qualified is None else entered_qualified
                observed['longest_qualified_dwell_s'] = max(observed['longest_qualified_dwell_s'],(tick-entered_qualified)/PHYSICS_HZ)
            else:
                entered_qualified = None
        return original_monitor(env, observe)

    simulation.monitor_physics_steps = monitor
    try:
        result, snapshot, _ = simulation.execute(s,delay,config=config,work=work)
    finally:
        simulation.monitor_physics_steps = original_monitor
    minimum = observed['min_x_at_north_altitude']
    observed['entire_north_altitude_pass_east_of_box'] = bool(minimum is not None and minimum > upper[0])
    observed['minimum_east_miss_m'] = None if minimum is None else float(max(0.,minimum-upper[0]))
    return result, snapshot, observed


def evaluate(out, config, start_index, count):
    cases = load_development(out/'calibrated_scenarios.json', config)
    assert len(cases) == 20 and 1 <= count <= 7
    rows = [] if not (out/'flights.jsonl').exists() else [json.loads(line) for line in (out/'flights.jsonl').read_text().splitlines()]
    completed = {(row['seed'],round(row['delay_s']*HZ)) for row in rows}
    assert len(completed) == len(rows)
    work = simulation.start_work(160,900)
    for s, stored, _ in cases[start_index:start_index+count]:
        for step in DELAY_STEPS:
            if (s.seed,step) in completed:
                continue
            result, snapshot, observed = fly_with_checkpoint_observations(s,step/HZ,config,work)
            np.testing.assert_array_equal(snapshot.state,stored.state)
            np.testing.assert_array_equal(snapshot.remaining_reference,stored.remaining_reference)
            for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
                np.testing.assert_array_equal(getattr(snapshot.pid_state,name),getattr(stored.pid_state,name))
            assert snapshot.command == stored.command == s.command
            assert result['event']['application_step'] == round(s.approval_s*HZ)+step
            assert result['completed'] == (observed['longest_qualified_dwell_s'] >= config.dwell_s)
            if step == 0:
                assert result['safe_completion'], result
            result['checkpoint_observations'] = observed
            append_json(out/'flights.jsonl',result)
            print(f"seed={s.seed} D={step/HZ:.2f} safe={result['safe']} completed={result['completed']} predispatch={result['pre_dispatch_violation_s']}",flush=True)
    print(f'Batch finished: {work["trials"]} new flights in {time.monotonic()-work["start"]:.1f} s.',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task',choices=('calibrate','evaluate'))
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--start-index',type=int,default=0)
    parser.add_argument('--count',type=int,default=7)
    args = parser.parse_args()
    out = args.out.resolve()
    config, existing = initialize(out)
    if args.task == 'calibrate':
        calibrate_additional(out,config,existing)
    else:
        with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            evaluate(out,config,args.start_index,args.count)


if __name__ == '__main__':
    main()
