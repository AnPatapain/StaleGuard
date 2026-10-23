"""Bounded, serial development commands. Run as python -m geofence_turn.cli."""
import argparse
import ctypes
import gc
from collections import Counter
from dataclasses import asdict, replace
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import platform as system_platform
import sys
import time
import numpy as np
from .mission import SCHEMA, Config, Command, Scenario, RAMP_S, scenario_dict, scenario_from_dict
from .scenario import generate_scenario, calibrate
from .simulation import execute, check_memory

ROOT = Path(__file__).resolve().parent


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def append_json(path, value):
    with Path(path).open('a') as stream:
        stream.write(json.dumps(value, allow_nan=False)+'\n')
        stream.flush()
        os.fsync(stream.fileno())


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def metadata(config):
    return dict(schema=SCHEMA, config=asdict(config), python=sys.version, executable=sys.executable,
                platform=system_platform.platform(), numpy=np.__version__,
                numerical_threads=1, cpu_only=True, simulation_workers=1,
                source_hashes={p.name:file_hash(p) for p in ROOT.glob('*.py')},
                started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))


class Budget:
    def __init__(self, max_trials, max_seconds, config):
        self.trials, self.max_trials = 0, max_trials
        self.start, self.max_seconds, self.config = time.monotonic(), max_seconds, config

    def execute(self, *args, **kwargs):
        if self.trials >= self.max_trials or time.monotonic()-self.start >= self.max_seconds:
            raise RuntimeError('Explicit development trial/time budget reached; partial logs preserved')
        # Match the existing serial runner's allocator cleanup before its guard.
        gc.collect()
        ctypes.CDLL(None).malloc_trim(0)
        check_memory(self.config)
        self.trials += 1
        return execute(*args, config=self.config, **kwargs)


def characterize(out, budget, config):
    # Preregistered engineering sweep; no policies evaluated in this stage.
    journal = out/'sweep.jsonl'
    results = [] if not journal.exists() else [json.loads(line) for line in journal.read_text().splitlines()]
    completed = {(row['speed'], row['radius']) for row in results}
    if len(completed) != len(results):
        raise ValueError('Duplicate characterization pair in saved journal')
    for speed in (.4, .6, .8):
        for radius in (.2, .4, .6):
            if (speed, radius) in completed:
                continue
            approval = 1.2
            center_x = speed*(approval-RAMP_S/2)+radius
            center_y = radius+speed*config.checkpoint_north_travel_s
            tol = config.position_tolerance_m
            cmd = Command(radius, speed,
                          (center_x-config.checkpoint_width_m/2, center_y-tol, 1.25-tol),
                          (center_x+config.checkpoint_width_m/2, center_y+tol, 1.25+tol))
            deadline = round((cmd.arc_s+config.checkpoint_north_travel_s+1.5)*50)/50
            s = Scenario(42, speed, 1.25, 0., 0., 10., deadline, cmd, approval)
            result, _, trace = budget.execute(s, save_trace=True)
            feasible = result['safe_completion'] and result['max_tracking_error_m'] <= config.tracking_limit_m
            row = dict(speed=speed, radius=radius, lateral_acceleration_mps2=speed**2/radius,
                       feasible=bool(feasible), scenario=scenario_dict(s), outcome=result)
            results.append(row)
            np.savez_compressed(out/f'trace_v{speed}_r{radius}.npz', controls=trace)
            append_json(out/'sweep.jsonl', row)
            print(f'v={speed:.1f} R={radius:.1f} feasible={feasible} max_error={result["max_tracking_error_m"]:.4f} completion={result["completion_s"]}', flush=True)
    write_json(out/'envelope.json', dict(config=asdict(config), results=results))


def generate(out, args, config):
    envelope = json.loads(Path(args.envelope).read_text())
    rows = []
    for seed in range(args.start_seed, args.start_seed+args.attempts):
        try:
            rows.append(scenario_dict(generate_scenario(seed, envelope, config)))
        except ValueError as error:
            if not str(error).startswith('checkpoint_geometry:'):
                raise
            rows.append(dict(seed=seed, generator_rejection=str(error)))
    write_json(out/'generated.json', rows)
    print(f'Generated {len(rows)} distinct seeds; no simulation or target adjustment.', flush=True)
    return rows


def development(out, args, budget, config):
    if args.resume:
        generated = json.loads((out/'generated.json').read_text())
    elif args.generated:
        generated = json.loads(Path(args.generated).read_text())
        if len(generated) > args.attempts:
            generated = generated[:args.attempts]
    else:
        generated = generate(out, args, config)
    if not (out/'generated.json').exists():
        write_json(out/'generated.json', generated)
    prior = [] if not (out/'attempts.jsonl').exists() else [json.loads(line) for line in (out/'attempts.jsonl').read_text().splitlines()]
    accepted = [row for row in prior if row['accepted']]
    completed_seeds = {row['seed'] for row in prior}
    attempted = len(prior)
    rejections = Counter() if not prior else Counter(json.loads((out/'summary.json').read_text())['candidate_rejections'])
    for row in generated:
        if len(accepted) >= args.target:
            break
        if row['seed'] in completed_seeds:
            continue
        attempted += 1
        if 'generator_rejection' in row:
            record = dict(seed=row['seed'], generated=row, accepted=False, candidate_count=0,
                          rejection_reason=row['generator_rejection'], selected=None)
            rejections['checkpoint_geometry'] += 1
            append_json(out/'attempts.jsonl', record)
            write_json(out/'summary.json', dict(attempted=attempted, accepted=len(accepted),
                       acceptance_rate=len(accepted)/attempted, candidate_rejections=dict(rejections), scenarios=accepted))
            print(f"seed={row['seed']} rejected at generation: {row['generator_rejection']}", flush=True)
            continue
        s = scenario_from_dict(row)
        directory = out/f'seed_{s.seed}'
        directory.mkdir(exist_ok=args.resume)
        journal = directory/'candidates.jsonl'
        cached = {} if not journal.exists() else {entry['approval_s']:entry for entry in
                  (json.loads(line) for line in journal.read_text().splitlines())}
        def audit(candidate):
            if candidate['approval_s'] in cached:
                assert json.dumps(candidate,sort_keys=True) == json.dumps(cached[candidate['approval_s']],sort_keys=True)
            else:
                append_json(journal, candidate)
        def evaluate(candidate):
            saved = cached.get(candidate.approval_s)
            if saved is None:
                return budget.execute(candidate)
            assert saved['command'] == json.loads(json.dumps(scenario_dict(candidate)['command']))
            snap = None
            if not saved['rejected_because']:
                with (directory/'approval.pkl').open('rb') as stream:
                    snap = pickle.load(stream)
            return saved['outcome'], snap, None
        # One real nominal prefix gives a rigorous scenario-specific upper bound:
        # every later approval already has an irrevocable pre-dispatch violation.
        # Probe past geometric arrival by the arc plus designated north travel;
        # if no crossing is observed, stop rather than inventing a search bound.
        predicted_arrival = (s.fence_x-s.initial_x)/s.speed + RAMP_S/2
        probe_s = predicted_arrival + s.command.arc_s + config.checkpoint_north_travel_s
        if (directory/'nominal_bound.json').exists():
            nominal = json.loads((directory/'nominal_bound.json').read_text())
        else:
            nominal, _, _ = budget.execute(replace(s, deadline_s=probe_s-s.approval_s), None)
            write_json(directory/'nominal_bound.json', nominal)
        if nominal['first_violation_s'] is None:
            raise RuntimeError('Nominal probe did not cross; no measured calibration upper bound')
        upper = math.ceil(nominal['first_violation_s']/config.approval_grid_s)*config.approval_grid_s
        selected, snapshot, rows = calibrate(s, evaluate, config, audit,
                                             latest_approval_s=upper)
        for candidate in rows:
            rejections.update(candidate['rejected_because'])
        record = dict(seed=s.seed, generated=row, accepted=selected is not None,
                      candidate_count=len(rows), search_upper_s=upper, grid_s=config.approval_grid_s, rejection_reason=None if selected else 'no_feasible_approval_on_grid',
                      selected=None if selected is None else scenario_dict(selected))
        if selected is not None:
            # Reuse the existing checkpoint approach: pickle full state/PID/reference
            # alongside human-readable JSON. Only load locally produced snapshots.
            with (directory/'approval.pkl').open('wb') as stream:
                pickle.dump(snapshot, stream)
            write_json(directory/'scenario.json', scenario_dict(selected))
            write_json(directory/'configuration.json', asdict(config))
            replay, replay_snapshot, trace = budget.execute(selected, save_trace=True)
            np.testing.assert_array_equal(snapshot.state, replay_snapshot.state)
            for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
                np.testing.assert_array_equal(getattr(snapshot.pid_state, name), getattr(replay_snapshot.pid_state, name))
            assert replay['safe_completion'], replay
            np.savez_compressed(directory/'d0_trace.npz', controls=trace)
            write_json(directory/'d0_replay.json', replay)
            record.update(replay=replay, snapshot_sha256=file_hash(directory/'approval.pkl'))
            accepted.append(record)
        append_json(out/'attempts.jsonl', record)
        print(f'seed={s.seed} accepted={selected is not None} candidates={len(rows)} latest_approval={None if selected is None else selected.approval_s}', flush=True)
        write_json(out/'summary.json', dict(attempted=attempted, accepted=len(accepted),
                   acceptance_rate=len(accepted)/attempted, candidate_rejections=dict(rejections), scenarios=accepted))
    print(f'Acceptance: {len(accepted)}/{attempted}; candidate rejection reasons={dict(rejections)}', flush=True)


def replay(out, args, budget, config):
    for directory in sorted(Path(args.development).glob('seed_*')):
        if not (directory/'scenario.json').exists():
            continue
        s = scenario_from_dict(json.loads((directory/'scenario.json').read_text()))
        with (directory/'approval.pkl').open('rb') as stream:
            stored = pickle.load(stream)
        assert stored.config == config
        result, snap, trace = budget.execute(s, save_trace=True)
        np.testing.assert_array_equal(snap.state, stored.state)
        np.testing.assert_array_equal(snap.remaining_reference, stored.remaining_reference)
        for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
            np.testing.assert_array_equal(getattr(snap.pid_state, name), getattr(stored.pid_state, name))
        assert snap.command == stored.command == s.command
        assert result['safe_completion'], result
        write_json(out/f'seed_{s.seed}.json', result)
        print(f'D=0 seed={s.seed} PASS clearance={result["min_clearance_m"]:.6f} completion={result["completion_s"]:.3f}', flush=True)


def smoke(out, args, budget, config):
    from .infrastructure import load_scg_platform_params
    from .prediction import predict_delays, predict_immediate, SMOKE_DELAYS
    platform = load_scg_platform_params()
    paths = sorted(Path(args.development).glob('seed_*/scenario.json'))
    if not paths:
        raise ValueError('No calibrated development scenarios')
    # Warm the exact shared Numba kernels; no timing sample includes first JIT.
    with paths[0].with_name('approval.pkl').open('rb') as stream:
        warm = pickle.load(stream)
    predict_delays(warm, platform)
    predict_immediate(warm, platform)
    counts = {p:Counter() for p in ('no_recovery', 'staleguard', 'use_time_recheck', 'fixed_ttl')}
    completed = set()
    journal = out/'executions.jsonl'
    if journal.exists():
        for line in journal.read_text().splitlines():
            result = json.loads(line)
            policy = result['event']['policy']
            key = (result['seed'], result['delay_s'], policy)
            if key in completed:
                raise ValueError(f'Duplicate saved flight: {key}')
            completed.add(key)
            counts[policy]['flights'] += 1
            counts[policy]['safe_completion'] += int(result['safe_completion'])
            counts[policy]['safe_noncompletion'] += int(result['safe'] and not result['completed'])
            counts[policy]['unsafe'] += int(not result['safe'])
            counts[policy]['fallback'] += int(result['event']['fallback_step'] is not None)
    for path in paths:
        s = scenario_from_dict(json.loads(path.read_text()))
        with path.with_name('approval.pkl').open('rb') as stream:
            snapshot = pickle.load(stream)
        assert snapshot.config == config
        # Deliberately before actual D enumeration; no future-delay argument.
        prediction_file = out/f'prediction_seed_{s.seed}.json'
        if prediction_file.exists():
            received = json.loads(prediction_file.read_text())
        else:
            received = predict_delays(snapshot, platform)
            write_json(prediction_file, received)
        for delay in SMOKE_DELAYS:
            for policy in counts:
                if (s.seed, delay, policy) in completed:
                    continue
                result, snap, trace = budget.execute(s, delay, policy=policy,
                                                     received=received, platform=platform,
                                                     save_trace=True)
                np.testing.assert_array_equal(snap.state, snapshot.state)
                assert snap.command == snapshot.command
                np.savez_compressed(out/f'seed{s.seed}_D{delay}_{policy}.npz', controls=trace)
                append_json(out/'executions.jsonl', result)
                counts[policy]['flights'] += 1
                counts[policy]['safe_completion'] += int(result['safe_completion'])
                counts[policy]['safe_noncompletion'] += int(result['safe'] and not result['completed'])
                counts[policy]['unsafe'] += int(not result['safe'])
                counts[policy]['fallback'] += int(result['event']['fallback_step'] is not None)
                print(f'seed={s.seed} D={delay:.1f} {policy} safe={result["safe"]} complete={result["completed"]} fallback={result["event"]["fallback_step"]}', flush=True)
                write_json(out/'summary.json', {p:dict(c) for p,c in counts.items()})


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('command', choices=('characterize', 'generate', 'calibrate', 'replay', 'smoke'))
    parser.add_argument('--out', required=True)
    parser.add_argument('--envelope', default=str(ROOT/'logs/northbound_review/characterization/envelope.json'))
    parser.add_argument('--generated')
    parser.add_argument('--resume', action='store_true', help='Resume interrupted characterization, calibration or smoke')
    parser.add_argument('--development', default=str(ROOT/'logs/northbound_review/development'))
    parser.add_argument('--start-seed', type=int, default=0)
    parser.add_argument('--attempts', type=int, default=10)
    parser.add_argument('--target', type=int, default=5)
    parser.add_argument('--max-trials', type=int, default=160)
    parser.add_argument('--max-seconds', type=float, default=900)
    args = parser.parse_args()
    if not 1 <= args.attempts <= 20 or not 1 <= args.target <= args.attempts:
        parser.error('Development cap: 1 <= target <= attempts <= 20')
    if not 1 <= args.max_trials <= 200 or not 0 < args.max_seconds <= 1800:
        parser.error('Development cap: 1..200 flights and at most 1800 seconds')
    config = Config()
    # Single-worker lock shared by every geofence CLI invocation.
    with (ROOT/'logs/.simulation.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Another geofence development command is running')
        out = Path(args.out)
        if args.resume:
            if args.command not in ('characterize', 'calibrate', 'smoke'):
                parser.error('--resume supports characterization, calibration and smoke logs')
            saved = json.loads((out/'metadata.json').read_text())
            if saved['config'] != asdict(config):
                raise ValueError('Resume configuration differs from original experiment')
            for name in ('mission.py', 'scenario.py', 'simulation.py', 'prediction.py', 'policies.py', 'infrastructure.py'):
                if saved['source_hashes'][name] != file_hash(ROOT/name):
                    raise ValueError(f'Resume physics/policy source changed: {name}')
            append_json(out/'resume_sessions.jsonl', metadata(config))
        else:
            out.mkdir(parents=True, exist_ok=False)  # Never overwrite prior experiments.
            write_json(out/'metadata.json', metadata(config))
        budget = Budget(args.max_trials, args.max_seconds, config)
        try:
            if args.command == 'characterize':
                characterize(out, budget, config)
            elif args.command == 'generate':
                generate(out, args, config)
            elif args.command == 'calibrate':
                development(out, args, budget, config)
            elif args.command == 'replay':
                replay(out, args, budget, config)
            else:
                smoke(out, args, budget, config)
        except Exception as error:
            append_json(out/'failures.jsonl', dict(type=type(error).__name__, message=str(error),
                                                  completed_trials=budget.trials))
            raise
        rss, available = check_memory(config)
        write_json(out/'finished.json', dict(trials=budget.trials, elapsed_s=time.monotonic()-budget.start,
                                            final_rss_mb=rss, available_mb=available))


if __name__ == '__main__':
    main()
