"""Generate, characterize, calibrate, export, or replay the small geofence set."""
from pathlib import Path
import sys
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geofence_turn  # Apply CPU/thread settings before importing NumPy.

import argparse
from collections import Counter
from dataclasses import asdict, replace
import fcntl
import hashlib
import json
import math
import os
import pickle
import platform as system_platform
import time
import numpy as np
from geofence_turn.scenario import (SCHEMA, Config, Command, Scenario, RAMP_S, HZ,
    scenario_dict, scenario_from_dict, generate_scenario, make_forward_trajectory)
from geofence_turn.simulation import execute, check_memory, start_work

ROOT = Path(__file__).resolve().parent
DEFAULT_DEVELOPMENT = ROOT/'support/logs/northbound_review/development'
DEFAULT_ENVELOPE = ROOT/'support/logs/northbound_review/characterization/envelope.json'


def approval_candidates(s, config=Config(), latest_approval_s=None):
    lo = round(config.approval_min_s*HZ)
    hi = round((config.approval_max_s if latest_approval_s is None else latest_approval_s)*HZ)
    stride = round(config.approval_grid_s*HZ)
    if lo < round(RAMP_S*HZ) or stride < 1:
        raise ValueError('Invalid post-ramp calibration grid')
    # Exhaustively descending finite grid; first feasible is its LATEST member.
    return [replace(s, approval_s=step/HZ) for step in range(hi, lo-1, -stride)]


def rejection_reasons(result):
    reasons = []
    if not result['safe']:
        reasons.append('body_clearance')
    if not result['completed']:
        reasons.append('checkpoint_deadline_velocity_or_dwell')
    return reasons


def calibrate(s, evaluate, config=Config(), on_candidate=None, latest_approval_s=None):
    """evaluate(candidate) returns the real D=0 simulation result and snapshot."""
    rows = []
    for candidate in approval_candidates(s, config, latest_approval_s):
        result, snapshot, trace = evaluate(candidate)
        reasons = rejection_reasons(result)
        row = dict(approval_s=candidate.approval_s, command=scenario_dict(candidate)['command'],
                   nominal_patrol_distance_m=candidate.speed*(candidate.approval_s-RAMP_S/2),
                   outcome=result, rejected_because=reasons)
        rows.append(row)
        if on_candidate is not None:
            on_candidate(row)
        if not reasons:
            return candidate, snapshot, rows
    return None, None, rows


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
    sources = {str(p.relative_to(ROOT)):file_hash(p) for p in ROOT.rglob('*.py')
               if not any(x in p.relative_to(ROOT).parts for x in ('logs','results','tests','__pycache__'))}
    return dict(schema=SCHEMA, config=asdict(config), python=sys.version, executable=sys.executable,
                platform=system_platform.platform(), numpy=np.__version__,
                numerical_threads=1, cpu_only=True, simulation_workers=1,
                source_hashes=sources,
                started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))


def prepare_output(out, config, resume=False):
    out = Path(out)
    current = metadata(config)
    if resume:
        saved = json.loads((out/'metadata.json').read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != current['config']:
            raise ValueError('Resume configuration or dataset schema differs')
        if saved['source_hashes'] != current['source_hashes']:
            raise ValueError('Resume source/layout changed; retain the old results and use a fresh output directory')
        append_json(out/'resume_sessions.jsonl', current)
    else:
        out.mkdir(parents=True, exist_ok=False)
        write_json(out/'metadata.json', current)
    return out


def load_snapshot(path):
    """Read frozen snapshots, including class names from the old mission.py.

    The mapping changes only where Python finds the same three data classes;
    snapshot bytes and all dataset compatibility checks stay unchanged.
    """
    class Reader(pickle.Unpickler):
        def find_class(self, module, name):
            if module == 'geofence_turn.mission' and name in ('Command','Config','Scenario'):
                return {'Command':Command, 'Config':Config, 'Scenario':Scenario}[name]
            return super().find_class(module, name)
    with Path(path).open('rb') as stream:
        return Reader(stream).load()


def load_development(development, config=Config()):
    """Read the five existing cases without changing their files or provenance."""
    development = Path(development)
    if development.is_file():
        saved = json.loads(development.read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != asdict(config):
            raise ValueError('Calibrated dataset schema/configuration mismatch')
        provenance = saved['provenance']
        if file_hash(development.parent/provenance['path']) != provenance['sha256']:
            raise ValueError('Calibrated dataset provenance checksum mismatch')
        entries = saved['scenarios']
        rows = [(entry['scenario'], development.parent/entry['approval_snapshot'], entry['snapshot_sha256'])
                for entry in entries]
    else:
        saved = json.loads((development/'metadata.json').read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != asdict(config):
            raise ValueError('Development dataset schema/configuration mismatch')
        summary = json.loads((development/'summary.json').read_text())
        rows = [(entry['selected'], development/f"seed_{entry['seed']}/approval.pkl", entry['snapshot_sha256'])
                for entry in summary['scenarios']]
    cases = []
    for row, path, checksum in rows:
        scenario = scenario_from_dict(row)
        if file_hash(path) != checksum:
            raise ValueError(f'Approval snapshot checksum mismatch: {path}')
        snapshot = load_snapshot(path)
        if snapshot.config != config or snapshot.command != scenario.command:
            raise ValueError(f'Approval snapshot does not match scenario/configuration: {path}')
        approval = round(scenario.approval_s*HZ)
        horizon = math.ceil(scenario.deadline_s*HZ-1e-10)
        expected = make_forward_trajectory(approval+horizon+1, scenario)[approval:]
        if (snapshot.approval_time != scenario.approval_s or snapshot.horizon_steps != horizon
                or not np.array_equal(snapshot.remaining_reference, expected)):
            raise ValueError(f'Approval time/reference does not match scenario: {path}')
        cases.append((scenario, snapshot, path))
    return sorted(cases, key=lambda item:item[0].seed)


def save_calibrated_scenarios(out, records, config):
    entries = [dict(scenario=entry['selected'],
                    approval_snapshot=f"seed_{entry['seed']}/approval.pkl",
                    snapshot_sha256=entry['snapshot_sha256']) for entry in records]
    write_json(out/'calibrated_scenarios.json', dict(schema=SCHEMA, config=asdict(config), scenarios=entries,
               provenance=dict(path='metadata.json', sha256=file_hash(out/'metadata.json'))))


def export_calibrated(out, development, config):
    entries = [dict(scenario=scenario_dict(s), approval_snapshot=os.path.relpath(path,out),
                    snapshot_sha256=file_hash(path)) for s,_,path in load_development(development,config)]
    source = Path(development)
    if source.is_dir():
        source = source/'metadata.json'
    write_json(out/'calibrated_scenarios.json', dict(schema=SCHEMA, config=asdict(config), scenarios=entries,
               provenance=dict(path=os.path.relpath(source,out), sha256=file_hash(source))))


def characterize_turns(out, work, config):
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
            result, _, trace = execute(s, save_trace=True, config=config, work=work)
            feasible = result['safe_completion'] and result['max_tracking_error_m'] <= config.tracking_limit_m
            row = dict(speed=speed, radius=radius, lateral_acceleration_mps2=speed**2/radius,
                       feasible=bool(feasible), scenario=scenario_dict(s), outcome=result)
            results.append(row)
            np.savez_compressed(out/f'trace_v{speed}_r{radius}.npz', controls=trace)
            append_json(out/'sweep.jsonl', row)
            print(f'v={speed:.1f} R={radius:.1f} feasible={feasible} max_error={result["max_tracking_error_m"]:.4f} completion={result["completion_s"]}', flush=True)
    write_json(out/'envelope.json', dict(config=asdict(config), results=results))


def generate_scenarios(out, args, config):
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


def calibrate_development(out, args, work, config):
    if args.resume:
        generated = json.loads((out/'generated.json').read_text())
    elif args.generated:
        generated = json.loads(Path(args.generated).read_text())
        if len(generated) > args.attempts:
            generated = generated[:args.attempts]
    else:
        generated = generate_scenarios(out, args, config)
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
                return execute(candidate, config=config, work=work)
            assert saved['command'] == json.loads(json.dumps(scenario_dict(candidate)['command']))
            snap = None
            if not saved['rejected_because']:
                snap = load_snapshot(directory/'approval.pkl')
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
            nominal, _, _ = execute(replace(s, deadline_s=probe_s-s.approval_s), None, config=config, work=work)
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
            replay, replay_snapshot, trace = execute(selected, save_trace=True, config=config, work=work)
            np.testing.assert_array_equal(snapshot.state, replay_snapshot.state)
            for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
                np.testing.assert_array_equal(getattr(snapshot.pid_state, name), getattr(replay_snapshot.pid_state, name))
            assert replay['safe_completion'], replay
            np.savez_compressed(directory/'d0_trace.npz', controls=trace)
            write_json(directory/'d0_replay.json', replay)
            record.update(replay=replay, snapshot_sha256=file_hash(directory/'approval.pkl'))
            accepted.append(record)
            save_calibrated_scenarios(out, accepted, config)
        append_json(out/'attempts.jsonl', record)
        print(f'seed={s.seed} accepted={selected is not None} candidates={len(rows)} latest_approval={None if selected is None else selected.approval_s}', flush=True)
        write_json(out/'summary.json', dict(attempted=attempted, accepted=len(accepted),
                   acceptance_rate=len(accepted)/attempted, candidate_rejections=dict(rejections), scenarios=accepted))
    print(f'Acceptance: {len(accepted)}/{attempted}; candidate rejection reasons={dict(rejections)}', flush=True)


def replay_calibrated(out, args, work, config):
    for s, stored, _ in load_development(args.development, config):
        assert stored.config == config
        result, snap, trace = execute(s, save_trace=True, config=config, work=work)
        np.testing.assert_array_equal(snap.state, stored.state)
        np.testing.assert_array_equal(snap.remaining_reference, stored.remaining_reference)
        for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
            np.testing.assert_array_equal(getattr(snap.pid_state, name), getattr(stored.pid_state, name))
        assert snap.command == stored.command == s.command
        assert result['safe_completion'], result
        write_json(out/f'seed_{s.seed}.json', result)
        print(f'D=0 seed={s.seed} PASS clearance={result["min_clearance_m"]:.6f} completion={result["completion_s"]:.3f}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=('calibrate','characterize','generate','replay','export'), default='calibrate')
    parser.add_argument('--out', required=True)
    parser.add_argument('--envelope', default=str(DEFAULT_ENVELOPE))
    parser.add_argument('--development', default=str(DEFAULT_DEVELOPMENT))
    parser.add_argument('--generated')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--start-seed', type=int, default=0)
    parser.add_argument('--attempts', type=int, default=10)
    parser.add_argument('--target', type=int, default=5)
    parser.add_argument('--max-trials', type=int, default=160)
    parser.add_argument('--max-seconds', type=float, default=900)
    args = parser.parse_args()
    if not 1 <= args.target <= args.attempts <= 20:
        parser.error('Development cap: 1 <= target <= attempts <= 20')
    if args.resume and args.task not in ('calibrate','characterize'):
        parser.error('Only calibration/characterization can resume')
    config = Config()
    work = start_work(args.max_trials,args.max_seconds)
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        out = prepare_output(args.out, config, args.resume)
        try:
            if args.task == 'characterize':
                characterize_turns(out, work, config)
            elif args.task == 'generate':
                generate_scenarios(out, args, config)
            elif args.task == 'calibrate':
                calibrate_development(out, args, work, config)
            elif args.task == 'replay':
                replay_calibrated(out, args, work, config)
            else:
                export_calibrated(out, args.development, config)
        except Exception as error:
            append_json(out/'failures.jsonl',dict(type=type(error).__name__,message=str(error),completed_trials=work['trials']))
            raise
        rss,available = check_memory(config)
        write_json(out/'finished.json',dict(trials=work['trials'],elapsed_s=time.monotonic()-work['start'],final_rss_mb=rss,available_mb=available))


if __name__ == '__main__':
    main()
