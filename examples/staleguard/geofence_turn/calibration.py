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
    with temporary.open('w') as stream:
        stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
        stream.flush()
        os.fsync(stream.fileno())
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
                shared_source_hashes={str(p):file_hash(p) for p in
                    [*(ROOT.parent/'inspection_descent/model').glob('*.py'),
                     ROOT.parent/'inspection_descent/experiment.py',
                     Path(sys.modules['physics_monitor'].__file__)]},
                started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))


def check_provenance(saved, config):
    current = metadata(config)
    for field in ('schema','config','source_hashes','shared_source_hashes','python','numpy'):
        if saved.get(field) != current[field]:
            raise ValueError(f'Resume source/layout changed or configuration/environment differs: {field}')


def prepare_output(out, config, resume=False):
    out = Path(out)
    current = metadata(config)
    if resume:
        saved = json.loads((out/'metadata.json').read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != current['config']:
            raise ValueError('Resume configuration or dataset schema differs')
        if saved['source_hashes'] != current['source_hashes']:
            raise ValueError('Resume source/layout changed; retain the old results and use a fresh output directory')
        check_provenance(saved,config)
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
    """Read compact or historical calibrated inputs, retaining compatibility checks."""
    development = Path(development)
    if development.is_dir() and (development/'calibrated_scenarios.json').exists():
        development = development/'calibrated_scenarios.json'
    if development.is_file():
        saved = json.loads(development.read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != asdict(config):
            raise ValueError('Calibrated dataset schema/configuration mismatch')
        provenance = saved['provenance']
        if saved.get('format') == 'compact_calibration_v1':
            if provenance.get('schema') != SCHEMA or provenance.get('config') != asdict(config) or not provenance.get('source_hashes') or not provenance.get('shared_source_hashes'):
                raise ValueError('Calibrated dataset provenance is incomplete or incompatible')
        elif file_hash(development.parent/provenance['path']) != provenance['sha256']:
            raise ValueError('Calibrated dataset provenance checksum mismatch')
        entries = saved['scenarios']
        if saved.get('format') == 'compact_calibration_v1':
            statistics = saved['statistics']
            attempted = statistics['attempted']
            if statistics['accepted'] != len(entries) or (attempted is not None and
                    (attempted < len(entries) or statistics['rejected'] != attempted-len(entries))):
                raise ValueError('Calibrated dataset acceptance statistics are inconsistent')
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
    seen = set()
    for row, path, checksum in rows:
        scenario = scenario_from_dict(row)
        if scenario.seed in seen:
            raise ValueError(f'Duplicate calibrated seed: {scenario.seed}')
        seen.add(scenario.seed)
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


def calibration_statistics(source, accepted):
    """Keep the original attempted-seed denominator when exporting old datasets."""
    source = Path(source)
    if source.is_dir():
        source = source/('calibrated_scenarios.json' if (source/'calibrated_scenarios.json').exists() else 'summary.json')
    saved = json.loads(source.read_text())
    if 'statistics' in saved:
        return dict(saved['statistics'])
    attempted = saved.get('generated_and_attempted',saved.get('attempted'))
    if attempted is None and 'path' in saved.get('provenance',{}):
        summary = (source.parent/saved['provenance']['path']).parent/'summary.json'
        if summary.exists():
            saved = json.loads(summary.read_text())
            attempted = saved.get('attempted')
    return dict(accepted=accepted, attempted=attempted,
                rejected=None if attempted is None else attempted-accepted,
                acceptance_rate=None if not attempted else accepted/attempted,
                candidate_rejections=saved.get('candidate_rejections',{}))


def export_calibrated(out, development, config):
    """Copy frozen snapshots byte-for-byte; never recalibrate or rewrite inputs."""
    import shutil
    out = Path(out)
    if any(out.iterdir()):
        raise FileExistsError('Export output must be empty; existing results are preserved')
    (out/'snapshots').mkdir()
    cases = load_development(development,config)
    entries = []
    for scenario, _, path in cases:
        target = out/f'snapshots/seed_{scenario.seed}.pkl'
        shutil.copyfile(path,target)
        entries.append(dict(scenario=scenario_dict(scenario),
                            approval_snapshot=str(target.relative_to(out)),
                            snapshot_sha256=file_hash(target)))
    source = Path(development)
    if source.is_dir():
        source = source/('calibrated_scenarios.json' if (source/'calibrated_scenarios.json').exists() else 'summary.json')
    write_json(out/'calibrated_scenarios.json',dict(format='compact_calibration_v1',schema=SCHEMA,
        config=asdict(config),provenance=metadata(config),
        source_dataset=dict(path=os.path.relpath(source,out),sha256=file_hash(source)),
        statistics=calibration_statistics(development,len(entries)),
        calibration=dict(status='complete',kind='export_existing'),scenarios=entries))


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


def generate_scenarios(out, args, config, save=True):
    envelope = json.loads(Path(args.envelope).read_text())
    rows = []
    for seed in range(args.start_seed, args.start_seed+args.attempts):
        try:
            rows.append(scenario_dict(generate_scenario(seed, envelope, config)))
        except ValueError as error:
            if not str(error).startswith('checkpoint_geometry:'):
                raise
            rows.append(dict(seed=seed, generator_rejection=str(error)))
    if save:
        write_json(out/'generated.json', rows)
    print(f'Generated {len(rows)} distinct seeds; no simulation or target adjustment.', flush=True)
    return rows


def calibrate_development(out, args, work, config):
    """The original D=0 search, with all bookkeeping in one scenario JSON.

    Only the current seed's candidate cache is retained for interrupted searches.
    Finished searches keep counts and selected scenarios, not candidate journals.
    """
    out = Path(out)
    path = out/'calibrated_scenarios.json'
    detailed = getattr(args,'diagnostics',False)
    generated_input = getattr(args,'generated',None)
    settings = dict(start_seed=getattr(args,'start_seed',0),attempts=args.attempts,
                    target=args.target,diagnostics=detailed,
                    generated_sha256=file_hash(generated_input) if generated_input else None,
                    envelope_sha256=None if generated_input else file_hash(getattr(args,'envelope',DEFAULT_ENVELOPE)))
    if args.resume:
        data = json.loads(path.read_text())
        if data.get('format') != 'compact_calibration_v1':
            raise ValueError('Old calibration layout cannot resume; retain it and use a fresh output directory')
        check_provenance(data['provenance'],config)
        if 'settings' not in data['calibration']:
            raise ValueError('Exported calibration is frozen input, not an interrupted calibration run')
        if data['calibration']['settings'] != settings:
            raise ValueError('Resume calibration inputs/settings differ')
        load_development(path,config)
        if data['calibration']['status'] == 'complete':
            return
    else:
        if out.exists() and any(out.iterdir()):
            raise FileExistsError('Calibration output contains results; use --resume or a fresh directory')
        out.mkdir(parents=True,exist_ok=True)
        (out/'snapshots').mkdir()
        generated = (json.loads(Path(generated_input).read_text())[:args.attempts]
                     if generated_input else generate_scenarios(out,args,config,save=False))
        if len({row['seed'] for row in generated}) != len(generated):
            raise ValueError('Duplicate generated seed')
        data = dict(format='compact_calibration_v1',schema=SCHEMA,config=asdict(config),
                    provenance=metadata(config),scenarios=[],
                    statistics=dict(generated=len(generated),attempted=0,accepted=0,rejected=0,
                                    acceptance_rate=None,candidate_rejections={}),
                    calibration=dict(status='running',settings=settings,attempts=[],sessions=[]),
                    progress=dict(generated=generated,current=None))
    session = dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),trials=0)
    data['calibration']['sessions'].append(session)
    data['calibration']['status'] = 'running'
    data['calibration'].pop('error',None)

    def checkpoint():
        session.update(trials=work['trials'],elapsed_s=time.monotonic()-work['start'])
        write_json(path,data)

    checkpoint()
    completed = {row['seed'] for row in data['calibration']['attempts']}
    diagnostics = out/'diagnostics'
    if detailed:
        diagnostics.mkdir(exist_ok=True)
    try:
        for generated in data['progress']['generated']:
            if len(data['scenarios']) >= args.target:
                break
            if generated['seed'] in completed:
                continue
            if 'generator_rejection' in generated:
                record = dict(seed=generated['seed'],accepted=False,candidate_count=0,
                              rejection_reason=generated['generator_rejection'])
                reasons = Counter(checkpoint_geometry=1)
            else:
                s = scenario_from_dict(generated)
                current = data['progress']['current']
                if current is None:
                    current = dict(seed=s.seed,candidates=[])
                    data['progress']['current'] = current
                if current['seed'] != s.seed:
                    raise ValueError('In-progress calibration seed does not match generated order')
                snapshot_path = out/f'snapshots/seed_{s.seed}.pkl'
                cached = {row['approval_s']:row for row in current['candidates']}

                def audit(candidate):
                    if candidate['approval_s'] in cached:
                        if json.dumps(candidate,sort_keys=True) != json.dumps(cached[candidate['approval_s']],sort_keys=True):
                            raise ValueError('Cached calibration candidate differs')
                    else:
                        current['candidates'].append(candidate)
                        checkpoint()

                def evaluate(candidate):
                    saved = cached.get(candidate.approval_s)
                    if saved is not None:
                        if saved['command'] != json.loads(json.dumps(scenario_dict(candidate)['command'])):
                            raise ValueError('Cached approved command differs')
                        snapshot = None
                        if not saved['rejected_because']:
                            if file_hash(snapshot_path) != current['snapshot_sha256']:
                                raise ValueError('In-progress approval snapshot checksum mismatch')
                            snapshot = load_snapshot(snapshot_path)
                        return saved['outcome'],snapshot,None
                    result,snapshot,trace = execute(candidate,config=config,work=work)
                    if result['safe_completion']:
                        temporary = snapshot_path.with_suffix('.pkl.tmp')
                        with temporary.open('wb') as stream:
                            pickle.dump(snapshot,stream)
                            stream.flush()
                            os.fsync(stream.fileno())
                        temporary.replace(snapshot_path)
                        current['snapshot_sha256'] = file_hash(snapshot_path)
                    return result,snapshot,trace

                # Unchanged nominal probe, approval grid and latest-feasible rule.
                predicted_arrival = (s.fence_x-s.initial_x)/s.speed + RAMP_S/2
                probe_s = predicted_arrival+s.command.arc_s+config.checkpoint_north_travel_s
                if 'nominal' not in current:
                    nominal,_,_ = execute(replace(s,deadline_s=probe_s-s.approval_s),None,config=config,work=work)
                    current['nominal'] = nominal
                    checkpoint()
                nominal = current['nominal']
                if nominal['first_violation_s'] is None:
                    raise RuntimeError('Nominal probe did not cross; no measured calibration upper bound')
                upper = math.ceil(nominal['first_violation_s']/config.approval_grid_s)*config.approval_grid_s
                selected,snapshot,rows = calibrate(s,evaluate,config,audit,latest_approval_s=upper)
                reasons = Counter(reason for row in rows for reason in row['rejected_because'])
                record = dict(seed=s.seed,accepted=selected is not None,candidate_count=len(rows),
                              search_upper_s=upper,grid_s=config.approval_grid_s,
                              rejection_reason=None if selected else 'no_feasible_approval_on_grid')
                if selected is not None:
                    replay,replayed,trace = execute(selected,save_trace=detailed,config=config,work=work)
                    np.testing.assert_array_equal(snapshot.state,replayed.state)
                    for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
                        np.testing.assert_array_equal(getattr(snapshot.pid_state,name),getattr(replayed.pid_state,name))
                    assert replay['safe_completion'],replay
                    data['scenarios'].append(dict(scenario=scenario_dict(selected),
                        approval_snapshot=str(snapshot_path.relative_to(out)),snapshot_sha256=file_hash(snapshot_path),
                        d0_verification=replay))
                    if detailed:
                        np.savez_compressed(diagnostics/f'seed_{s.seed}_d0.npz',controls=trace)
                if detailed:
                    details_path = diagnostics/'calibration.json'
                    details = json.loads(details_path.read_text()) if details_path.exists() else {}
                    details[str(s.seed)] = dict(nominal_bound=nominal,candidates=rows)
                    write_json(details_path,details)
            data['calibration']['attempts'].append(record)
            statistics = data['statistics']
            counts = Counter(statistics['candidate_rejections'])
            counts.update(reasons)
            statistics.update(attempted=len(data['calibration']['attempts']),accepted=len(data['scenarios']),
                              rejected=len(data['calibration']['attempts'])-len(data['scenarios']),
                              candidate_rejections=dict(counts))
            statistics['acceptance_rate'] = statistics['accepted']/statistics['attempted']
            data['progress']['current'] = None
            checkpoint()
            print(f"seed={record['seed']} accepted={record['accepted']} candidates={record['candidate_count']}",flush=True)
        rss,available = check_memory(config)
        session.update(final_rss_mb=rss,available_mb=available)
        data['calibration']['status'] = 'complete'
        data.pop('progress')
        checkpoint()
    except BaseException as error:
        data['calibration'].update(status='interrupted',error=dict(type=type(error).__name__,message=str(error)))
        checkpoint()
        raise


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
    parser.add_argument('--out',default=str(ROOT/'results/development/calibration'))
    parser.add_argument('--envelope', default=str(DEFAULT_ENVELOPE))
    parser.add_argument('--development', default=str(DEFAULT_DEVELOPMENT))
    parser.add_argument('--generated')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--diagnostics',action='store_true',help='Save detailed calibration candidates and D=0 traces')
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
        if args.task == 'calibrate':
            calibrate_development(Path(args.out),args,work,config)
            return
        if args.task == 'export':
            out = Path(args.out)
            out.mkdir(parents=True,exist_ok=True)
            export_calibrated(out,args.development,config)
            check_memory(config)
            return
        out = prepare_output(args.out,config,args.resume)
        try:
            if args.task == 'characterize':
                characterize_turns(out, work, config)
            elif args.task == 'generate':
                generate_scenarios(out, args, config)
            elif args.task == 'replay':
                replay_calibrated(out, args, work, config)
        except Exception as error:
            append_json(out/'failures.jsonl',dict(type=type(error).__name__,message=str(error),completed_trials=work['trials']))
            raise
        rss,available = check_memory(config)
        write_json(out/'finished.json',dict(trials=work['trials'],elapsed_s=time.monotonic()-work['start'],final_rss_mb=rss,available_mb=available))


if __name__ == '__main__':
    main()
