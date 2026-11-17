"""Resumable E01 tasks; atomic records are authoritative, reports are regenerable."""

from concurrent.futures import ProcessPoolExecutor, as_completed, wait
from contextlib import contextmanager
from dataclasses import asdict
from multiprocessing import get_context
from pathlib import Path
import argparse
import fcntl
import json
import os
import time
import traceback
import uuid
import shlex
import shutil
import subprocess
import sys

import numpy as np

from staleguard.io import file_hash, load_pickle
from staleguard.reproduce.common import Data
from staleguard.underpass_inspection.scenario import Scenario
from staleguard.underpass_inspection.storage import check_approval_snapshot
from staleguard.model.scg import load_scg_platform_params
from . import physics
from .protocol import (
    ROOT,
    REPO,
    MISSION,
    CONDITIONS,
    POLICIES,
    freeze,
    perturbation,
    plant_name,
    digest,
    atomic_json,
    save_record,
    load_record,
    environment,
)

_PLATFORM = None
_WARMED = False
_POOL = None


@contextmanager
def exclusive(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def prepare_archive(data_root, spec):
    """Extract selected immutable records, retaining original journal hashes."""
    data = Data(data_root)
    selected = set(spec['selected_seeds'])
    rows = {s: dict(h2=[], h3=[], forecasts={}) for s in selected}
    for kind, dataset, filename in (
        ('h2', 'feasibility', 'executions.jsonl'),
        ('h3', 'benchmark', 'executions.jsonl'),
        ('forecasts', 'benchmark', 'predictions.jsonl'),
    ):
        path = data.file(data.entry(MISSION + '/' + dataset)['files'][filename])
        keys = set()
        for line in path.open():
            row = json.loads(line)
            if row['seed'] not in selected:
                continue
            if kind == 'forecasts':
                if row['policy'] in ('staleguard', 'staleguard_dual_horizon'):
                    rows[row['seed']]['forecasts'][row['policy']] = row['forecast']
            elif kind == 'h2':
                key = row['seed'], row['mode'], round(row['start_s'] * 50)
                if key in keys:
                    raise ValueError('Duplicate archived physical comparison')
                keys.add(key)
                rows[row['seed']]['h2'].append(row)
            elif row['event']['policy'] in POLICIES:
                key = row['seed'], row['event']['policy'], round(row['delay_s'] * 50)
                if key in keys:
                    raise ValueError('Duplicate archived policy outcome')
                keys.add(key)
                rows[row['seed']]['h3'].append(row)
    for seed, payload in rows.items():
        assert len(payload['h2']) == 38 and len(payload['h3']) == 78
        payload['source_sha256'] = spec['original_inputs']
        save_record(ROOT / 'archive' / f'{seed}.json', payload)


class Cache:
    def __init__(self, spec, scenario, condition, data_root, implementation):
        self.spec, self.scenario = spec, scenario
        self.condition = condition
        self.uncertainty = perturbation(spec, condition, scenario.seed)
        self.plant = plant_name(condition)
        self.data_root = Path(data_root)
        self.implementation = implementation
        self.completed_attempts = {}
        for path in sorted((ROOT / 'attempts' / str(scenario.seed)).glob('*.json')):
            candidate = load_record(path)
            if candidate['uncertainty']['mass_factor'] == self.uncertainty[
                'mass_factor'
            ] and digest(candidate['scenario']) == digest(asdict(scenario)):
                action = tuple(candidate['action'])
                previous = self.completed_attempts.get(action)
                if (
                    previous is not None
                    and previous[1]['record']['physics_sha256']
                    != candidate['record']['physics_sha256']
                ):
                    raise ValueError('Independent identical-input flights have different hashes')
                self.completed_attempts[action] = (path, candidate)
        self.stats = dict(
            new_physical_rollouts=0,
            archive_imports=0,
            cache_hits=0,
            simulation_runtime_s=0.0,
            h2_physical_runtime_s=0.0,
        )

    def path(self, action):
        key = dict(
            scenario=asdict(self.scenario),
            mass_factor=self.uncertainty['mass_factor'],
            action=list(action),
            nominal_parameters=self.spec['nominal_physical_parameters'],
            implementation=self.implementation,
            reference='original apply_maneuver(true state); full flight from reset',
        )
        return (
            ROOT / 'physical' / self.plant / str(self.scenario.seed) / (digest(key) + '.json'),
            key,
        )

    def get(self, action, allow_archive=True, simulate=True):
        path, key = self.path(action)
        if path.exists():
            row = load_record(path)
            assert row['key'] == key
            self.stats['cache_hits'] += 1
            return dict(row, cache_hit=True)
        if self.plant == 'N0' and allow_archive:
            verified = ROOT / 'snapshots/N0' / f'{self.scenario.seed}.json'
            if not verified.exists() or not load_record(verified)['archive_equivalence_verified']:
                raise ValueError(
                    'Nominal archive reuse requires independently verified equivalence'
                )
            archive = load_record(ROOT / 'archive' / f'{self.scenario.seed}.json')
            for original in archive['h2'] + archive['h3']:
                if original['mode'] == action[0] and original['initiation_step'] == action[1]:
                    value = dict(
                        key=key,
                        record=original,
                        source='verified immutable nominal physical execution',
                        source_journal_sha256=archive['source_sha256'],
                        cache_id=str(path.relative_to(ROOT)),
                    )
                    save_record(path, value)
                    self.stats['archive_imports'] += 1
                    return dict(value, cache_hit=True)
        if not simulate:
            if action not in self.completed_attempts:
                return None
        if action in self.completed_attempts:
            origin, attempt = self.completed_attempts[action]
            value = dict(
                key=key,
                record=attempt['record'],
                source='recovered completed physical execution',
                source_attempt=str(origin.relative_to(ROOT)),
                cache_id=str(path.relative_to(ROOT)),
            )
            save_record(path, value)
            self.stats['cache_hits'] += 1
            return dict(value, cache_hit=True)
        record, _ = self.execute_physical(action)
        return self.store_physical(action, record)

    def execute_physical(self, action, capture_steps=()):
        if _POOL is None:
            return physics.physical(self.scenario, self.uncertainty, action, capture_steps)
        return _POOL.submit(
            physics.physical, self.scenario, self.uncertainty, action, capture_steps
        ).result()

    def store_physical(self, action, record):
        path, key = self.path(action)
        self.stats['simulation_runtime_s'] += record['physical_runtime_s']
        self.stats['new_physical_rollouts'] += 1
        value = dict(
            key=key,
            record=record,
            source='new PyBullet physical execution',
            cache_id=str(path.relative_to(ROOT)),
        )
        save_record(path, value)
        return dict(value, cache_hit=False)

    def many(self, actions):
        result, missing = {}, []
        for action in dict.fromkeys(actions):
            value = self.get(action, simulate=False)
            if value is None:
                missing.append(action)
            else:
                result[action] = value
        if _POOL is None:
            for action in missing:
                result[action] = self.get(action)
        else:
            futures = {
                _POOL.submit(physics.physical, self.scenario, self.uncertainty, action): action
                for action in missing
            }
            try:
                for future in as_completed(futures):
                    action = futures[future]
                    record, _ = future.result()
                    result[action] = self.store_physical(action, record)
            finally:
                for future in futures:
                    future.cancel()
                wait(futures)
        return result

    def snapshots(self):
        seed = self.scenario.seed
        path = ROOT / 'snapshots' / self.plant / f'{seed}.npz'
        meta = path.with_suffix('.json')
        if meta.exists():
            info = load_record(meta)
            assert info['sha256'] == file_hash(path)
            return path
        steps = [round((self.scenario.approval_s + d) * 50) for d in self.spec['policy_delays']]
        record, snapshots = self.execute_physical(('nominal', None), steps)
        assert set(snapshots) == set(steps)
        physics.save_snapshots(path, snapshots)
        cache_path, key = self.path(('nominal', None))
        if not cache_path.exists():
            save_record(
                cache_path,
                dict(
                    key=key,
                    record=record,
                    source='new PyBullet snapshot flight',
                    cache_id=str(cache_path.relative_to(ROOT)),
                ),
            )
            self.stats['new_physical_rollouts'] += 1
            self.stats['simulation_runtime_s'] += record['physical_runtime_s']
        verification = []
        if self.plant == 'N0':
            data = Data(self.data_root)
            relative = data.entry(MISSION + '/calibration')['snapshots'][str(seed)]
            archived_snapshot = load_pickle(data.file(relative))
            check_approval_snapshot(
                snapshots[round(self.scenario.approval_s * 50)], archived_snapshot
            )
        # Independent replay validates cache equivalence for both maneuvers in every plant/scenario.
        for mode in ('dive', 'sideways'):
            action = (mode, round(self.scenario.approval_s * 50))
            value = self.get(action, allow_archive=False)
            if self.plant == 'N0':
                archived = load_record(ROOT / 'archive' / f'{seed}.json')
                original = next(
                    r for r in archived['h2'] if r['mode'] == mode and r['start_s'] == 0
                )
                for field in ('physics_sha256', 'safe', 'safe_completion'):
                    if value['record'][field] != original[field]:
                        raise ValueError(f'N0 physical mismatch: seed={seed}, {mode}, {field}')
            else:
                independent, _ = self.execute_physical(action)
                self.stats['new_physical_rollouts'] += 1
                self.stats['simulation_runtime_s'] += independent['physical_runtime_s']
                assert independent['physics_sha256'] == value['record']['physics_sha256']
                assert independent['safe'] == value['record']['safe']
                assert independent['safe_completion'] == value['record']['safe_completion']
            verification.append(dict(mode=mode, physics_sha256=value['record']['physics_sha256']))
        save_record(
            meta,
            dict(
                sha256=file_hash(path),
                plant=self.plant,
                seed=seed,
                physical_plant=record['physical_plant'],
                archive_equivalence_verified=self.plant == 'N0',
                independent_replays=verification,
            ),
        )
        return path


def outcome(decision, rollout, uncertainty):
    row = rollout['record']
    event = decision['event']
    approval = row['approval_step']
    action_step = decision['action'][1]
    first = row['first_violation']
    violation = None if first is None else first['time_flight_s']
    recovery = decision['action'][0] == 'sideways'
    failure = None
    if recovery and not row['safe']:
        failure = (
            'pre_approval_violation'
            if violation < approval / 50
            else (
                'lost_before_recovery'
                if violation <= action_step / 50
                else 'violation_after_recovery_start'
            )
        )
    return dict(
        seed=decision['seed'],
        condition=decision['condition'],
        policy=decision['policy'],
        delay_s=decision['delay_s'],
        mass_sign=uncertainty['mass_sign'],
        safe=row['safe'],
        completed=row['inspection_completed'],
        safe_completion=row['safe_completion'],
        safe_non_completion=row['safe'] and not row['safe_completion'],
        recovery_attempted=recovery,
        unsafe_recovery=recovery and not row['safe'],
        recovery_failure_category=failure,
        intervention_control_step=action_step,
        application_step=event['application_step'],
        fallback_step=event['fallback_step'],
        approval_step=approval,
        dispatch_step=event['scheduled_dispatch_step'],
        command_canceled=event['command_canceled'],
        cancellation_reason=event.get('recovery_reason'),
        prediction_ready=event.get('prediction_ready'),
        prediction_ready_step=event.get(
            'prediction_ready_control_step', event.get('decision_step')
        ),
        prediction_triggered=recovery
        and decision['policy'].startswith('staleguard')
        and event.get('prediction_ready') is True,
        best_effort_recovery=event.get('best_effort_recovery'),
        predicted_recovery_verified=event.get('predicted_recovery_verified_at_start'),
        first_violation_s=violation,
        first_violation=first,
        violation_before_approval=violation is not None and violation < approval / 50,
        violation_while_pending=violation is not None
        and approval / 50
        <= violation
        <= (action_step / 50 if action_step is not None else row['horizon_flight_s']),
        physics_sha256=row['physics_sha256'],
        physical_cache_id=rollout['cache_id'],
        physical_source=rollout['source'],
        physical_reused=rollout.get('cache_hit', False),
    )


def seed_job(job):
    global _PLATFORM, _WARMED
    spec, seed, conditions, experiment, delays, policies, data_root, implementation = job
    if _PLATFORM is None:
        _PLATFORM = load_scg_platform_params()
    scenario = Scenario(**next(r for r in spec['scenarios'] if r['seed'] == seed))
    result = []
    for condition in conditions:
        began = time.perf_counter()
        try:
            cache = Cache(spec, scenario, condition, data_root, implementation)
            snapshots = cache.snapshots()
            approval = round(scenario.approval_s * 50)
            snap = physics.load_snapshot(snapshots, scenario, approval)
            warmups = 0
            path = ROOT / 'forecasts' / condition / f'{seed}.json'
            needs_recheck = (
                experiment in ('h3', 'both')
                and 'use_time_recheck' in policies
                and any(
                    not (
                        ROOT
                        / 'decisions'
                        / condition
                        / str(seed)
                        / f'{round(d * 50):03d}_use_time_recheck.json'
                    ).exists()
                    for d in delays
                )
            )
            if not _WARMED and (not path.exists() or needs_recheck):
                physics.warm(scenario, snap, _PLATFORM)
                _WARMED = True
                warmups = 1
            computed = not path.exists()
            if computed:
                value = physics.forecast(snap, cache.uncertainty, _PLATFORM)
                save_record(
                    path,
                    dict(
                        seed=seed,
                        condition=condition,
                        forecast=value,
                        perturbation=cache.uncertainty,
                        true_snapshot_sha256=file_hash(snapshots),
                        implementation=implementation,
                    ),
                )
            forecast = load_record(path)['forecast']
            if forecast['nominal_predictor_mass_kg'] != spec['nominal_physical_parameters']['mass']:
                raise ValueError('Predictor received a non-nominal mass')
            truth_path = ROOT / 'h2' / condition / f'{seed}.json'
            if experiment in ('h2', 'both') and not truth_path.exists():
                truth = []
                truth_began = time.perf_counter()
                rollouts = cache.many(
                    [
                        (mode, approval + round(d * 50))
                        for d in spec['prediction_delays']
                        for mode in ('dive', 'sideways')
                    ]
                )
                for d in spec['prediction_delays']:
                    for mode in ('dive', 'sideways'):
                        physical = rollouts[mode, approval + round(d * 50)]
                        row = physical['record']
                        truth.append(
                            dict(
                                delay_s=d,
                                mode=mode,
                                safe=row['safe'],
                                safe_completion=row['safe_completion'],
                                physics_sha256=row['physics_sha256'],
                                physical_cache_id=physical['cache_id'],
                            )
                        )
                cache.stats['h2_physical_runtime_s'] = time.perf_counter() - truth_began
                save_record(truth_path, dict(seed=seed, condition=condition, comparisons=truth))
            new_outcomes = 0
            pending = []
            for delay in delays if experiment in ('h3', 'both') else ():
                for policy in policies:
                    base = Path(condition) / str(seed) / f'{round(delay * 50):03d}_{policy}.json'
                    target = ROOT / 'outcomes' / base
                    if target.exists():
                        load_record(target)
                        continue
                    decision_path = ROOT / 'decisions' / base
                    if not decision_path.exists():
                        dispatch_snap = physics.load_snapshot(
                            snapshots, scenario, approval + round(delay * 50)
                        )
                        decision = physics.decide(
                            scenario,
                            delay,
                            policy,
                            forecast,
                            dispatch_snap,
                            cache.uncertainty,
                            _PLATFORM,
                        )
                        decision['forecast_sha256'] = file_hash(path)
                        save_record(decision_path, decision)
                    decision = load_record(decision_path)
                    pending.append((target, decision))
            rollouts = cache.many([tuple(d['action']) for _, d in pending])
            used = set()
            for target, decision in pending:
                rollout = rollouts[tuple(decision['action'])]
                if rollout['cache_id'] in used:
                    rollout = dict(rollout, cache_hit=True)
                save_record(target, outcome(decision, rollout, cache.uncertainty))
                used.add(rollout['cache_id'])
                new_outcomes += 1
            stats = dict(
                seed=seed,
                condition=condition,
                elapsed_s=time.perf_counter() - began,
                predictions_computed=int(computed),
                warmup_full_forecasts=warmups,
                new_outcomes=new_outcomes,
                **cache.stats,
            )
            save_record(ROOT / 'execution' / (str(uuid.uuid4()) + '.json'), stats)
            result.append(stats)
        except Exception:
            error = dict(seed=seed, condition=condition, traceback=traceback.format_exc())
            atomic_json(ROOT / 'errors' / f'{condition}_{seed}_{time.time_ns()}.json', error)
            result.append(error)
    return result


def status():
    return dict(
        forecasts=len(list((ROOT / 'forecasts').glob('*/*.json'))),
        h2_scenario_conditions=len(list((ROOT / 'h2').glob('*/*.json'))),
        h3_outcomes=len(list((ROOT / 'outcomes').glob('*/*/*.json'))),
        expected_full_forecasts=320,
        expected_h2_scenario_conditions=320,
        expected_h3_outcomes=24960,
        errors=len(list((ROOT / 'errors').glob('*.json'))),
    )


def run(args):
    global _POOL
    from .analysis import summarize

    spec = freeze(args.data_root)
    prepare_archive(args.data_root, spec)
    current = environment()
    assets = {
        str(p.relative_to(REPO)): file_hash(p)
        for p in sorted((REPO / 'safe_control_gym').rglob('*'))
        if p.is_file() and p.suffix.lower() in ('.yaml', '.urdf', '.obj', '.mtl', '.stl', '.xml')
    }
    asset_manifest = ROOT / 'simulator_assets.json'
    if asset_manifest.exists() and json.loads(asset_manifest.read_text()) != assets:
        raise ValueError('Simulator assets/configuration changed; refusing cached physical reuse')
    if not asset_manifest.exists():
        atomic_json(asset_manifest, assets)
    current['simulator_assets_sha256'] = assets
    scientific_source = {
        name: value
        for name, value in current['source_sha256'].items()
        if name.startswith(('staleguard/model/', 'staleguard/underpass_inspection/'))
        or name
        in (
            'staleguard/robustness/physics.py',
            'staleguard/robustness/protocol.py',
            'staleguard/physics_monitor.py',
        )
    }
    implementation = digest(
        dict(
            source=scientific_source,
            simulator=current['simulator_sha256'],
            packages=current['packages'],
            python=sys.version.split()[0],
        )
    )
    config = dict(implementation=implementation, environment=current)
    path = ROOT / 'implementation.json'
    if path.exists():
        previous = json.loads(path.read_text())
        origin = previous['environment']
        if (
            any(
                origin['source_sha256'].get(name) != value
                for name, value in scientific_source.items()
            )
            or origin['simulator_sha256'] != current['simulator_sha256']
            or origin['packages'] != current['packages']
            or origin['python'].split()[0] != sys.version.split()[0]
        ):
            raise ValueError(
                'Physical/prediction implementation changed; refusing scientific cache reuse'
            )
        # Keep the original cache namespace and provenance. Resource scheduling and
        # analytical code may change without altering physical inputs or policy rules.
        implementation = previous['implementation']
    if not path.exists():
        atomic_json(path, config)
    selected = spec['execution_order']
    delays = spec['policy_delays']
    if args.command == 'pilot':
        first = spec['selected_seeds'][0]
        second = next(
            s
            for s in spec['selected_seeds']
            if spec['mass_signs'][str(s)] != spec['mass_signs'][str(first)]
        )
        selected = sorted((first, second))
        delays = [0.0, 0.2, 1.2, 2.0]
    if args.seeds:
        if not set(args.seeds) <= set(selected):
            raise ValueError('Seeds must be a subset of the locked selection (or pilot subset)')
        selected = sorted(args.seeds)
    invocation = dict(
        command=args.command,
        seeds=selected,
        conditions=args.conditions,
        experiment=args.experiment,
        policies=args.policies,
        delays=delays,
        workers=args.workers,
        started_unix_s=time.time(),
        provenance=current,
    )
    identifier = str(uuid.uuid4())
    atomic_json(ROOT / 'invocations' / f'{identifier}.json', invocation)
    began = time.perf_counter()
    jobs = [
        (
            spec,
            seed,
            args.conditions,
            args.experiment,
            delays,
            args.policies,
            str(args.data_root.resolve()),
            implementation,
        )
        for seed in selected
    ]
    errors = []
    with exclusive(ROOT / 'runner.lock'):
        atomic_json(
            ROOT / 'progress.json',
            dict(status(), running=True, elapsed_s=0.0, started_unix_s=time.time()),
        )
        # Forecast/recheck timing is serial and independent of the physical worker count.
        # Only deterministic physical continuations run in parallel; all are joined
        # before the next timed prediction or policy-decision batch.
        for job in jobs:
            # Bound native SCG/CasADi allocator growth by retiring workers per scenario.
            _POOL = ProcessPoolExecutor(max_workers=args.workers, mp_context=get_context('spawn'))
            try:
                value = seed_job(job)
                errors.extend(r for r in value if 'traceback' in r)
                atomic_json(
                    ROOT / 'progress.json',
                    dict(status(), running=True, elapsed_s=time.perf_counter() - began),
                )
                print('Checkpoint', job[1], status(), flush=True)
            finally:
                if _POOL is not None:
                    _POOL.shutdown(wait=True, cancel_futures=True)
                    _POOL = None
        invocation.update(
            elapsed_s=time.perf_counter() - began, errors=len(errors), finished_unix_s=time.time()
        )
        atomic_json(ROOT / 'invocations' / f'{identifier}.json', invocation)
        if args.command == 'pilot' and not errors and not (ROOT / 'PILOT.json').exists():
            atomic_json(ROOT / 'PILOT.json', invocation)
        atomic_json(
            ROOT / 'progress.json', dict(status(), running=False, elapsed_s=invocation['elapsed_s'])
        )
        summarize()
    if errors:
        raise RuntimeError(
            f'{len(errors)} condition tasks failed; see {ROOT / "errors"}. Resume retries missing work.'
        )


def launch(args):
    try:
        with exclusive(ROOT / 'runner.lock'):
            pass
    except BlockingIOError:
        print('An E01 runner already holds the execution lock; no duplicate launched')
        return
    socket = ROOT / 'tmux.sock'
    tmux_available = shutil.which('tmux') is not None
    check = (
        subprocess.run(['tmux', '-S', str(socket), 'has-session', '-t', 'e01'], capture_output=True)
        if tmux_available
        else None
    )
    if check is not None and check.returncode == 0:
        print('E01 tmux session already exists; no duplicate runner launched')
        return
    logs = ROOT / 'logs'
    logs.mkdir(parents=True, exist_ok=True)
    log = logs / ('pilot.log' if args.pilot_run else 'overnight.log')
    command = [
        sys.executable,
        '-B',
        '-u',
        '-m',
        'staleguard.robustness',
        'pilot' if args.pilot_run else 'run',
        '--workers',
        str(args.workers),
        '--data-root',
        str(args.data_root.resolve()),
        '--experiment',
        args.experiment,
        '--conditions',
        *args.conditions,
        '--policies',
        *args.policies,
    ]
    if args.seeds:
        command.extend(['--seeds', *map(str, args.seeds)])
    shell = 'exec ' + shlex.join(command) + ' >> ' + shlex.quote(str(log)) + ' 2>&1'
    tmux = (
        subprocess.run(
            ['tmux', '-S', str(socket), 'new-session', '-d', '-s', 'e01', '-c', str(REPO), shell],
            capture_output=True,
            text=True,
        )
        if tmux_available
        else subprocess.CompletedProcess([], 127, stderr='tmux is not installed')
    )
    if tmux.returncode:
        # No socket service is needed for the POSIX detached-process fallback.
        launched = time.time()
        with log.open('a') as stream:
            child = subprocess.Popen(
                command,
                cwd=REPO,
                stdin=subprocess.DEVNULL,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        atomic_json(
            ROOT / 'process.json',
            dict(
                command=command,
                method='detached POSIX session',
                pid=child.pid,
                log=str(log),
                tmux_error=tmux.stderr.strip(),
                launched_unix_s=launched,
            ),
        )
        for _ in range(150):
            progress = ROOT / 'progress.json'
            if progress.exists() and progress.stat().st_mtime >= launched:
                print('Detached process started; progress and log verified:', log)
                return
            if child.poll() is not None:
                if child.returncode:
                    raise RuntimeError('Detached runner failed; inspect ' + str(log))
                print('Detached runner completed; inspect', log)
                return
            time.sleep(0.1)
        raise RuntimeError('Runner did not produce a startup checkpoint; inspect ' + str(log))
    subprocess.run(['tmux', '-S', str(socket), 'has-session', '-t', 'e01'], check=True)
    atomic_json(
        ROOT / 'process.json',
        dict(
            command=command,
            tmux_socket=str(socket),
            tmux_session='e01',
            log=str(log),
            launched_unix_s=time.time(),
        ),
    )
    print('Verified detached tmux session e01; log:', log)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'command',
        choices=(
            'init',
            'test',
            'pilot',
            'run',
            'launch',
            'summarize',
            'status',
            'archive',
            'verify-archive',
            'reproduce',
        ),
    )
    parser.add_argument('--data-root', type=Path, default=REPO / 'data')
    parser.add_argument(
        '--out', type=Path, help='reproduce only: fresh directory for archived-data reconstruction'
    )
    parser.add_argument('--conditions', nargs='+', choices=CONDITIONS, default=list(CONDITIONS))
    parser.add_argument('--policies', nargs='+', choices=POLICIES, default=list(POLICIES))
    parser.add_argument('--experiment', choices=('h2', 'h3', 'both'), default='both')
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--seeds', nargs='+', type=int)
    parser.add_argument(
        '--pilot-run', action='store_true', help='launch only: detached bounded pilot'
    )
    args = parser.parse_args()
    if args.out is not None and args.command != 'reproduce':
        parser.error('--out is supported only by the archived-data reproduce command')
    if not 1 <= args.workers <= 4:
        parser.error('Use 1--4 isolated PyBullet workers to bound memory use')
    if args.seeds and len(set(args.seeds)) != len(args.seeds):
        parser.error('Repeated scenario seeds are not allowed')
    if args.pilot_run and args.command != 'launch':
        parser.error('Use pilot for a foreground pilot; --pilot-run is launch-only')
    if len(set(args.conditions)) != len(args.conditions) or len(set(args.policies)) != len(
        args.policies
    ):
        parser.error('Repeated conditions/policies are not allowed')
    if args.command in ('archive', 'verify-archive', 'reproduce'):
        from .archive import freeze_archive, verify_archive, reproduce_archive

        if args.command == 'archive':
            freeze_archive(args.data_root)
        elif args.command == 'verify-archive':
            verify_archive(args.data_root)
        else:
            reproduce_archive(args.data_root, args.out)
    elif args.command == 'init':
        spec = freeze(args.data_root)
        prepare_archive(args.data_root, spec)
        print('Frozen 64 scenarios, 5 conditions, 6 policies, 24960 H3 outcomes')
    elif args.command == 'test':
        from .tests import run_tests

        run_tests(args.data_root)
    elif args.command == 'summarize':
        from .analysis import summarize

        summarize()
    elif args.command == 'status':
        print(json.dumps(status(), indent=2))
    elif args.command == 'launch':
        launch(args)
    else:
        run(args)
