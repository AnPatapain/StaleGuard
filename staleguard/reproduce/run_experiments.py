"""Fresh physical reproduction with frozen published cohorts/grids; never writes canonical data.

Use --max-records 1 and --workers 1 for a bounded smoke run. Fresh warmed runtime
measurements belong to this run and never replace the paper's archived timings.
"""

from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from dataclasses import asdict
import argparse, importlib, json, time
import numpy as np
from .common import Data, POLICIES, MISSIONS, file_hash
from staleguard.io import load_pickle, metadata, write_json, append_json
from staleguard.model.scg import load_scg_platform_params
from staleguard.model.mission_evaluation import prediction, clock_action, physical


def load_cases(data, mission, cohort='evaluation'):
    entry = data.entry(mission + '/calibration')
    manifest = data.read_json(mission + '/calibration', 'calibrated_scenarios.json')
    selected = set(data.entry(mission + '/benchmark')['scenarios'])
    result = []
    if cohort != 'evaluation':
        raise ValueError('Only the published frozen evaluation cohort is supported')
    for row in manifest['scenarios']:
        seed = row.get('seed', row['scenario'].get('seed'))
        if seed not in selected:
            continue
        relative = entry['snapshots'][str(seed)]
        path = data.file(relative)
        if file_hash(path) != row['snapshot_sha256']:
            raise ValueError('Frozen snapshot differs from original calibration')
        snap = load_pickle(path)
        if mission == 'inspection_descent':
            from staleguard.inspection_descent.scenario import (
                Scenario,
                configuration,
                make_forward_trajectory,
                MEASURE_SECONDS,
            )

            if manifest['config'] != configuration():
                raise ValueError('Inspection configuration mismatch')
            s = Scenario(**row['scenario'])
            a = round(s.approval_time * 50)
            reference = make_forward_trajectory(
                SimpleNamespace(
                    CTRL_FREQ=50, X_GOAL=np.zeros((a + len(snap.remaining_reference), 12))
                ),
                s,
            )[a:]
            np.testing.assert_array_equal(reference, snap.remaining_reference)
            assert snap.horizon_steps == round(MEASURE_SECONDS * 50) - a
        elif mission == 'geofence_turn':
            from staleguard.geofence_turn.scenario import scenario_from_dict, Config

            if manifest['config'] != asdict(Config()):
                raise ValueError('Geofence configuration mismatch')
            s = scenario_from_dict(row['scenario'])
            assert snap.command == s.command and snap.fence_x == s.fence_x
        else:
            from staleguard.underpass_inspection.scenario import (
                Scenario,
                body_clearances,
                POS,
                make_nominal,
            )
            from staleguard.underpass_inspection.simulation import Snapshot

            s = Scenario(**row['scenario'])
            prefix = min(body_clearances(snap['state'][POS], s.obstacles).values())
            snap = Snapshot(**snap, prefix_min_clearance_m=prefix)
            np.testing.assert_array_equal(
                snap.remaining_reference,
                make_nominal(round((s.approval_s + s.observation_s) * 50) + 1, s)[
                    round(s.approval_s * 50) :
                ],
            )
        result.append((seed, s, snap))
    if {r[0] for r in result} != selected:
        raise ValueError('Frozen cohort/snapshot dependencies incomplete')
    return sorted(result, key=lambda r: r[0])


def full_prediction(mission, s, snap, platform, **kwargs):
    if mission == 'underpass_inspection':
        from staleguard.underpass_inspection.baselines.recovery_horizons import predict

        return predict(snap, platform, **kwargs)
    return prediction(mission, s, snap, platform, **kwargs)


def command_prediction(mission, snap, platform):
    if mission == 'inspection_descent':
        from staleguard.inspection_descent.baselines.staleguard import predict_at_reception

        value, runtime = predict_at_reception(snap, *platform)
        return dict(prediction=value, computation_s=runtime)
    module = importlib.import_module('staleguard.' + mission + '.baselines.staleguard')
    if mission == 'geofence_turn':
        from staleguard.geofence_turn.baselines.recovery_horizons import TTL_GRID

        began = time.perf_counter()
        result = module.predict_delays(snap, platform, delays=TTL_GRID)
    else:
        began = time.perf_counter()
        result = module.predict_delays(snap, platform)
    result['computation_s'] = time.perf_counter() - began
    return result


def execute_policy(mission, seed, s, snap, delay, policy, received, platform, save_trace=False):
    native = importlib.import_module('staleguard.' + mission + '.simulation').execute
    if mission == 'underpass_inspection':
        row, _ = native(
            s,
            policy=policy,
            delay_s=delay,
            received=received,
            platform=platform,
            expected_snapshot=vars(snap),
        )
    elif (
        policy in ('staleguard_recovery_aware', 'staleguard_dual_horizon')
        or mission == 'inspection_descent'
        and policy == 'fixed_early_recovery'
    ):
        action, event = clock_action(
            policy,
            snap,
            delay,
            received,
            horizon_steps=210 if mission == 'inspection_descent' else snap.horizon_steps,
        )
        row = physical((mission, seed, s, snap, delay, action, event))
    else:
        kwargs = dict(
            delay=delay, policy=policy, received=received, platform=platform, save_trace=save_trace
        )
        if mission == 'inspection_descent':
            kwargs.update(snapshot=snap, seed=seed)
        row, actual, _ = native(s, **kwargs)
        from importlib import import_module

        import_module('staleguard.' + mission + '.storage').check_approval_snapshot(actual, snap)
    return row


def execute_job(job):
    mission, seed, s, snap, delay, kind, policy, forecast, platform = job
    if policy is not None:
        return execute_policy(mission, seed, s, snap, delay, policy, forecast, platform)
    if mission == 'underpass_inspection':
        from staleguard.underpass_inspection.simulation import execute

        row, _ = execute(s, 'dive' if kind == 'command' else 'sideways', delay)
        np.testing.assert_array_equal(row['approval_state'], snap.state)
    else:
        row = physical(
            (mission, seed, s, snap, delay, (kind, round((snap.approval_time + delay) * 50)), None)
        )
    row.update(kind=kind, mission=mission, seed=seed, delay_s=delay)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=Path('data'))
    parser.add_argument('--mission', choices=MISSIONS, required=True)
    parser.add_argument(
        '--experiment',
        choices=('benchmark', 'feasibility', 'prediction', 'calibration'),
        required=True,
    )
    parser.add_argument('--cohort', choices=('evaluation',), default='evaluation')
    parser.add_argument(
        '--seeds',
        type=int,
        nargs='+',
        help='Subset of the published frozen cohort; no replacement or regeneration',
    )
    parser.add_argument('--policies', choices=POLICIES, nargs='+', default=POLICIES)
    parser.add_argument(
        '--delays', type=float, nargs='+', help='Subset of the original experiment grid'
    )
    parser.add_argument('--workers', type=int, default=1)
    parser.add_argument(
        '--max-records',
        type=int,
        default=0,
        help='0 means all; positive values bound the number of new records',
    )
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.workers < 1 or args.max_records < 0:
        parser.error('Use positive workers and nonnegative --max-records')
    if args.out.resolve().is_relative_to(args.data_root.resolve()):
        parser.error('Never write into immutable data')
    if args.out.exists() and any(args.out.iterdir()):
        parser.error('Use a fresh output directory; historical runs are not resumed')
    data = Data(args.data_root)
    cases = load_cases(data, args.mission, args.cohort)
    if args.seeds:
        if not set(args.seeds) <= {s for s, _, _ in cases}:
            parser.error('Unknown frozen evaluation seed')
        cases = [r for r in cases if r[0] in args.seeds]
    args.out.mkdir(parents=True, exist_ok=True)
    settings = dict(
        mission=args.mission,
        experiment=args.experiment,
        cohort=args.cohort,
        seeds=[r[0] for r in cases],
        policies=list(args.policies),
        control_hz=50,
        physics_hz=1000,
        model_hz=100,
        workers=args.workers,
        evidence='fresh simulation; archived data read-only',
        provenance=metadata(),
    )
    write_json(args.out / 'run.json', settings)
    if args.experiment == 'calibration':
        module = importlib.import_module('staleguard.' + args.mission + '.calibration')
        seeds = settings['seeds'][: args.max_records or None]
        if args.mission == 'inspection_descent':
            module.calibrate_scenarios(seeds, args.out / 'calibration')
        elif args.mission == 'geofence_turn':
            module.calibrate_scenarios(
                seeds,
                len(seeds),
                args.out / 'calibration',
                envelope_path=data.file(
                    data.entry(args.mission + '/calibration')['files']['envelope.json']
                ),
            )
        else:
            module.calibrate(seeds, args.out / 'calibration', split='evaluation')
        write_json(args.out / 'progress.json', dict(status='complete', calibration=True))
        return
    platform = load_scg_platform_params()
    settings['physical_parameters'] = asdict(platform[0])
    settings['pid_parameters'] = asdict(platform[1])
    settings['scenarios'] = {str(seed): asdict(s) for seed, s, _ in cases}
    first = cases[0]
    warm = full_prediction(
        args.mission, first[1], first[2], platform
    )  # Warm before timed production forecasts.
    if args.experiment == 'benchmark':
        command_prediction(args.mission, first[2], platform)
    grid = list(warm['grid_s'])
    delays = (
        data.run(args.mission + '/benchmark')['protocol']['delays']
        if args.experiment == 'benchmark'
        else grid
    )
    if args.delays:
        if not set(round(d * 50) for d in args.delays) <= {round(d * 50) for d in delays}:
            parser.error('Delay outside published grid')
        delays = [d for d in delays if round(d * 50) in {round(x * 50) for x in args.delays}]
    settings.update(delays=delays, prediction_grid=grid, input_sha256=data.used)
    write_json(args.out / 'run.json', settings)
    jobs = []
    predictions = 0
    for seed, s, snap in cases:
        full = full_prediction(args.mission, s, snap, platform)
        append_json(
            args.out / 'predictions.jsonl',
            dict(seed=seed, policy='staleguard_dual_horizon', forecast=full),
        )
        predictions += 1
        if args.experiment == 'prediction':
            if args.max_records and predictions >= args.max_records:
                break
            continue
        command_only = (
            command_prediction(args.mission, snap, platform)
            if args.experiment == 'benchmark'
            else None
        )
        recovery_aware = None
        if args.experiment == 'benchmark' and 'staleguard_recovery_aware' in args.policies:
            recovery_aware = full_prediction(args.mission, s, snap, platform)
            append_json(
                args.out / 'predictions.jsonl',
                dict(seed=seed, policy='staleguard_recovery_aware', forecast=recovery_aware),
            )
            predictions += 1
        if command_only is not None:
            append_json(
                args.out / 'predictions.jsonl',
                dict(seed=seed, policy='staleguard', forecast=command_only),
            )
            predictions += 1
        for d in delays:
            for p in args.policies if args.experiment == 'benchmark' else ('command', 'recovery'):
                forecast = (
                    recovery_aware
                    if p == 'staleguard_recovery_aware'
                    else (
                        full
                        if p == 'staleguard_dual_horizon'
                        else (
                            command_only
                            if p in ('staleguard', 'staleguard_early_recovery')
                            else None
                        )
                    )
                )
                jobs.append(
                    (
                        args.mission,
                        seed,
                        s,
                        snap,
                        d,
                        p if args.experiment == 'feasibility' else None,
                        p if args.experiment == 'benchmark' else None,
                        forecast,
                        platform,
                    )
                )
                if args.max_records and len(jobs) >= args.max_records:
                    break
            if args.max_records and len(jobs) >= args.max_records:
                break
        if args.max_records and len(jobs) >= args.max_records:
            break
    completed = 0
    try:
        if args.workers == 1:
            iterator = map(execute_job, jobs)
            for row in iterator:
                append_json(args.out / 'executions.jsonl', row)
                completed += 1
        else:
            with ProcessPoolExecutor(
                max_workers=args.workers, mp_context=get_context('spawn')
            ) as pool:
                for row in pool.map(execute_job, jobs):
                    append_json(args.out / 'executions.jsonl', row)
                    completed += 1
    finally:
        files = {
            p.name: dict(bytes=p.stat().st_size, sha256=file_hash(p))
            for p in args.out.glob('*.jsonl')
        }
        write_json(
            args.out / 'progress.json',
            dict(
                status='complete' if completed == len(jobs) else 'partial',
                flights=completed,
                policy_trial_outcomes=completed if args.experiment == 'benchmark' else 0,
                new_physical_rollouts=completed,
                reused_physical_rollouts=0,
                decision_only_replays=0,
                predictions=predictions,
                files=files,
            ),
        )
    print('Fresh physical executions:', completed, '; forecasts:', predictions, ';', args.out)


if __name__ == '__main__':
    main()
