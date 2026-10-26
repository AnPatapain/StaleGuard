"""Geofence Turn: run six baseline policies on identical frozen missions."""
from pathlib import Path
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import geofence_turn  # Configure numerical threads before other imports.
import argparse
from dataclasses import asdict
import ctypes
import fcntl
import gc
import hashlib
import json
import math

from model.scg import load_scg_platform_params
from geofence_turn.scenario import Config, HZ, scenario_dict
from geofence_turn.simulation import execute
from geofence_turn.storage import (
    ROOT, load_calibrated_scenarios, check_approval_snapshot,
    metadata, check_provenance, file_hash, append_json, write_json,
)
from geofence_turn.baselines.staleguard import predict_delays, LightweightPredictiveModel
from geofence_turn.baseline_benchmark.metrics import (
    POLICIES, read_executions, flight_key, summarize,
)


# Frozen StaleGuard prediction grid.
TTL_GRID = (
    0., .02, .04, .06, .08, .10, .12, .14, .16, .18, .20,
    .30, .40, .50, .60, .70, .80, .90, 1.,
)

# Preserve every TTL-grid delay and add four distributed off-grid delays.
DISPATCH_DELAYS = tuple(sorted(set(TTL_GRID) | {.24, .44, .64, .84}))


def run(args):
    config = Config()
    cases = load_calibrated_scenarios(args.calibration, config)

    if args.seeds:
        cases = [case for case in cases if case[0].seed in args.seeds]
        if {s.seed for s, _, _ in cases} != set(args.seeds):
            raise ValueError('Requested seed absent from calibrated dataset')

    if not cases:
        raise ValueError('No calibrated scenarios')

    source = Path(args.calibration)
    if source.is_dir():
        source /= (
            'calibrated_scenarios.json'
            if (source / 'calibrated_scenarios.json').exists()
            else 'summary.json'
        )

    protocol = dict(
        scenarios=[scenario_dict(s) for s, _, _ in cases],
        snapshot_sha256={
            str(s.seed): file_hash(p) for s, _, p in cases
        },
        dataset_sha256=file_hash(source),
        config=asdict(config),
        control_hz=HZ,
        policies=list(args.policies),
        delays=list(args.delays),
        ttl_grid_s=list(TTL_GRID),
    )
    protocol = json.loads(json.dumps(protocol))  # Match JSON lists when rerunning.

    folder = (
        args.calibration
        if args.calibration.is_dir()
        else args.calibration.parent
    )

    out = args.out or folder.parent / 'baseline_benchmark'
    out.mkdir(parents=True, exist_ok=True)

    run_path = out / 'run.json'
    journal = out / 'executions.jsonl'

    # Rerunning the same command skips completed flights.
    if run_path.exists():
        saved = json.loads(run_path.read_text())

        if saved['protocol'] != protocol:
            raise ValueError(
                'Saved scenarios, delays, or policies differ: '
                'choose a fresh --out'
            )

        check_provenance(saved['provenance'], config)

        raw = journal.read_bytes()
        size = saved.get('journal_bytes', 0)
        if size and (len(raw) < size or hashlib.sha256(raw[:size]).hexdigest() != saved['journal_sha256']):
            raise ValueError('Previously committed execution records changed or are missing')
        rows = read_executions(journal, repair_tail=True)
        for row in rows:
            if row['event']['policy'] == 'staleguard':
                prediction = saved['predictions'].get(str(row['seed']))
                if prediction is None or any(row['event'].get(name) != prediction[name]
                                             for name in ('computation_s', 'expiry_steps')):
                    raise ValueError('Saved flight differs from its frozen approval prediction')
        completed = {flight_key(row) for row in rows}

    else:
        if any(out.iterdir()):
            raise FileExistsError(
                'Output directory is not empty; choose a fresh --out'
            )

        saved = dict(
            protocol=protocol,
            provenance=metadata(config),
            predictions={},
        )

        write_json(run_path, saved)
        journal.touch()
        completed = set()

    expected = {
        (s.seed, d, p)
        for s, _, _ in cases
        for d in args.delays
        for p in args.policies
    }

    if not completed <= expected:
        raise ValueError('Saved journal contains unexpected flights')

    # Warm up only the predictive models actually needed.
    platform = None

    if completed != expected and {'staleguard', 'use_time_recheck'} & set(args.policies):
        platform = load_scg_platform_params()
        model = LightweightPredictiveModel(*platform)
        try:  # Warm the shared kernels without an extra approval forecast.
            controller, observation, info, _ = model.start(cases[0][1])
            model.step(controller.select_action(observation, info), observation, info)
        finally:
            model.close({})

    new_flights = 0

    try:
        # ===============================================
        # CORE EXPERIMENT
        # ===============================================
        for scenario, snapshot, _ in cases:

            if all(
                (scenario.seed, d, p) in completed
                for d in args.delays
                for p in args.policies
            ):
                continue

            # 1. Predict ONCE at approval.
            # Actual dispatch delay remains unknown.
            prediction = None

            if 'staleguard' in args.policies:
                prediction = saved['predictions'].get(
                    str(scenario.seed)
                )

                if prediction is None:
                    prediction = predict_delays(
                        snapshot,
                        platform,
                        delays=TTL_GRID,
                    )

                    saved['predictions'][str(scenario.seed)] = prediction
                    write_json(run_path, saved)

            # 2. Evaluate identical delays for all policies.
            for delay in args.delays:
                for policy in args.policies:

                    key = (scenario.seed, delay, policy)

                    if key in completed:
                        continue

                    # 3. Execute actual PyBullet flight.
                    result, replayed, _ = execute(
                        scenario,
                        delay,
                        policy=policy,
                        received=prediction,
                        platform=platform,
                        config=config,
                    )

                    check_approval_snapshot(replayed, snapshot)

                    # 4. Save outcome.
                    append_json(journal, result)

                    completed.add(key)
                    new_flights += 1

                    print(
                        f'seed={scenario.seed} '
                        f'D={delay:.2f} '
                        f'{policy} '
                        f'safe={result["safe"]} '
                        f'completed={result["completed"]} '
                        f'recover={result["event"]["fallback_step"] is not None} '
                        f'[{len(completed)}/{len(expected)}]',
                        flush=True,
                    )

                    # Release native PyBullet memory periodically.
                    if new_flights % 20 == 0:
                        gc.collect()
                        ctypes.CDLL(None).malloc_trim(0)

                    if args.max_flights and new_flights >= args.max_flights:
                        return

    finally:
        # 5. Save final metrics (also on interruption).
        raw = journal.read_bytes()

        saved.update(
            status=(
                'complete'
                if completed == expected
                else 'partial'
            ),
            journal_bytes=len(raw),
            journal_sha256=hashlib.sha256(raw).hexdigest(),
        )

        write_json(run_path, saved)
        write_json(
            out / 'metrics.json',
            summarize(journal, protocol),
        )

        print(
            f'Flights: {len(completed)}/{len(expected)}; '
            f'new this run: {new_flights}',
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        '--calibration', '--development',
        type=Path,
        default=ROOT / 'results/development/calibration',
        help='Calibrated dataset directory or JSON: development or evaluation',
    )

    parser.add_argument('--out', type=Path)
    parser.add_argument('--seeds', nargs='+', type=int)

    parser.add_argument(
        '--delays',
        nargs='+',
        type=float,
        default=DISPATCH_DELAYS,
    )

    parser.add_argument(
        '--policies',
        nargs='+',
        choices=POLICIES,
        default=POLICIES,
    )

    parser.add_argument(
        '--max-flights',
        type=int,
        default=0,
        help='0 = all flights; 160 = execute 160, then rerun to continue',
    )

    args = parser.parse_args()

    if (
        not args.delays
        or not args.policies
        or args.max_flights < 0
        or len(set(args.delays)) != len(args.delays)
        or len(set(args.policies)) != len(args.policies)
    ):
        parser.error('Empty/duplicate delays or policies, or invalid --max-flights')

    if any(
        not math.isfinite(d)
        or d < 0
        or not math.isclose(d * HZ, round(d * HZ), abs_tol=1e-9)
        for d in args.delays
    ):
        parser.error('Delays must be multiples of 0.02 seconds')

    with (ROOT / 'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run(args)


if __name__ == '__main__':
    main()
