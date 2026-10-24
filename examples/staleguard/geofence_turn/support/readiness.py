"""Existing warmed prediction-timing diagnostic, not a TTL accuracy experiment.

The geofence mission-run prediction-accuracy protocol remains unfinished. This
script only relocates the existing saved-snapshot readiness benchmark.
"""
from pathlib import Path
import sys
if __package__ in (None, ""):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

import geofence_turn  # Apply CPU/thread settings before importing NumPy.

import argparse
import fcntl
import json
import math
import os
import pickle
import time
import numpy as np
from geofence_turn.scenario import Config
from geofence_turn.simulation import start_work, check_work, check_memory
from geofence_turn.calibration import ROOT, DEFAULT_DEVELOPMENT, prepare_output, write_json, append_json, file_hash, load_development


def measure_readiness(out, args, work, config):
    """Time the unchanged forecast with cached approval snapshots, not flights."""
    from model.scg import load_scg_platform_params
    from geofence_turn.baselines.staleguard import PREDICTION_DELAYS, predict_delays
    from geofence_turn.baselines.use_time_recheck import predict_immediate
    cases = load_development(args.development,config)
    if not 1 <= len(cases) <= 5:
        raise ValueError('Readiness benchmark requires 1..5 development snapshots')
    paths = [path for _,_,path in cases]
    snapshots = [snapshot for _,snapshot,_ in cases]
    platform = load_scg_platform_params()

    def measured(function, snapshot):
        check_work(work,config)
        began = time.perf_counter()
        result = function(snapshot, platform)
        wall_s = time.perf_counter()-began
        # Charge the exact clocks used by StaleGuard and Use-Time Recheck.
        return result, result.get('computation_s', wall_s)*1000

    first, first_ms = measured(predict_delays, snapshots[0])
    measured(predict_immediate, snapshots[0])
    fingerprints = [pickle.dumps(snapshot) for snapshot in snapshots]
    signatures, samples = {}, []
    for repeat in range(args.repeats):
        for path, snapshot in zip(paths, snapshots):
            seed = int(path.parent.name.split('_')[1])
            forecast, full_ms = measured(predict_delays, snapshot)
            immediate, immediate_ms = measured(predict_immediate, snapshot)
            signature = (forecast['results'], forecast['expiry_steps'])
            if seed not in signatures:
                signatures[seed] = signature
                write_json(out/f'prediction_seed_{seed}.json', forecast)
            assert signature == signatures[seed], 'Forecast changed across timing repetitions'
            assert immediate == forecast['results'][0], 'Immediate and sweep D=0 disagree'
            row = dict(seed=seed, repeat=repeat, full_sweep_ms=full_ms,
                       immediate_ms=immediate_ms)
            samples.append(row)
            append_json(out/'timings.jsonl', row)
    assert [pickle.dumps(snapshot) for snapshot in snapshots] == fingerprints

    def stats(values):
        ordered = sorted(values)
        return dict(count=len(values), min_ms=min(values), median_ms=float(np.median(values)),
                    p95_ms=ordered[math.ceil(.95*len(values))-1], max_ms=max(values))

    summary = dict(development=str(Path(args.development).resolve()),
                   snapshot_sha256={path.parent.name:file_hash(path) for path in paths},
                   delay_grid_s=list(PREDICTION_DELAYS), repeats=args.repeats,
                   first_sweep_existing_numba_disk_cache_ms=first_ms,
                   startup_policy='Warm sweep and immediate model before approval; first call excluded from warm statistics',
                   warm_full_sweep=stats([r['full_sweep_ms'] for r in samples]),
                   warm_immediate=stats([r['immediate_ms'] for r in samples]),
                   per_seed={str(seed):stats([r['full_sweep_ms'] for r in samples if r['seed']==seed])
                             for seed in signatures},
                   forecasts_identical_across_repetitions=True, snapshots_unchanged=True,
                   forecast_calls=work['trials'], physics_flights=0, host_load_average=os.getloadavg())
    write_json(out/'summary.json', summary)
    print(json.dumps(summary, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    parser.add_argument('--development',default=str(DEFAULT_DEVELOPMENT))
    parser.add_argument('--repeats',type=int,default=10)
    parser.add_argument('--max-trials',type=int,default=160)
    parser.add_argument('--max-seconds',type=float,default=900)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 10:
        parser.error('Readiness cap: 1..10 repetitions')
    config = Config()
    work = start_work(args.max_trials, args.max_seconds)
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        out = prepare_output(args.out,config)
        print('READINESS DIAGNOSTIC ONLY: the mission-run TTL accuracy experiment is not implemented.',flush=True)
        try:
            measure_readiness(out, args, work, config)
        except Exception as error:
            append_json(out/'failures.jsonl', dict(type=type(error).__name__, message=str(error),
                                                 completed_trials=work['trials']))
            raise
        rss,available = check_memory(config)
        write_json(out/'finished.json',dict(trials=work['trials'],elapsed_s=time.monotonic()-work['start'],final_rss_mb=rss,available_mb=available))


if __name__ == '__main__':
    main()
