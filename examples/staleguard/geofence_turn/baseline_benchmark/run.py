"""Existing paired four-policy evaluation; no Always-Recover or FBS."""
from pathlib import Path
import sys
if __package__ in (None, ""):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

import geofence_turn  # Apply CPU/thread settings before importing NumPy.

import argparse
from collections import Counter
import fcntl
import json
import time
import numpy as np
from geofence_turn.scenario import Config
from geofence_turn.simulation import start_work, check_memory, execute
from geofence_turn.calibration import (ROOT, DEFAULT_DEVELOPMENT, prepare_output,
                                      write_json, append_json, load_development)
from geofence_turn.scenario import scenario_dict
from geofence_turn.baseline_benchmark.metrics import summarize

DISPATCH_DELAYS = (0., .2, .6, 1.)  # Existing experiment grid; subset with --delays.

def run_comparison(out, args, work, config):
    from model.scg import load_scg_platform_params
    from geofence_turn.baselines.staleguard import predict_delays
    from geofence_turn.baselines.use_time_recheck import predict_immediate
    platform = load_scg_platform_params()
    cases = load_development(args.development,config)
    if args.seeds is not None:
        cases = [case for case in cases if case[0].seed in args.seeds]
        if {case[0].seed for case in cases} != set(args.seeds):
            raise ValueError('Requested seed is absent from the calibrated dataset')
    if not cases:
        raise ValueError('No calibrated development scenarios')
    protocol = json.loads(json.dumps(dict(scenarios=[scenario_dict(s) for s,_,_ in cases],
                                        delays=list(args.delays))))
    if (out/'protocol.json').exists():
        if json.loads((out/'protocol.json').read_text()) != protocol:
            raise ValueError('Resume cases or delay grid differ from the recorded comparison')
    else:
        write_json(out/'protocol.json', protocol)
    warm = cases[0][1]
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
    for s, snapshot, path in cases:
        # Deliberately before actual D enumeration; no future-delay argument.
        prediction_file = out/f'prediction_seed_{s.seed}.json'
        if prediction_file.exists():
            received = json.loads(prediction_file.read_text())
        else:
            received = predict_delays(snapshot, platform)
            write_json(prediction_file, received)
        for delay in args.delays:
            for policy in counts:
                if (s.seed, delay, policy) in completed:
                    continue
                result, snap, trace = execute(s, delay, policy=policy, received=received,
                                             platform=platform, save_trace=True,
                                             config=config, work=work)
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    parser.add_argument('--development',default=str(DEFAULT_DEVELOPMENT))
    parser.add_argument('--seeds',nargs='+',type=int)
    parser.add_argument('--delays',nargs='+',type=float,choices=DISPATCH_DELAYS,default=DISPATCH_DELAYS)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--max-trials',type=int,default=160)
    parser.add_argument('--max-seconds',type=float,default=900)
    args = parser.parse_args()
    if len(set(args.delays)) != len(args.delays):
        parser.error('Duplicate delay')
    config = Config()
    work = start_work(args.max_trials, args.max_seconds)
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        out = prepare_output(args.out,config,args.resume)
        try:
            run_comparison(out, args, work, config)
        except Exception as error:
            append_json(out/'failures.jsonl', dict(type=type(error).__name__, message=str(error),
                                                 completed_trials=work['trials']))
            raise
        write_json(out/'paired_summary.json',summarize(out))
        rss,available = check_memory(config)
        write_json(out/'finished.json',dict(trials=work['trials'],elapsed_s=time.monotonic()-work['start'],final_rss_mb=rss,available_mb=available))


if __name__ == '__main__':
    main()
