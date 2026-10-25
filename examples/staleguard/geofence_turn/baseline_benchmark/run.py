"""Comparison of selected existing policies; defaults remain the four-policy grid."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

import geofence_turn  # Apply CPU/thread settings before importing NumPy.
import argparse
import fcntl
import hashlib
import json
import math
import time
import numpy as np
from geofence_turn.scenario import Config, HZ, scenario_dict
from geofence_turn.simulation import start_work, check_memory, execute
from geofence_turn.storage import (ROOT, check_provenance, metadata, file_hash,
                                      write_json, append_json, load_development)
from geofence_turn.baseline_benchmark.metrics import DEFAULT_POLICIES, POLICIES, flight_key, read_executions, summarize

DISPATCH_DELAYS = (0., .2, .6, 1.)  # Original experiment grid, unchanged.


def run_comparison(out, args, work, config):
    from model.scg import load_scg_platform_params
    from geofence_turn.baselines.staleguard import predict_delays
    from geofence_turn.baselines.use_time_recheck import predict_immediate
    out = Path(out)
    cases = load_development(args.development,config)
    if args.seeds is not None:
        cases = [case for case in cases if case[0].seed in args.seeds]
        if {case[0].seed for case in cases} != set(args.seeds):
            raise ValueError('Requested seed is absent from the calibrated dataset')
    if not cases:
        raise ValueError('No calibrated development scenarios')
    policies = tuple(getattr(args,'policies',DEFAULT_POLICIES))
    if not policies or len(set(policies)) != len(policies) or not set(policies) <= set(POLICIES):
        raise ValueError('Select distinct implemented policies')
    source = Path(args.development)
    if source.is_dir():
        source = source/('calibrated_scenarios.json' if (source/'calibrated_scenarios.json').exists() else 'summary.json')
    detailed = getattr(args,'diagnostics',False)
    protocol = json.loads(json.dumps(dict(
        scenarios=[scenario_dict(s) for s,_,_ in cases],delays=list(args.delays),
        policies=list(policies),control_hz=HZ,diagnostics=detailed,
        snapshot_sha256={str(s.seed):file_hash(p) for s,_,p in cases},
        dataset_sha256=file_hash(source))))
    path = out/'run.json'
    journal = out/'executions.jsonl'
    if getattr(args,'resume',False):
        run = json.loads(path.read_text())
        if run.get('format') != 'compact_benchmark_v1':
            raise ValueError('Old benchmark layout cannot resume; retain it and use a fresh output directory')
        check_provenance(run['provenance'],config)
        if run['protocol'] != protocol:
            raise ValueError('Resume cases, snapshots, delays or protocol differ')
        raw = journal.read_bytes() if journal.exists() else b''
        committed = run['journal_bytes']
        if len(raw) < committed or hashlib.sha256(raw[:committed]).hexdigest() != run['journal_sha256']:
            raise ValueError('Previously committed execution records changed or are missing')
        rows = read_executions(journal,repair_tail=True)
    else:
        if out.exists() and any(out.iterdir()):
            raise FileExistsError('Benchmark output contains results; use --resume or a fresh directory')
        out.mkdir(parents=True,exist_ok=True)
        journal.touch()
        rows = []
        run = dict(format='compact_benchmark_v1',provenance=metadata(config),protocol=protocol,
                   predictions={},sessions=[],status='running',journal_bytes=0,
                   journal_sha256=hashlib.sha256(b'').hexdigest())
    expected = {(s.seed,delay,policy) for s,_,_ in cases for delay in args.delays for policy in policies}
    completed = {flight_key(row) for row in rows}
    if not completed <= expected:
        raise ValueError('Journal contains a flight outside the fixed protocol')
    for row in rows:
        if row['event']['policy'] == 'staleguard':
            prediction = run['predictions'].get(str(row['seed']))
            if prediction is None or any(row['event'].get(key) != prediction[key] for key in ('computation_s','expiry_steps')):
                raise ValueError('Saved flight does not match its frozen approval prediction')
    raw = journal.read_bytes() if journal.exists() else b''
    digest = hashlib.sha256(raw)
    byte_count = len(raw)
    session = dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),trials=0)
    run['sessions'].append(session)
    run['status'] = 'running'
    run.pop('error',None)

    def checkpoint():
        run.update(journal_bytes=byte_count,journal_sha256=digest.hexdigest())
        session.update(trials=work['trials'],elapsed_s=time.monotonic()-work['start'])
        write_json(path,run)

    checkpoint()
    try:
        if completed != expected:
            platform = None
            if 'staleguard' in policies or 'use_time_recheck' in policies:
                platform = load_scg_platform_params()
                warm = cases[0][1]
                predict_delays(warm,platform)
                predict_immediate(warm,platform)
            for s,snapshot,_ in cases:
                if all((s.seed,delay,policy) in completed for delay in args.delays for policy in policies):
                    continue
                # Freeze the approval-only forecast before enumerating actual D.
                received = run['predictions'].get(str(s.seed))
                if received is None and 'staleguard' in policies:
                    received = predict_delays(snapshot,platform)
                    run['predictions'][str(s.seed)] = received
                    checkpoint()
                for delay in args.delays:
                    for policy in policies:
                        key = s.seed,delay,policy
                        if key in completed:
                            continue
                        result,snap,trace = execute(s,delay,policy=policy,received=received,
                            platform=platform,save_trace=detailed,config=config,work=work)
                        np.testing.assert_array_equal(snap.state,snapshot.state)
                        np.testing.assert_array_equal(snap.remaining_reference,snapshot.remaining_reference)
                        assert snap.command == snapshot.command
                        if detailed:
                            traces = out/'diagnostics'
                            traces.mkdir(exist_ok=True)
                            np.savez_compressed(traces/f'seed{s.seed}_D{delay}_{policy}.npz',controls=trace)
                        append_json(journal,result)
                        line = (json.dumps(result,allow_nan=False)+'\n').encode()
                        digest.update(line)
                        byte_count += len(line)
                        completed.add(key)
                        checkpoint()
                        print(f'seed={s.seed} D={delay:.2f} {policy} safe={result["safe"]} complete={result["completed"]} fallback={result["event"]["fallback_step"]}',flush=True)
        rss,available = check_memory(config)
        session.update(final_rss_mb=rss,available_mb=available)
        run['status'] = 'complete'
    except BaseException as error:
        run.update(status='interrupted',error=dict(type=type(error).__name__,message=str(error)))
        raise
    finally:
        checkpoint()
        write_json(out/'metrics.json',summarize(journal,protocol))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',default=str(ROOT/'results/development/baseline_benchmark'))
    parser.add_argument('--development',default=str(ROOT/'results/development/calibration'))
    parser.add_argument('--seeds',nargs='+',type=int)
    parser.add_argument('--delays',nargs='+',type=float,default=DISPATCH_DELAYS)
    parser.add_argument('--policies',nargs='+',choices=POLICIES,default=DEFAULT_POLICIES)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--diagnostics',action='store_true',help='Save individual flight traces under diagnostics/')
    parser.add_argument('--max-trials',type=int,default=160)
    parser.add_argument('--max-seconds',type=float,default=900)
    args = parser.parse_args()
    if len(set(args.delays)) != len(args.delays):
        parser.error('Duplicate delay')
    if any(not math.isfinite(d) or d < 0 or not math.isclose(d*HZ,round(d*HZ),abs_tol=1e-9) for d in args.delays):
        parser.error('Dispatch delays must be nonnegative multiples of the 0.02 s control step')
    if len(set(args.policies)) != len(args.policies):
        parser.error('Duplicate policy')
    config = Config()
    work = start_work(args.max_trials,args.max_seconds)
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        run_comparison(Path(args.out),args,work,config)


if __name__ == '__main__':
    main()
