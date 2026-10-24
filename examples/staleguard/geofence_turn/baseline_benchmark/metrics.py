"""Derive benchmark metrics from authoritative flight records, without simulation."""
import argparse
from collections import defaultdict
from enum import Enum
import hashlib
import json
import math
import os
from pathlib import Path
import statistics

POLICIES = ('no_recovery', 'staleguard', 'use_time_recheck', 'fixed_ttl')
CONTROL_HZ = 50  # Fixed Geofence control frequency; also recorded in run.json.


class Metric(Enum):
    # Number of completed, recorded flights.
    FLIGHTS = 'flights'
    # Checkpoint objective completed before the deadline, with safety throughout.
    SAFE_COMPLETION = 'safe_completion'
    # Safe throughout, but checkpoint objective not completed before the deadline.
    SAFE_NONCOMPLETION = 'safe_noncompletion'
    # Required body clearance violated at some point during the mission.
    UNSAFE = 'unsafe'
    # Safe completions / flights; null when no flights are recorded.
    SAFE_COMPLETION_RATE = 'safe_completion_rate'
    # Safe non-completions / flights; null when no flights are recorded.
    SAFE_NONCOMPLETION_RATE = 'safe_noncompletion_rate'
    # Unsafe flights / flights; null when no flights are recorded.
    UNSAFE_RATE = 'unsafe_rate'
    # Recovery actually activated, identified by event.fallback_step.
    RECOVERY_ATTEMPTS = 'recovery_attempts'
    # Recovery activated and no safety violation anywhere in the mission.
    RECOVERY_SUCCESSES = 'recovery_successes'
    # Recovery activated and a safety violation occurred during the mission.
    RECOVERY_FAILURES = 'recovery_failures'
    # Recovery activations / flights.
    RECOVERY_ATTEMPT_RATE = 'recovery_attempt_rate'
    # Safe recoveries / recovery attempts; null when none were activated.
    RECOVERY_SUCCESS_RATE = 'recovery_success_rate'
    # Unsafe recoveries / recovery attempts; null when none were activated.
    RECOVERY_FAILURE_RATE = 'recovery_failure_rate'
    # Measured prediction duration in seconds: mean, median and nearest-rank p95.
    PREDICTION_TIME = 'prediction_computation_s'
    # Actual action tick minus scheduled dispatch tick, divided by control Hz.
    ACTION_DELAY = 'dispatch_to_action_s'
    # StaleGuard result available at dispatch; null for other policies.
    READY = 'prediction_ready'
    # StaleGuard result unavailable at dispatch; null for other policies.
    UNAVAILABLE = 'prediction_unavailable'
    # StaleGuard readiness absent from the record; never assume availability.
    READINESS_MISSING = 'prediction_readiness_unrecorded'
    # Scenario-delay groups containing both StaleGuard and Use-Time Recheck.
    COMPLETE_PAIRS = 'complete_pairs'
    # Groups missing either compared policy; excluded from paired calculations.
    INCOMPLETE_PAIRS = 'incomplete_pairs'
    # Planned groups without any flight record, when a fixed protocol is present.
    UNSTARTED_PAIRS = 'unstarted_pairs'
    # StaleGuard completes safely and Use-Time Recheck does not.
    COMPLETION_BENEFITS = 'completion_benefits'
    # Use-Time Recheck completes safely and StaleGuard does not.
    COMPLETION_HARMS = 'completion_harms'
    # StaleGuard stays safe and Use-Time Recheck is unsafe.
    UNSAFE_PREVENTED = 'unsafe_prevented'
    # Use-Time Recheck stays safe and StaleGuard is unsafe.
    UNSAFE_INTRODUCED = 'unsafe_introduced'
    # Completion benefits minus completion harms; a signed count, not a rate.
    NET_COMPLETION_BENEFIT = 'net_completion_benefit'
    # Unsafe prevented minus unsafe introduced; a signed count, not a rate.
    NET_UNSAFE_REDUCTION = 'net_unsafe_reduction'


def flight_key(row):
    return row['seed'], row['delay_s'], row['event']['policy']


def read_executions(path, repair_tail=False):
    """Reject duplicates/corruption; resume may remove only an unfinished last line."""
    path = Path(path)
    if not path.exists():
        return []
    raw = path.read_bytes()
    rows, seen, offset = [], set(), 0
    lines = raw.splitlines(keepends=True)
    for index, line in enumerate(lines):
        try:
            row = json.loads(line)
        except (ValueError, UnicodeError) as error:
            if repair_tail and index == len(lines)-1 and not line.endswith(b'\n'):
                with path.open('r+b') as stream:
                    stream.truncate(offset)
                    stream.flush()
                    os.fsync(stream.fileno())
                break
            raise ValueError(f'Invalid execution JSON on line {index+1}') from error
        key = flight_key(row)
        if key in seen:
            raise ValueError(f'Duplicate flight in policy journal: {key}')
        if key[2] not in POLICIES or not math.isfinite(key[1]) or key[1] < 0:
            raise ValueError(f'Invalid policy or delay: {key}')
        if any(not isinstance(row[name], bool) for name in ('safe','completed','safe_completion')):
            raise ValueError(f'Outcome flags must be Boolean: {key}')
        if row['safe_completion'] != (row['safe'] and row['completed']):
            raise ValueError(f'Inconsistent safety/completion flags: {key}')
        seen.add(key)
        rows.append(row)
        offset += len(line)
    else:
        if repair_tail and raw and not raw.endswith(b'\n'):
            with path.open('ab') as stream:
                stream.write(b'\n')
                stream.flush()
                os.fsync(stream.fileno())
    return rows


def timing_summary(values):
    """Missing samples stay missing; p95 uses the documented nearest-rank rule."""
    values = [value for value in values if value is not None]
    if any(not isinstance(x,(int,float)) or isinstance(x,bool) or not math.isfinite(x) or x < 0 for x in values):
        raise ValueError('Recorded timing must be finite and nonnegative')
    ordered = sorted(values)
    return dict(samples=len(values), mean_s=statistics.mean(values) if values else None,
                median_s=statistics.median(values) if values else None,
                p95_s=ordered[math.ceil(.95*len(values))-1] if values else None)


def policy_metrics(rows, policy, control_hz):
    n = len(rows)
    completed = sum(row['safe_completion'] for row in rows)
    unsafe = sum(not row['safe'] for row in rows)
    recovery = [row for row in rows if row['event'].get('fallback_step') is not None]
    prediction_times, action_delays = {}, []
    for row in rows:
        event = row['event']
        duration = event.get('computation_s')
        # One frozen approval forecast per scenario for StaleGuard; one check per
        # flight for Use-Time Recheck. Do not count a cached forecast repeatedly.
        key = row['seed'] if policy == 'staleguard' else flight_key(row)
        if duration is not None:
            if key in prediction_times and prediction_times[key] != duration:
                raise ValueError('Conflicting computation times for a reused approval prediction')
            prediction_times[key] = duration
        dispatch = event.get('scheduled_dispatch_step')
        action = event.get('fallback_step')
        if action is None:
            action = event.get('application_step')
        # A decision tick alone is not proof that an action actually occurred.
        action_delays.append(None if dispatch is None or action is None else (action-dispatch)/control_hz)
    value = {
        Metric.FLIGHTS: n,
        Metric.SAFE_COMPLETION: completed,
        Metric.SAFE_NONCOMPLETION: sum(row['safe'] and not row['completed'] for row in rows),
        Metric.UNSAFE: unsafe,
        Metric.SAFE_COMPLETION_RATE: completed/n if n else None,
        Metric.SAFE_NONCOMPLETION_RATE: sum(row['safe'] and not row['completed'] for row in rows)/n if n else None,
        Metric.UNSAFE_RATE: unsafe/n if n else None,
        Metric.RECOVERY_ATTEMPTS: len(recovery),
        Metric.RECOVERY_SUCCESSES: sum(row['safe'] for row in recovery),
        Metric.RECOVERY_FAILURES: sum(not row['safe'] for row in recovery),
        Metric.RECOVERY_ATTEMPT_RATE: len(recovery)/n if n else None,
        Metric.RECOVERY_SUCCESS_RATE: sum(row['safe'] for row in recovery)/len(recovery) if recovery else None,
        Metric.RECOVERY_FAILURE_RATE: sum(not row['safe'] for row in recovery)/len(recovery) if recovery else None,
        Metric.PREDICTION_TIME: timing_summary(prediction_times.values()),
        Metric.ACTION_DELAY: timing_summary(action_delays),
        Metric.READY: sum(row['event'].get('prediction_ready') is True for row in rows) if policy=='staleguard' else None,
        Metric.UNAVAILABLE: sum(row['event'].get('prediction_ready') is False for row in rows) if policy=='staleguard' else None,
        Metric.READINESS_MISSING: sum(row['event'].get('prediction_ready') is None for row in rows) if policy=='staleguard' else None,
    }
    return {metric.value: result for metric,result in value.items()}


def paired_metrics(rows, expected_pairs=None):
    blocks = defaultdict(dict)
    for row in rows:
        blocks[row['seed'],row['delay_s']][row['event']['policy']] = row
    keys = set(blocks) if expected_pairs is None else set(expected_pairs)
    pairs = [blocks[key] for key in sorted(keys)
             if {'staleguard','use_time_recheck'} <= blocks[key].keys()]
    benefits = harms = prevented = introduced = 0
    for pair in pairs:
        sg, ut = pair['staleguard'], pair['use_time_recheck']
        benefits += sg['safe_completion'] and not ut['safe_completion']
        harms += ut['safe_completion'] and not sg['safe_completion']
        prevented += sg['safe'] and not ut['safe']
        introduced += ut['safe'] and not sg['safe']
    values = {
        Metric.COMPLETE_PAIRS: len(pairs),
        Metric.INCOMPLETE_PAIRS: len(keys)-len(pairs),
        Metric.UNSTARTED_PAIRS: sum(not blocks[key] for key in keys),
        Metric.COMPLETION_BENEFITS: benefits if pairs else None,
        Metric.COMPLETION_HARMS: harms if pairs else None,
        Metric.UNSAFE_PREVENTED: prevented if pairs else None,
        Metric.UNSAFE_INTRODUCED: introduced if pairs else None,
        Metric.NET_COMPLETION_BENEFIT: benefits-harms if pairs else None,
        Metric.NET_UNSAFE_REDUCTION: prevented-introduced if pairs else None,
    }
    return {metric.value: value for metric,value in values.items()}


def summarize(path, protocol=None):
    """Reproducible from JSONL and fixed protocol; includes all recorded flights."""
    path = Path(path)
    if path.is_dir():
        path = path/'executions.jsonl'
    if not path.is_file():
        raise FileNotFoundError(f'Execution journal does not exist: {path}')
    if protocol is None:
        run = path.with_name('run.json')
        protocol = json.loads(run.read_text())['protocol'] if run.exists() else {}
    control_hz = protocol.get('control_hz',CONTROL_HZ)
    if not isinstance(control_hz,(int,float)) or not math.isfinite(control_hz) or control_hz <= 0:
        raise ValueError('Invalid protocol control frequency')
    rows = read_executions(path)
    policies = protocol.get('policies') or [p for p in POLICIES if any(r['event']['policy']==p for r in rows)]
    delays = protocol.get('delays') or sorted({row['delay_s'] for row in rows})
    seeds = {s['seed'] for s in protocol.get('scenarios',[])}
    expected = {(seed,delay) for seed in seeds for delay in delays} if seeds else None
    # Protocol membership is also checked by the runner before resuming.
    if any(row['event']['policy'] not in policies or row['delay_s'] not in delays
           or seeds and row['seed'] not in seeds for row in rows):
        raise ValueError('Journal contains a flight outside the fixed protocol')
    by_policy = {p:policy_metrics([r for r in rows if r['event']['policy']==p],p,control_hz) for p in policies}
    by_delay = {}
    for delay in delays:
        selected = [row for row in rows if row['delay_s']==delay]
        subset = None if expected is None else {key for key in expected if key[1]==delay}
        by_delay[str(float(delay))] = dict(
            by_policy={p:policy_metrics([r for r in selected if r['event']['policy']==p],p,control_hz) for p in policies},
            paired_comparison=paired_metrics(selected,subset))
    return dict(schema='geofence_benchmark_metrics_v1', recorded_flights=len(rows),
                control_hz=control_hz, timing_units='seconds',
                prediction_timing_samples='StaleGuard: one recorded approval forecast per seed; other policies: recorded per-flight checks.',
                journal_sha256=hashlib.sha256(path.read_bytes() if path.exists() else b'').hexdigest(),
                by_policy=by_policy, by_delay=by_delay,
                paired_comparison=paired_metrics(rows,expected))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executions',type=Path)
    parser.add_argument('--out',type=Path)
    args = parser.parse_args()
    result = summarize(args.executions)
    journal = args.executions/'executions.jsonl' if args.executions.is_dir() else args.executions
    target = args.out or journal.with_name('metrics.json')
    target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(f'Wrote {target}: {result["recorded_flights"]} recorded flights.')
