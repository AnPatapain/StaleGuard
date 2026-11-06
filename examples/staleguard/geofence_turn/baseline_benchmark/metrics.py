"""Summarize Geofence Turn flights and paired baseline comparisons."""
import argparse
from collections import defaultdict
from enum import Enum
import hashlib
import json
import math
from pathlib import Path
import statistics

POLICIES = (
    'no_recovery',
    'always_recover',
    'fixed_ttl',
    'fbs',
    'use_time_recheck',
    'staleguard',
    'staleguard_early_recovery',
    'fixed_early_recovery',
    'ppr',
)
CONTROL_HZ = 50


class Metric(Enum):
    FLIGHTS = 'flights'
    SAFE_COMPLETION = 'safe_completion'
    SAFE_NONCOMPLETION = 'safe_noncompletion'
    UNSAFE = 'unsafe'
    SAFE_COMPLETION_RATE = 'safe_completion_rate'
    SAFE_NONCOMPLETION_RATE = 'safe_noncompletion_rate'
    UNSAFE_RATE = 'unsafe_rate'
    RECOVERY_ATTEMPTS = 'recovery_attempts'
    RECOVERY_SUCCESSES = 'recovery_successes'
    RECOVERY_FAILURES = 'recovery_failures'
    RECOVERY_ATTEMPT_RATE = 'recovery_attempt_rate'
    RECOVERY_SUCCESS_RATE = 'recovery_success_rate'
    RECOVERY_FAILURE_RATE = 'recovery_failure_rate'
    PREDICTION_TIME = 'prediction_computation_s'
    PREDICTION_TIME_MS = 'prediction_computation_ms'
    FBS_CUMULATIVE_TIME = 'fbs_cumulative_observation_computation_s'
    FBS_CUMULATIVE_TIME_MS = 'fbs_cumulative_observation_computation_ms'
    PPR_CHECKS = 'predictive_checks'
    PPR_CHECK_COUNT = 'predictive_checks_per_flight'
    PPR_TOTAL_TIME = 'prediction_computation_total_s'  # Distribution of per-flight sums.
    PPR_MAX_TIME = 'prediction_computation_max_s'  # Distribution of per-flight maxima.
    PPR_SUM_TIME = 'prediction_computation_sum_s'  # Sum over all actual checks.
    PPR_DECISION_LATENCY = 'prediction_to_decision_s'  # Observation to consumed result, including ticks.
    PPR_RECOVERY_LATENCY = 'prediction_to_recovery_s'  # Observation to actual recovery activation.
    PPR_CONSUMED = 'predictions_consumed'
    PPR_DISCARDED = 'predictions_discarded_at_dispatch'
    PPR_OVERRUNS = 'prediction_timing_bound_exceeded'
    ACTION_DELAY = 'dispatch_to_action_s'
    ACTION_DELAY_MS = 'dispatch_to_action_ms'
    EARLY_RECOVERIES = 'early_recoveries'
    EARLY_LEAD = 'early_recovery_lead_s'
    EARLY_LEAD_MS = 'early_recovery_lead_ms'
    READY = 'prediction_ready'
    UNAVAILABLE = 'prediction_unavailable'
    READINESS_MISSING = 'prediction_readiness_unrecorded'
    UNVERIFIED_EXECUTIONS = 'unverified_executions'
    COMPLETE_PAIRS = 'complete_pairs'
    INCOMPLETE_PAIRS = 'incomplete_pairs'
    UNSTARTED_PAIRS = 'unstarted_pairs'
    COMPLETION_BENEFITS = 'completion_benefits'
    COMPLETION_HARMS = 'completion_harms'
    UNSAFE_PREVENTED = 'unsafe_prevented'
    UNSAFE_INTRODUCED = 'unsafe_introduced'
    NET_COMPLETION_BENEFIT = 'net_completion_benefit'
    NET_UNSAFE_REDUCTION = 'net_unsafe_reduction'


def flight_key(row):
    return row['seed'], row['delay_s'], row['event']['policy']


def read_executions(path, repair_tail=False):
    """Read flight journal, rejecting duplicate/inconsistent outcomes."""
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
                break
            raise ValueError(f'Invalid execution JSON on line {index+1}') from error
        key = flight_key(row)
        if key in seen:
            raise ValueError(f'Duplicate flight in policy journal: {key}')
        if key[2] not in POLICIES or not math.isfinite(key[1]) or key[1] < 0:
            raise ValueError(f'Invalid policy or delay: {key}')
        if any(type(row[name]) is not bool for name in ('safe', 'completed', 'safe_completion')):
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
    return rows


def timing_summary(values):
    values = [value for value in values if value is not None]
    if any(not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x) or x < 0
           for x in values):
        raise ValueError('Recorded timing must be finite and nonnegative')
    ordered = sorted(values)
    return dict(samples=len(values),
                mean_s=statistics.mean(values) if values else None,
                median_s=statistics.median(values) if values else None,
                p95_s=ordered[math.ceil(.95 * len(values)) - 1] if values else None)


def milliseconds(summary):
    return dict(samples=summary['samples'], **{
        name + '_ms': None if summary[name + '_s'] is None else 1000 * summary[name + '_s']
        for name in ('mean', 'median', 'p95')
    })


def policy_metrics(rows, policy, control_hz):
    n = len(rows)
    completed = sum(row['safe_completion'] for row in rows)
    safe_incomplete = sum(row['safe'] and not row['completed'] for row in rows)
    unsafe = sum(not row['safe'] for row in rows)
    recovery = [row for row in rows if row['event'].get('fallback_step') is not None]

    prediction_times, action_delays, fbs_cumulative_times, early_leads = {}, [], [], []
    for row in rows:
        event = row['event']
        duration = event.get('computation_s')
        # Approval-time StaleGuard forecasts are reused across dispatch delays.
        key = row['seed'] if policy in ('staleguard', 'staleguard_early_recovery') else flight_key(row)
        if duration is not None:
            if key in prediction_times and prediction_times[key] != duration:
                raise ValueError('Conflicting timing for reused StaleGuard forecast')
            prediction_times[key] = duration
        if policy == 'fbs' and event.get('observation_update_s'):
            fbs_cumulative_times.append(math.fsum(event['observation_update_s']))
        dispatch = event.get('scheduled_dispatch_step')
        fallback = event.get('fallback_step')
        action = fallback if fallback is not None else event.get('application_step')
        is_early = fallback is not None and dispatch is not None and fallback < dispatch
        if is_early != (event.get('early_recovery') is True):
            raise ValueError('Early-recovery flag disagrees with fallback timing')
        if is_early:
            if (policy not in ('staleguard_early_recovery', 'fixed_early_recovery', 'ppr')
                    or event.get('application_step') is not None):
                raise ValueError('Early recovery must cancel the pending command')
            early_leads.append((dispatch - fallback) / control_hz)
        # Pre-dispatch intervention is not a negative dispatch-to-action latency.
        action_delays.append(None if dispatch is None or action is None or is_early
                             else (action - dispatch) / control_hz)

    checks = [check for row in rows for check in row['event']['ppr_checks']] if policy == 'ppr' else []
    timed = timing_summary([c['computation_s'] for c in checks] if policy == 'ppr' else prediction_times.values())
    action = timing_summary(action_delays)
    guarded = policy in ('staleguard', 'staleguard_early_recovery', 'fbs')
    ready = sum(r['event'].get('prediction_ready') is True for r in rows)
    unavailable = sum(r['event'].get('prediction_ready') is False for r in rows)
    missing = n - ready - unavailable
    unverified = sum(r['event'].get('prediction_ready') is False
                     and r['event'].get('application_step') is not None for r in rows)

    values = {
        Metric.FLIGHTS: n,
        Metric.SAFE_COMPLETION: completed,
        Metric.SAFE_NONCOMPLETION: safe_incomplete,
        Metric.UNSAFE: unsafe,
        Metric.SAFE_COMPLETION_RATE: completed / n if n else None,
        Metric.SAFE_NONCOMPLETION_RATE: safe_incomplete / n if n else None,
        Metric.UNSAFE_RATE: unsafe / n if n else None,
        Metric.RECOVERY_ATTEMPTS: len(recovery),
        Metric.RECOVERY_SUCCESSES: sum(row['safe'] for row in recovery),
        Metric.RECOVERY_FAILURES: sum(not row['safe'] for row in recovery),
        Metric.RECOVERY_ATTEMPT_RATE: len(recovery) / n if n else None,
        Metric.RECOVERY_SUCCESS_RATE: sum(row['safe'] for row in recovery) / len(recovery) if recovery else None,
        Metric.RECOVERY_FAILURE_RATE: sum(not row['safe'] for row in recovery) / len(recovery) if recovery else None,
        Metric.PREDICTION_TIME: timed,
        Metric.PREDICTION_TIME_MS: milliseconds(timed),
        Metric.ACTION_DELAY: action,
        Metric.ACTION_DELAY_MS: milliseconds(action),
        Metric.READY: ready if guarded else None,
        Metric.UNAVAILABLE: unavailable if guarded else None,
        Metric.READINESS_MISSING: missing if guarded else None,
        Metric.UNVERIFIED_EXECUTIONS: unverified if guarded else None,
    }
    if policy == 'fbs':
        cumulative = timing_summary(fbs_cumulative_times)
        values.update({Metric.FBS_CUMULATIVE_TIME: cumulative,
                       Metric.FBS_CUMULATIVE_TIME_MS: milliseconds(cumulative)})
    if policy in ('staleguard_early_recovery', 'fixed_early_recovery', 'ppr'):
        lead = timing_summary(early_leads)
        values.update({Metric.EARLY_RECOVERIES: len(early_leads),
                       Metric.EARLY_LEAD: lead,
                       Metric.EARLY_LEAD_MS: milliseconds(lead)})
    if policy == 'ppr':
        counts, totals, maxima = [], [], []
        for row in rows:
            event = row['event']
            times = [c['computation_s'] for c in event['ppr_checks']]
            if (event['prediction_check_count'] != len(times)
                    or not math.isclose(event['prediction_computation_total_s'], math.fsum(times), abs_tol=1e-12)
                    or event['prediction_computation_max_s'] != max(times, default=None)):
                raise ValueError('PPR check counts/timing totals disagree with actual checks')
            counts.append(len(times)); totals.append(event['prediction_computation_total_s'])
            maxima.append(event['prediction_computation_max_s'])
        ordered = sorted(counts)
        values.update({
            Metric.PPR_CHECKS: len(checks),
            Metric.PPR_CHECK_COUNT: dict(samples=n, mean=statistics.mean(counts) if n else None,
                median=statistics.median(counts) if n else None,
                p95=ordered[math.ceil(.95*n)-1] if n else None, maximum=max(counts, default=None)),
            Metric.PPR_TOTAL_TIME: timing_summary(totals),
            Metric.PPR_MAX_TIME: timing_summary(maxima),
            Metric.PPR_SUM_TIME: math.fsum(c['computation_s'] for c in checks),
            Metric.PPR_DECISION_LATENCY: timing_summary([
                (c['decision_step']-c['observation_step'])/control_hz
                for c in checks if c['decision_step'] is not None]),
            Metric.PPR_RECOVERY_LATENCY: timing_summary([
                (c['action_step']-c['observation_step'])/control_hz
                for c in checks if c['action_step'] is not None]),
            Metric.PPR_CONSUMED: sum(c['decision_step'] is not None for c in checks),
            Metric.PPR_DISCARDED: sum(c['discarded_at_dispatch'] for c in checks),
            Metric.PPR_OVERRUNS: sum(c['timing_bound_exceeded'] for c in checks),
        })
    return {metric.value: value for metric, value in values.items()}


def paired_metrics(rows, expected_pairs=None, other='use_time_recheck', reference='staleguard'):
    """Paired difference: positive means 'reference' outperforms 'other'."""
    blocks = defaultdict(dict)
    for row in rows:
        blocks[row['seed'], row['delay_s']][row['event']['policy']] = row
    keys = set(blocks) if expected_pairs is None else set(expected_pairs)
    unstarted = sum(key not in blocks for key in keys)
    pairs = [blocks[key] for key in sorted(keys)
             if key in blocks and {reference, other} <= blocks[key].keys()]

    benefits = harms = prevented = introduced = 0
    for pair in pairs:
        first, baseline = pair[reference], pair[other]
        benefits += bool(first['safe_completion'] and not baseline['safe_completion'])
        harms += bool(baseline['safe_completion'] and not first['safe_completion'])
        prevented += bool(first['safe'] and not baseline['safe'])
        introduced += bool(baseline['safe'] and not first['safe'])

    values = {
        Metric.COMPLETE_PAIRS: len(pairs),
        Metric.INCOMPLETE_PAIRS: len(keys) - len(pairs),
        Metric.UNSTARTED_PAIRS: unstarted,
        Metric.COMPLETION_BENEFITS: benefits if pairs else None,
        Metric.COMPLETION_HARMS: harms if pairs else None,
        Metric.UNSAFE_PREVENTED: prevented if pairs else None,
        Metric.UNSAFE_INTRODUCED: introduced if pairs else None,
        Metric.NET_COMPLETION_BENEFIT: benefits - harms if pairs else None,
        Metric.NET_UNSAFE_REDUCTION: prevented - introduced if pairs else None,
    }
    return {metric.value: value for metric, value in values.items()}


def summarize(path, protocol=None):
    """Compute aggregate and per-delay results from saved PyBullet flights."""
    path = Path(path)
    if path.is_dir():
        path /= 'executions.jsonl'
    if not path.is_file():
        raise FileNotFoundError(f'Execution journal does not exist: {path}')
    if protocol is None:
        run = path.with_name('run.json')
        protocol = json.loads(run.read_text())['protocol'] if run.exists() else {}
    control_hz = protocol.get('control_hz', CONTROL_HZ)
    if not isinstance(control_hz, (int, float)) or not math.isfinite(control_hz) or control_hz <= 0:
        raise ValueError('Invalid protocol control frequency')
    rows = read_executions(path)
    policies = protocol.get('policies') or [p for p in POLICIES if any(r['event']['policy'] == p for r in rows)]
    delays = protocol.get('delays') or sorted({row['delay_s'] for row in rows})
    seeds = {s['seed'] for s in protocol.get('scenarios', [])}
    expected = {(seed, delay) for seed in seeds for delay in delays} if seeds else None
    if any(row['event']['policy'] not in policies or row['delay_s'] not in delays
           or (seeds and row['seed'] not in seeds) for row in rows):
        raise ValueError('Journal contains a flight outside the fixed protocol')

    if 'staleguard' in policies:
        comparison_reference = 'staleguard'
        comparison_other = ('use_time_recheck' if 'use_time_recheck' in policies
                            else 'staleguard_early_recovery'
                            if 'staleguard_early_recovery' in policies
                            else 'fixed_early_recovery'
                            if 'fixed_early_recovery' in policies else None)
    elif 'staleguard_early_recovery' in policies and 'fixed_early_recovery' in policies:
        comparison_reference = 'staleguard_early_recovery'
        comparison_other = 'fixed_early_recovery'
    elif 'staleguard_early_recovery' in policies and 'ppr' in policies:
        comparison_reference = 'staleguard_early_recovery'
        comparison_other = 'ppr'
    else:
        comparison_reference, comparison_other = None, None

    def comparison(selected, expected_keys):
        if comparison_reference is not None and comparison_other is not None:
            return paired_metrics(selected, expected_keys,
                                  other=comparison_other, reference=comparison_reference)
        return paired_metrics(selected, expected_keys)

    def pairs_against_staleguard(selected, expected_keys):
        if 'staleguard' not in policies:
            return {}
        return {p: paired_metrics(selected, expected_keys, p)
                for p in policies if p != 'staleguard'}

    def early_vs_fixed(selected, expected_keys):
        if not {'staleguard_early_recovery', 'fixed_early_recovery'} <= set(policies):
            return None
        return paired_metrics(selected, expected_keys,
                              other='fixed_early_recovery',
                              reference='staleguard_early_recovery')

    by_policy = {p: policy_metrics([r for r in rows if r['event']['policy'] == p], p, control_hz)
                 for p in policies}
    by_delay = {}
    for delay in delays:
        selected = [row for row in rows if row['delay_s'] == delay]
        subset = None if expected is None else {key for key in expected if key[1] == delay}
        by_delay[str(float(delay))] = dict(
            by_policy={p: policy_metrics([r for r in selected if r['event']['policy'] == p], p, control_hz)
                       for p in policies},
            paired_comparison=comparison(selected, subset),
            paired_against_staleguard=pairs_against_staleguard(selected, subset),
            paired_early_vs_fixed=early_vs_fixed(selected, subset),
        )

    return dict(
        schema='geofence_benchmark_metrics_v1',
        recorded_flights=len(rows),
        control_hz=control_hz,
        timing_units='seconds',
        prediction_timing_samples=(
            'StaleGuard and EarlyRecovery: one approval forecast per seed; '
            'Fixed-EarlyRecovery: no prediction computation. '
            'Pre-dispatch recovery lead is recorded separately, not as negative action latency. '
            'Use-Time Recheck: one check per flight. '
            'PPR: one sample per actual periodic check; per-flight total/max computation, '
            'observation-to-decision/recovery delay and timing-bound overruns are separate. '
            'FBS: approval-time feature/EMA update and TTL freeze per flight; '
            'FBS cumulative observation computation is reported only if recorded.'
        ),
        journal_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        by_policy=by_policy,
        by_delay=by_delay,
        paired_comparison_reference_policy=comparison_reference,
        paired_comparison_other_policy=comparison_other,
        paired_comparison=comparison(rows, expected),
        paired_against_staleguard=pairs_against_staleguard(rows, expected),
        paired_early_vs_fixed=early_vs_fixed(rows, expected),
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executions', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    result = summarize(args.executions)
    journal = args.executions / 'executions.jsonl' if args.executions.is_dir() else args.executions
    target = args.out or journal.with_name('metrics.json')
    target.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Wrote {target}: {result["recorded_flights"]} recorded flights.')
