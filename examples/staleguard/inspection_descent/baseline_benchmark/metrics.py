"""Descriptive mission outcomes and paired comparisons, from saved flights only."""
from pathlib import Path
import sys
if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import inspection_descent
import argparse
import json
import math
import numpy as np
from geofence_turn.baseline_benchmark.metrics import (
    read_executions, paired_metrics, milliseconds)

POLICIES=('no_recovery','always_recover','fixed_ttl','fbs','use_time_recheck','staleguard',
          'staleguard_early_recovery','fixed_early_recovery','ppr')


def timing_summary(values):
    """Keep Inspection-Descent's original NumPy-interpolated percentiles."""
    values=[v for v in values if v is not None]
    if any(not math.isfinite(v) or v<0 for v in values):
        raise ValueError('Invalid computation/latency measurement')
    return dict(samples=len(values),mean_s=float(np.mean(values)) if values else None,
        median_s=float(np.median(values)) if values else None,
        p95_s=float(np.percentile(values,95)) if values else None)


def policy_metrics(rows,policy):
    n=len(rows); events=[r['event'] for r in rows]
    recovery=[r for r in rows if r['event']['fallback_step'] is not None]
    early=[e for e in events if e['fallback_step'] is not None and
           e['fallback_step']<e['scheduled_dispatch_step']]
    for e in early:
        if e['application_step'] is not None:
            raise ValueError('Early recovery failed to cancel the pending descent')
    times={}
    for row in rows:
        value=row['event'].get('computation_s')
        if value is not None:
            key=row['seed'] if policy in ('staleguard','staleguard_early_recovery') else (row['seed'],row['delay_s'])
            if key in times and times[key]!=value:
                raise ValueError('Conflicting frozen forecast computation times')
            times[key]=value
    checks=[c for e in events for c in e.get('ppr_checks',[])]
    computation=timing_summary([c['computation_s'] for c in checks] if policy=='ppr' else times.values())
    action=timing_summary([(e['fallback_step'] if e['fallback_step'] is not None else e['application_step'])/50-
        e['scheduled_dispatch_step']/50 for e in events
        if not (e['fallback_step'] is not None and e['fallback_step']<e['scheduled_dispatch_step'])
        and (e['fallback_step'] is not None or e['application_step'] is not None)])
    success=sum(r['safe_completion'] for r in rows); unsafe=sum(not r['safe'] for r in rows)
    incomplete=sum(r['safe'] and not r['completed'] for r in rows)
    guarded=policy in ('staleguard','staleguard_early_recovery','fbs','use_time_recheck')
    result=dict(flights=n,safe_completion=success,safe_noncompletion=incomplete,unsafe=unsafe,
        safe_completion_rate=success/n if n else None,unsafe_rate=unsafe/n if n else None,
        safe_noncompletion_rate=incomplete/n if n else None,
        recovery_attempts=len(recovery),recovery_successes=sum(r['safe'] for r in recovery),
        recovery_failures=sum(not r['safe'] for r in recovery),
        recovery_attempt_rate=len(recovery)/n if n else None,
        recovery_success_rate=sum(r['safe'] for r in recovery)/len(recovery) if recovery else None,
        prediction_computation_s=computation,prediction_computation_ms=milliseconds(computation),
        dispatch_to_action_s=action,dispatch_to_action_ms=milliseconds(action),
        early_recoveries=len(early),early_recovery_lead_s=timing_summary([
            (e['scheduled_dispatch_step']-e['fallback_step'])/50 for e in early]),
        prediction_ready=sum(e.get('prediction_ready') is True for e in events) if guarded else None,
        prediction_unavailable=sum(e.get('prediction_ready') is False for e in events) if guarded else None,
        prediction_readiness_unrecorded=sum(e.get('prediction_ready') is None for e in events) if guarded else None,
        unverified_executions=sum(e.get('prediction_ready') is False and e['application_step'] is not None
                                 for e in events) if guarded else None)
    if policy=='fbs':
        result['fbs_cumulative_observation_computation_s']=timing_summary([
            math.fsum(e['observation_update_s']) for e in events if 'observation_update_s' in e])
    if policy=='ppr':
        result.update(predictive_checks=len(checks),
            predictive_checks_per_flight=[len(e['ppr_checks']) for e in events],
            prediction_computation_total_s=timing_summary([e['prediction_computation_total_s'] for e in events]),
            prediction_computation_max_s=timing_summary([e['prediction_computation_max_s'] for e in events]),
            prediction_computation_sum_s=math.fsum(c['computation_s'] for c in checks),
            prediction_timing_bound_exceeded=sum(c['timing_bound_exceeded'] for c in checks),
            predictions_discarded_at_dispatch=sum(c['discarded_at_dispatch'] for c in checks),
            prediction_to_decision_s=timing_summary([(c['decision_step']-c['observation_step'])/50
                for c in checks if c['decision_step'] is not None]))
    return result


def summarize(journal,protocol=None):
    journal=Path(journal)
    if protocol is None:
        protocol=json.loads(journal.with_name('run.json').read_text())['protocol']
    rows=read_executions(journal); policies=protocol['policies']; delays=protocol['delays']
    seeds={s['seed'] for s in protocol['scenarios']}; expected={(s,d) for s in seeds for d in delays}
    if any((r['seed'],r['delay_s']) not in expected or r['event']['policy'] not in policies for r in rows):
        raise ValueError('Flight outside frozen protocol')
    reference='staleguard' if 'staleguard' in policies else 'staleguard_early_recovery' if 'staleguard_early_recovery' in policies else policies[0]
    def group(selected,keys):
        return dict(by_policy={p:policy_metrics([r for r in selected if r['event']['policy']==p],p) for p in policies},
            paired_comparisons={p:paired_metrics(selected,keys,other=p,reference=reference) for p in policies if p!=reference})
    result=group(rows,expected)
    result.update(schema='inspection_benchmark_metrics_v1',recorded_flights=len(rows),
        selected_scenarios=len(seeds),calibration_statistics=protocol['calibration_statistics'],
        expected_flights=len(expected)*len(policies),comparison_reference_policy=reference,
        by_delay={str(d):group([r for r in rows if r['delay_s']==d],{k for k in expected if k[1]==d}) for d in delays},
        interpretation='Descriptive rates; repeated delays are paired within scenarios, not independent samples.',
        timing_note='StaleGuard: one approval forecast/seed; PPR: one sample/check, separate flight totals. FBS: approval update/freeze only; history sum is separate.')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executions',type=Path)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    from inspection_descent.storage import write_json
    write_json(args.out or args.executions.with_name('metrics.json'),summarize(args.executions))


if __name__=='__main__':
    main()
