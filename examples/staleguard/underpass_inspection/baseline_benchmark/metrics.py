"""Actual paired-flight summaries, predictor diagnostics, CSVs and short report."""
from pathlib import Path
from collections import Counter, defaultdict
import csv
import json
import math
import statistics
from ..storage import read_executions, flight_key, write_json
from ..baselines.policies import POLICIES,APPROVAL_POLICIES
from ..scenario import HZ


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
        'complete_pairs': len(pairs),
        'incomplete_pairs': len(keys) - len(pairs),
        'unstarted_pairs': unstarted,
        'completion_benefits': benefits if pairs else None,
        'completion_harms': harms if pairs else None,
        'unsafe_prevented': prevented if pairs else None,
        'unsafe_introduced': introduced if pairs else None,
        'net_completion_benefit': benefits - harms if pairs else None,
        'net_unsafe_reduction': prevented - introduced if pairs else None,
    }
    return values


def prediction_availability_report(m):
    lines=['Prediction availability means the approval-time forecast has finished by the scheduled dispatch '
        'time (including equality). For EarlyRecovery, the denominator also includes commands already canceled '
        'by an earlier recovery.','',
        '| Policy | Available by scheduled dispatch | Availability rate |',
        '|---|---:|---:|']
    names={'staleguard':'StaleGuard Dispatch','staleguard_early_recovery':'StaleGuard EarlyRecovery'}
    for policy,name in names.items():
        if policy not in m['by_policy']:continue
        values=m['by_policy'][policy];total=values['flights']
        available=values['approval_prediction_available_before_dispatch']
        rate=f'{available/total:.1%}' if total else 'undefined'
        lines.append(f'| {name} | {available}/{total} cases | {rate} |')
    prediction=m['predictor']
    ready=prediction['prediction_ready_by_predicted_expiry']
    late=prediction['prediction_late_for_predicted_expiry'];finite=ready+late
    rate=f'{ready/finite:.1%}' if finite else 'undefined'
    lines+=['','Both variants share the same forecasts and computation times, so their availability rates '
        'are identical for the same scenario-delay cases. For proactive recovery, a second measure is readiness '
        f'by predicted command expiration: **{ready}/{finite} scenario forecasts ({rate})** finish by that time; '
        f'{late}/{finite} finish later. '
        'Scenarios with no predicted expiration within the prediction grid are excluded only from this second '
        f'denominator ({prediction["unobserved_prediction_boundaries"]} here). These rates use measured '
        'warmed computation times replayed on the simulation clock; they do not describe cold-start availability. '
        'Availability alone does not establish prediction correctness or safe recovery.']
    unavailable=[]
    for policy,name in names.items():
        if policy not in m['by_policy']:continue
        values=m['by_policy'][policy]
        missing=values['flights']-values['approval_prediction_available_before_dispatch']
        unavailable.append(f'{name}: {missing} cases without a forecast by dispatch, '
                           f'{values["unavailable_unsafe_dispatches"]} unsafe dispatches')
    if unavailable:
        lines+=['','An unavailable forecast permits dispatch under the established policy rule. '
                +'; '.join(unavailable)+'. These counts overlap across variants and must not be added together.']
    return lines


def policy_metrics(rows,policy,truth):
    recovery=[r for r in rows if r['event']['fallback_step'] is not None]
    dispatched=[r for r in rows if r['event']['application_step'] is not None]
    compared=[r for r in rows if (r['seed'],'dive',r['delay_s']) in truth]
    false_safe=[r for r in compared if r['event']['application_step'] is not None
                and r['event'].get('prediction_ready') is True
                and not truth[(r['seed'],'dive',r['delay_s'])]['safe']]
    false_reject=[r for r in compared if r['event']['command_rejected']
                  and truth[(r['seed'],'dive',r['delay_s'])]['safe']]
    guarded=policy in APPROVAL_POLICIES or policy in ('fbs','use_time_recheck')
    durations={r['seed'] if policy in APPROVAL_POLICIES else flight_key(r):r['event']['computation_s']
               for r in rows if 'computation_s' in r['event']}
    return dict(flights=len(rows),safe=sum(r['safe'] for r in rows),unsafe=sum(not r['safe'] for r in rows),
        safe_completion=sum(r['safe_completion'] for r in rows),
        unsafe_inspection_completions=sum(r['completed'] and not r['safe'] for r in rows),
        safe_noncompletion=sum(r['safe'] and not r['completed'] for r in rows),
        abandoned_inspections=sum(r['event']['command_canceled'] for r in rows),
        recovery_without_inspection=sum(not r['completed'] for r in recovery),
        commands_dispatched=len(dispatched),commands_rejected=sum(r['event']['command_rejected'] for r in rows),
        recovery_attempts=len(recovery),recovery_failures=sum(not r['safe'] for r in recovery),
        early_recoveries=sum(r['event']['early_recovery'] for r in rows),
        recoveries_exactly_at_dispatch=sum(r['event']['recovery_at_dispatch'] for r in rows),
        dispatch_triggered_recoveries=sum(r['event'].get('recovery_reason') in ('dispatch','recheck') for r in recovery),
        recoveries_after_violation=sum(r['violation_before_intervention'] for r in recovery),
        commands_still_pending_at_horizon=sum(r['event']['application_step'] is None and r['event']['fallback_step'] is None for r in rows),
        prediction_ready_at_dispatch=sum(r['event'].get('prediction_ready_at_dispatch') is True for r in rows) if guarded else None,
        approval_prediction_available_before_dispatch=sum(r['event'].get('prediction_available_before_dispatch') is True for r in rows) if policy in APPROVAL_POLICIES else None,
        unavailable_unsafe_dispatches=sum(r['event'].get('prediction_ready') is False and not r['safe'] for r in dispatched) if guarded else None,
        false_safe_decisions=len(false_safe) if guarded else None,
        false_reject_decisions=len(false_reject) if guarded else None,
        ready_accepts_with_unsafe_actual_outcome=sum(r['event'].get('prediction_ready') is True and not r['safe'] for r in dispatched) if guarded else None,
        counterfactual_comparisons=len(compared),
        prediction_computation_s=timing_summary(durations.values()),
        fbs_cumulative_observation_s=timing_summary([sum(r['event']['observation_update_s']) for r in rows]) if policy=='fbs' else None,
        first_violation_parts=dict(Counter(r['first_violation']['part'] for r in rows if r['first_violation'])),
        minimum_body_clearance_m=min((r['min_clearance_m'] for r in rows),default=None))


def predictor_metrics(run,truth):
    per_seed=[]
    for seed,forecast in sorted(run['predictions'].items(),key=lambda item:int(item[0])):
        seed=int(seed);expiry=forecast['expiry_steps']
        actual=sorted((r for (s,m,d),r in truth.items() if s==seed and m=='dive'),key=lambda r:r['start_s'])
        first=next((r['start_s'] for r in actual if not r['safe']),None)
        predicted=None if expiry is None else expiry/HZ
        comparisons=[(d,expiry is None or round(d*HZ)<expiry,truth[(seed,'dive',d)]['safe'])
                     for d in run['protocol']['delays'] if (seed,'dive',d) in truth]
        direct={r['delay_s']:r['safe'] for r in forecast['results']}
        side=sorted((r for (s,m,d),r in truth.items() if s==seed and m=='sideways'),key=lambda r:r['start_s'])
        last=max((r['start_s'] for r in side if r['safe']),default=None)
        per_seed.append(dict(seed=seed,predicted_ttl_s=predicted,true_first_unsafe_s=first,
            ttl_error_s=predicted-first if predicted is not None and first is not None else None,
            prediction_right_censored=expiry is None,truth_right_censored=first is None,
            predicted_non_monotonic=forecast['non_monotonic'],computation_s=forecast['computation_s'],
            false_safe_forecast_labels=sum(p and not t for d,p,t in comparisons),
            false_reject_forecast_labels=sum(not p and t for d,p,t in comparisons),
            direct_grid_false_safe=sum(d in direct and direct[d] and not t for d,p,t in comparisons),
            direct_grid_false_reject=sum(d in direct and not direct[d] and t for d,p,t in comparisons),
            delay_comparisons=[dict(delay_s=d,ground_truth_command_safe=t,
                ttl_admission_safe=p,direct_model_safe=direct.get(d)) for d,p,t in comparisons],
            compared_delays=len(comparisons),
            prediction_available_at_predicted_expiry=None if expiry is None else forecast['computation_s']<=predicted,
            last_tested_safe_side_s=last,side_expiration_observed=any(not r['safe'] for r in side),
            prediction_available_by_last_tested_safe_side=None if last is None else forecast['computation_s']<=last))
    errors=[r['ttl_error_s'] for r in per_seed if r['ttl_error_s'] is not None]
    times=[r['computation_s'] for r in per_seed]
    return dict(samples=len(per_seed),timing=timing_summary(times),
        first_forecast_s=times[0] if times else None,
        subsequent_forecasts=timing_summary(times[1:]),
        comparable_finite_ttl_errors=len(errors),
        mean_signed_ttl_error_s=statistics.mean(errors) if errors else None,
        median_absolute_ttl_error_s=statistics.median(map(abs,errors)) if errors else None,
        maximum_absolute_ttl_error_s=max(map(abs,errors),default=None),
        forecast_false_safe_labels=sum(r['false_safe_forecast_labels'] for r in per_seed),
        forecast_false_reject_labels=sum(r['false_reject_forecast_labels'] for r in per_seed),
        direct_grid_false_safe=sum(r['direct_grid_false_safe'] for r in per_seed),
        direct_grid_false_reject=sum(r['direct_grid_false_reject'] for r in per_seed),
        prediction_ready_by_predicted_expiry=sum(r['prediction_available_at_predicted_expiry'] is True for r in per_seed),
        prediction_late_for_predicted_expiry=sum(r['prediction_available_at_predicted_expiry'] is False for r in per_seed),
        prediction_ready_by_observed_side_deadline=sum(r['side_expiration_observed'] and r['prediction_available_by_last_tested_safe_side'] is True for r in per_seed),
        unobserved_truth_boundaries=sum(r['truth_right_censored'] for r in per_seed),
        unobserved_prediction_boundaries=sum(r['prediction_right_censored'] for r in per_seed),
        by_scenario=per_seed)


def summarize(rows,run,truth):
    protocol=run['protocol'];policies=protocol['policies']
    expected={(s['seed'],d,p) for s in protocol['scenarios'] for d in protocol['delays'] for p in policies}
    keys={flight_key(r) for r in rows}
    if len(keys)!=len(rows) or not keys<=expected:raise ValueError('Unexpected or duplicate flight')
    expected_pairs={(s['seed'],d) for s in protocol['scenarios'] for d in protocol['delays']}
    def policy_group(selected):
        return {p:policy_metrics([r for r in selected if r['event']['policy']==p],p,truth) for p in policies}
    def against_no_recovery(selected,pairs):
        if 'no_recovery' not in policies:return {}
        result={p:paired_metrics(selected,pairs,other='no_recovery',reference=p) for p in policies if p!='no_recovery'}
        blocks={flight_key(r):r for r in selected}
        for p,data in result.items():
            data['safe_completions_preserved']=sum(blocks[(s,d,p)]['safe_completion'] and blocks[(s,d,'no_recovery')]['safe_completion']
                for s,d in pairs if (s,d,p) in blocks and (s,d,'no_recovery') in blocks)
        return result
    by_scenario={}
    for s in protocol['scenarios']:
        selected=[r for r in rows if r['seed']==s['seed']]
        by_scenario[str(s['seed'])]=dict(by_policy=policy_group(selected),
            paired_against_no_recovery=against_no_recovery(selected,{p for p in expected_pairs if p[0]==s['seed']}))
    early={}
    for p in ('staleguard','use_time_recheck','fixed_early_recovery'):
        if p in policies and 'staleguard_early_recovery' in policies:
            early[p]=paired_metrics(rows,expected_pairs,other=p,reference='staleguard_early_recovery')
    return dict(status='complete' if keys==expected else 'partial',recorded_flights=len(rows),expected_flights=len(expected),
        independent_scenario_count=len(protocol['scenarios']),
        denominator_note='Scenario is the independent unit. Delay/policy cells are repeated deterministic flights, not independent sampled scenarios.',
        false_decision_definition='False-safe: ready check/forecast permits a command whose original scheduled-delay No-Recovery truth is unsafe. False-reject: rejected/canceled command whose scheduled-delay No-Recovery truth is safe. Unavailable unsafe dispatches and actual unsafe accepted commands are separate.',
        by_policy=policy_group(rows),by_scenario=by_scenario,
        by_delay={str(d):policy_group([r for r in rows if r['delay_s']==d]) for d in protocol['delays']},
        paired_against_no_recovery=against_no_recovery(rows,expected_pairs),paired_early_recovery=early,
        predictor=predictor_metrics(run,truth))


def write_outputs(out,run,rows,truth):
    out=Path(out);m=summarize(rows,run,truth);write_json(out/'metrics.json',m)
    fields=('seed','delay_s','policy','safe','safe_completion','completed','min_clearance_m',
        'command_step','recovery_step','early_recovery','first_violation_s','obstacle','part',
        'computation_s','prediction_ready','prediction_ready_control_step','expiry_steps')
    with (out/'per_delay.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for r in rows:
            e=r['event'];v=r['first_violation'];entry={k:r[k] for k in fields if k in r}
            entry.update({k:e[k] for k in fields if k in e})
            entry.update(policy=e['policy'],command_step=e['application_step'],recovery_step=e['fallback_step'],
                obstacle=v['obstacle'] if v else None,part=v['part'] if v else None)
            writer.writerow(entry)
    fields=('seed','policy','flights','unsafe','safe_completion','recovery_attempts','early_recoveries',
        'false_safe_decisions','false_reject_decisions','unsafe_prevented','unsafe_introduced','safe_completions_lost')
    with (out/'per_scenario.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for seed,group in m['by_scenario'].items():
            for p,values in group['by_policy'].items():
                entry={k:values[k] for k in fields if k in values};entry.update(seed=seed,policy=p)
                pair=group['paired_against_no_recovery'].get(p,{})
                entry.update(unsafe_prevented=pair.get('unsafe_prevented'),unsafe_introduced=pair.get('unsafe_introduced'),
                             safe_completions_lost=pair.get('completion_harms'))
                writer.writerow(entry)
    count=m['independent_scenario_count'];n=m['recorded_flights'];sg=m['by_policy'].get('staleguard',{})
    early=m['by_policy'].get('staleguard_early_recovery',{})
    lines=['# Underpass seven-policy development benchmark','',
        f'Implemented seven policies on {count} frozen development scenarios. The UAV flies North toward a bridge; '
        'its approved command dives underneath to inspect and swerves East around a pier. Every recovery instead '
        'abandons inspection and uses the same independent Eastward sideways maneuver. '
        f'This journal contains {n}/{m["expected_flights"]} actual PyBullet executions. '
        f'StaleGuard has {sg.get("unsafe",0)} unsafe flights and its EarlyRecovery variant has '
        f'{early.get("unsafe",0)}. Prediction readiness, false-safe decisions and abandoned inspections are reported '
        f'separately. This {count}-scenario development cohort does not establish held-out or real-world performance; '
        'inspect the timing/model failures before freezing a later evaluation.','',
        'All policies use the same physical scenario, PID and eight-second horizon after approval. Safety is monitored '
        'at 1,000 Hz from reset, including before intervention and after a violation. Unsafe inspection dwell is never '
        'counted as safe completion. Dispatch delays are unknown to the policy until the event occurs. '
        'No offline deadline or other policy outcome is supplied to a policy.','',
        '**PARTIAL: pending cells are not outcomes.**' if m['status']!='complete' else '**Complete paired matrix.**','',
        '| Policy | Flights | Unsafe | Safe inspection | Recovery | Early recovery | Ready false-safe decisions |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for p,v in m['by_policy'].items():
        lines.append(f'| {p} | {v["flights"]} | {v["unsafe"]} | {v["safe_completion"]} | '
            f'{v["recovery_attempts"]} | {v["early_recoveries"]} | {v["false_safe_decisions"]} |')
    lines+=['','Early means recovery starts strictly before the scheduled dispatch, including Always Recover at '
        'approval for positive delays. At D=0 Always Recover still cancels immediately. Fixed EarlyRecovery uses '
        '0.20 s with >= equality. Climb is never selected. A rejected command can still lead to an unsafe flight '
        'when sideways recovery starts too late.','',
        '| Policy versus No Recovery | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |',
        '|---|---:|---:|---:|---:|']
    for p,v in m['paired_against_no_recovery'].items():
        lines.append(f'| {p} | {v["unsafe_prevented"]} | {v["unsafe_introduced"]} | '
                     f'{v["safe_completions_preserved"]} | {v["completion_harms"]} |')
    lines+=['']+prediction_availability_report(m)
    prediction=m['predictor'];timing=prediction['timing']
    lines+=['','The StaleGuard variants share one measured approval forecast per scenario and the same runtime. '
        'The shared 100 Hz lightweight model preserves PID memory, predicts nominal waiting and then anchors the '
        'same dive to each predicted dispatch state. A swept 3D Euclidean box/body checker applies the unchanged '
        'clearance threshold. The fixed prediction grid follows Geofence (19 candidate delays); actual dispatch '
        'uses 51 delays. Expiration is the first predicted unsafe grid point; an all-safe grid has unobserved expiration.','',
        f'Approval forecast runtime: {timing["samples"]} scenario samples, '
        f'median {timing["median_s"]} s, p95 {timing["p95_s"]} s. '
        f'Finite comparable TTL errors: {prediction["comparable_finite_ttl_errors"]}; '
        f'median absolute error {prediction["median_absolute_ttl_error_s"]} s. '
        'Unobserved boundaries are excluded from scalar TTL errors, not from flight outcomes. '
        'First-forecast and subsequent timing, startup warmup and per-scenario forecasts are stored separately.','',
        f'The TTL admission rule has {prediction["forecast_false_safe_labels"]} false-safe and '
        f'{prediction["forecast_false_reject_labels"]} false-reject scenario/delay labels. '
        f'Direct model labels at sampled prediction-grid points have {prediction["direct_grid_false_safe"]} '
        f'false-safe and {prediction["direct_grid_false_reject"]} false-reject labels. '
        'The distinction exposes errors between coarse prediction-grid points as well as dynamic-model mismatch. '
        f'{prediction["prediction_late_for_predicted_expiry"]} forecasts finish after their predicted expiry; '
        'unavailable dispatches remain separate from ready false-safe decisions.','',
        'Computation is measured offline, then its duration is replayed on the physical control clock, matching '
        'Geofence. Nominal flight continues until the result becomes actionable. An unavailable approval forecast '
        'permits dispatch and cannot cancel an already started command later. EarlyRecovery waits for both readiness '
        'and expiration; a late result acts at its actual ready control step while still pending, never retroactively. '
        'Use-Time Recheck performs one dispatch-state rollout and acts strictly after measured computation on the '
        'control grid; that movement during checking can change its eventual outcome.','',
        'Seeds with negative physical recovery gaps and unobserved command expiration remain in the cohort. '
        'Scenario is the independent experimental unit; aggregate flight counts are repeated-case descriptions. '
        'Per-scenario results, all cell outcomes and paired EarlyRecovery comparisons are in the CSVs and metrics.json. '
        'The [technical appendix](TECHNICAL_APPENDIX.md) documents policy rules, source equivalence, tests, '
        'timing limitations and reproduction. No held-out scenarios were generated or evaluated.']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return m
