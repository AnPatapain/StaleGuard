"""Actual paired-flight summaries, predictor diagnostics, CSVs and short report."""
from pathlib import Path
from collections import Counter, defaultdict
from itertools import groupby
import csv
import json
import math
import statistics
import numpy as np
from ..storage import flight_key, write_json, iter_committed
from ..baselines.policies import POLICIES,APPROVAL_POLICIES,HORIZON_POLICIES
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
    names={'staleguard':'StaleGuard Dispatch','staleguard_early_recovery':'StaleGuard EarlyRecovery',
        'staleguard_recovery_aware':'StaleGuard RecoveryAware','staleguard_dual_horizon':'StaleGuard DualHorizon'}
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
    lines+=['','The original StaleGuard pair shares forecasts/runtime. Each new policy has an independently timed '
        'run of the same three-profile predictor; no cross-policy computation cache advances readiness. '
        'For the root predictor summary (scope_policy in metrics.json), a second measure is readiness '
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
    result=dict(flights=len(rows),safe=sum(r['safe'] for r in rows),unsafe=sum(not r['safe'] for r in rows),
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
    if any('inspection_observation' in r for r in rows):
        result.update(inspections_not_observed_by_deadline=sum(r.get('inspection_observation')=='not_observed_by_fixed_deadline' for r in rows),
            maneuver_profiles_truncated_by_horizon=sum(r.get('maneuver_profile_truncated_by_horizon',False) for r in rows))
    return result


def predictor_metrics(run,truth):
    per_seed=[];indexed=defaultdict(list)
    for (seed,mode,delay),row in truth.items():indexed[seed,mode].append(row)
    for seed,forecast in sorted(run['predictions'].items(),key=lambda item:int(item[0])):
        seed=int(seed);expiry=forecast['expiry_steps']
        actual=sorted(indexed[seed,'dive'],key=lambda r:r['start_s'])
        first=next((r['start_s'] for r in actual if not r['safe']),None)
        predicted=None if expiry is None else expiry/HZ
        comparisons=[(d,expiry is None or round(d*HZ)<expiry,truth[(seed,'dive',d)]['safe'])
                     for d in run['protocol']['delays'] if (seed,'dive',d) in truth]
        direct={r['delay_s']:r['safe'] for r in forecast['results']}
        side=sorted(indexed[seed,'sideways'],key=lambda r:r['start_s'])
        last=max((r['start_s'] for r in side if r['safe']),default=None)
        complete=all((seed,'dive',d) in truth for d in run['protocol']['delays'])
        per_seed.append(dict(seed=seed,predicted_ttl_s=predicted,true_first_unsafe_s=first,
            ttl_error_s=predicted-first if complete and predicted is not None and first is not None else None,
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
    if run['protocol'].get('ground_truth_source')=='no_recovery':
        for row in per_seed:
            actual=sorted(indexed[row['seed'],'dive'],key=lambda r:r['start_s'])
            complete=len(actual)==len(run['protocol']['delays'])
            intervals=[]
            for flight in actual:
                if not intervals or intervals[-1]['safe']!=flight['safe']:
                    intervals.append(dict(first_tested_s=flight['start_s'],last_tested_s=flight['start_s'],safe=flight['safe']))
                else:intervals[-1]['last_tested_s']=flight['start_s']
            row.update(observations_complete=complete,
                truth_right_censored=complete and row['true_first_unsafe_s'] is None,
                last_tested_safe_s=max((r['start_s'] for r in actual if r['safe']),default=None),
                last_tested_safe_completable_s=max((r['start_s'] for r in actual if r['safe_completion']),default=None),
                safety_intervals=intervals,non_monotonic=any(not a['safe'] and b['safe'] for a,b in zip(actual,actual[1:])),
                recovery_deadline_measured=False)
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


def command_truth(rows):
    return {(r['seed'],'dive',r['delay_s']):dict(start_s=r['delay_s'],safe=r['safe'],
        safe_completion=r['safe_completion']) for r in rows if r['event']['policy']=='no_recovery'}


def extension_pairs(rows,pairs,policies):
    comparisons=[]
    for policy in HORIZON_POLICIES:
        if policy not in policies:continue
        for other in ('staleguard_early_recovery','fixed_early_recovery','use_time_recheck'):
            if other in policies:comparisons.append((policy,other))
    if set(HORIZON_POLICIES)<=set(policies):comparisons.append((HORIZON_POLICIES[1],HORIZON_POLICIES[0]))
    return {p+'__vs__'+other:paired_metrics(rows,pairs,reference=p,other=other) for p,other in comparisons}


def summarize(rows,run,truth=None):
    if truth is None:truth=command_truth(rows)
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
    result=dict(status='complete' if keys==expected else 'partial',recorded_flights=len(rows),expected_flights=len(expected),
        independent_scenario_count=len(protocol['scenarios']),
        denominator_note='Scenario is the independent unit. Delay/policy cells are repeated deterministic flights, not independent sampled scenarios.',
        false_decision_definition='False-safe: ready check/forecast permits a command whose original scheduled-delay No-Recovery truth is unsafe. False-reject: rejected/canceled command whose scheduled-delay No-Recovery truth is safe. Unavailable unsafe dispatches and actual unsafe accepted commands are separate.',
        by_policy=policy_group(rows),by_scenario=by_scenario,
        by_delay={str(d):policy_group([r for r in rows if r['delay_s']==d]) for d in protocol['delays']},
        paired_against_no_recovery=against_no_recovery(rows,expected_pairs),paired_early_recovery=early,
        predictor=predictor_metrics(run,truth))
    if set(policies)&set(HORIZON_POLICIES):
        result['paired_extensions']=extension_pairs(rows,expected_pairs,policies)
        for seed,group in by_scenario.items():
            selected=[r for r in rows if r['seed']==int(seed)]
            group['paired_extensions']=extension_pairs(selected,{x for x in expected_pairs if x[0]==int(seed)},policies)
    return result


def summarize_journal(out,run,truth=None,delay_subset=None):
    """Use the existing metrics one scenario at a time, keeping RAM bounded."""
    original_run=run
    if delay_subset is not None:
        run=dict(run,protocol=dict(run['protocol'],delays=list(delay_subset)))
    protocol=run['protocol'];policies=protocol['policies'];delays=protocol['delays'];selected_delays=set(delays)
    groups=iter(groupby(iter_committed(out,run),key=lambda r:r['seed']))
    current=next(groups,None);by_scenario={};by_policy={p:{} for p in policies}
    by_delay={str(d):{p:{} for p in policies} for d in delays}
    timings={p:{} for p in policies};fbs_times={p:[] for p in policies}
    delay_times={str(d):{p:{} for p in policies} for d in delays}
    delay_fbs={str(d):{p:[] for p in policies} for d in delays}
    pairs={};early={};extended={};labels={} if truth is None else truth;recorded=0

    def merge(target,value):
        for key,item in value.items():
            if key in ('prediction_computation_s','fbs_cumulative_observation_s'):continue
            if key=='first_violation_parts':
                target[key]=dict(Counter(target.get(key,{}))+Counter(item))
            elif key=='minimum_body_clearance_m':
                old=target.get(key);target[key]=item if old is None else old if item is None else min(old,item)
            elif item is None:target.setdefault(key,None)
            else:target[key]=(target.get(key) or 0)+item

    def collect(rows,clock,cumulative):
        for row in rows:
            p=row['event']['policy'];event=row['event']
            if 'computation_s' in event:
                key=row['seed'] if p in APPROVAL_POLICIES else flight_key(row)
                if key in clock[p] and clock[p][key]!=event['computation_s']:
                    raise ValueError('Conflicting shared forecast timing')
                clock[p][key]=event['computation_s']
            if p=='fbs':cumulative[p].append(sum(event['observation_update_s']))

    for scenario in protocol['scenarios']:
        seed=scenario['seed'];rows=[]
        if current is not None and current[0]==seed:
            rows=list(current[1]);current=next(groups,None)
        local_truth=command_truth(rows) if truth is None else truth
        if truth is None:labels.update(local_truth)
        rows=[r for r in rows if r['delay_s'] in selected_delays]
        local_run=dict(run,protocol=dict(protocol,scenarios=[scenario]),
            predictions={str(seed):run['predictions'][str(seed)]} if str(seed) in run['predictions'] else {})
        summary=summarize(rows,local_run,local_truth)
        if protocol.get('analysis_subsets'):
            summary['by_scenario'][str(seed)]['paired_early_recovery']=summary['paired_early_recovery']
        by_scenario.update(summary['by_scenario']);recorded+=len(rows)
        for p,value in summary['by_policy'].items():merge(by_policy[p],value)
        for delay,values in summary['by_delay'].items():
            for p,value in values.items():merge(by_delay[delay][p],value)
        for p,value in summary['paired_against_no_recovery'].items():merge(pairs.setdefault(p,{}),value)
        for p,value in summary['paired_early_recovery'].items():merge(early.setdefault(p,{}),value)
        for p,value in summary.get('paired_extensions',{}).items():merge(extended.setdefault(p,{}),value)
        collect(rows,timings,fbs_times)
        for delay,selected in groupby(rows,key=lambda r:str(r['delay_s'])):
            collect(list(selected),delay_times[delay],delay_fbs[delay])
    if current is not None:raise ValueError('Journal scenario order differs from frozen protocol')
    for values,clock,cumulative in [(by_policy,timings,fbs_times)]+[
            (by_delay[str(d)],delay_times[str(d)],delay_fbs[str(d)]) for d in delays]:
        for p,value in values.items():
            value['prediction_computation_s']=timing_summary(clock[p].values())
            value['fbs_cumulative_observation_s']=timing_summary(cumulative[p]) if p=='fbs' else None
    expected=len(protocol['scenarios'])*len(delays)*len(policies)
    result=dict(status='complete' if recorded==expected else 'partial',recorded_flights=recorded,
        expected_flights=expected,independent_scenario_count=len(protocol['scenarios']),
        denominator_note=summary['denominator_note'],false_decision_definition=summary['false_decision_definition'],
        by_policy=by_policy,by_scenario=by_scenario,by_delay=by_delay,
        paired_against_no_recovery=pairs,paired_early_recovery=early,predictor=predictor_metrics(original_run,labels))
    if set(policies)&set(HORIZON_POLICIES):result['paired_extensions']=extended
    if protocol.get('analysis_subsets'):
        result['uncertainty']=scenario_uncertainty(result,protocol)
    if protocol.get('ground_truth_source')=='no_recovery':
        result['ground_truth_source']='Actual paired No Recovery flights; no independent recovery deadline sweep'
        result['scenario_rates']={}
        for p in policies:
            complete=[g['by_policy'][p] for g in by_scenario.values() if g['by_policy'][p]['flights']==len(delays)]
            result['scenario_rates'][p]={'complete_scenarios':len(complete)}
            for field in ('unsafe','safe_completion'):
                rates=[v[field]/len(delays) for v in complete]
                result['scenario_rates'][p][field]=dict(mean=statistics.mean(rates) if rates else None,
                    median=statistics.median(rates) if rates else None,min=min(rates,default=None),max=max(rates,default=None))
    return result


def scenario_uncertainty(result,protocol):
    """Resample whole scenarios, preserving all within-scenario delay/policy pairs."""
    policies=protocol['policies'];n_delays=len(protocol['delays'])
    groups=[g for g in result['by_scenario'].values()
            if all(g['by_policy'][p]['flights']==n_delays for p in policies)]
    names=[];columns=[]
    for p in policies:
        for field in ('unsafe','safe_completion','recovery_attempts'):
            names.append(('by_policy',p,field))
            columns.append([g['by_policy'][p][field]/n_delays for g in groups])
    for category in ('paired_against_no_recovery','paired_early_recovery')+ (('paired_extensions',) if 'paired_extensions' in result else ()):
        comparisons=result[category]
        for p in comparisons:
            for field in ('net_unsafe_reduction','net_completion_benefit'):
                names.append((category,p,field))
                columns.append([g[category][p][field]/n_delays for g in groups])
    settings=protocol['uncertainty'];count=len(groups)
    output=dict(settings,complete_scenarios=count,by_policy={},paired_against_no_recovery={},paired_early_recovery={})
    if 'paired_extensions' in result:output['paired_extensions']={}
    means=np.asarray(columns,dtype=float).T if count else np.empty((0,len(names)))
    intervals=None
    if count>=2:
        rng=np.random.default_rng(settings['seed']);boot=np.empty((settings['resamples'],len(names)))
        for start in range(0,len(boot),32):
            selected=rng.integers(0,count,size=(min(32,len(boot)-start),count))
            boot[start:start+len(selected)]=means[selected].mean(axis=1)
        tail=(1-settings['confidence'])/2
        intervals=np.quantile(boot,[tail,1-tail],axis=0)
    for i,(category,p,field) in enumerate(names):
        output[category].setdefault(p,{})[field]=dict(mean=float(means[:,i].mean()) if count else None,
            ci95=None if intervals is None else intervals[:,i].tolist())
    return output


def quality(rows,run,truth):
    result={};forecasts=run.get('predictions_by_policy',{})
    for policy,cache in forecasts.items():
        scores=dict(command_false_safe=0,command_false_reject=0,completion_false_safe=0,completion_false_reject=0,
                    command_completion_label_pairs=0,recovery_sampled_pairs=0,recovery_false_safe=0,recovery_false_reject=0,
                    completion_differs_from_safety=0)
        for seed,forecast in cache.items():
            for item in forecast['results']:
                key=int(seed),'dive',item['delay_s']
                scores['completion_differs_from_safety']+=item['command_complete']!=item['command_safe']
                if key not in truth:continue
                actual=truth[key];scores['command_completion_label_pairs']+=1
                for name,predicted,observed in [('command',item['command_safe'],actual['safe']),('completion',item['command_complete'],actual['safe_completion'])]:
                    scores[name+'_false_safe']+=predicted and not observed
                    scores[name+'_false_reject']+=observed and not predicted
        scores['runtime']=timing_summary(f['computation_s'] for f in cache.values())
        result[policy]=scores
    failures={p:dict(unsafe_accepted_commands=0,unsafe_recovery_attempts=0,
        unsafe_recovery_predicted_verified=0,unsafe_best_effort_recovery=0,
        predictions_late_for_trigger=0) for p in run['protocol']['policies']}
    seen={};seed=None
    for row in rows:
        if seed!=row['seed']:seed=row['seed'];seen={}
        event=row['event'];policy=event['policy'];f=failures[policy]
        f['unsafe_accepted_commands']+=not row['safe'] and event['application_step'] is not None
        f['unsafe_recovery_attempts']+=not row['safe'] and event['fallback_step'] is not None
        f['unsafe_recovery_predicted_verified']+=not row['safe'] and event.get('predicted_recovery_verified_at_start') is True
        f['unsafe_best_effort_recovery']+=not row['safe'] and event.get('best_effort_recovery') is True
        f['predictions_late_for_trigger']+=event.get('prediction_available_by_trigger') is False
        if policy not in forecasts or row['mode']!='sideways':continue
        forecast=forecasts[policy][str(seed)]
        sampled=next((r['recovery_safe'] for r in forecast['results'] if r['delay_s']==row['start_s']),None)
        if sampled is None:continue
        key=policy,row['start_s'];label=sampled,row['safe']
        if key in seen:
            if seen[key]!=label:raise ValueError('Inconsistent repeated physical recovery label')
            continue
        seen[key]=label;scores=result[policy]
        scores['recovery_sampled_pairs']+=1
        scores['recovery_false_safe']+=sampled and not row['safe']
        scores['recovery_false_reject']+=row['safe'] and not sampled
    return result,failures



def journal_metrics(out,run,truth=None):
    root_scope=next((p for p in ('staleguard','staleguard_early_recovery') if p in run['protocol']['policies']),None)
    if not root_scope and run.get('predictions_by_policy'):
        root_scope=next(iter(run['predictions_by_policy']))
    if not run.get('predictions') and run.get('predictions_by_policy'):
        run=dict(run,predictions=run['predictions_by_policy'][root_scope])
    metrics=summarize_journal(out,run,truth)
    if run['protocol'].get('analysis_subsets'):
        metrics['analysis_subsets']={}
        for name,(lower,upper) in run['protocol']['analysis_subsets'].items():
            if name=='full':continue
            delays=[d for d in run['protocol']['delays'] if lower<=d<=upper]
            if not delays:continue
            part=summarize_journal(out,run,truth,delays)
            # Prediction-boundary diagnostics describe the full observed command grid.
            # Subset admission/readiness/paired counts remain in each by_policy block.
            part.pop('predictor')
            part['dispatch_delays_s']=delays
            metrics['analysis_subsets'][name]=part
    if set(run['protocol']['policies'])&set(HORIZON_POLICIES):
        labels=command_truth(iter_committed(out,run)) if truth is None else truth
        ids={s['seed'] for s in run['protocol']['scenarios']};delays=set(run['protocol']['delays'])
        labels={k:v for k,v in labels.items() if k[0] in ids and k[2] in delays}
        metrics['prediction_by_policy'],metrics['failure_decomposition']=quality(iter_committed(out,run),run,labels)
        metrics['predictor']['scope_policy']=root_scope
    return metrics


def write_journal_outputs(out,run,truth=None):
    return write_outputs(out,run,iter_committed(out,run),truth,metrics=journal_metrics(out,run,truth))


def write_outputs(out,run,rows,truth=None,metrics=None):
    out=Path(out);m=summarize(rows,run,truth) if metrics is None else metrics;write_json(out/'metrics.json',m)
    fields=('seed','delay_s','policy','safe','safe_completion','completed','min_clearance_m',
        'command_step','recovery_step','early_recovery','first_violation_s','obstacle','part',
        'computation_s','prediction_ready','prediction_ready_control_step','expiry_steps')
    if run['protocol'].get('analysis_subsets'):
        fields+=('prediction_grid_region','inspection_observation','maneuver_profile_truncated_by_horizon')
    with (out/'per_delay.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for r in rows:
            e=r['event'];v=r['first_violation'];entry={k:r[k] for k in fields if k in r}
            entry.update({k:e[k] for k in fields if k in e})
            entry.update(policy=e['policy'],command_step=e['application_step'],recovery_step=e['fallback_step'],
                obstacle=v['obstacle'] if v else None,part=v['part'] if v else None)
            if 'prediction_grid_region' in fields:
                entry['prediction_grid_region']='within_prediction_horizon' if r['delay_s']<=1. else 'beyond_prediction_horizon'
            writer.writerow(entry)
    fields=('seed','policy','flights','unsafe','safe_completion','recovery_attempts','early_recoveries',
        'false_safe_decisions','false_reject_decisions','unsafe_prevented','unsafe_introduced','safe_completions_lost')
    if run['protocol'].get('analysis_subsets'):fields+=('analysis_subset',)
    with (out/'per_scenario.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        windows={'full':m,**m.get('analysis_subsets',{})}
        for name,window in windows.items():
            for seed,group in window['by_scenario'].items():
                for p,values in group['by_policy'].items():
                    entry={k:values[k] for k in fields if k in values};entry.update(seed=seed,policy=p)
                    if 'analysis_subset' in fields:entry['analysis_subset']=name
                    pair=group['paired_against_no_recovery'].get(p,{})
                    entry.update(unsafe_prevented=pair.get('unsafe_prevented'),unsafe_introduced=pair.get('unsafe_introduced'),
                                 safe_completions_lost=pair.get('completion_harms'))
                    writer.writerow(entry)
    count=m['independent_scenario_count'];n=m['recorded_flights'];sg=m['by_policy'].get('staleguard',{})
    early=m['by_policy'].get('staleguard_early_recovery',{})
    cohort=run['protocol'].get('dataset_split','development')
    lines=[f'# Underpass {len(run["protocol"]["policies"])}-policy {cohort} benchmark','',
        f'Implemented {len(run["protocol"]["policies"])} policies on {count} frozen {cohort} scenarios. The UAV flies North toward a bridge; '
        'its approved command dives underneath to inspect and swerves East around a pier. Every recovery instead '
        'abandons inspection and uses the same independent Eastward sideways maneuver. '
        f'This journal contains {n}/{m["expected_flights"]} actual PyBullet executions. ' +
        (f'StaleGuard has {sg["unsafe"]} unsafe flights and its EarlyRecovery variant has {early["unsafe"]}. '
         if sg and early else 'The two new policies are physically executed; original policy outcomes are not rerun. ') +
        'Prediction readiness, false-safe decisions and abandoned inspections are reported '
        f'separately. The scenario is the independent experimental unit; '
        'simulation results do not establish real-world safety.','',
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
        'when sideways recovery starts too late.','']
    if m['paired_against_no_recovery']:
        lines+=['| Policy versus No Recovery | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |',
                '|---|---:|---:|---:|---:|']
    else:
        lines+=['No paired No Recovery comparison was collected in this policy subset.','']
    for p,v in m['paired_against_no_recovery'].items():
        lines.append(f'| {p} | {v["unsafe_prevented"]} | {v["unsafe_introduced"]} | '
                     f'{v["safe_completions_preserved"]} | {v["completion_harms"]} |')
    sampling=run['protocol'].get('scenario_sampling')
    if sampling and 'excluded_seeds' in sampling:
        lines+=['',f'**Fresh evaluation:** {sampling["sample_count"]} scenarios sampled without replacement from '
            f'{sampling["eligible_count"]} eligible accepted scenarios, with RNG seed {sampling["rng_seed"]}. '
            f'The {len(sampling["excluded_seeds"])} previously evaluated scenarios are excluded. '
            'Selection used accepted IDs only, without policy outcomes.']
    if m.get('analysis_subsets'):
        if len(run['protocol']['delays'])==13 and run['protocol'].get('protocol_version')!='reduced_100x13_v3':
            lines+=['','**Temporal resolution:** 13 selected dispatch delays; gaps range from 0.04 to 0.40 s. '
                'Tested expiration boundaries are coarse observations. Rates weight the selected delays equally, '
                'not a uniform continuous delay distribution. The 19-point prediction grid is unchanged.']
        if run['protocol'].get('protocol_version')=='reduced_100x13_v3':
            sample=run['protocol'].get('scenario_sampling')
            lines+=['','**Reduced protocol:** '+('100 scenarios uniformly sampled without replacement from 865 accepted, '
                'using RNG seed 20261108. ' if sample else 'Development-only operational smoke. ')+
                'The full reduced grid has 13 delays: 0, 0.04, 0.08, 0.12, 0.16, 0.20, 0.40, 0.60, '
                '0.80, 1.00, 1.20, 1.60, 2.00 s. Gaps range from 0.04 to 0.40 s. Observed expiration '
                'thresholds are first tested unsafe points, not exact deadlines. All ten selected delays '
                'through 1 s are prediction-grid points, so this reduced design does not test between-grid '
                'admissions within 1 s. Rates average the selected points equally; they do not assume a '
                'uniform continuous dispatch-delay distribution.']
        lines+=['','The physical grid covers 0–2.00 s; the predictor still tests only its original 19 delays through '
            '1.00 s. These windows are subsets of one benchmark. With no predicted expiration, the unchanged '
            'StaleGuard gate permits dispatch even beyond 1 s; this is not a safety certificate for those delays. '
            'A ready finite TTL still rejects at/after expiry, and unavailable predictions still permit dispatch.','',
            'The fixed eight-second horizon is unchanged. Missing inspection dwell means incomplete by that '
            'deadline, not proof it could never complete later. Truncated maneuver profiles are recorded separately.']
        windows={'full':m,**m['analysis_subsets']}
        for name,window in windows.items():
            lines+=['',f'Analysis subset: **{name}**','',
                '| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |',
                '|---|---:|---:|---:|---|']
            for p,value in window['by_policy'].items():
                stat=window['uncertainty']['by_policy'][p]['unsafe'];ci=stat['ci95']
                interval='undefined' if ci is None else f'{ci[0]:.3f}–{ci[1]:.3f}'
                mean='undefined' if stat['mean'] is None else f'{stat["mean"]:.3f}'
                lines.append(f'| {p} | {value["flights"]} | {value["unsafe"]} | {value["safe_completion"]} | {mean} [{interval}] |')
        lines+=['','95% percentile bootstrap intervals resample whole scenarios (2,000 resamples, fixed seed '
            '2027). Complete scenario grids contribute; fewer than two complete scenarios gives no interval. '
            'Paired net safety/completion differences use the same scenario resamples and are in metrics.json; '
            'correlated delay trials are never treated as independent scenarios.']
    if run['protocol'].get('exploratory'):
        lines+=['','**Exploratory extension:** these scenarios were examined in the earlier seven-policy study. '
            'New policies were developed/profiler-tested only on development scenarios, but this is not a fresh '
            'held-out superiority test. Existing flights were not rerun. New policy forecasts are independently '
            'computed/timed; all online waiting, both continuations, completion checks and boundary extraction '
            'count toward their atomic readiness.']
    lines+=['']+prediction_availability_report(m)
    prediction=m['predictor'];timing=prediction['timing']
    lines+=['','The original StaleGuard pair shares one measured approval forecast per scenario and runtime. '
        'Each new policy independently computes/times the shared three-profile predictor. '
        'The shared 100 Hz lightweight model preserves PID memory, predicts nominal waiting and then anchors the '
        'same dive to each predicted dispatch state. A swept 3D Euclidean box/body checker applies the unchanged '
        'clearance threshold. The fixed prediction grid follows Geofence (19 candidate delays); actual dispatch '
        f'uses {len(run["protocol"]["delays"])} delays. Expiration is the first predicted unsafe grid point; an all-safe grid has unobserved expiration.','',
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
        'timing limitations and reproduction.']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    appendix=out/'TECHNICAL_APPENDIX.md'
    if not appendix.exists() or run['protocol'].get('analysis_subsets'):
        appendix.write_text('# Underpass frozen benchmark protocol\n\n'
            'Each recorded outcome is an actual PID/PyBullet flight. Safety is monitored at 1,000 Hz over the '
            'whole fixed horizon; earlier violations persist. All recovery is the same Eastward sideways maneuver.\n\n'
            'StaleGuard uses the fixed 19-point grid and first-unsafe cutoff; equality rejects only when ready. '
            'The original two variants share one warmed forecast/runtime per scenario. New variants independently '
            'time the same three-profile predictor; neither consumes the other policy forecast. The early variant can recover while pending. '
            'Unavailable forecasts permit dispatch. Fixed EarlyRecovery uses 0.20 s; FBS uses alpha 0.1, epsilon '
            '1e-12 and a 210-step cap, with strict age > TTL expiration. Use-Time Recheck observes at dispatch, '
            'continues nominal motion while computing and acts at dispatch + floor(runtime*50) + 1.\n\n'
            'No future dispatch time is supplied to policies. Counterfactual command labels come from paired '
            'No Recovery flights when no external truth is configured. Command expiration is the first tested '
            'unsafe delay; all-safe complete grids remain censored. No independent recovery deadline is inferred. '
            'Full intervals expose non-monotonic command safety. Scenario-level rates use complete scenario grids only.\n\n'
            'Each flight/forecast append is fsynced before its offset/hash is atomically committed in progress.json. '
            'Resume checks those prefixes and rolls back only uncommitted tail bytes. Metrics are regenerated one scenario '
            'at a time, without simulation. Source/environment, frozen protocol and cached forecast runtimes '
            'must match on resume.\n\n'
            + 'Frozen configuration SHA-256: '+str(run.get('config_sha256','recorded in progress.json'))+'\n\n'
            + 'Protocol and source/environment information are recorded in the immutable run.json. '
            'Actual approval state, PID memory and references are bundled in approval_snapshots.npz. '
            'Validation and metrics use only this folder; no original benchmark directories or calibration '
            'folder are required. Historical experiment provenance remains in original_experiments, with '
            'source/preparation evidence in source_versions.zip.\n\n'
            + 'For the 2 s protocol the predictor candidate grid is still 19 points through 1 s. '
            'No predicted expiration continues to permit dispatch beyond 1 s; finite TTL and readiness gates '
            'are unchanged. The displacement profile ends 3.8 s after a dive starts, so D=2 leaves 2.2 s '
            'after that profile before the unchanged 8 s deadline. Observed completion and horizon truncation '
            'are reported separately. One journal is analyzed over the full grid, within <=1 s and beyond '
            '>1 s (sampled delays only). Confidence intervals resample entire paired scenario vectors, not individual flights.\n')

    if m.get('analysis_subsets'):
        with appendix.open('a') as stream:
            stream.write('\n## Paired scenario uncertainty by analysis window\n\n'
                'Positive net unsafe-rate reduction favors EarlyRecovery. Intervals resample whole scenarios '
                'with all paired delay trials retained; no interval is estimated from fewer than two complete '
                'scenarios. These are analyses of one physical journal.\n\n'
                '| Window | EarlyRecovery versus | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |\n'
                '|---|---|---:|---:|---|\n')
            for name,window in {'full':m,**m['analysis_subsets']}.items():
                for p,value in window['paired_early_recovery'].items():
                    stat=window['uncertainty']['paired_early_recovery'][p]['net_unsafe_reduction']
                    ci=stat['ci95'];interval='undefined' if ci is None else f'{ci[0]:.4f}–{ci[1]:.4f}'
                    mean='undefined' if stat['mean'] is None else f'{stat["mean"]:.4f}'
                    stream.write(f'| {name} | {p} | {value["unsafe_prevented"]} | {value["unsafe_introduced"]} | {mean} [{interval}] |\n')
            if m.get('paired_extensions'):
                stream.write('\n## Exploratory new-policy paired comparisons\n\n'
                    '| Window | Comparison | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |\n'
                    '|---|---|---:|---:|---|\n')
                for name,window in {'full':m,**m['analysis_subsets']}.items():
                    for comparison,value in window.get('paired_extensions',{}).items():
                        stat=window['uncertainty']['paired_extensions'][comparison]['net_unsafe_reduction']
                        ci=stat['ci95'];bounds='undefined' if ci is None else f'{ci[0]:.4f}–{ci[1]:.4f}'
                        mean='undefined' if stat['mean'] is None else f'{stat["mean"]:.4f}'
                        stream.write(f'| {name} | {comparison} | {value["unsafe_prevented"]} | {value["unsafe_introduced"]} | {mean} [{bounds}] |\n')
            stream.write('\nPaired completion-rate intervals and all scenario-level values are retained in metrics.json. '
                'Inspection abandonment is separate from an inspection not observed by the fixed deadline.\n')

    if m.get('prediction_by_policy'):
        with (Path(out)/'REPORT.md').open('a') as stream:
            stream.write('\nNew policy predictor runtime (warmed, independently computed per policy/scenario):\n\n'
                '| Policy | Forecasts | Median ms | p95 ms |\n|---|---:|---:|---:|\n')
            for p,q in m['prediction_by_policy'].items():
                t=q['runtime'];med='undefined' if t['median_s'] is None else f'{1000*t["median_s"]:.1f}'
                hi='undefined' if t['p95_s'] is None else f'{1000*t["p95_s"]:.1f}'
                stream.write(f'| {p} | {t["samples"]} | {med} | {hi} |\n')
            stream.write('\nRecoveryAware proactively uses the earlier of command expiry and the last safe recovery-prefix '
                'grid point. DualHorizon also considers completion expiry. A late prediction does not retroactively act. '
                'A missed recovery prefix does not invalidate an otherwise safe command; known-expired commands use '
                'the existing recovery as best-effort. Completion-only abandonment requires verified recovery. '
                'All-safe grids are censored; no horizon-end rejection is invented.\n\n'
                'Whole-flight failure decomposition and command/completion/recovery prediction labels are in metrics.json. '
                'Recovery-label comparisons include only actual starts that coincide with predicted grid points. '
                'The root predictor summary refers to the policy named by predictor.scope_policy.\n')
    return m
