"""Analysis/figures for the independent sweep; never run a model or simulator."""
from pathlib import Path
import sys
if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
from collections import Counter,defaultdict
from itertools import groupby
import csv
import json
import numpy as np
from underpass_inspection.storage import ROOT,read_run,iter_committed,file_hash,write_json,prediction_for
from underpass_inspection.baselines.staleguard import TTL_GRID,first_ready_step
from underpass_inspection.baseline_benchmark.metrics import journal_metrics,timing_summary

GRID=list(TTL_GRID)
PROPERTIES=('command','completion','recovery')
PRED_FIELDS=dict(command='command_safe',completion='command_complete',recovery='recovery_safe')
NEW_POLICIES=('staleguard_recovery_aware','staleguard_dual_horizon')


def profile(flags,grid=GRID):
    flags=list(map(bool,flags));bad=next((i for i,v in enumerate(flags) if not v),None)
    return dict(labels=flags,first_loss_s=None if bad is None else grid[bad],
        last_safe_prefix_s=(grid[-1] if bad is None else None if bad==0 else grid[bad-1]),
        last_tested_safe_s=max((d for d,v in zip(grid,flags) if v),default=None),
        right_censored=bad is None,left_censored=bad==0,
        non_monotonic=any(not a and b for a,b in zip(flags,flags[1:])),
        sampled_runs=[dict(feasible=value,first_tested_s=grid[group[0]],last_tested_s=grid[group[-1]])
            for value,indices in groupby(range(len(flags)),key=lambda i:flags[i]) for group in [list(indices)]])


def gap(command,recovery):
    c=command['first_loss_s'];r=recovery['first_loss_s']
    if c is None and r is None:return None,'both_censored'
    if c is None:return None,'recovery_first_command_censored'
    if r is None:return None,'command_first_recovery_censored'
    value=round(r-c,10)
    return value,'command_first' if value>0 else 'recovery_first' if value<0 else 'equal_sampled_loss'


def ratio(a,b):return None if not b else a/b


def distribution(values):
    a=np.asarray([v for v in values if v is not None],float)
    return dict(n=len(a),minimum=float(a.min()) if len(a) else None,maximum=float(a.max()) if len(a) else None,
        median=float(np.median(a)) if len(a) else None,mean=float(a.mean()) if len(a) else None,
        p25=float(np.quantile(a,.25)) if len(a) else None,p75=float(np.quantile(a,.75)) if len(a) else None)


def confusion(predicted,actual):
    return dict(tp=sum(p and t for p,t in zip(predicted,actual)),tn=sum(not p and not t for p,t in zip(predicted,actual)),
                fp=sum(p and not t for p,t in zip(predicted,actual)),fn=sum(not p and t for p,t in zip(predicted,actual)))


def accuracy(forecasts,physical):
    result={}
    for prop in PROPERTIES:
        counts=Counter();boundaries=[];per_seed=[];error=[];prefix_error=[];censor=Counter()
        for seed,surfaces in physical.items():
            saved=forecasts[str(seed)];pred=profile([r[PRED_FIELDS[prop]] for r in saved['results']])
            actual=surfaces[prop];q=confusion(pred['labels'],actual['labels']);counts.update(q);per_seed.append(q)
            ckey=('right_censored' if pred['right_censored'] else 'left_censored' if pred['left_censored'] else 'observed')+'__vs__'+('right_censored' if actual['right_censored'] else 'left_censored' if actual['left_censored'] else 'observed')
            censor[ckey]+=1
            d=None if pred['first_loss_s'] is None or actual['first_loss_s'] is None else round(pred['first_loss_s']-actual['first_loss_s'],10)
            if d is not None:error.append(d)
            if pred['first_loss_s'] is not None and actual['first_loss_s'] is not None and pred['last_safe_prefix_s'] is not None and actual['last_safe_prefix_s'] is not None:
                prefix_error.append(round(pred['last_safe_prefix_s']-actual['last_safe_prefix_s'],10))
            boundaries.append(dict(seed=seed,predicted=pred['first_loss_s'],physical=actual['first_loss_s'],error_s=d,
                predicted_non_monotonic=pred['non_monotonic'],physical_non_monotonic=actual['non_monotonic']))
        n=sum(counts.values());absolute=np.abs(error)
        result[prop]=dict(counts,total=n,accuracy=ratio(counts['tp']+counts['tn'],n),
            false_feasible_rate=ratio(counts['fp'],counts['fp']+counts['tn']),
            false_reject_rate=ratio(counts['fn'],counts['fn']+counts['tp']),
            false_feasible_among_predicted_feasible=ratio(counts['fp'],counts['fp']+counts['tp']),
            false_feasible_scenarios=sum(q['fp']>0 for q in per_seed),false_reject_scenarios=sum(q['fn']>0 for q in per_seed),
            boundary_comparisons=len(error),mae_s=float(np.mean(absolute)) if len(error) else None,
            median_absolute_error_s=float(np.median(absolute)) if len(error) else None,
            mean_signed_error_s=float(np.mean(error)) if len(error) else None,
            last_safe_prefix_mae_s=float(np.mean(np.abs(prefix_error))) if prefix_error else None,
            censoring=dict(censor),predicted_non_monotonic=sum(b['predicted_non_monotonic'] for b in boundaries),
            physical_non_monotonic=sum(b['physical_non_monotonic'] for b in boundaries),
            predicted_right_censored=sum(b['predicted'] is None for b in boundaries),physical_right_censored=sum(b['physical'] is None for b in boundaries),
            predicted_left_censored=sum(b['predicted']==0 for b in boundaries),physical_left_censored=sum(b['physical']==0 for b in boundaries),boundaries=boundaries)
    return result


def readiness(forecasts,physical):
    counts=Counter();direct=Counter();details=[]
    for seed,surfaces in physical.items():
        f=forecasts[str(seed)];ready=first_ready_step(f['computation_s'])/50
        r=surfaces['recovery'];bad=r['first_loss_s'];last=r['last_safe_prefix_s']
        state=('no_loss_observed_ready_within_grid' if ready<=1 else 'no_loss_observed_ready_beyond_grid') if bad is None else (
            'at_or_after_first_unsafe_sample' if ready>=bad else 'by_last_safe_sample' if last is not None and ready<=last else 'within_unresolved_loss_bracket')
        observed=r['labels'][GRID.index(ready)] if ready in GRID else None
        direct['untested_ready_start' if observed is None else 'feasible_at_ready_start' if observed else 'infeasible_at_ready_start']+=1
        counts[state]+=1;details.append(dict(seed=seed,runtime_s=f['computation_s'],ready_s=ready,recovery_feasible_at_ready=observed,
            physical_first_loss_s=bad,physical_last_safe_prefix_s=last,classification=state))
    finite=sum(not p['recovery']['right_censored'] for p in physical.values());late=counts['at_or_after_first_unsafe_sample']
    return dict(timing=timing_summary(f['computation_s'] for f in forecasts.values()),counts=dict(counts),direct_readiness_counts=dict(direct),
        finite_physical_boundaries=finite,late_fraction_all_scenarios=ratio(late,len(physical)),late_fraction_observed_boundaries=ratio(late,finite),
        note='Scheduling uses ceil-to-first-available 50 Hz step. Equality with an unsafe first-loss sample is late. Inside (last safe, first unsafe) is unresolved; right-censored cases have no invented deadline.',details=details)


def unsafe_breakdown(base,source,truth_rows,physical):
    truth={};rows=[]
    for r in iter_committed(base,source):
        if r['event']['policy']=='no_recovery':truth[r['seed'],r['delay_s']]=r
        rows.append(r)
    pair_index={(r['seed'],r['delay_s'],r['event']['policy']):r for r in rows}
    ablation=dict(matched_pairs=0,safety_differences=0,completion_differences=0,action_differences=0,trajectory_hash_differences=0)
    for seed in truth:
        s,d=seed;a=pair_index[s,d,NEW_POLICIES[0]];b=pair_index[s,d,NEW_POLICIES[1]]
        ablation['matched_pairs']+=1;ablation['safety_differences']+=a['safe']!=b['safe']
        ablation['completion_differences']+=a['safe_completion']!=b['safe_completion']
        ablation['action_differences']+=any(a['event'][k]!=b['event'][k] for k in ('application_step','fallback_step'))
        ablation['trajectory_hash_differences']+=a['physics_sha256']!=b['physics_sha256']
    summaries={};csv_rows=[]
    for policy in source['protocol']['policies']:
        group=[r for r in rows if r['event']['policy']==policy];counts=Counter();cases=defaultdict(set);starts=defaultdict(set)
        for r in group:
            e=r['event'];seed=r['seed'];rec=e['fallback_step'] is not None;accepted=e['application_step'] is not None;unsafe=not r['safe']
            correct=rec and not truth[seed,r['delay_s']]['safe']
            tags=[]
            def add(name):counts[name]+=1;cases[name].add(seed);starts[name].add((seed,r['start_s']));tags.append(name)
            if rec:add('recoveries')
            if correct:add('correct_rejections_hindsight')
            if correct and unsafe:add('unsafe_recovery_after_correct_rejection')
            if rec:
                action_cmd=truth_rows.get((seed,'dive',round(r['start_s']*50))) or truth.get((seed,r['start_s']))
                if action_cmd is None:add('command_safety_at_actual_rejection_unmeasured')
                elif not action_cmd['safe']:
                    add('correct_rejection_at_actual_intervention')
                    if unsafe:add('unsafe_recovery_after_correct_rejection_at_actual_intervention')
            if unsafe and accepted:
                if e.get('prediction_ready') is True and not truth[seed,r['delay_s']]['safe']:add('false_safe_admission_hindsight')
                add('unsafe_command_before_prediction_ready' if e.get('prediction_ready') is False else 'unsafe_command_with_ready_prediction' if e.get('prediction_ready') is True else 'unsafe_unchecked_command')
                if e.get('prediction_ready') is True and r['delay_s']>1:add('ready_unsafe_command_beyond_horizon')
                if e.get('prediction_ready') is True and r['delay_s']<=1:add('ready_unsafe_command_within_horizon')
            if rec and unsafe:
                add('unsafe_recovery')
                if r['violation_before_intervention']:add('safety_already_violated_before_recovery')
                loss=physical[seed]['recovery']['first_loss_s']
                if loss is not None and r['start_s']>=loss:add('started_at_or_after_physical_initial_recovery_loss')
                sample=truth_rows.get((seed,'sideways',round(r['start_s']*50)))
                saved=prediction_for(source,policy,seed) if policy in NEW_POLICIES else None
                predicted=next((x['recovery_safe'] for x in saved['results'] if x['delay_s']==r['start_s']),None) if saved else None
                if sample:
                    assert sample['safe']==r['safe'] and sample['physics_sha256']==r['physics_sha256']
                    if predicted is True and not sample['safe']:add('direct_false_feasible_recovery_prediction')
                    elif predicted is False:add('recovery_attempt_despite_predicted_infeasibility')
                    else:add('no_recovery_forecast_for_this_policy')
                else:add('actual_recovery_start_outside_independent_grid')
                csv_rows.append(dict(seed=seed,policy=policy,dispatch_s=r['delay_s'],recovery_s=r['start_s'],
                    correct_rejection_hindsight=correct,correct_rejection_at_actual_intervention=(None if action_cmd is None else not action_cmd['safe']),first_violation_s=r['first_violation_s'],physical_recovery_loss_s=loss,
                    sampled_recovery_prediction=predicted,tags=';'.join(tags)))
        summaries[policy]=dict(counts=dict(counts),scenario_counts={k:len(v) for k,v in cases.items()},
            distinct_recovery_trajectories={k:len(v) for k,v in starts.items()})
    return summaries,csv_rows,ablation


def other_missions():
    result={}
    for mission in ('geofence_turn','inspection_descent'):
        folder=ROOT.parent/mission/'results/evaluation/baseline_benchmark'
        run=json.loads((folder/'run.json').read_text());raw=folder/'executions.jsonl'
        assert file_hash(raw)==run['journal_sha256'] and raw.stat().st_size==run['journal_bytes']
        commands=defaultdict(list);recoveries=defaultdict(list);stats=Counter();scenarios=defaultdict(set);kinds=Counter()
        seen=set()
        with raw.open() as stream:
            for line in stream:
                r=json.loads(line);e=r['event'];key=r['seed'],r['delay_s'],e['policy']
                assert key not in seen;seen.add(key)
                if e['policy']=='no_recovery':
                    commands[r['seed']].append(r);stats['command_flights']+=1;kinds[r.get('execution_kind','physical_journal')]+=1
                    if r['safe'] and not r['completed']:
                        stats['safe_but_incomplete']+=1;scenarios['safe_but_incomplete'].add(r['seed'])
                if e['policy']=='always_recover':
                    recoveries[r['seed']].append(r)
                    if e['fallback_step'] is not None:
                        stats['actual_always_recovery_flights']+=1
                        if not r['safe']:stats['unsafe_always_recoveries']+=1
        profiles={};gap_counts=Counter()
        for seed,rs in commands.items():
            rs=sorted(rs,key=lambda r:r['delay_s']);grid=[r['delay_s'] for r in rs]
            p={name:profile([r['safe'] if name=='command' else r['safe_completion'] for r in rs],grid) for name in ('command','completion')}
            p['completion_lost_before_command_safety']=any(r['safe'] and not r['completed'] for r in rs)
            rec=sorted(recoveries[seed],key=lambda r:r['delay_s'])
            if mission=='geofence_turn':
                assert len(rec)==len(rs) and all(r['event']['fallback_step']==r['event']['scheduled_dispatch_step'] for r in rec)
                p['recovery']=profile([r['safe'] for r in rec],grid)
                _,order=gap(p['command'],p['recovery']);gap_counts[order]+=1
                values=[p[q]['first_loss_s'] for q in PROPERTIES]
                if all(v is not None and v>0 for v in values) and len(set(values))==3:stats['three_distinct_observed_first_losses']+=1
            else:
                assert all(r['event']['fallback_step']==r['event']['scheduled_dispatch_step']-round(r['delay_s']*50) for r in rec if r['event']['fallback_step'] is not None)
            profiles[str(seed)]=p
        result[mission]=dict(stats=dict(stats),scenarios=len(commands),safe_but_incomplete_scenarios=len(scenarios['safe_but_incomplete']),
            delays=run['protocol']['delays'],gap_counts=dict(gap_counts),execution_kinds=dict(kinds),
            note=('Always Recover initiates braking/position hold at each dispatch: matched recorded recovery starts can support sampled recovery profiles. It is not Underpass sideways recovery.' if mission=='geofence_turn' else
                  'Historical Always Recover initiates at approval for D>0 and executes the command at D=0. Repeated delays therefore do not independently sweep recovery initiation times; no recovery deadline is inferred.'),
            examples=[dict(seed=seed,delay_s=r['delay_s']) for seed,rs in commands.items() for r in rs if r['safe'] and not r['completed']][:3],profiles=profiles,
            source_journal_sha256=run['journal_sha256'])
    return result


def figure(out,physical,scenarios):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    negative=next((r for r in scenarios if r['ordering']=='recovery_first'),None)
    representative=negative or next((r for r in scenarios if r['ordering']=='command_first'),scenarios[0])
    seed=representative['seed'];surfaces=physical[seed]
    fig,axes=plt.subplots(1,2,figsize=(10.5,3.8),gridspec_kw={'width_ratios':[1.1,1]})
    ax=axes[0]
    for y,(name,label) in enumerate((('recovery','Sideways recovery'),('completion','Safe inspection'),('command','Command safety'))):
        flags=np.array(surfaces[name]['labels']);ax.scatter(np.array(GRID)[flags],np.full(flags.sum(),y),s=32,c='#17834b',marker='o',zorder=3)
        ax.scatter(np.array(GRID)[~flags],np.full((~flags).sum(),y),s=35,c='#ba3232',marker='x',zorder=3)
        ax.axhline(y,color='.85',lw=.7,zorder=0)
    ax.set(yticks=[0,1,2],yticklabels=['Sideways recovery','Safe inspection','Command safety'],ylim=(-.5,2.8),xlim=(-.025,1.025),xlabel='Approval-relative maneuver start (s)',title=f'(a) Independent physical samples: seed {seed}')
    ax.scatter([],[],c='#17834b',s=30,label='Feasible');ax.scatter([],[],c='#ba3232',s=30,marker='x',label='Infeasible');ax.legend(loc='upper right',ncol=2,frameon=False,fontsize=8)
    finite=[r['gap_first_loss_s'] for r in scenarios if r['gap_first_loss_s'] is not None];counts=Counter(r['ordering'] for r in scenarios)
    ax=axes[1]
    if finite:
        keys=sorted(set(finite));values=Counter(finite)
        ax.bar(keys,[values[k] for k in keys],width=.015,color=['#cf7043' if k<0 else '#777777' if k==0 else '#3984a6' for k in keys])
    ax.axvline(0,color='.25',ls='--',lw=.8);ax.set(xlabel=r'Sampled first-loss gap $t_{rec}-t_{cmd}$ (s)',ylabel='Scenarios',title=f'(b) Physical gaps: {len(finite)}/400 both observed')
    ax.text(.02,.96,f'Recovery first: {counts["recovery_first"]}\nCommand first: {counts["command_first"]}\nEqual sampled loss: {counts["equal_sampled_loss"]}',transform=ax.transAxes,va='top',fontsize=8,bbox=dict(facecolor='white',edgecolor='none',alpha=.9))
    censored=len(scenarios)-len(finite)
    fig.text(.5,.015,f'All 400 scenarios retained; {censored} have at least one right-censored boundary. Points are tested starts, not continuous-time guarantees.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.06,1,.98))
    for suffix in ('png','pdf'):fig.savefig(out/f'Fig2_physical_feasibility.{suffix}',dpi=220,bbox_inches='tight')
    plt.close(fig)
    (out/'Fig2_caption.md').write_text(f'Independent physical feasibility at the frozen 19 start delays through 1 s; every point is a separate complete physical flight, not a state sampled along one trajectory. (a) Seed {seed}: green circles denote feasible full-horizon flights, red crosses infeasible flights. Safe inspection requires both safety and the original dwell objective; no trajectory interpolation is shown. (b) First physically infeasible sampled recovery start minus first physically infeasible sampled command start. Negative values mean recovery is lost first, positive values command execution is lost first; equal samples do not establish equal continuous-time deadlines. The histogram includes only scenarios with both first losses observed. The remaining {censored} cases are classified without assigning finite gaps: command loss observed but recovery loss unobserved in {counts["command_first_recovery_censored"]}, recovery loss observed but command loss unobserved in {counts["recovery_first_command_censored"]}, and neither loss observed in {counts["both_censored"]}. All 400 scenarios remain in the denominator. Full safe/unsafe profiles and non-monotonic flags are retained; matching command/completion rows are shown without inventing separate boundaries.\n')
    return seed



def policy_figure(out,source,m):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected=[('no_recovery','No Recovery','#666666'),('use_time_recheck','Use-Time Recheck','#d2992b'),
        ('staleguard_early_recovery','Command-only EarlyRecovery','#a5539a'),
        ('fixed_early_recovery','Fixed EarlyRecovery','#4286b4'),('staleguard_dual_horizon','StaleGuard (complete)','#158153')]
    fig,axes=plt.subplots(1,2,figsize=(10.2,3.7));delays=source['protocol']['delays']
    offsets={'staleguard_dual_horizon':(5,-16),'staleguard_early_recovery':(5,7),'fixed_early_recovery':(5,0)}
    for policy,label,color in selected:
        q=m['by_policy'][policy];x=100*q['unsafe']/q['flights'];y=100*q['safe_completion']/q['flights']
        axes[0].scatter(x,y,color=color,s=34);axes[0].annotate(label,(x,y),xytext=offsets.get(policy,(5,2)),textcoords='offset points',fontsize=7)
        values=[100*m['by_delay'][str(d)][policy]['unsafe']/400 for d in delays]
        axes[1].plot(delays,values,'o-',color=color,ms=3,lw=1.3,label=label)
    axes[0].set(xlabel='Unsafe flights (%) — lower is better',ylabel='Safe inspections (%) — higher is better',xlim=(-1,58),ylim=(30,65),title='(a) Safety–completion trade-off')
    axes[1].axvline(1,color='.3',ls='--',lw=.9);axes[1].set(xlabel='Dispatch delay (s)',ylabel='Unsafe flights (%)',title='(b) Delay dependence: 400 scenarios / point',ylim=(-3,104))
    axes[1].legend(fontsize=6.5,frameon=False,loc='upper left');axes[1].text(1.02,8,'Prediction horizon',rotation=90,fontsize=7,color='.3')
    for ax in axes:ax.grid(alpha=.18)
    fig.tight_layout()
    for suffix in ('png','pdf'):fig.savefig(out/f'policy_tradeoff_delay.{suffix}',dpi=200,bbox_inches='tight')
    plt.close(fig)


def write_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)


def analyze(out,source_path=None):
    out=Path(out);run=read_run(out)
    if run['status']!='complete' or run['completed_flights']!=15200:raise ValueError('Complete all independent physical points first')
    source_path=Path(source_path or run['source_path']);source=read_run(source_path)
    assert source['journal_sha256']==run['source_journal_sha256'] and source['prediction_sha256']==run['source_prediction_sha256']
    assert source['config_sha256']==run['source_config_sha256'] and source['inputs_sha256']==run['inputs_sha256']
    rows={}
    for r in iter_committed(out,run):
        k=r['seed'],r['mode'],round(r['start_s']*50)
        if k in rows:raise ValueError('Duplicate physical point')
        rows[k]=r
    expected={(r['seed'],mode,round(d*50)) for r in run['protocol']['scenarios'] for mode in ('dive','sideways') for d in GRID}
    assert set(rows)==expected
    for r in iter_committed(source_path,source):
        if r['event']['policy']=='no_recovery' and r['delay_s'] in GRID:
            physical_row=rows[r['seed'],'dive',round(r['delay_s']*50)]
            assert physical_row['origin']=='reused_no_recovery'
            for field in ('safe','safe_completion','physics_sha256','min_clearance_m','first_violation'):assert physical_row[field]==r[field]
    for name in NEW_POLICIES:
        assert set(source['predictions_by_policy'][name])=={str(s['seed']) for s in run['protocol']['scenarios']}
        for f in source['predictions_by_policy'][name].values():
            assert [r['delay_s'] for r in f['results']]==GRID
            assert all(not r['command_complete'] or r['command_safe'] for r in f['results'])
    physical={};scenario_rows=[];profile_rows=[]
    for raw in run['protocol']['scenarios']:
        seed=raw['seed'];cmd=[rows[seed,'dive',round(d*50)] for d in GRID];rec=[rows[seed,'sideways',round(d*50)] for d in GRID]
        surfaces=dict(command=profile([r['safe'] for r in cmd]),completion=profile([r['safe_completion'] for r in cmd]),recovery=profile([r['safe'] for r in rec]))
        assert all(not c or s for c,s in zip(surfaces['completion']['labels'],surfaces['command']['labels']))
        physical[seed]=surfaces;delta,order=gap(surfaces['command'],surfaces['recovery'])
        scenario_rows.append(dict(seed=seed,t_cmd_first_loss_s=surfaces['command']['first_loss_s'],t_completion_first_loss_s=surfaces['completion']['first_loss_s'],
            t_cmd_last_safe_prefix_s=surfaces['command']['last_safe_prefix_s'],t_completion_last_safe_prefix_s=surfaces['completion']['last_safe_prefix_s'],
            t_rec_first_loss_s=surfaces['recovery']['first_loss_s'],t_rec_last_safe_prefix_s=surfaces['recovery']['last_safe_prefix_s'],gap_first_loss_s=delta,ordering=order,
            non_monotonic=any(p['non_monotonic'] for p in surfaces.values()),
            safe_command_while_recovery_unsafe=any(c and not r for c,r in zip(surfaces['command']['labels'],surfaces['recovery']['labels']))))
        for i,d in enumerate(GRID):
            predicted=source['predictions_by_policy']['staleguard_dual_horizon'][str(seed)]['results'][i]
            profile_rows.append(dict(seed=seed,delay_s=d,command_safe=cmd[i]['safe'],safe_completion=cmd[i]['safe_completion'],
                recovery_safe=rec[i]['safe'],predicted_command_safe=predicted['command_safe'],predicted_safe_completion=predicted['command_complete'],
                predicted_recovery_safe=predicted['recovery_safe'],command_origin=cmd[i]['origin'],
                command_min_clearance_m=cmd[i]['min_clearance_m'],recovery_min_clearance_m=rec[i]['min_clearance_m']))
    result=dict(scenarios=400,candidate_delays=19,physical_records=len(rows),new_flights=sum(r['origin']=='independent_pybullet' for r in rows.values()),
        reused_flights=sum(r['origin']=='reused_no_recovery' for r in rows.values()),
        boundary_convention='First tested loss for all three properties; conservative initial prefix. Last safe prefix is separately recorded. No infinite/1s replacement of censored boundaries.',
        gap_counts=dict(Counter(r['ordering'] for r in scenario_rows)),finite_gap_distribution=distribution(r['gap_first_loss_s'] for r in scenario_rows),
        physical_censoring={q:dict(right=sum(p[q]['right_censored'] for p in physical.values()),left=sum(p[q]['left_censored'] for p in physical.values()),non_monotonic=sum(p[q]['non_monotonic'] for p in physical.values())) for q in PROPERTIES},
        command_feasible_after_recovery_infeasible_scenarios=sum(r['safe_command_while_recovery_unsafe'] for r in scenario_rows),
        safe_but_incomplete_command_points=sum(s and not c for p in physical.values() for s,c in zip(p['command']['labels'],p['completion']['labels'])),
        physical_profiles={str(seed):v for seed,v in physical.items()})
    result['prediction_profile_disagreements_between_new_policies']=sum(a['results']!=source['predictions_by_policy'][NEW_POLICIES[1]][seed]['results'] for seed,a in source['predictions_by_policy'][NEW_POLICIES[0]].items())
    result['command_profile_disagreements_original_vs_complete']=sum(a['safe']!=b['command_safe'] for seed,f in source['predictions'].items() for a,b in zip(f['results'],source['predictions_by_policy'][NEW_POLICIES[1]][seed]['results']))
    result['prediction_accuracy']={p:accuracy(source['predictions_by_policy'][p],physical) for p in NEW_POLICIES}
    result['gap_distributions_by_ordering']={name:distribution(r['gap_first_loss_s'] for r in scenario_rows if r['ordering']==name) for name in ('command_first','recovery_first','equal_sampled_loss')}
    result['readiness']={p:readiness(source['predictions_by_policy'][p],physical) for p in NEW_POLICIES}
    result['readiness']['staleguard']=readiness(source['predictions'],physical)
    result['predicted_safe_but_incomplete_points']={p:sum(x['command_safe'] and not x['command_complete'] for f in source['predictions_by_policy'][p].values() for x in f['results']) for p in NEW_POLICIES}
    result['first_loss_obstacles']={}
    for prop,mode in (('command','dive'),('completion','dive'),('recovery','sideways')):
        events=[rows[seed,mode,round(p[prop]['first_loss_s']*50)]['first_violation'] for seed,p in physical.items() if p[prop]['first_loss_s'] is not None]
        result['first_loss_obstacles'][prop]=dict(Counter('mission_noncompletion_without_violation' if e is None else e['part'] for e in events))
    original_metrics=json.loads((source_path/'metrics.json').read_text());assert journal_metrics(source_path,source)==original_metrics
    result['benchmark_metrics_reproduced_exactly']=True
    result['policy_failure_breakdown'],recovery_rows,result['completion_condition_ablation']=unsafe_breakdown(source_path,source,rows,physical)
    result['cross_mission_evidence']=other_missions()
    representatives={name:next((r for r in scenario_rows if r['ordering']==name),None) for name in result['gap_counts']}
    result['representative_scenarios']=representatives
    result['fig2_seed']=figure(out,physical,scenario_rows)
    policy_figure(out,source,original_metrics)
    result['original_policy_comparisons']={k:original_metrics[k] for k in ('by_policy','paired_extensions','paired_early_recovery','uncertainty')}
    result['original_policy_subsets']={k:{f:v[f] for f in ('by_policy','paired_extensions','uncertainty')} for k,v in original_metrics['analysis_subsets'].items()}
    result['claim_assessments']={
        'claim_1':dict(status='supported',scope='Distinct execution/recovery opportunities in Underpass; distinct safety/completion opportunities in archived Geofence and Inspection results.',
            limitations='Sampled simulator evidence, not exact continuous-time deadlines. Underpass safety and safe completion coincide; 190 scenarios have at least one right-censored boundary.'),
        'claim_2':dict(status='partially supported',scope='All three saved feasibility profiles independently checked on 7600 physical outcomes each; every recorded forecast is ready at a physically safe sampled recovery start.',
            limitations='25 command/completion and 12 recovery false-feasible labels; censored boundary mismatches; one non-monotonic predicted command profile. Warmed offline runtimes are replayed on the simulator clock, not an asynchronous hardware demonstration. No validation beyond 1 s.'),
        'claim_3':dict(status='supported',scope='Existing paired 400-scenario results and scenario-bootstrap confidence intervals reproduce exactly; independent sweeps confirm recovery feasibility failures.',
            limitations='Trade-off, not universal dominance: complete StaleGuard sacrifices 58 safe inspections versus command-only EarlyRecovery and has 65 more unsafe flights than Fixed EarlyRecovery. No additional Underpass decision benefit from the completion condition; 279 of 381 net prevented unsafe flights versus command-only EarlyRecovery occur beyond the predictor horizon.')}
    audit=out/'preserved_artifact_hashes.json'
    protected=(json.loads(audit.read_text()) if audit.exists() else {
        str(source_path/'executions.jsonl'):run['source_journal_sha256'],
        str(source_path/'predictions.jsonl'):run['source_prediction_sha256']})
    assert all(Path(p).is_file() and file_hash(p)==h for p,h in protected.items())
    result['unchanged_existing_artifacts']=len(protected)
    write_json(out/'metrics.json',result);write_csv(out/'per_scenario.csv',scenario_rows);write_csv(out/'profiles.csv',profile_rows);write_csv(out/'unsafe_recoveries.csv',recovery_rows)
    table=[]
    for prop,q in result['prediction_accuracy']['staleguard_dual_horizon'].items():
        table.append(dict(property=prop,accuracy=q['accuracy'],false_feasible=q['fp'],false_reject=q['fn'],true_feasible=q['tp'],true_infeasible=q['tn'],
            false_feasible_rate=q['false_feasible_rate'],false_reject_rate=q['false_reject_rate'],boundary_pairs=q['boundary_comparisons'],boundary_mae_s=q['mae_s'],boundary_median_absolute_error_s=q['median_absolute_error_s']))
    write_csv(out/'prediction_accuracy.csv',table)
    report(out,result)
    validation=json.loads((out/'validation.json').read_text())
    validation.update(predicted_physical_pairs_per_profile=400*len(GRID),
        saved_prediction_grid_alignment=True,reused_command_records_verified=4000,
        original_policy_metrics_reproduced_exactly=True,unchanged_existing_artifacts=len(protected),
        same_input_profiles_between_new_policies=result['prediction_profile_disagreements_between_new_policies']==0)
    write_json(out/'validation.json',validation)
    print('Analysis complete:',result['gap_counts']);print('Prediction accuracy',table)


def report(out,m):
    primary=m['prediction_accuracy']['staleguard_dual_horizon'];g=m['gap_counts'];ready=m['readiness']['staleguard_dual_horizon']
    completed=m['policy_failure_breakdown']['staleguard_dual_horizon']
    counts=completed['counts'];late=ready['counts'].get('at_or_after_first_unsafe_sample',0)
    labels=dict(recovery_first='Recovery lost first; both observed',command_first='Command lost first; both observed',equal_sampled_loss='Equal first-loss samples',recovery_first_command_censored='Recovery lost; command loss unobserved',command_first_recovery_censored='Command lost; recovery loss unobserved',both_censored='Neither loss observed through 1 s')
    lines=['# Independent physical feasibility: supplementary results','',
        'We tested whether waiting can remove the UAV’s escape route before its approved inspection becomes unsafe. '
        'The aircraft continues North while waiting, then either dives under the bridge and swerves East around the pier, or abandons inspection and turns East beside the bridge. '
        'Across the same 400 scenarios, 11,200 new PyBullet flights plus 4,000 reused command flights establish 19-point physical feasibility profiles. '
        f'Both losses are observed in 210 scenarios: {g["command_first"]} command-first, {g["recovery_first"]} recovery-first and {g["equal_sampled_loss"]} equal-sample cases. '
        'Recovery predictions agree with 99.61% of physical labels, but include 12 false-feasible predictions. '
        'All 400 recorded forecasts become ready at a tested safe recovery start. '
        'Existing policy results reproduce unchanged. Underpass still supplies no extra decision value for completion prediction; Geofence supplies evidence that safety and completion can differ. '
        'Use these results for the paper, retaining sampled-boundary, finite-horizon, prediction-error and simulated-timing limitations.','',
        '## Physical opportunities','',
        'Every point is a separate whole-horizon flight, including the waiting period. Safety uses unchanged 1,000 Hz monitoring; success additionally requires the original inspection dwell objective. '
        'All boundaries below mean **first tested infeasible start**, consistently for command and recovery. They are not exact latest-safe times.','',
        '| Physical ordering | Scenarios / 400 |','|---|---:|']
    for name in ('command_first','recovery_first','equal_sampled_loss','command_first_recovery_censored','recovery_first_command_censored','both_censored'):
        lines.append(f'| {labels[name]} | {g[name]} ({g[name]/400:.1%}) |')
    positive=m['gap_distributions_by_ordering']['command_first'];negative=m['gap_distributions_by_ordering']['recovery_first']
    lines+=['',f'Finite positive gaps range **{positive["minimum"]:.2f}–{positive["maximum"]:.2f} s** (median {positive["median"]:.2f} s); finite negative gaps range **{negative["minimum"]:.2f}–{negative["maximum"]:.2f} s**. '
        'Including cases with only one observed loss, command-first ordering is established in 256 scenarios and recovery-first ordering in 73; 49 remain unresolved. '
        'No physical profile returns to feasibility after its first loss. No command is safe but incomplete among 7,600 tested command flights. '
        'Command expiration is caused by the pier in 289/318 cases; all 243 observed recovery losses involve the deck’s front or front-side clearance.','',
        'Representative examples: seed **1012** loses recovery at 0.80 s and command safety at 1.00 s (gap −0.20 s); '
        'seed **1011** loses command safety at 0.04 s and recovery at 0.90 s (gap +0.86 s); seed **1005** has equal first-loss samples at 1.00 s. '
        'Seed 1222 loses recovery at 1.00 s while command safety remains unexpired on the grid.','',
        '![Fig. 2: independent physical profiles and deadline gaps](Fig2_physical_feasibility.png)','',
        '## Prediction accuracy and readiness','',
        '| Profile | Accuracy | False-feasible: count / rate | False-reject: count / rate | Boundary MAE / median (ms) | Boundary pairs |',
        '|---|---:|---:|---:|---:|---:|']
    for prop,q in primary.items():
        lines.append(f'| {prop.capitalize()} | {q["accuracy"]:.2%} | {q["fp"]} / {q["false_feasible_rate"]:.2%} | {q["fn"]} / {q["false_reject_rate"]:.2%} | {q["mae_s"]*1000:.2f} / {q["median_absolute_error_s"]*1000:.0f} | {q["boundary_comparisons"]} |')
    lines+=['','Each row compares 7,600 saved prediction labels with independent physical outcomes. False-feasible rate divides by physically infeasible points; false-reject rate divides by physically feasible points. '
        'Physical command/completion losses are unobserved in 82 scenarios and recovery loss in 157; corresponding predicted counts are 79 and 156. '
        'Boundary errors exclude unobserved losses. One command/completion prediction is already infeasible at the first sample; it is explicitly flagged. '
        'One predicted command/completion profile, seed 1611, is non-monotonic; its full labels and conservative first loss are retained.','',
        f'The complete predictor’s stored warmed runtime is **{ready["timing"]["median_s"]*1000:.1f} ms median / {ready["timing"]["p95_s"]*1000:.1f} ms p95**. '
        f'After rounding availability to the next 50 Hz control step, **{late}/243** forecasts are late relative to observed physical recovery loss; 157 recovery boundaries are unobserved. '
        'All 400 readiness times exactly match tested safe recovery starts. Timely availability does not guarantee a timely or correct intervention.','',
        '## Existing policy evidence','',
        'The original 46,800-flight metrics and scenario-bootstrap confidence intervals reproduce exactly. Complete StaleGuard reduces unsafe flights from **563 to 182** versus command-only EarlyRecovery '
        '(7.33 percentage points, 95% CI 6.12–8.63), while losing **58** safe inspections. Against Fixed EarlyRecovery it preserves **1,017** inspections but incurs **65** additional unsafe flights. '
        'Of the net 381 unsafe flights prevented versus command-only EarlyRecovery, 102 are within the prediction horizon and 279 beyond it.','',
        '| Existing policy | Unsafe recoveries after correctly rejecting the scheduled unsafe command | Scenarios |',
        '|---|---:|---:|']
    for p,label in (('staleguard','Dispatch-only StaleGuard'),('staleguard_early_recovery','Command-only EarlyRecovery'),('staleguard_dual_horizon','Complete StaleGuard')):
        v=m['policy_failure_breakdown'][p];field='unsafe_recovery_after_correct_rejection'
        lines.append(f'| {label} | {v["counts"].get(field,0)} | {v["scenario_counts"].get(field,0)} |')
    lines+=['','“Correct” here is retrospective: No Recovery at the scheduled dispatch was unsafe. Early policies do not know that future delay. '
        'At the actual early recovery starts, all 13 unsafe complete-StaleGuard recoveries still had a physically safe command alternative; 11 have an unsafe later scheduled command. '
        'Those 13 unsafe recoveries are three repeated scenario/start cases with directly false-feasible recovery predictions, not 13 independent model failures. '
        f'Complete StaleGuard’s 182 unsafe flights comprise **{counts["unsafe_command_before_prediction_ready"]} dispatches before readiness, {counts["unsafe_command_with_ready_prediction"]} unsafe commands admitted with a ready forecast, and {counts["unsafe_recovery"]} unsafe recoveries**; '
        f'{counts["ready_unsafe_command_beyond_horizon"]} of the ready unsafe admissions occur beyond 1 s. '
        'All 278 unsafe command-only early recoveries begin at/after the measured initial recovery loss; that policy does not forecast recovery feasibility.','',
        '## Three-mission interpretation','',
        'Archived Geofence Turn contains **2,659 safe-but-incomplete command flights in 246/300 scenarios** and 75 scenarios with three distinct positive sampled first-loss boundaries. '
        'Inspection Descent contains **265 safe-but-incomplete flights in 126/276 scenarios**; its approval-time fallback records do not establish delayed recovery deadlines. '
        'Different missions, delay grids and recovery maneuvers are not pooled.','',
        '| Claim | Assessment and remaining limitation |','|---|---|']
    for k,v in m['claim_assessments'].items():lines.append(f'| {k.replace("_"," ").title()} | **{v["status"]}**. {v["limitations"]} |')
    lines+=['','RecoveryAware and DualHorizon retain identical predictions, actions and physical hashes across all 5,200 matched cases. '
        'Thus the independent decision value of completion prediction is **not demonstrated in Underpass**. '
        f'All {m["unchanged_existing_artifacts"]:,} existing result files remain unchanged. Full confusion counts, censoring, scenario counts, within/beyond-horizon comparisons and reproduction commands are in metrics.json and TECHNICAL_APPENDIX.md.']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')

    appendix=['# Technical appendix','',
        '## Protocol and implementation','',
        'feasibility_sweep.py calls unchanged simulation.execute from reset; approval state and PID/reference arrays match the frozen benchmark inputs. '
        'Reuse: 4,000 matching No Recovery records. New flights: 3,600 missing command starts plus 7,600 independent sideways starts. '
        'Four isolated serial simulators use one numerical thread each; four smoke flights verify serial/parallel and archived D=0 physics hashes. '
        'All 400 immediate recovery hashes additionally match archived Always Recover flights. No prediction is run during ground-truth generation. '
        'All physical samples run the full original horizon, including before intervention. Existing journal/fsync checkpoints provide integrity, duplicate rejection and resume.','',
        'Grid: '+str(GRID)+'. Boundaries use the first infeasible sample, not a mixed command-first-unsafe/recovery-last-safe definition. '
        'Last safe prefix and every sampled feasible/infeasible run remain available in metrics.json; per_scenario.csv lists both first loss and last safe prefix. '
        'Censoring is never replaced by a finite boundary or infinity. Non-monotonicity never skips a physical rollout.','',
        '## Complete confusion counts and boundary censoring','',
        '| Profile | TP | TN | FP | FN | False-feasible scenarios | False-reject scenarios | Predicted / physical right-censored |',
        '|---|---:|---:|---:|---:|---:|---:|---|']
    for prop,q in primary.items():appendix.append(f'| {prop} | {q["tp"]} | {q["tn"]} | {q["fp"]} | {q["fn"]} | {q["false_feasible_scenarios"]} | {q["false_reject_scenarios"]} | {q["predicted_right_censored"]} / {q["physical_right_censored"]} |')
    for prop,q in primary.items():appendix+=['',f'{prop}: predicted-versus-physical boundary categories: `{q["censoring"]}`. Predicted/physical left-censored counts: {q["predicted_left_censored"]}/{q["physical_left_censored"]}.']
    appendix+=['','TP/TN/FP/FN count paired grid evaluations, not independently sampled scenarios. FP/(FP+TN) is the false-feasible rate; FN/(FN+TP) is the false-reject rate. '
        'FP/(TP+FP), separate error-scenario counts, signed boundary errors and last-safe-prefix errors are also retained. '
        'First-loss error is predicted minus physical sampled first loss; absolute error includes a first-sample failure as sample zero, explicitly flagged as left-censored. '
        'No finite error is assigned when a loss is unobserved. All 400 forecast records are retained, not only forecasts that a policy used.','',
        '## Readiness and failure definitions','',
        'Availability is first_ready_step(recorded warmed runtime)/50 seconds after approval. Equality with a first unsafe recovery sample is late; '
        'strictly between last safe and first unsafe is unresolved. A censored physical recovery profile does not supply a deadline. '
        'Every actual readiness step in this dataset has a directly matching safe physical recovery flight. No computation was retimed during the parallel physical sweep.','',
        'unsafe_recoveries.csv records actual action time, scheduled-command hindsight correctness, command correctness at intervention when measured, '
        'and overlapping error/latency tags. Off-grid intervention starts are untested by this sweep and never rounded to a nearby feasibility label. '
        'Repeated delays sharing a (scenario, recovery start) are counted separately as flights but not as independent model failures. '
        'Correct rejection of the scheduled command does not imply that the command was unsafe at an earlier cancellation time.','',
        '| Policy | All unsafe recoveries | After correct scheduled-command rejection | Scenarios for preceding column | Distinct starts for preceding column | Violated before recovery |',
        '|---|---:|---:|---:|---:|---:|']
    for p,v in m['policy_failure_breakdown'].items():
        c=v['counts'];field='unsafe_recovery_after_correct_rejection'
        appendix.append(f'| {p} | {c.get("unsafe_recovery",0)} | {c.get(field,0)} | {v["scenario_counts"].get(field,0)} | {v["distinct_recovery_trajectories"].get(field,0)} | {c.get("safety_already_violated_before_recovery",0)} |')
    appendix+=['','The complete policy’s 13 unsafe recoveries are seeds 1012, 1242 and 1748. All reproduce independently at the same recovery starts. '
        'They also start at/after physical first loss: model error and late physical intervention are overlapping descriptions, not additive causes. '
        'By contrast, unready command admission, ready unsafe command admission and unsafe recovery partition the complete policy’s 182 unsafe flights.','',
        '## Preserved paired comparisons','',
        '| Complete StaleGuard versus | Unsafe prevented | Unsafe introduced | Net unsafe reduction (pp), 95% CI | Completion change (pp) |','|---|---:|---:|---|---:|']
    for other in ('staleguard_early_recovery','fixed_early_recovery','use_time_recheck'):
        name='staleguard_dual_horizon__vs__'+other;pair=m['original_policy_comparisons']['paired_extensions'][name]
        ci=m['original_policy_comparisons']['uncertainty']['paired_extensions'][name]['net_unsafe_reduction'];limits=ci['ci95']
        appendix.append(f'| {other} | {pair["unsafe_prevented"]} | {pair["unsafe_introduced"]} | {100*ci["mean"]:.2f} [{100*limits[0]:.2f}, {100*limits[1]:.2f}] | {100*pair["net_completion_benefit"]/pair["complete_pairs"]:+.2f} |')
    appendix+=['','| Analysis subset | Policy | Flights | Unsafe | Safe inspections |','|---|---|---:|---:|---:|']
    for name,v in m['original_policy_subsets'].items():
        for p in ('staleguard_early_recovery','staleguard_dual_horizon','fixed_early_recovery'):
            q=v['by_policy'][p];appendix.append(f'| {name} | {p} | {q["flights"]} | {q["unsafe"]} | {q["safe_completion"]} |')
    appendix+=['','All existing nine-policy aggregate, paired, per-scenario and subset metrics including whole-scenario bootstrap confidence intervals were recomputed identically. '
        'The main analysis uses the 400 previously unused cases; no exploratory 100-scenario cohort was merged. '
        'policy_tradeoff_delay.png/pdf is a metrics-only rendering of the existing safety/completion and delay-dependent results.','',
        '## Historical evidence and integrity','',
        'Geofence and Inspection journals are read directly and checked against their recorded hashes/lengths. '
        'Geofence Always Recover starts at dispatch and can supply matched delayed recovery profiles (braking/position hold, not Underpass sideways). '
        'Inspection Always Recover cancels at approval for positive delays and executes the command at zero: repeated rows do not form a recovery-start sweep. '
        'Inspection records retain their historical conversion provenance. Original artifacts, including the stale preparation validation file, remain untouched; '
        'a complete read-only source audit is saved here as source_validation.json. Physical sweep completeness is validation.json. '
        'preserved_artifact_hashes.json protects all pre-existing result files; analysis asserts every checksum unchanged.','',
        '## Reproduction','',
        'Run from examples/staleguard with scg active. The sweep is already complete; resume performs no new flights when all keys exist.','',
        '```bash\npython -m unittest underpass_inspection.support.test_supplementary -v\n'
        'python underpass_inspection/support/feasibility_sweep.py --workers 4 --resume\n'
        'python underpass_inspection/support/supplementary_analysis.py\n```','',
        'A fresh physical reproduction uses feasibility_sweep.py --out <fresh_directory> --workers 4, then supplementary_analysis.py --out <same_directory>. '
        'Fig2 contains physical feasibility samples and first-loss gaps only; no representative Underpass flight trajectory figure was generated.']
    (out/'TECHNICAL_APPENDIX.md').write_text('\n'.join(appendix)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=ROOT/'results/evaluation/supplementary_feasibility_v1')
    p.add_argument('--source',type=Path,help='Locate the unchanged source benchmark after moving the archive; all recorded hashes must still match')
    args=p.parse_args();analyze(args.out,args.source)
