"""Read-only scientific aggregation; no simulator or predictor invocation."""
from pathlib import Path
from collections import Counter,defaultdict
import csv,json,math
import numpy as np
from staleguard.tables import complete_experiments as W
from staleguard.underpass_inspection.storage import read_run,iter_committed
from staleguard.underpass_inspection.baselines.staleguard import first_ready_step
ROOT=W.ROOT
PROPERTIES={'command':'command_safe','completion':'command_complete','recovery':'recovery_safe'}
POLICIES=('no_recovery','always_recover','fbs','fixed_ttl','use_time_recheck','staleguard','staleguard_early_recovery','fixed_early_recovery','staleguard_recovery_aware','staleguard_dual_horizon')
LABELS=dict(no_recovery='No Recovery',always_recover='Always Recover (approval)',fbs='FBS',fixed_ttl='Fixed TTL (legacy)',use_time_recheck='Use-Time Recheck',staleguard='StaleGuard (dispatch)',staleguard_early_recovery='Command-only EarlyRecovery',fixed_early_recovery='Fixed EarlyRecovery (0.20 s)',staleguard_recovery_aware='RecoveryAware (ablation)',staleguard_dual_horizon='StaleGuard (full)')
NAMES={'inspection_descent':'Inspection Descent','geofence_turn':'Geofence Turn','underpass_inspection':'Underpass'}

def write_csv(path,rows):
 fields=list(dict.fromkeys(k for r in rows for k in r))
 with path.open('w',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)

def update_geofence_figure_data(rows,seeds,delays):
 policy='staleguard_dual_horizon'
 groups=defaultdict(list)
 for r in rows:
  if r['policy']==policy:groups[r['delay_s']].append(r)
 assert set(groups)==set(delays) and all(len(group)==len(seeds) for group in groups.values())
 for filename,metric in (('geofence_unsafe_rates.csv',lambda r:not r['safe']),
                         ('geofence_safe_completion_rates.csv',lambda r:r['safe_completion'])):
  path=ROOT/'figures/figure4'/filename
  with path.open(newline='') as f:
   reader=csv.DictReader(f);fields=reader.fieldnames;data=list(reader)
  assert [float(r['delay_s']) for r in data]==list(delays)
  if policy not in fields:fields.append(policy)
  for r in data:
   group=groups[float(r['delay_s'])]
   r[policy]=f'{100*sum(metric(x) for x in group)/len(group):.10f}'
  temporary=path.with_suffix('.csv.tmp')
  with temporary.open('w',newline='') as f:
   writer=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows(data)
  temporary.replace(path)

def ratio(a,b):return None if not b else a/b

def boundary(flags,grid):
 i=next((i for i,v in enumerate(flags) if not v),None)
 return dict(first_loss_s=None if i is None else grid[i],last_safe_prefix_s=grid[-1] if i is None else None if i==0 else grid[i-1],
  initially_infeasible=i==0,right_censored=i is None,non_monotonic=any(not a and b for a,b in zip(flags,flags[1:])),labels=flags)

def profile_metrics(forecasts,truth,grid,seeds):
 result={};physical={};errors_by_seed={}
 for seed in seeds:
  physical[seed]={p:boundary([truth[seed,'recovery' if p=='recovery' else 'command',round(d*50)]['safe' if p!='completion' else 'safe_completion'] for d in grid],grid) for p in PROPERTIES}
 for prop,field in PROPERTIES.items():
  counts=Counter();errors=[];details=[];initial=Counter();censored=Counter();nonmono=Counter();scenarios=Counter()
  for seed in seeds:
   f=forecasts[str(seed)];assert [round(r['delay_s']*50) for r in f['results']]==[round(d*50) for d in grid]
   pred=boundary([r[field] for r in f['results']],grid);act=physical[seed][prop]
   q=Counter('tp' if p and t else 'tn' if not p and not t else 'fp' if p else 'fn' for p,t in zip(pred['labels'],act['labels']))
   counts.update(q);scenarios['false_feasible']+=q['fp']>0;scenarios['false_reject']+=q['fn']>0
   initial['predicted']+=pred['initially_infeasible'];initial['physical']+=act['initially_infeasible']
   censored['predicted']+=pred['right_censored'];censored['physical']+=act['right_censored']
   nonmono['predicted']+=pred['non_monotonic'];nonmono['physical']+=act['non_monotonic']
   error=None if pred['first_loss_s'] is None or act['first_loss_s'] is None else round(pred['first_loss_s']-act['first_loss_s'],10)
   if error is not None:errors.append(error)
   details.append(dict(seed=seed,predicted=pred,physical=act,sampled_boundary_error_s=error))
  total=sum(counts.values());assert total==len(seeds)*len(grid)
  # Keep optional errors null when no finite sampled boundary pair exists.
  result[prop]=dict(status='COMPLETE',scenarios=len(seeds),physical_comparisons=total,counts={k:counts[k] for k in ('tp','tn','fp','fn')},
   accuracy=ratio(counts['tp']+counts['tn'],total),false_feasible_rate=ratio(counts['fp'],counts['fp']+counts['tn']),
   false_reject_rate=ratio(counts['fn'],counts['fn']+counts['tp']),error_scenarios=dict(scenarios),
   boundary_pairs=len(errors),boundary_mae_s=float(np.mean(np.abs(errors))) if errors else None,
   boundary_median_absolute_error_s=float(np.median(np.abs(errors))) if errors else None,
   initial_cases=dict(initial),right_censored=dict(censored),non_monotonic=dict(nonmono),per_scenario=details)
 # All forecasts, not just policy-selected continuations, are timed once per seed.
 runtimes=np.array([forecasts[str(seed)]['computation_s'] for seed in seeds]);readiness=Counter();readiness_rows=[]
 for seed in seeds:
  r=physical[seed]['recovery'];ready=first_ready_step(forecasts[str(seed)]['computation_s'])/50
  if r['first_loss_s'] is None:state='recovery_loss_unobserved'
  elif ready>=r['first_loss_s']:state='late_at_or_after_first_unsafe_sample'
  elif r['last_safe_prefix_s'] is not None and ready<=r['last_safe_prefix_s']:state='ready_by_last_safe_sample'
  else:state='ready_inside_unresolved_boundary_bracket'
  readiness[state]+=1;readiness_rows.append(dict(seed=seed,ready_s=ready,classification=state,physical_recovery_first_loss_s=r['first_loss_s']))
 observed=len(seeds)-readiness['recovery_loss_unobserved']
 return result,dict(timed_forecasts=len(seeds),runtime_median_s=float(np.median(runtimes)),runtime_p95_s=float(np.quantile(runtimes,.95)),
  observed_physical_recovery_boundaries=observed,
  late_recovery_loss_rate_over_observed=ratio(readiness['late_at_or_after_first_unsafe_sample'],observed),
  ready_by_last_safe_sample_rate_over_observed=ratio(readiness['ready_by_last_safe_sample'],observed),
  physical_readiness=dict(readiness),physical_readiness_per_scenario=readiness_rows),physical

def compact(row):
 e=row['event'];a=row.get('approval_step',e['scheduled_dispatch_step']-round(row['delay_s']*50))
 return dict(seed=row['seed'],delay_s=round(row['delay_s'],2),policy=e['policy'],safe=row['safe'],safe_completion=row['safe_completion'],completed=row['completed'],
  application_step=e['application_step'],fallback_step=e['fallback_step'],approval_step=a,first_violation_s=row.get('first_violation_s'),
  kind=row.get('execution_kind','archived_physical_flight'),physical_source=row.get('physical_source'),
  canceled=e.get('command_canceled'),physics_sha256=row.get('physics_sha256'))

def group_metrics(rows):
 n=len(rows);recoveries=[r for r in rows if r['fallback_step'] is not None]
 origins=Counter(r['kind'] for r in rows)
 sources={(r['physical_source'].get('journal'),r['physical_source']['seed'],r['physical_source']['delay_s'],r['physical_source']['policy'],r['physical_source'].get('kind') or ('recovery' if r['fallback_step'] is not None else 'command' if r['application_step'] is not None else 'unapplied')) for r in rows if r['physical_source']}
 # Inspection stops at its first physical violation. A precomputed clock log
 # can describe a later planned cancellation that was never physically reached.
 explicit=sum(r['canceled'] is True and r['fallback_step'] is not None and r['application_step'] is None for r in rows)
 inferred=sum(r['canceled'] is None and r['fallback_step'] is not None and r['application_step'] is None for r in rows)
 return dict(case_count=n,unsafe=sum(not r['safe'] for r in rows),safe_completion=sum(r['safe_completion'] for r in rows),
  unsafe_rate=sum(not r['safe'] for r in rows)/n,safe_completion_rate=sum(r['safe_completion'] for r in rows)/n,
  recovery_attempts=len(recoveries),recovery_failures=sum(not r['safe'] for r in recoveries),
  early_recoveries=sum(r['fallback_step']<r['approval_step']+round(r['delay_s']*50) for r in recoveries),
  command_cancellations=explicit+inferred,explicit_cancellations=explicit,inferred_cancellations=inferred,
  planned_cancellations_not_physically_reached=sum(r['canceled'] is True and r['fallback_step'] is None for r in rows),
  new_physical_flights=origins['new_physical_flight'],source_kind_counts=dict(origins),unique_reused_physical_source_records=len(sources))

def paired(rows,seeds,delays):
 indexed={(r['seed'],round(r['delay_s']*50),r['policy']):r for r in rows};out={};decisions={}
 for other in ('staleguard_early_recovery','staleguard_recovery_aware','fixed_early_recovery','use_time_recheck'):
  if not all((s,round(d*50),p) in indexed for s in seeds for d in delays for p in ('staleguard_dual_horizon',other)):
   out[other]=dict(status='MISSING_PAIRED_CELLS');continue
  counts=Counter();vectors=[];action_scenarios=set();outcome_scenarios=set()
  for seed in seeds:
   uv=[];cv=[]
   for d in delays:
    a=indexed[seed,round(d*50),'staleguard_dual_horizon'];b=indexed[seed,round(d*50),other]
    counts['pairs']+=1;counts['unsafe_prevented']+=not b['safe'] and a['safe'];counts['unsafe_introduced']+=b['safe'] and not a['safe']
    counts['safe_completions_preserved']+=a['safe_completion'] and b['safe_completion'];counts['safe_completions_sacrificed']+=b['safe_completion'] and not a['safe_completion'];counts['safe_completions_gained']+=a['safe_completion'] and not b['safe_completion']
    change=(a['application_step'],a['fallback_step'])!=(b['application_step'],b['fallback_step']);counts['different_actions']+=change
    if change:action_scenarios.add(seed)
    if (a['safe'],a['safe_completion'])!=(b['safe'],b['safe_completion']):outcome_scenarios.add(seed)
    uv.append(int(not b['safe'])-int(not a['safe']));cv.append(int(a['safe_completion'])-int(b['safe_completion']))
   vectors.append([np.mean(uv),np.mean(cv)])
  v=np.asarray(vectors);rng=np.random.default_rng(2027);samples=v[rng.integers(0,len(seeds),(2000,len(seeds)))].mean(axis=1)
  out[other]=dict(status='COMPLETE',**dict(counts),scenarios=len(seeds),different_action_scenarios=len(action_scenarios),different_outcome_scenarios=len(outcome_scenarios),
   net_unsafe_rate_reduction=float(v[:,0].mean()),net_safe_completion_change=float(v[:,1].mean()),
   unsafe_reduction_ci95=np.quantile(samples[:,0],[.025,.975]).tolist(),completion_change_ci95=np.quantile(samples[:,1],[.025,.975]).tolist(),
   bootstrap=dict(unit='whole scenario',resamples=2000,seed=2027))
 return out

def completion_condition_diagnostic(rows,forecasts,seeds,delays):
 """Clock-only attribution with identical forecasts/timing; no extra flights.

 Actual policy results retain their independently measured computation times.
 This diagnostic changes only which existing intervention condition is enabled.
 It must never be interpreted as an alternative physical outcome measurement.
 """
 from types import SimpleNamespace
 from staleguard.model.mission_evaluation import clock_action
 approval={r['seed']:r['approval_step'] for r in rows}
 changed=[];different_profile_seeds=[]
 for seed in seeds:
  f=forecasts[str(seed)]
  if any(r['command_safe']!=r['command_complete'] for r in f['results']):different_profile_seeds.append(seed)
  snap=SimpleNamespace(approval_time=approval[seed]/50,horizon_steps=round(max(delays)*50)+1)
  for d in delays:
   a,_=clock_action('staleguard_recovery_aware',snap,d,f)
   b,_=clock_action('staleguard_dual_horizon',snap,d,f)
   if a!=b:changed.append(dict(seed=seed,delay_s=d,recovery_aware_action=a,full_action=b))
 return dict(type='clock-only diagnostic; no physical outcome inferred',forecast_and_runtime='same recorded full forecast for both conditions',
  scenario_delay_pairs=len(seeds)*len(delays),different_actions=len(changed),different_action_scenarios=len({r['seed'] for r in changed}),
  distinct_command_completion_profile_scenarios=len(different_profile_seeds),profile_scenarios=different_profile_seeds,changed_cases=changed)

def validate_new_records(mission,source,old,stages):
 """Independently recheck actual actions and every reused physical outcome."""
 from staleguard.model.mission_evaluation import cases,clock_action
 data={s:(scenario,snap,p) for s,scenario,snap,p in cases(mission)}
 reference_path=W.stage_path(mission,W.FOLDERS[1]);reference=read_run(reference_path)
 assert reference['status']=='complete'
 assert reference['protocol']['calibration_sha256']==W.sha(ROOT/mission/'results/evaluation/calibration/calibrated_scenarios.json')
 for seed,(_,snap,p) in data.items():assert W.sha(p)==reference['protocol']['snapshots'][str(seed)]
 cache={str((ROOT/mission/'results/evaluation/baseline_benchmark').relative_to(ROOT)):old,
        str(reference_path.relative_to(ROOT)):list(iter_committed(reference_path,reference))}
 cache.update({str(path.relative_to(ROOT)):rows for path,rows in stages.items()})
 indices={}
 for path,rows in cache.items():
  index=defaultdict(list)
  for r in rows:index[r['seed'],round(r['delay_s']*50),r['event']['policy']].append(r)
  indices[path]=index
 checked=0;actions=0
 for path,rows in stages.items():
  for r in rows:
   seed=r['seed'];s,snap,_=data[seed];p=r['event']['policy']
   f=reference['predictions_by_policy'][p][str(seed)] if p in reference['predictions_by_policy'] else W.legacy_forecast(source,seed) if p=='staleguard_early_recovery' else None
   action,_=clock_action(p,snap,r['delay_s'],f,horizon_steps=210 if mission=='inspection_descent' else snap.horizon_steps)
   actual=W.applied_action(r)
   if actual!=action:
    assert mission=='inspection_descent' and not r['safe'] and actual is None
    assert round(r['monitored_until_s']*50)<=action[1]
   actions+=1
   if r.get('physical_source'):
    ps=r['physical_source'];origin=ps.get('journal',str((ROOT/mission/'results/evaluation/baseline_benchmark').relative_to(ROOT)))
    candidates=indices[origin][ps['seed'],round(ps['delay_s']*50),ps['policy']]
    candidates=[x for x in candidates if W.applied_action(x)==actual and all(x[field]==r[field] for field in ('safe','completed','safe_completion','horizon_s'))]
    assert candidates,(mission,seed,p,r['delay_s'],origin)
    for field in ('physics_sha256','control_sha256','min_clearance_m','final_position','final_velocity'):
     if field in r:assert any(x.get(field)==r[field] for x in candidates),(mission,seed,p,field)
    checked+=1
 return dict(passed=True,physical_reuse_records_verified=checked,policy_actions_recomputed=actions,frozen_snapshots_verified=len(data))

def aggregate(out):
 out=Path(out);out.mkdir(parents=True,exist_ok=True);missing=[];allmetrics={};allpairs={};table1=[];table2=[];checks={};sources={}
 replay=json.loads((out/'run.json').read_text()).get('dual_horizon_replay',{}) if (out/'run.json').exists() else {}
 geofence_figure_data=None
 for mission in (*W.MISSIONS,'underpass_inspection'):
  label=NAMES[mission];is_u=mission=='underpass_inspection';scenario_n=400 if is_u else 276 if mission=='inspection_descent' else 300
  bench=ROOT/mission/'results/evaluation'/('baseline_benchmark_v2' if is_u else 'baseline_benchmark')
  if is_u:
   oldrun=read_run(bench);old=list(iter_committed(bench,oldrun));source=oldrun
  else:source,old=W.original(mission)
  seeds=sorted(s['seed'] for s in source['protocol']['scenarios']);delays=[round(d,2) for d in source['protocol']['delays']]
  assert len(seeds)==scenario_n and len(set(seeds))==scenario_n
  sources[str(bench.relative_to(ROOT))]=W.sha(bench/'executions.jsonl')
  combined=[compact(r) for r in old if is_u or r['event']['policy']!='always_recover']
  if mission=='geofence_turn':
   folder=ROOT/mission/'results/evaluation/fixed_early_benchmark';run=json.loads((folder/'run.json').read_text());assert W.sha(folder/'executions.jsonl')==run['journal_sha256']
   sources[str(folder.relative_to(ROOT))]=run['journal_sha256']
   assert sorted(s['seed'] for s in run['protocol']['scenarios'])==seeds
   assert run['protocol']['delays']==source['protocol']['delays']
   for line in (folder/'executions.jsonl').open():combined.append(compact(json.loads(line)))
  new_stages={}
  if not is_u:
   for name in (W.FOLDERS[0],W.FOLDERS[2]):
    folder=W.stage_path(mission,name)
    if not (folder/'progress.json').exists():missing.append(f'{mission}/{name}');continue
    run=read_run(folder)
    if run['status']!='complete':missing.append(f'{mission}/{name}: incomplete');continue
    assert run['protocol']['seeds']==seeds and run['protocol']['dispatch_grid_s']==source['protocol']['delays']
    native_rows=list(iter_committed(folder,run));new_stages[folder]=native_rows
    combined.extend(compact(r) for r in native_rows);sources[str(folder.relative_to(ROOT))]=run['journal_sha256']
  seen=set();groups=defaultdict(list)
  for r in combined:
   k=r['seed'],round(r['delay_s']*50),r['policy'];assert k not in seen;seen.add(k);groups[r['policy']].append(r)
  by_policy={}
  archived_metrics=json.loads((bench/'metrics.json').read_text())['by_policy']
  expected_pairs={(s,round(d*50)) for s in seeds for d in delays}
  for p in POLICIES:
   if p not in groups:
    if p!='fixed_ttl':missing.append(f'{mission}/{p}: missing cases')
    continue
   rows=groups[p];assert {(r['seed'],round(r['delay_s']*50)) for r in rows}==expected_pairs
   assert len(rows)==len(expected_pairs)
   q=group_metrics(rows);by_policy[p]=q
   if p in archived_metrics and (is_u or p!='always_recover'):
    archived=archived_metrics[p]
    assert q['case_count']==archived['flights']
    for field in ('unsafe','safe_completion','recovery_attempts','recovery_failures'):
     assert q[field]==archived[field],(mission,p,field,q[field],archived[field])
   if p=='always_recover':
    assert all(r['fallback_step']==r['approval_step'] and r['application_step'] is None for r in rows)
   table2.append(dict(mission=mission,policy=p,policy_label=LABELS[p],status='COMPLETE',scenarios=len(seeds),dispatch_delays=len(delays),**{k:v for k,v in q.items() if k!='source_kind_counts'},source_kinds=json.dumps(q['source_kind_counts'],sort_keys=True)))
  allpairs[mission]=paired(combined,seeds,delays)
  subsets={name:paired([r for r in combined if (r['delay_s']<=1 if name=='within_prediction_horizon' else r['delay_s']>1)],seeds,[d for d in delays if (d<=1 if name=='within_prediction_horizon' else d>1)]) for name in ('within_prediction_horizon','beyond_prediction_horizon') if any(d<=1 if name=='within_prediction_horizon' else d>1 for d in delays)}
  allmetrics[mission]=dict(scenarios=len(seeds),delays=delays,by_policy=by_policy,paired=allpairs[mission],paired_subsets=subsets)
  checks[mission]=dict(unique_cases=len(seen),per_policy_complete=sorted(by_policy),scenarios=len(seeds),delay_grid=delays)
  if mission in replay:
   q=replay[mission];n=len(seeds)*len(delays)
   assert q['decisions_re_evaluated']==n==q['physical_executions_reused']+q['physical_executions_rerun']
   checks[mission]['dual_horizon_replay']=q
   allmetrics[mission]['dual_horizon_replay']=q
   if mission=='geofence_turn':geofence_figure_data=(combined,seeds,delays)
  feasibility=ROOT/mission/'results/evaluation/supplementary_feasibility_v1'
  if not (feasibility/'progress.json').exists():missing.append(f'{mission}/physical_feasibility');continue
  frun=read_run(feasibility)
  if frun['status']!='complete':missing.append(f'{mission}/physical_feasibility: incomplete');continue
  if not is_u:checks[mission]['independent_record_validation']=validate_new_records(mission,source,old,new_stages)
  if is_u:
   forecasts=source['predictions_by_policy']['staleguard_dual_horizon'];grid=list(next(iter(forecasts.values()))['grid_s'])
  else:forecasts=frun['predictions_by_policy']['staleguard_dual_horizon'];grid=frun['protocol']['prediction_grid_s']
  assert sorted(map(int,forecasts))==seeds
  if not is_u:
   other=frun['predictions_by_policy']['staleguard_recovery_aware']
   assert sorted(map(int,other))==seeds
   assert all(other[str(s)]['results']==forecasts[str(s)]['results'] for s in seeds)
  truth={}
  for r in iter_committed(feasibility,frun):
   kind=('command' if r['mode']=='dive' else 'recovery') if is_u else r['kind']
   key=r['seed'],kind,round((r['start_s'] if is_u else r['delay_s'])*50)
   assert key not in truth;truth[key]=r
  expected_truth={(s,k,round(d*50)) for s in seeds for k in ('command','recovery') for d in grid}
  assert set(truth)==expected_truth
  acc,timing,physical=profile_metrics(forecasts,truth,grid,seeds)
  readiness_available=sum(forecasts[str(seed)]['computation_s']<=d for seed in seeds for d in delays)
  timing.update(available_by_dispatch=readiness_available,dispatch_cases=len(seeds)*len(delays),availability_rate=readiness_available/(len(seeds)*len(delays)))
  if is_u:
   archived=json.loads((feasibility/'metrics.json').read_text())['prediction_accuracy']['staleguard_dual_horizon']
   for p,q in acc.items():
    assert q['counts']=={k:archived[p][k] for k in ('tp','tn','fp','fn')}
    assert abs(q['boundary_mae_s']-archived[p]['mae_s'])<1e-12
  metrics=dict(status='COMPLETE',cohort='evaluation',scenarios=len(seeds),prediction_grid_s=grid,accuracy=acc,readiness=timing,
   physical_record_count=len(truth),initial_and_censored_cases_explicit=True,
   new_physical_flights=0 if is_u else sum(r.get('execution_kind')=='new_physical_flight' for r in truth.values()),
   horizon_note='Full profiles use original operational horizon; unchanged legacy command-only forecast uses 8.5 s from flight start.' if mission=='inspection_descent' else 'Prediction and original physical operational horizons match.')
  allmetrics[mission]['prediction']=metrics
  allmetrics[mission]['completion_condition_diagnostic']=completion_condition_diagnostic(combined,forecasts,seeds,delays)
  if not is_u:
   W.save(feasibility/'metrics.json',metrics)
   W.save(feasibility/'validation.json',dict(passed=True,physical_records=len(truth),expected=len(expected_truth),predicted_labels_per_property=len(seeds)*len(grid),cohort='evaluation',duplicates=0,all_three_properties_compared=True))
  sources[str(feasibility.relative_to(ROOT))]=frun['journal_sha256']
  for prop,q in acc.items():
   table1.append(dict(mission=mission,property=prop,status='COMPLETE',scenarios=len(seeds),physical_comparisons=q['physical_comparisons'],
    accuracy_pct=100*q['accuracy'],false_feasible_count=q['counts'].get('fp',0),false_feasible_pct=None if q['false_feasible_rate'] is None else 100*q['false_feasible_rate'],
    false_feasible_denominator=q['counts']['fp']+q['counts']['tn'],false_reject_denominator=q['counts']['fn']+q['counts']['tp'],
    false_reject_count=q['counts'].get('fn',0),false_reject_pct=None if q['false_reject_rate'] is None else 100*q['false_reject_rate'],
    boundary_mae_ms=None if q['boundary_mae_s'] is None else 1000*q['boundary_mae_s'],boundary_median_ae_ms=None if q['boundary_median_absolute_error_s'] is None else 1000*q['boundary_median_absolute_error_s'],boundary_pairs=q['boundary_pairs'],
    predicted_initially_infeasible=q['initial_cases'].get('predicted',0),physical_initially_infeasible=q['initial_cases'].get('physical',0),
    predicted_right_censored=q['right_censored'].get('predicted',0),physical_right_censored=q['right_censored'].get('physical',0),
    predicted_non_monotonic=q['non_monotonic'].get('predicted',0),physical_non_monotonic=q['non_monotonic'].get('physical',0),
    timed_forecasts=timing['timed_forecasts'],runtime_median_ms=1000*timing['runtime_median_s'],runtime_p95_ms=1000*timing['runtime_p95_s'],
    available_by_dispatch=readiness_available,dispatch_cases=timing['dispatch_cases'],
    ready_by_last_safe_recovery_sample=timing['physical_readiness'].get('ready_by_last_safe_sample',0),
    late_at_sampled_recovery_loss=timing['physical_readiness'].get('late_at_or_after_first_unsafe_sample',0),
    observed_physical_recovery_boundaries=timing['observed_physical_recovery_boundaries'],
    late_recovery_loss_rate_pct=None if timing['late_recovery_loss_rate_over_observed'] is None else 100*timing['late_recovery_loss_rate_over_observed'],
    ready_inside_recovery_boundary_bracket=timing['physical_readiness'].get('ready_inside_unresolved_boundary_bracket',0),
    recovery_boundary_unobserved=timing['physical_readiness'].get('recovery_loss_unobserved',0)))
 complete_I=len(table1)==9
 complete_II=all(len(q['by_policy'])==(9 if m=='underpass_inspection' else 10) for m,q in allmetrics.items())
 complete=not missing and complete_I and complete_II
 for mission in NAMES:
  for prop in PROPERTIES:
   if not any(r['mission']==mission and r['property']==prop for r in table1):table1.append(dict(mission=mission,property=prop,status='MISSING'))
  for p in POLICIES:
   if p=='fixed_ttl' and mission=='underpass_inspection':continue
   if not any(r['mission']==mission and r['policy']==p for r in table2):table2.append(dict(mission=mission,policy=p,policy_label=LABELS[p],status='MISSING'))
 W.save(out/'metrics.json',dict(status='COMPLETE' if complete else 'PARTIALLY COMPLETE',missions=allmetrics,missing=missing,
  source_journal_sha256=sources,bootstrap_unit='scenario',first_loss='first sampled infeasible start; initially infeasible at zero; right-censored errors null',
  analysis_source_sha256={str(p.relative_to(ROOT)):W.sha(p) for p in (Path(__file__),ROOT/'tables/complete_experiments.py',ROOT/'tables/tables_test_complete.py')},
  provenance=W.provenance()))
 W.save(out/'paired_comparisons.json',allpairs);write_csv(out/'table_I.csv',table1);write_csv(out/'table_II.csv',table2)
 # Explicit missing cells: never silently represent an incomplete mission/property as zero.
 lines=['# Table I — Prediction accuracy and computational readiness','',f'Status: {"COMPLETE" if complete_I else "PARTIALLY COMPLETE"}. Evaluation cohorts only.','',
  '| Mission / property | Scenarios | Physical pairs | Accuracy (%) | FF n (%) | FR n (%) | Boundary MAE / median (ms) | Boundary pairs |','|---|---:|---:|---:|---:|---:|---:|---:|']
 def number(x,empty='unobserved'):return empty if x is None else f'{x:.2f}'
 for mission in NAMES:
  for p in PROPERTIES:
   r=next((r for r in table1 if r['mission']==mission and r['property']==p and r['status']=='COMPLETE'),None)
   if r:lines.append(f'| {NAMES[mission]} / {p} | {r["scenarios"]} | {r["physical_comparisons"]} | {number(r["accuracy_pct"])} | {r["false_feasible_count"]} ({number(r["false_feasible_pct"],"N/A")}) | {r["false_reject_count"]} ({number(r["false_reject_pct"],"N/A")}) | {number(r["boundary_mae_ms"])} / {number(r["boundary_median_ae_ms"])} | {r["boundary_pairs"]} |')
   else:lines.append(f'| {NAMES[mission]} / {p} | MISSING | MISSING | MISSING | MISSING | MISSING | MISSING | MISSING |')
 lines+=['','| Full predictor | Timed forecasts | Median / p95 (ms) | Ready by dispatch | Ready by last safe recovery sample | Inside unresolved bracket | Late / observed recovery losses | Recovery loss unobserved |','|---|---:|---:|---:|---:|---:|---:|---:|']
 for mission in NAMES:
  r=next((r for r in table1 if r['mission']==mission and r['status']=='COMPLETE'),None)
  if r:
   late=f'{r["late_at_sampled_recovery_loss"]}/{r["observed_physical_recovery_boundaries"]}' if r['observed_physical_recovery_boundaries'] else 'N/A'
   lines.append(f'| {NAMES[mission]} | {r["timed_forecasts"]} | {r["runtime_median_ms"]:.2f} / {r["runtime_p95_ms"]:.2f} | {r["available_by_dispatch"]}/{r["dispatch_cases"]} | {r["ready_by_last_safe_recovery_sample"]} | {r["ready_inside_recovery_boundary_bracket"]} | {late} | {r["recovery_boundary_unobserved"]} |')
 lines+=['','FF divides by physically infeasible labels; FR by physically feasible labels. N/A means the conditional denominator is zero. Boundary errors are sampled-grid errors, not continuous-time guarantees. Initially infeasible, censored, and non-monotonic cases are retained in CSV/JSON. Right-censored cases have no artificial finite error. Runtime is warmed offline computation replayed on the simulator clock, not an onboard timing validation. No development scenarios enter these tables.']
 (out/'table_I.md').write_text('\n'.join(lines)+'\n')
 lines=['# Table II — Safety and safe mission completion','',f'Status: {"COMPLETE" if complete_II else "PARTIALLY COMPLETE"}. Each mission uses its own frozen scenario/delay grid.','',
  '| Policy | Inspection unsafe / safe completion (%) | Geofence unsafe / safe completion (%) | Underpass unsafe / safe completion (%) |','|---|---:|---:|---:|']
 for p in POLICIES:
  cells=[]
  for mission in NAMES:
   q=allmetrics[mission]['by_policy'].get(p)
   cells.append(f'{100*q["unsafe_rate"]:.2f} / {100*q["safe_completion_rate"]:.2f}' if q else 'Not evaluated' if p=='fixed_ttl' and mission=='underpass_inspection' else 'MISSING')
  lines.append('| '+LABELS[p]+' | '+' | '.join(cells)+' |')
 lines+=['','Always Recover cancels at approval for every D, including D=0. Corrected Inspection/Geofence cells reuse physically equivalent existing approval recoveries; represented delay cases are not claimed as new physical flights. Source reuse and new-flight counts are separate columns in the CSV. Legacy Fixed TTL is retained where already evaluated, not introduced for Underpass. Rates do not pool missions or assume a uniform continuous dispatch-delay distribution.','',
 '| Mission: full versus | Unsafe prevented / introduced | Completion preserved / sacrificed / gained | Unsafe-rate reduction pp [95% CI] | Completion change pp [95% CI] | Different actions / scenarios |','|---|---:|---:|---:|---:|---:|']
 for mission,comparisons in allpairs.items():
  for other,q in comparisons.items():
   if q['status']!='COMPLETE':continue
   uc=q['unsafe_reduction_ci95'];cc=q['completion_change_ci95']
   lines.append(f'| {NAMES[mission]}: {LABELS[other]} | {q.get("unsafe_prevented",0)} / {q.get("unsafe_introduced",0)} | {q.get("safe_completions_preserved",0)} / {q.get("safe_completions_sacrificed",0)} / {q.get("safe_completions_gained",0)} | {100*q["net_unsafe_rate_reduction"]:.2f} [{100*uc[0]:.2f}, {100*uc[1]:.2f}] | {100*q["net_safe_completion_change"]:.2f} [{100*cc[0]:.2f}, {100*cc[1]:.2f}] | {q.get("different_actions",0)} / {q["different_action_scenarios"]} |')
 lines+=['','Intervals resample whole paired scenario vectors (2000 resamples, seed 2027). Full/within-prediction-horizon/beyond-horizon comparisons are in metrics.json. RecoveryAware/full forecasts are computed independently; actual decision differences may involve readiness as well as the completion condition. Raw trigger/profile and event data remain archived for attribution.']
 (out/'table_II.md').write_text('\n'.join(lines)+'\n')
 W.save(out/'VALIDATION.json',dict(passed=complete,status='COMPLETE' if complete else 'PARTIALLY COMPLETE',missions=checks,table_I_rows=len(table1),table_II_rows=len(table2),missing=missing,
  duplicate_checks=True,evaluation_cohorts_only=True,original_artifact_hashes_verified=True,paired_resampling_unit='scenario'))
 report(out,complete,allmetrics,missing)
 if complete:
  if geofence_figure_data:update_geofence_figure_data(*geofence_figure_data)
  total=sum(q['case_count'] for m in allmetrics.values() for q in m['by_policy'].values())
  native=sum(q['new_physical_flights'] for m in allmetrics.values() for q in m['by_policy'].values())+sum(m['prediction']['new_physical_flights'] for m in allmetrics.values())
  W.status(last_error=None,validated_case_count=total,completed=total,pending=0,
           new_physical_records=native,forecasts=1152,forecast_target=1152,new_physical_this_stage=None)
 (ROOT/'tables/CURRENT_RESULTS.md').write_text('# Current experimental tables\n\nStatus: '+('COMPLETE' if complete else 'PARTIALLY COMPLETE')+'\n\n'
  '[Table I: CSV](results/evaluation/complete_tables_v1/table_I.csv) | [Markdown](results/evaluation/complete_tables_v1/table_I.md)\n\n'
  '[Table II: CSV](results/evaluation/complete_tables_v1/table_II.csv) | [Markdown](results/evaluation/complete_tables_v1/table_II.md)\n\n'
  '[Scientific report](results/evaluation/complete_tables_v1/REPORT.md) | [Validation](results/evaluation/complete_tables_v1/VALIDATION.json)\n\n'
  'The original table1.tex/table2.tex fragments retain the earlier measurements. Use these newly aggregated scientific values; no table-layout work is performed by this pipeline.\n')
 print('Aggregation', 'COMPLETE' if complete else 'PARTIALLY COMPLETE','missing:',missing,flush=True)
 return complete

def report(out,complete,metrics,missing):
 facts=[]
 for m,q in metrics.items():
  full=q['by_policy'].get('staleguard_dual_horizon')
  if full:facts.append(f'{NAMES[m]} full StaleGuard: {full["unsafe"]}/{full["case_count"]} unsafe flights and {full["safe_completion"]} safe completions.')
 summary=('The workflow fills the missing three-opportunity prediction and policy evaluations on frozen cohorts of 276, 300 and 400 scenarios. '
 'For example, while the UAV approaches a fence, it can cancel the pending turn and start its existing recovery before that escape becomes unsafe. '
 'Existing valid physical flights are reused; missing continuations are flown independently in PyBullet. '+ ' '.join(facts)+' '
 +('All required experiments and both tables validate.' if complete else 'Work remains incomplete; missing cells are labeled explicitly.')+' '
 'These are sampled simulation results with offline prediction timing, not hardware safety guarantees. Inspection’s legacy command-only predictor has a different forecast horizon, so that comparison does not isolate intervention logic alone. '
 +('Next, transfer the verified values to the paper and retain these limitations.' if complete else 'Next, finish the resumable workload and regenerate the tables automatically.'))
 lines=['# Final experimental tables','',summary,'',
  '| Mission | Scenarios × delays | Full unsafe / cases | Full safe completions | Predictor median / p95 (ms) |','|---|---:|---:|---:|---:|']
 for m,q in metrics.items():
  full=q['by_policy'].get('staleguard_dual_horizon');timing=q.get('prediction',{}).get('readiness')
  lines.append(f'| {NAMES[m]} | {q["scenarios"]} × {len(q["delays"])} | '+(f'{full["unsafe"]} / {full["case_count"]} | {full["safe_completion"]}' if full else 'MISSING | MISSING')+' | '+(f'{1000*timing["runtime_median_s"]:.2f} / {1000*timing["runtime_p95_s"]:.2f}' if timing else 'MISSING')+' |')
 if complete:
  native=sum(q['new_physical_flights'] for m in metrics.values() for q in m['by_policy'].values())
  lines+=['',f'The new policy matrices contain 25,944 cases: {native} new native physical records and {25944-native} cases linked to physically identical measured continuations. Inspection additionally required 2,760 delayed-recovery physical records; Geofence reused its complete existing independent recovery grid. Prediction timing remains independently measured for each policy. Smoke flights are separate validation evidence. Underpass was not rerun.']
 lines+=['','See [Table I](table_I.md), [Table II](table_II.md), machine-readable CSVs, metrics.json and paired_comparisons.json.','',
 '## Scientific limitations','',
 'Command-safety and recovery boundaries use first sampled losses. A pending command retains its completion opportunity through the final feasible sampled interval, with exhaustion at the next sample. At dispatch, the most recent sample supplies the completion label. Completion-based recovery requires a ready forecast and verified recovery. Unobserved boundaries remain censored. Inspection uses an operational horizon of approval plus 4.2 s for its three profiles. Its command-only forecast extends to 8.5 s from flight start, so horizon length also differs in that comparison. The full policy and RecoveryAware use the same predictor horizon and numerical profile. Geofence completion includes the original velocity and dwell conditions. Timing uses recorded offline computation replayed on the simulator clock.','',
 'Always Recover is now unconditional at approval, including a dispatch at D=0. Original historical outcomes remain immutable. Corrected legacy cases are physically equivalent reuses, not newly flown delay-specific trials. Their physical source references are recorded.','',
 'All 276 Inspection recovery profiles remain physically feasible throughout the sampled 0–1 s grid. Its 100% recovery classification agreement therefore does not estimate a recovery expiration error: no physical loss was observed, and the false-feasible conditional rate has no denominator. These fields remain inapplicable or censored rather than being reported as zero error.','',
 '## Completion condition: Geofence','']
 q=metrics['geofence_turn']['paired'].get('staleguard_recovery_aware',{})
 if q.get('status')=='COMPLETE':lines.append(f'Full StaleGuard and RecoveryAware differ in {q.get("different_actions",0)} actual scenario–delay actions across {q["different_action_scenarios"]} scenarios, and in safety/completion outcomes across {q["different_outcome_scenarios"]} scenarios. Identical outcomes, if observed, are a negative ablation finding; no trigger is tuned to produce a difference.')
 else:lines.append('Not yet complete; no comparative outcome is inferred.')
 diagnostic=metrics['geofence_turn'].get('completion_condition_diagnostic')
 if diagnostic:lines.append(f'With the same recorded forecast and runtime supplied to both existing decision rules, enabling the completion condition changes {diagnostic["different_actions"]} actions across {diagnostic["different_action_scenarios"]} scenarios. This clock-only diagnostic separates the completion condition from timing differences; it does not supply additional physical outcomes. Details are in metrics.json.')
 replay=metrics['geofence_turn'].get('dual_horizon_replay')
 if replay:
  lines+=['',f'The Geofence DualHorizon matrix contains {replay["decisions_re_evaluated"]:,} cases. '
          f'Physical simulation covers {replay["physical_executions_rerun"]} cases with revised actions, and the other {replay["physical_executions_reused"]:,} use their measured physical records. '
          f'The reruns changed {replay["outcomes_changed"]["safe"]} safety labels and {replay["outcomes_changed"]["safe_completion"]} safe-completion labels. '
          'The run manifest identifies each simulated seed and dispatch delay. Figure 4 rates use these same execution records.']
 if missing:lines+=['','## Incomplete work','']+[f'- {s}' for s in missing]
 (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
