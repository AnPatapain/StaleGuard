#!/usr/bin/env python3
"""One resumable workflow: frozen-cohort predictions, missing physical flights, tables.

From the git root: conda run --no-capture-output -n scg python
examples/staleguard/tables/complete_experiments.py --launch
"""
from pathlib import Path
import os,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS','NUMBA_NUM_THREADS'):os.environ[name]='1'
os.environ['CUDA_VISIBLE_DEVICES']=''
os.environ.setdefault('MPLCONFIGDIR','/tmp/staleguard-complete-tables-mpl')
import argparse,json,hashlib,time,subprocess,shlex,fcntl,traceback,platform
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
OUT=ROOT/'tables/results/evaluation/complete_tables_v1'
SESSION='staleguard-tables-v1'
MISSIONS=('inspection_descent','geofence_turn')
NEW={'inspection_descent':('staleguard_early_recovery','fixed_early_recovery','staleguard_recovery_aware','staleguard_dual_horizon'),
     'geofence_turn':('staleguard_recovery_aware','staleguard_dual_horizon')}
FOLDERS=('always_recover_corrected_v1','supplementary_feasibility_v1','three_opportunity_benchmark_v1')

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for block in iter(lambda:f.read(1048576),b''):h.update(block)
 return h.hexdigest()

def save(path,value):
 from geofence_turn.storage import write_json
 Path(path).parent.mkdir(parents=True,exist_ok=True);write_json(path,value)

def log(message):print(time.strftime('%Y-%m-%d %H:%M:%S'),message,flush=True)

def status(**values):
 OUT.mkdir(parents=True,exist_ok=True);p=OUT/'status.json'
 old=json.loads(p.read_text()) if p.exists() else {}
 old.update(values,updated_unix_s=time.time());save(p,old)

def provenance():
 import numpy as np
 from underpass_inspection.storage import metadata as physical_metadata
 from importlib.metadata import version
 names=['model/lightweight.py','model/common.py','model/scg.py','model/feasibility_profiles.py','model/mission_evaluation.py','physics_monitor.py',
 'underpass_inspection/baselines/policies.py','underpass_inspection/baselines/recovery_horizons.py','underpass_inspection/baselines/staleguard.py','underpass_inspection/baselines/fixed_early_recovery.py']
 for m in MISSIONS:names += [m+'/'+n for n in ('scenario.py','simulation.py','baselines/always_recover.py','baselines/recovery_horizons.py')]
 return dict(python=sys.version,numpy=np.__version__,platform=platform.platform(),packages={n:version(n) for n in ('numba','pybullet')},
  physics_environment={k:v for k,v in physical_metadata().items() if k in ('environment_sha256','shared_sha256')},
  source_sha256={n:sha(ROOT/n) for n in names},git_revision=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),numerical_threads=1)

def compatible(a,b):
 return {k:v for k,v in a.items() if k!='git_revision'}=={k:v for k,v in b.items() if k!='git_revision'}

def stage_path(mission,name):return ROOT/mission/'results/evaluation'/name

def key(r):return r['seed'],r.get('kind',r['event']['policy']),round(r['delay_s']*50)

def open_stage(path,protocol):
 from underpass_inspection.storage import initialize_run,read_run,committed_hasher
 p=path/'run.json';config=dict(format='three_mission_completion_v1',protocol=protocol,provenance=provenance())
 config=json.loads(json.dumps(config))
 if p.exists():
  r=read_run(path,repair=True)
  if r['protocol']!=config['protocol'] or not compatible(r['provenance'],config['provenance']):raise ValueError('Frozen stage source/configuration differs: '+str(path))
 else:r=initialize_run(path,config)
 return r,committed_hasher(path,r),committed_hasher(path,r,kind='prediction')

def records(path,run):
 from underpass_inspection.storage import iter_committed
 return list(iter_committed(path,run))

def finish(path,run,expected):
 from underpass_inspection.storage import write_progress
 rows=records(path,run);seen={key(r) for r in rows}
 assert len(seen)==len(rows) and seen==expected,(path,len(seen),len(expected))
 for r in rows:assert r['safe_completion']==(r['safe'] and r['completed'])
 run['status']='complete';write_progress(path,run)
 save(path/'validation.json',dict(passed=True,expected_cases=len(expected),recorded_cases=len(rows),duplicates=0,
  new_physical_runs=sum(r.get('execution_kind')=='new_physical_flight' for r in rows),
  reused_cases=sum(r.get('execution_kind')!='new_physical_flight' for r in rows),
  source_and_input_checks=True,monitoring_hz=1000))

def original(mission):
 folder=ROOT/mission/'results/evaluation/baseline_benchmark'
 run=json.loads((folder/'run.json').read_text());progress=json.loads((folder/'progress.json').read_text()) if (folder/'progress.json').exists() else run
 assert sha(folder/'executions.jsonl')==progress['journal_sha256']
 rows=[]
 with (folder/'executions.jsonl').open() as f:
  for line in f:rows.append(json.loads(line))
 return run,rows

def inputs(mission):
 from model.mission_evaluation import cases
 data=cases(mission);source,rows=original(mission)
 assert len(data)==(276 if mission=='inspection_descent' else 300)
 assert {r[0] for r in data}=={s['seed'] for s in source['protocol']['scenarios']}
 return data,source,rows

def protocol(mission,data,source):
 import importlib
 grid=importlib.import_module(mission+'.baselines.recovery_horizons').adapter(data[0][1],data[0][2])['grid']
 return dict(mission=mission,cohort='evaluation',scenarios=source['protocol']['scenarios'],seeds=[r[0] for r in data],
  source_journal_sha256=source['journal_sha256'],source_manifest_sha256=sha(ROOT/mission/'results/evaluation/baseline_benchmark/run.json'),
  snapshots={str(seed):sha(p) for seed,s,snap,p in data},
  calibration_sha256=sha(ROOT/mission/'results/evaluation/calibration/calibrated_scenarios.json'),
  dispatch_grid_s=source['protocol']['delays'],prediction_grid_s=grid,
  recovery='existing mission safety-only reference',monitoring_hz=1000,control_hz=50,model_hz=100,
  full_predictor_horizon='original operational horizon: approval+4.2 s' if mission=='inspection_descent' else 'original approval+scenario.deadline_s',
  legacy_command_predictor_horizon='8.5 s from flight start (unchanged)' if mission=='inspection_descent' else 'original operational horizon',
  policy_core='underpass_inspection.baselines.policies.make_policy',unknown_future_dispatch=True,
  fixed_early_trigger_s=.20,always_recover_zero_delay='recover at approval, cancellation precedes simultaneous dispatch')

def reuse(row,**values):
 import copy
 r=copy.deepcopy(row);r['physical_source']=dict(seed=row['seed'],delay_s=row['delay_s'],policy=row['event']['policy'],physics_sha256=row.get('physics_sha256'))
 r['source_event']=r['event'].copy();r.update(values,execution_kind='verified_physical_equivalence_reuse');return r

def corrected(mission,data,source,old):
 from underpass_inspection.storage import commit_record
 from model.mission_evaluation import clock_action
 proto=protocol(mission,data,source);proto.update(stage='always_recover',source_journal_sha256=sha(ROOT/mission/'results/evaluation/baseline_benchmark/executions.jsonl'))
 path=stage_path(mission,FOLDERS[0]);run,digest,_=open_stage(path,proto);done={key(r) for r in records(path,run)}
 available={}
 for r in old:
  if r['event']['policy']=='always_recover' and (r['delay_s']>0 if mission=='inspection_descent' else r['delay_s']==0):available.setdefault(r['seed'],r)
 assert set(available)=={r[0] for r in data}
 for seed,s,snap,p in data:
  oldrow=available[seed];a=round(snap.approval_time*50)
  assert oldrow['event']['fallback_step']==a and oldrow['event']['application_step'] is None
  for delay in proto['dispatch_grid_s']:
   k=seed,'always_recover',round(delay*50)
   if k in done:continue
   action,event=clock_action('always_recover',snap,delay,horizon_steps=210 if mission=='inspection_descent' else snap.horizon_steps)
   assert action==('recovery',a) and event['application_step'] is None
   row=reuse(oldrow,delay_s=delay,event=event,mission=mission,start_s=0.,mode='recovery',approval_step=a,
     reuse_reason='Identical initial state, approval state/PID, nominal prefix, recovery at approval, fixed horizon; dispatch never changes reference after cancellation')
   commit_record(path,run,row,digest);done.add(k)
 expected={(seed,'always_recover',round(d*50)) for seed,s,snap,p in data for d in proto['dispatch_grid_s']}
 finish(path,run,expected);log(mission+' Always Recover correction complete (existing physical approval recoveries reused)')

def forecast_phase(mission,data,source):
 from model.mission_evaluation import prediction
 from model.scg import load_scg_platform_params
 from underpass_inspection.storage import commit_record
 proto=protocol(mission,data,source);proto.update(stage='physical_feasibility_and_forecasts')
 path=stage_path(mission,FOLDERS[1]);run,digest,pdigest=open_stage(path,proto)
 wanted={(p,seed) for p in ('staleguard_recovery_aware','staleguard_dual_horizon') for seed,s,snap,path in data}
 have={(p,int(seed)) for p,cache in run['predictions_by_policy'].items() for seed in cache}
 if wanted-have:
  platform=load_scg_platform_params();seed,s,snap,p=data[0]
  began=time.perf_counter();prediction(mission,s,snap,platform);log(f'{mission} predictor warmup (excluded from latency): {time.perf_counter()-began:.2f}s')
  for seed,s,snap,p in data:
   results=[]
   for policy in ('staleguard_recovery_aware','staleguard_dual_horizon'):
    if (policy,seed) in have:f=run['predictions_by_policy'][policy][str(seed)]
    else:
     f=prediction(mission,s,snap,platform)
     commit_record(path,run,dict(policy=policy,seed=seed,forecast=f),pdigest,kind='prediction')
     run['predictions_by_policy'].setdefault(policy,{})[str(seed)]=f
    results.append(f['results'])
   assert results[0]==results[1]
   if seed%10==0:
    log(f'{mission} forecast seed {seed}; {run["completed_predictions"]}/{len(wanted)} cached')
    status(phase='prediction',mission=mission,forecasts=run['completed_predictions'],forecast_target=len(wanted))
 return path,run,digest

def pool_flights(jobs,path,run,digest,workers):
 from model.mission_evaluation import physical
 from underpass_inspection.storage import commit_record,write_progress
 done={key(r) for r in records(path,run)};todo=[j for j in jobs if j[0] not in done]
 run['status']='running';write_progress(path,run)
 with ProcessPoolExecutor(max_workers=workers,mp_context=get_context('spawn')) as pool:
  pending=deque();iterator=iter(todo)
  def submit():
   item=next(iterator,None)
   if item is not None:pending.append((item,pool.submit(physical,item[1])))
  for _ in range(2*workers):submit()
  count=0
  while pending:
   item,future=pending.popleft();k,job,extra=item
   for attempt in range(3):
    try:
     row=future.result();break
    except Exception:
     log(traceback.format_exc())
     if attempt==2:raise
     time.sleep(5);future=pool.submit(physical,job)
   row.update(extra)
   assert key(row)==k and k not in done
   commit_record(path,run,row,digest);done.add(k);count+=1;submit()
   if count%100==0:
    log(f'{path.parent.parent.parent.name}/{path.name}: {len(done)} cases, {count} new physical flights this invocation')
    status(phase=path.name,mission=job[0],completed=len(done),new_physical_this_stage=count,pending=len(todo)-count)
 return done

def physical_phase(mission,data,source,old,path,run,digest,workers):
 from underpass_inspection.storage import commit_record
 grid=run['protocol']['prediction_grid_s'];done={key(r) for r in records(path,run)};available={}
 for r in old:
  policy=r['event']['policy'];d=r['delay_s']
  if policy=='no_recovery' and round(d*50) in {round(x*50) for x in grid}:available[r['seed'],'command',round(d*50)]=r
  if mission=='geofence_turn' and policy=='always_recover' and round(d*50) in {round(x*50) for x in grid}:available[r['seed'],'recovery',round(d*50)]=r
  if mission=='inspection_descent' and policy=='always_recover' and d>0:available.setdefault((r['seed'],'recovery',0),r)
 expected=set();jobs=[]
 for seed,s,snap,p in data:
  for kind in ('command','recovery'):
   for d in grid:
    k=seed,kind,round(d*50);expected.add(k)
    if k in done:continue
    if k in available:
     row=reuse(available[k],kind=kind,delay_s=d,mission=mission)
     commit_record(path,run,row,digest);done.add(k)
    else:
     start=round(snap.approval_time*50)+round(d*50)
     jobs.append((k,(mission,seed,s,snap,d,(kind,start),None),dict(kind=kind)))
 pool_flights(jobs,path,run,digest,workers);finish(path,run,expected)
 log(f'{mission} independent feasibility complete; {len(expected)} physical outcome records, {len(jobs)} new candidate flights')

def legacy_forecast(source,seed):
 f=source['predictions'][str(seed)];p=f.get('prediction')
 if p is None:return f
 bad=0 if p['classification'] in ('INVALID_AT_APPROVAL','INVALID_NOMINAL') else None if p['first_bad_delay'] is None else round(p['first_bad_delay']*50)
 return dict(f,expiry_steps=bad)

def applied_action(row):
 e=row['event']
 if e['fallback_step'] is not None:return ('recovery',e['fallback_step'])
 if e['application_step'] is not None:return ('command',e['application_step'])
 return None

def policy_reuse(row,mission,delay,event,action,source_path,*,identical_planned_action=False):
 """Reuse physics only after independent policy prediction and action selection.

 The action completely fixes the reference thereafter in these latched guards.
 No prediction timing, decisions, or future-dispatch knowledge is shared.
 """
 assert (row.get('planned_action')==list(action)) if identical_planned_action else applied_action(row)==action
 r=reuse(row,mission=mission,delay_s=delay,event=dict(event))
 r.pop('kind',None)
 r['physical_source']['journal']=str(source_path.relative_to(ROOT))
 r['physical_source']['kind']=row.get('kind',row.get('mode'))
 r['physical_source']['action']=list(action)
 r['native_event']=row.get('native_event',row['event']).copy()
 for name in ('application_step','fallback_step'):r['event'][name]=row['event'][name]
 end=round((row.get('monitored_until_s',row['horizon_s']) if not row['safe'] else row['horizon_s'])*50)
 r['event']['events']=[e for e in event.get('events',[]) if e['control_step']<=end]
 a=event['scheduled_dispatch_step']-round(delay*50)
 r['approval_step']=a
 r['event']['early_recovery']=r['event']['fallback_step'] is not None and r['event']['fallback_step']<event['scheduled_dispatch_step']
 r['pre_dispatch_violation_s']=row['first_violation_s'] if row['first_violation_s'] is not None and row['first_violation_s']<=event['scheduled_dispatch_step']/50 else None
 r['source_runtime_s']=row.get('runtime_s');r['runtime_s']=None
 if 'max_arc_tracking_error_m' in r:
  r['source_max_arc_tracking_error_m']=r['max_arc_tracking_error_m'];r['max_arc_tracking_error_m']=None
 r['reuse_reason']='Same frozen scenario, initial/PID state, monitoring, horizon, and latched physical action/start; independent policy forecast and timing retained'
 r['planned_action']=list(action)
 return r

def benchmark_phase(mission,data,source,workers):
 from underpass_inspection.storage import read_run,commit_record
 from model.mission_evaluation import clock_action
 forecasts=read_run(stage_path(mission,FOLDERS[1]))['predictions_by_policy']
 proto=protocol(mission,data,source);proto.update(stage='missing_policies',policies=list(NEW[mission]),
  forecasts_sha256=sha(stage_path(mission,FOLDERS[1])/'predictions.jsonl'))
 path=stage_path(mission,FOLDERS[2]);run,digest,_=open_stage(path,proto)
 # The independent feasibility sweep already measured most possible actions.
 # Only actual command/recovery starts enter this index; no inferred labels.
 reference_path=stage_path(mission,FOLDERS[1]);reference_run=read_run(reference_path)
 physical_index={}
 old_path=ROOT/mission/'results/evaluation/baseline_benchmark'
 _,old_rows=original(mission)
 for row in old_rows:
  if row['event']['policy'] not in ('no_recovery','always_recover'):continue
  action=applied_action(row)
  if action is not None:physical_index.setdefault((row['seed'],*action),(row,old_path,False))
 for row in records(reference_path,reference_run):
  action=applied_action(row)
  if action is not None:physical_index.setdefault((row['seed'],*action),(row,reference_path,False))
 previous=records(path,run);done={key(r) for r in previous}
 for row in previous:
  action=row.get('planned_action') or applied_action(row)
  if action is not None:physical_index.setdefault((row['seed'],*action),(row,path,'planned_action' in row))
 jobs=[];expected=set();pending={};reused=0
 for seed,s,snap,p in data:
  for delay in proto['dispatch_grid_s']:
   for policy in NEW[mission]:
    k=seed,policy,round(delay*50);expected.add(k)
    if k in done:continue
    f=forecasts[policy][str(seed)] if policy in forecasts else legacy_forecast(source,seed) if policy=='staleguard_early_recovery' else None
    action,event=clock_action(policy,snap,delay,f,horizon_steps=210 if mission=='inspection_descent' else snap.horizon_steps)
    physical_key=(seed,*action)
    if physical_key in physical_index:
     row,origin,planned=physical_index[physical_key]
     commit_record(path,run,policy_reuse(row,mission,delay,event,action,origin,identical_planned_action=planned),digest);done.add(k);reused+=1
    else:
     if physical_key not in pending:
      jobs.append((k,(mission,seed,s,snap,delay,action,event),dict(planned_action=list(action))))
     pending.setdefault(physical_key,[]).append((k,delay,event,action))
 log(f'{mission}: {reused} policy cases reuse measured identical actions; {len(jobs)} distinct missing physical trajectories')
 pool_flights(jobs,path,run,digest,workers)
 completed={key(r):r for r in records(path,run)}
 for group in pending.values():
  representative=completed[group[0][0]]
  for k,delay,event,action in group[1:]:
   if k in completed:continue
   row=policy_reuse(representative,mission,delay,event,action,path,identical_planned_action=True)
   commit_record(path,run,row,digest);completed[k]=row
 finish(path,run,expected);log(mission+' missing policy matrix complete')

def reuse_smoke(bundles):
 """Compare equivalence reuse with four fresh native physical continuations."""
 from model.mission_evaluation import prediction,physical,clock_action
 from model.scg import load_scg_platform_params
 from underpass_inspection.storage import read_run
 marker=OUT/'reuse_smoke_validation.json'
 signature=hashlib.sha256(__import__('inspect').getsource(policy_reuse).encode()).hexdigest()
 if marker.exists():
  previous=json.loads(marker.read_text())
  if previous.get('passed') and previous.get('reuse_function_sha256')==signature and compatible(previous.get('provenance',{}),provenance()):return
 platform=load_scg_platform_params();checks=[]
 for mission,(data,source,old) in bundles.items():
  reference_path=stage_path(mission,FOLDERS[1]);available=[(r,ROOT/mission/'results/evaluation/baseline_benchmark') for r in old if r['event']['policy'] in ('no_recovery','always_recover')]
  if (reference_path/'progress.json').exists():available += [(r,reference_path) for r in records(reference_path,read_run(reference_path))]
  for seed,s,snap,p in data[:2]:
   f=prediction(mission,s,snap,platform)
   for delay in (.2,.6,1.,0.):
    action,event=clock_action('staleguard_dual_horizon',snap,delay,f,horizon_steps=210 if mission=='inspection_descent' else snap.horizon_steps)
    match=next(((r,path) for r,path in available if r['seed']==seed and applied_action(r)==action),None)
    if match is not None:break
   assert match is not None
   base,origin=match;reused=policy_reuse(base,mission,delay,event,action,origin)
   actual=physical((mission,seed,s,snap,delay,action,event))
   for field in ('safe','completed','safe_completion'):assert reused[field]==actual[field]
   for field in ('application_step','fallback_step','computation_s'):assert reused['event'][field]==actual['event'][field]
   if 'physics_sha256' in actual:assert reused['physics_sha256']==actual['physics_sha256']
   if 'min_clearance_m' in actual:assert abs(reused['min_clearance_m']-actual['min_clearance_m'])<1e-12
   for field in ('final_position','final_velocity'):
    if field in actual:
     import numpy as np
     np.testing.assert_allclose(reused[field],actual[field],rtol=0,atol=1e-12)
   checks.append(dict(mission=mission,seed=seed,delay_s=delay,action=action,physical_outcome_and_event_match=True))
 save(marker,dict(passed=True,reuse_function_sha256=signature,provenance=provenance(),physical_validation_flights=len(checks),checks=checks))
 log('Physical trajectory reuse smoke passed: '+str(len(checks))+' flights')

def smoke():
 from model.mission_evaluation import prediction,physical,clock_action
 from model.scg import load_scg_platform_params
 from underpass_inspection.storage import read_run
 path=OUT/'smoke_validation.json'
 if path.exists():
  old=json.loads(path.read_text())
  if old.get('passed') and compatible(old.get('provenance',{}),provenance()):return
 initial_provenance=provenance()
 if path.exists():
  import shutil
  old_sha=sha(path)[:12];shutil.copyfile(path,OUT/f'smoke_validation_{old_sha}.json')
 platform=load_scg_platform_params();flights=0;checks=[]
 for mission in MISSIONS:
  data,source,old=inputs(mission);by={(r['seed'],r['event']['policy'],round(r['delay_s']*50)):r for r in old}
  for seed,s,snap,p in data[:2]:
   # Compile/warm before measured tests; full vs no-early-exit equivalence.
   prediction(mission,s,snap,platform)
   f=prediction(mission,s,snap,platform,diagnostics=True)
   ref=prediction(mission,s,snap,platform,early_exit=False,diagnostics=True)
   assert f['results']==ref['results']
   for x,y in zip(f['diagnostics'],ref['diagnostics']):
    if f['results'][f['grid_s'].index(x['delay_s'])]['recovery_safe' if x['recovery'] else 'command_safe']:
     import numpy as np
     np.testing.assert_array_equal(x['state'],y['state']);np.testing.assert_array_equal(x['memory'],y['memory'])
   for d in (0.,.2,1.):
    for policy in NEW[mission]:
     received=f if policy in ('staleguard_recovery_aware','staleguard_dual_horizon') else legacy_forecast(source,seed) if policy=='staleguard_early_recovery' else None
     action,event=clock_action(policy,snap,d,received,horizon_steps=210 if mission=='inspection_descent' else snap.horizon_steps)
     row=physical((mission,seed,s,snap,d,action,event));assert row['event']['policy']==policy;flights+=1
   # Native-command replay proves the new optional hook does not alter dynamics.
   row=physical((mission,seed,s,snap,0.,('command',round(snap.approval_time*50)),None));flights+=1
   baseline=by[seed,'no_recovery',0]
   assert (row['safe'],row['safe_completion'])==(baseline['safe'],baseline['safe_completion'])
   if 'physics_sha256' in baseline:assert row['physics_sha256']==baseline['physics_sha256']
   if 'min_clearance_m' in baseline:assert abs(row['min_clearance_m']-baseline['min_clearance_m'])<1e-12
   # Standardized Always Recover, including D=0, cannot dispatch inspection later.
   a,e=clock_action('always_recover',snap,.2,horizon_steps=210 if mission=='inspection_descent' else snap.horizon_steps)
   row=physical((mission,seed,s,snap,.2,a,e));flights+=1
   b=by[seed,'always_recover',5 if mission=='inspection_descent' else 0]
   assert (row['safe'],row['safe_completion'])==(b['safe'],b['safe_completion'])
   if 'physics_sha256' in b:assert row['physics_sha256']==b['physics_sha256']
   if 'min_clearance_m' in b:assert abs(row['min_clearance_m']-b['min_clearance_m'])<1e-12
   assert row['event']['application_step'] is None
   for native_delay in (0.,.2):
    native=physical((mission,seed,s,snap,native_delay,None,None));flights+=1
    assert native['event']['fallback_step']==round(snap.approval_time*50)
    assert native['event']['application_step'] is None
    assert (native['safe'],native['safe_completion'])==(b['safe'],b['safe_completion'])
    if 'physics_sha256' in b:assert native['physics_sha256']==b['physics_sha256']
    if 'min_clearance_m' in b:assert abs(native['min_clearance_m']-b['min_clearance_m'])<1e-12
   checks.append(dict(mission=mission,seed=seed,grid=len(f['results']),clock_policy_and_reference_replay=True))
 assert provenance()==initial_provenance
 save(path,dict(passed=True,provenance=initial_provenance,physical_validation_flights=flights,checks=checks));log('Physical smoke tests passed: '+str(flights)+' flights')

def aggregate():
 sys.path.insert(0,str(ROOT/'tables'))
 from aggregate_results import aggregate as run
 return run(OUT)

def execute(workers):
 from contextlib import ExitStack
 OUT.mkdir(parents=True,exist_ok=True)
 with (OUT/'.lock').open('a') as lock,ExitStack() as stack:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  for p in (ROOT/'inspection_descent/support/.simulation.lock',ROOT/'geofence_turn/support/logs/.simulation.lock'):
   p.parent.mkdir(parents=True,exist_ok=True);f=stack.enter_context(p.open('a'));fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
  previous_errors=OUT/'errors.json'
  if previous_errors.exists() and json.loads(previous_errors.read_text()):
   save(OUT/('errors_previous_'+sha(previous_errors)[:12]+'.json'),json.loads(previous_errors.read_text()))
  save(previous_errors,[])
  status(status='RUNNING',pid=os.getpid(),phase='preflight',errors=[],last_error=None)
  save(OUT/'process.json',dict(pid=os.getpid(),session=SESSION,hostname=platform.node(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
   pid1=Path('/proc/1/comm').read_text().strip(),cwd=str(ROOT),python=sys.executable))
  manifest_path=OUT/'run.json'
  if not manifest_path.exists():
   save(manifest_path,dict(format='three_mission_table_completion_v1',provenance=provenance(),
    coordinator_sha256=sha(__file__),aggregator_sha256=sha(ROOT/'tables/aggregate_results.py'),
    expected_new_policy_flights=25944,expected_new_feasibility_flights=2760,
    underpass='reuse completed 400-scenario benchmark and validation',
    always_recover='cancel at approval for all D, including D=0; reuse equivalent physical recoveries'))
  errors=[]
  try:
   subprocess.run([sys.executable,'-m','unittest','tables_test_complete'],cwd=ROOT/'tables',check=True)
   smoke();bundles={m:inputs(m) for m in MISSIONS};reuse_smoke(bundles)
   for mission,(data,source,old) in bundles.items():corrected(mission,data,source,old)
   ready=[]
   for mission,(data,source,old) in bundles.items():
    try:
     status(phase='feasibility',mission=mission)
     path,run,digest=forecast_phase(mission,data,source)
     physical_phase(mission,data,source,old,path,run,digest,workers);ready.append(mission)
    except Exception:
     error=dict(mission=mission,phase='feasibility',traceback=traceback.format_exc());errors.append(error);log(error['traceback'])
     save(OUT/'errors.json',errors)
   aggregate()  # Publish completed Table I before the longer policy matrix.
   for mission in ready:
    data,source,old=bundles[mission]
    try:benchmark_phase(mission,data,source,workers)
    except Exception:
     error=dict(mission=mission,phase='policy_benchmark',traceback=traceback.format_exc());errors.append(error);log(error['traceback']);save(OUT/'errors.json',errors)
   result=aggregate()
   status(status='COMPLETE' if result and not errors else 'PARTIALLY COMPLETE',phase='finished',errors=errors)
  except BaseException:
   message=traceback.format_exc();log(message);status(status='BLOCKED',phase='exception',last_error=message)
   try:aggregate()
   except Exception:log(traceback.format_exc())
   raise

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 group=parser.add_mutually_exclusive_group();group.add_argument('--launch',action='store_true');group.add_argument('--run',action='store_true');group.add_argument('--status',action='store_true');group.add_argument('--metrics-only',action='store_true');group.add_argument('--smoke',action='store_true')
 parser.add_argument('--workers',type=int,default=4);a=parser.parse_args();os.chdir(ROOT);OUT.mkdir(parents=True,exist_ok=True)
 if a.status:
  p=OUT/'status.json';current=json.loads(p.read_text()) if p.exists() else {}
  print(json.dumps(current,indent=2) if current else 'NOT STARTED')
  if current.get('status')=='RUNNING':
   active=subprocess.run(['tmux','has-session','-t',SESSION],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
   print('Persistent terminal session: '+('running' if active else 'absent; inspect logs and resume'))
  return
 if a.metrics_only:aggregate();return
 if a.smoke:smoke();return
 if a.run:execute(a.workers);return
 if subprocess.run(['tmux','has-session','-t',SESSION],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:
  print('Already running in tmux '+SESSION);return
 command=shlex.join([sys.executable,'-u',str(Path(__file__).resolve()),'--run','--workers',str(a.workers)])+' >> '+shlex.quote(str(OUT/'execution.log'))+' 2>&1'
 subprocess.run(['tmux','new-session','-d','-s',SESSION,'-c',str(ROOT),command],check=True)
 time.sleep(2)
 subprocess.run(['tmux','has-session','-t',SESSION],check=True)
 print('Verified persistent tmux session:',SESSION);print('Log:',OUT/'execution.log')
 print((OUT/'status.json').read_text() if (OUT/'status.json').exists() else 'Worker starting; inspect execution.log')

if __name__=='__main__':main()
