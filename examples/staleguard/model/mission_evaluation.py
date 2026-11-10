"""Small adapters to frozen inputs and the existing two physical engines.

Clock-only guards run the exact Underpass state machine. Only dispatch EVENTS,
never future dispatch times, reach a guard. Physical engines replay the resulting
command/recovery start; no outcome is assembled from alternative trajectories.
"""
from pathlib import Path
from types import SimpleNamespace
import importlib,json,hashlib,time
import numpy as np
from model.scg import load_scg_platform_params
from underpass_inspection.baselines.policies import make_policy
ROOT=Path(__file__).resolve().parents[1]
HORIZON=('staleguard_recovery_aware','staleguard_dual_horizon')


def cases(mission):
    path=ROOT/mission/'results/evaluation/calibration/calibrated_scenarios.json'
    if mission=='geofence_turn':
        from geofence_turn.storage import load_calibrated_scenarios
        return [(s.seed,s,snap,p) for s,snap,p in load_calibrated_scenarios(path)]
    from inspection_descent import storage
    from inspection_descent.scenario import Scenario,configuration,make_forward_trajectory,MEASURE_SECONDS
    raw=json.loads(path.read_text());assert raw['config']==configuration() and raw['dataset_kind']=='evaluation'
    # These two intentionally changed sources implement the authorized policy
    # correction and diagnostic start hook; every other scientific source stays pinned.
    allowed={'inspection_descent/simulation.py','inspection_descent/baselines/always_recover.py'}
    moves=json.loads((ROOT/'source_relocation.json').read_text())['sources']
    for name,sha in raw['scientific_sources'].items():
        if name in ('simulation.py','baselines/always_recover.py'):continue
        try:storage.check_source(storage.ROOT/name,sha)
        except ValueError:
            key=str((storage.ROOT/name).resolve().relative_to(ROOT))
            move=moves.get(key)
            if not move or move['sha256']!=sha:raise
            if not set(move['canonical']) & allowed:raise
            for canonical,expected_sha in move['canonical'].items():
                if canonical not in allowed:
                    assert hashlib.sha256((ROOT/canonical).read_bytes()).hexdigest()==expected_sha

    result=[]
    for r in raw['scenarios']:
        p=path.parent/r['approval_snapshot'];assert hashlib.sha256(p.read_bytes()).hexdigest()==r['snapshot_sha256']
        s=Scenario(**r['scenario']);snap=storage.load_pickle(p);a=round(s.approval_time*50)
        expected=make_forward_trajectory(SimpleNamespace(CTRL_FREQ=50,X_GOAL=np.zeros((a+len(snap.remaining_reference),12))),s)[a:]
        np.testing.assert_array_equal(snap.remaining_reference,expected)
        assert snap.approval_time==s.approval_time and snap.horizon_steps==round(MEASURE_SECONDS*50)-a
        assert snap.low_steps==round(s.low_seconds*50)
        for name in ('low_height','box_x_min','box_x_max','box_y_limit','unsafe_below_z'):assert getattr(snap,name)==getattr(s,name)
        result.append((r['seed'],s,snap,p))
    assert len(result)==raw['statistics']['accepted']==276
    return sorted(result,key=lambda r:r[0])


def prediction(mission,s,snap,platform,**kwargs):
    module=importlib.import_module(mission+'.baselines.recovery_horizons')
    began=time.perf_counter();wall=time.time()
    result=module.predict(s,snap,platform,**kwargs)
    result.update(computation_s=time.perf_counter()-began,wall_started_unix_s=wall,wall_finished_unix_s=time.time())
    return result


def clock_action(policy,snapshot,delay,forecast=None,horizon_steps=None):
    """Simulator schedules events; the unchanged guard sees only elapsed clock."""
    approval=round(snapshot.approval_time*50);dispatch=approval+round(delay*50)
    scenario=SimpleNamespace(approval_s=snapshot.approval_time)
    agent=make_policy(policy,scenario,forecast);agent.approve();selected=None
    horizon=approval+(snapshot.horizon_steps if horizon_steps is None else horizon_steps)
    for step in range(approval,horizon):
        agent.announce_readiness(step)
        if step==dispatch:agent.dispatch(step,None,None)
        agent.tick(step);action=agent.take_action()
        if action is not None:
            assert selected is None
            selected=('recovery' if action=='sideways' else 'command',step)
    assert selected is not None
    return selected,agent.finish(dispatch,horizon)


def physical(job):
    mission,seed,s,snapshot,delay,action,event=job
    import gc,ctypes
    gc.collect();ctypes.CDLL(None).malloc_trim(0)
    if mission=='geofence_turn':
        from geofence_turn.simulation import execute
        from geofence_turn.storage import check_approval_snapshot
        row,actual,_=execute(s,delay=delay,policy='always_recover' if action is None else 'no_recovery',forced_action=action)
        check_approval_snapshot(actual,snapshot)
    else:
        from inspection_descent.simulation import execute
        row,actual,_=execute(s,delay=delay,policy='always_recover' if action is None else 'no_recovery',snapshot=snapshot,seed=seed,forced_action=action)
    native=row['event'].copy()
    if event is not None:
        row['event']=dict(event)
        # Physical early termination cannot be erased by precomputed timer logs.
        for key in ('application_step','fallback_step'):row['event'][key]=native.get(key)
        end=round(row['horizon_s']*50) if row['safe'] else round(row.get('monitored_until_s',row['horizon_s'])*50)
        row['event']['events']=[e for e in event.get('events',[]) if e['control_step']<=end]
        row['event']['early_recovery']=native.get('fallback_step') is not None and native['fallback_step']<native['scheduled_dispatch_step']
        row['native_event']=native
    a=round(snapshot.approval_time*50);start=native.get('fallback_step') if native.get('fallback_step') is not None else native.get('application_step')
    row.update(mission=mission,seed=seed,delay_s=round(delay,2),approval_step=a,
        start_s=None if start is None else (start-a)/50,mode='recovery' if native.get('fallback_step') is not None else 'command',
        execution_kind='new_physical_flight',approval_snapshot_verified=True,
        observation_horizon_s=4.2 if mission=='inspection_descent' else s.deadline_s)
    assert row['safe_completion']==(row['safe'] and row['completed'])
    return row
