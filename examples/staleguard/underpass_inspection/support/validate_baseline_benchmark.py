"""Independent audit of one self-contained benchmark; no model or physical flights."""
from dataclasses import asdict
from itertools import groupby
import hashlib
import json
import math
import zipfile
from pathlib import Path
import numpy as np
from underpass_inspection.storage import (read_run,load_inputs,prediction_for,flight_key,canonical,file_hash)
from underpass_inspection.baselines.policies import POLICIES,APPROVAL_POLICIES,HORIZON_POLICIES,make_policy
from underpass_inspection.baselines.staleguard import first_ready_step,TTL_GRID
from underpass_inspection.baselines import fbs
from underpass_inspection.baseline_benchmark.metrics import journal_metrics
from underpass_inspection.storage import iter_committed
from underpass_inspection.scenario import BODY_RADIUS_M,displacement


def check_decision(row,scenario,forecast):
    """Replay recorded information/timing through the actual policy, never a future schedule."""
    e=row['event'];policy=e['policy'];approval=row['approval_step'];dispatch=e['scheduled_dispatch_step']
    if policy=='use_time_recheck':
        assert e['check_started_step']==dispatch and e['prediction_ready_at_dispatch'] is False
        assert e['decision_step']==dispatch+math.floor(e['computation_s']*50)+1
        assert row['initiation_step']==e['decision_step']
        assert row['mode']==('dive' if e['check']['safe'] else 'sideways')
        return
    agent=make_policy(policy,scenario,forecast)
    if policy=='fbs':
        volatility=0.;previous=None
        for feature in e['features_m']:
            volatility=fbs.update_volatility(feature,previous,volatility);previous=feature
        assert fbs.freeze_prediction(previous,volatility)==e['prediction']
        assert len(e['features_m'])==approval+1
        agent.event.update(prediction=e['prediction'],computation_s=e['computation_s'])
    agent.approve();action=None;started=None
    for step in range(approval,dispatch+1):
        agent.announce_readiness(step)
        if step==dispatch:agent.dispatch(step,None,None)
        agent.tick(step);value=agent.take_action()
        if value is not None:
            assert action is None;action=value;started=step
    assert (action,started)==(row['mode'],row['initiation_step'])
    assert agent.event['application_step']==e['application_step'] and agent.event['fallback_step']==e['fallback_step']


def validate(folder):
    folder=Path(folder);run=read_run(folder);protocol=run['protocol'];cases=load_inputs(folder,run)
    if run.get('source_archive_sha256'):
        assert file_hash(folder/'source_versions.zip')==run['source_archive_sha256']
        with zipfile.ZipFile(folder/'source_versions.zip') as archive:assert archive.testzip() is None
    wanted={s['seed']:s for s in protocol['scenarios']}
    assert len(wanted)==len(cases) and len(set(protocol['policies']))==len(protocol['policies'])
    assert set(protocol['policies'])<=set(POLICIES) and protocol['ttl_grid_s']==list(TTL_GRID)
    sampling=protocol.get('scenario_sampling')
    if sampling and 'excluded_seeds' in sampling:
        ids=sorted(set(sampling['population_seeds'])-set(sampling['excluded_seeds']))
        chosen=sorted(np.random.default_rng(sampling['rng_seed']).choice(ids,sampling['sample_count'],replace=False).tolist())
        assert chosen==sorted(wanted) and not set(chosen)&set(sampling['excluded_seeds'])
        assert len(chosen)==400 and len(ids)==765 and len(sampling['excluded_seeds'])==100
        assert sampling['selection_uses_policy_outcomes'] is False
    expected=len(wanted)*len(protocol['delays'])*len(protocol['policies'])
    if run['status']=='prepared':
        assert run['completed_flights']==run['completed_predictions']==0
        return dict(passed=True,status='prepared',scenarios=len(wanted),expected_flights=expected,actual_flights=0,
                    disjoint_from_historical=bool(sampling and 'excluded_seeds' in sampling),inputs_verified=True)
    assert run['status']=='complete'
    indexed={s.seed:(s,snap) for s,snap,_ in cases};seen=set();count=0;representative={}
    rep=folder/'representative_trajectories.npz';rep_keys=set()
    if rep.exists():
        with np.load(rep) as archive:
            for key in archive.files:
                if key.endswith('__physics'):
                    seed,delay,policy=key[:-len('__physics')].split('_',2)
                    rep_keys.add((int(seed[4:]),int(delay)/50,policy))
    for seed,group in groupby(iter_committed(folder,run),key=lambda r:r['seed']):
        assert seed in wanted and seed not in seen;seen.add(seed);rows=list(group)
        s,snapshot=indexed[seed];assert canonical(asdict(s))==wanted[seed]
        lookup={flight_key(r):r for r in rows}
        keys={(seed,d,p) for d in protocol['delays'] for p in protocol['policies']}
        assert len(rows)==len(lookup) and set(lookup)==keys
        count+=len(rows);physical={}
        for r in rows:
            e=r['event'];policy=e['policy'];approval=round(s.approval_s*50)
            dispatch=approval+round(r['delay_s']*50);total=round((s.approval_s+s.observation_s)*50)
            assert e['scheduled_dispatch_step']==dispatch==e['dispatch_observed_step']
            assert r['approval_step']==approval and r['horizon_step']==total
            assert r['physics_samples']==20*total+1 and r['control_samples']==total+1
            assert r['safe']==(r['first_violation'] is None)==(r['min_clearance_m']>=s.clearance_m)
            assert r['safe_completion']==(r['safe'] and r['completed'])
            np.testing.assert_array_equal(r['approval_state'],snapshot.state)
            assert r['approval_snapshot_verified'] and r['geometry_unchanged'] and r['max_physics_position_increment_m']<.01
            actions=[v for v in (e['application_step'],e['fallback_step']) if v is not None]
            assert len(actions)==1 and r['initiation_step']==actions[0]
            assert r['pid_unchanged_at_initiation'] and r['physical_state_unchanged_at_initiation']
            assert r['mode'] in ('dive','sideways') and r['final_position_m'][1]<r['maneuver_anchor_m'][1]
            key=r['mode'],r['start_s'];outcome=(r['physics_sha256'],r['safe'],r['safe_completion'],r['min_clearance_m'])
            if key in physical:assert physical[key]==outcome
            physical[key]=outcome
            duration=s.swerve_start_s+s.swerve_s if r['mode']=='dive' else s.sideways_s
            assert r['maneuver_profile_end_approval_s']==r['start_s']+duration
            assert r['maneuver_profile_truncated_by_horizon']==(r['start_s']+duration>s.observation_s)
            assert r['inspection_observation']==('completed' if r['completed'] else 'abandoned_by_recovery' if e['command_canceled'] else 'not_observed_by_fixed_deadline')
            if e['fallback_step'] is not None:
                assert e['command_canceled'] and e['command_rejected'] and e['application_step'] is None
            assert e['early_recovery']==(e['fallback_step'] is not None and e['fallback_step']<dispatch)
            times=[v['flight_s'] for v in e['events']]
            assert all(a<=b+1e-12 for a,b in zip(times,times[1:]))
            assert sum(v['kind']=='approval' for v in e['events'])==1
            assert sum(v['kind']=='command_dispatch' for v in e['events'])==1
            assert sum(v['kind'] in ('command_started','recovery_started') for v in e['events'])==1
            assert e['events'][-1]['kind']=='observation_horizon_end'
            bad=[v for v in e['events'] if v['kind']=='safety_violation']
            assert len(bad)==int(r['first_violation'] is not None)
            if bad:assert bad[0]['physics_tick']==r['first_violation']['physics_tick']
            assert r['violation_before_intervention']==(r['first_violation'] is not None and r['first_violation']['time_flight_s']<=r['initiation_step']/50)
            forecast=prediction_for(run,policy,seed)
            if policy in APPROVAL_POLICIES:
                assert forecast is not None
                assert e['computation_s']==forecast['computation_s'] and e['expiry_steps']==forecast['expiry_steps']
                ready=approval+first_ready_step(forecast['computation_s'])
                assert e['prediction_ready_control_step']==ready
                if ready<total:assert e['prediction_ready_step']==ready
                if e['fallback_step'] is not None:assert e['fallback_step']>=ready
                assert e['prediction_available_before_dispatch']==(forecast['computation_s']<=r['delay_s'])
            check_decision(r,s,forecast)
            if flight_key(r) in rep_keys:representative[flight_key(r)]=r
    assert seen==set(wanted) and count==expected==run['completed_flights']
    checked=[]
    if rep.exists():
        with np.load(rep) as archive:
            for seed,delay,policy in sorted(rep_keys):
                prefix=f'seed{seed}_{round(delay*50):03d}_{policy}';r=representative[seed,delay,policy];s=indexed[seed][0]
                raw=archive[prefix+'__physics'];t=raw[:,0];positions=raw[:,1:]
                if policy=='use_time_recheck':
                    np.testing.assert_array_equal(r['event']['check_state'],archive[prefix+'__control'][r['event']['scheduled_dispatch_step'],1:])
                if policy=='fbs':
                    from underpass_inspection.baselines.fbs import safety_feature
                    history=archive[prefix+'__control'][:r['approval_step']+1,1:]
                    np.testing.assert_array_equal(r['event']['features_m'],[safety_feature(state,s) for state in history])
                np.testing.assert_array_equal(t,np.arange(len(t))/1000)
                assert len(t)==r['physics_samples'] and hashlib.sha256(raw.tobytes()).hexdigest()==r['physics_sha256']
                distances=[]
                for box in s.obstacles:
                    lo,hi=np.array(box.lower),np.array(box.upper);q=np.abs(positions-(lo+hi)/2)-(hi-lo)/2
                    distances.append(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0)-BODY_RADIUS_M)
                distances=np.array(distances);np.testing.assert_allclose(distances.min(),r['min_clearance_m'],atol=1e-12,rtol=0)
                bad=np.flatnonzero(distances.min(axis=0)<s.clearance_m);assert r['safe']==(len(bad)==0)
                if len(bad):assert bad[0]==r['first_violation']['physics_tick']
                mask=(t>=s.approval_s)&np.all(positions>=s.inspection_lower,axis=1)&np.all(positions<=s.inspection_upper,axis=1)
                edges=np.diff(np.r_[False,mask,False].astype(int));dwell=max(np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)-1,default=0)
                assert r['completed']==(dwell>=round(s.inspection_dwell_s*1000))
                step=r['initiation_step'];ref=archive[prefix+'__reference'];dp,_=displacement(np.arange(len(ref)-step)/50,s,r['mode'])
                np.testing.assert_allclose(ref[step:,1:4]-r['maneuver_anchor_m'],dp,atol=1e-12,rtol=0);checked.append(prefix)
    assert journal_metrics(folder,run)==json.loads((folder/'metrics.json').read_text())
    return dict(passed=True,status='complete',flights=count,scenarios=len(wanted),no_duplicates=True,
        full_horizon_1khz=True,approval_inputs_identical=True,decisions_replayed=True,
        policy_runtime_and_cancellation_verified=True,physical_replay_hash_consistency=True,
        representative_physics_independently_checked=len(checked),metrics_recomputed_identically=True)
