"""Stream-audit completed actual policy journals; never execute a physical flight."""
from pathlib import Path
import sys
if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
from dataclasses import asdict
from itertools import groupby
import hashlib
import json
import math
import numpy as np
from underpass_inspection.storage import (ROOT,load_calibrated_scenarios,load_ground_truth,
    iter_committed,committed_hasher,check_provenance,verify_physical_reference,canonical,write_json,flight_key,file_hash,
    historical_result_matches)
from underpass_inspection.scenario import BODY_RADIUS_M,displacement
from underpass_inspection.baselines.policies import APPROVAL_POLICIES
from underpass_inspection.baselines.staleguard import first_ready_step
from underpass_inspection.baseline_benchmark.metrics import journal_metrics
from underpass_inspection.baseline_benchmark.run import verify_freeze,reduced_cases,protocol_hash


def validate(folder,calibration,ground_truth=None,prior_hashes=None):
    folder=Path(folder);run=json.loads((folder/'run.json').read_text());protocol=run['protocol']
    check_provenance(run['provenance']);committed_hasher(folder,run)
    assert run['status']=='complete'
    if run.get('freeze_path'):
        assert file_hash(run['freeze_path'])==run['freeze_sha256']
        verify_freeze(run['freeze_path'],protocol,protocol['stage']=='physical_smoke')
    calibration=Path(calibration);path=calibration/'calibrated_scenarios.json' if calibration.is_dir() else calibration
    assert file_hash(path)==protocol['dataset_sha256']
    data=json.loads(path.read_text());entries={e['seed']:e for e in data['scenarios']}
    population=load_calibrated_scenarios(calibration)
    cases={s.seed:(s,snapshot) for s,snapshot,_ in population}
    sampling=protocol.get('scenario_sampling')
    if sampling is not None:
        chosen=[s.seed for s,_,_ in reduced_cases(population)]
        assert chosen==sampling['selected_seeds'] and len(chosen)==100
        assert sampling['rng_seed']==20261108 and sampling['selection_uses_policy_outcomes'] is False
        assert sampling['population_ids_sha256']==protocol_hash([s.seed for s,_,_ in population])
        assert [s['seed'] for s in protocol['scenarios']]==(chosen[:2] if protocol['stage']=='physical_smoke' else chosen)
    truth,_=load_ground_truth(ground_truth) if ground_truth else (None,None)
    wanted={s['seed']:s for s in protocol['scenarios']};seen=set();count=0
    no_recovery_matches=always_matches=paired_admissions=native_references=0
    representative={};rep=folder/'representative_trajectories.npz';rep_keys=set()
    if rep.exists():
        with np.load(rep) as archive:
            for key in archive.files:
                if key.endswith('__physics'):
                    seed,delay,policy=key[:-len('__physics')].split('_',2)
                    rep_keys.add((int(seed[4:]),int(delay)/50,policy))
    for seed,group in groupby(iter_committed(folder,run),key=lambda r:r['seed']):
        assert seed in wanted and seed not in seen;seen.add(seed);rows=list(group)
        s,snapshot=cases[seed];assert canonical(asdict(s))==wanted[seed]
        lookup={flight_key(r):r for r in rows}
        expected={(seed,d,p) for d in protocol['delays'] for p in protocol['policies']}
        assert len(rows)==len(lookup) and set(lookup)==expected
        physical={};count+=len(rows)
        for r in rows:
            e=r['event'];p=e['policy'];approval=round(s.approval_s*50)
            dispatch=approval+round(r['delay_s']*50);total=round((s.approval_s+s.observation_s)*50)
            assert e['scheduled_dispatch_step']==dispatch==e['dispatch_observed_step']
            assert r['approval_step']==approval and r['horizon_step']==total
            assert r['physics_samples']==total*20+1 and r['control_samples']==total+1
            assert r['safe']==(r['first_violation'] is None)==(r['min_clearance_m']>=s.clearance_m)
            assert r['safe_completion']==(r['safe'] and r['completed'])
            np.testing.assert_array_equal(r['approval_state'],snapshot.state)
            assert r['approval_snapshot_verified'] and r['geometry_unchanged'] and r['max_physics_position_increment_m']<.01
            actions=[v for v in (e['application_step'],e['fallback_step']) if v is not None]
            assert len(actions)==1 and r['initiation_step']==actions[0]
            assert r['pid_unchanged_at_initiation'] and r['physical_state_unchanged_at_initiation']
            assert r['final_position_m'][1]<r['maneuver_anchor_m'][1] and r['mode'] in ('dive','sideways')
            key=r['mode'],r['start_s'];value=(r['physics_sha256'],r['safe'],r['safe_completion'],r['min_clearance_m'])
            if key in physical:assert physical[key]==value
            physical[key]=value
            if protocol.get('protocol_version')=='dispatch_2s_v2':
                duration=s.swerve_start_s+s.swerve_s if r['mode']=='dive' else s.sideways_s
                assert r['maneuver_profile_end_approval_s']==r['start_s']+duration
                assert r['maneuver_profile_truncated_by_horizon']==(r['start_s']+duration>s.observation_s)
                assert r['inspection_observation']==('completed' if r['completed'] else
                    'abandoned_by_recovery' if e['command_canceled'] else 'not_observed_by_fixed_deadline')
            if e['fallback_step'] is not None:
                assert e['command_canceled'] and e['command_rejected'] and r['mode']=='sideways' and e['application_step'] is None
            assert e['early_recovery']==(e['fallback_step'] is not None and e['fallback_step']<dispatch)
            events=e['events'];times=[v['flight_s'] for v in events]
            assert all(a<=b+1e-12 for a,b in zip(times,times[1:]))
            assert sum(v['kind']=='approval' for v in events)==1
            assert sum(v['kind']=='command_dispatch' for v in events)==1
            assert sum(v['kind'] in ('command_started','recovery_started') for v in events)==1
            assert events[-1]['kind']=='observation_horizon_end'
            if r['first_violation']:
                bad=[v for v in events if v['kind']=='safety_violation']
                assert len(bad)==1 and bad[0]['physics_tick']==r['first_violation']['physics_tick']
            assert r['violation_before_intervention']==(r['first_violation'] is not None and
                r['first_violation']['time_flight_s']<=r['initiation_step']/50)
            if p=='no_recovery':
                assert e['application_step']==dispatch and e['fallback_step'] is None
                if r['delay_s']==0.:
                    assert r['physics_sha256']==entries[seed]['d0_verification']['physics_sha256']
                    assert r['safe_completion'];no_recovery_matches+=1
            if p=='always_recover':
                assert e['fallback_step']==approval and e['application_step'] is None
                assert r['physics_sha256']==entries[seed]['initial_sideways']['physics_sha256'];always_matches+=1
            if p=='fixed_early_recovery':
                assert e['fixed_early_ttl_s']==.2
                if dispatch<approval+10:assert e['application_step']==dispatch
                else:assert e['fallback_step']==approval+10
            if p in APPROVAL_POLICIES:
                forecast=run['predictions'][str(seed)]
                assert e['computation_s']==forecast['computation_s'] and e['expiry_steps']==forecast['expiry_steps']
                ready=approval+first_ready_step(forecast['computation_s'])
                assert e['prediction_ready_control_step']==ready
                if ready<total:assert e['prediction_ready_step']==ready
                else:assert 'prediction_ready_step' not in e
                assert e['prediction_available_before_dispatch']==(forecast['computation_s']<=r['delay_s'])
                if e['fallback_step'] is not None:assert e['fallback_step']>=ready
                if p=='staleguard':assert not e['early_recovery']
                if e['early_recovery']:
                    assert forecast['expiry_steps'] is not None and e['fallback_step']>=approval+forecast['expiry_steps']
            if p=='use_time_recheck':
                assert e['check_started_step']==dispatch and e['prediction_ready_at_dispatch'] is False
                assert e['decision_step']==dispatch+math.floor(e['computation_s']*50)+1==actions[0]
            if truth is not None:
                verified=verify_physical_reference(r,truth)
                assert r['archived_physical_reference_verified']==verified;native_references+=verified
            if flight_key(r) in rep_keys:representative[flight_key(r)]=r
        for delay in protocol['delays']:
            if {'staleguard','staleguard_early_recovery'}<=set(protocol['policies']):
                first=lookup[seed,delay,'staleguard'];early=lookup[seed,delay,'staleguard_early_recovery']
                a,b=first['event'],early['event']
                assert a['computation_s']==b['computation_s'] and a['expiry_steps']==b['expiry_steps']
                assert a['application_step']==b['application_step']
                if a['application_step'] is not None:assert first['physics_sha256']==early['physics_sha256']
                else:assert b['fallback_step']<=a['fallback_step']
                paired_admissions+=1
    assert seen==set(wanted) and count==len(wanted)*len(protocol['delays'])*len(protocol['policies'])
    checked=[]
    if rep.exists():
        with np.load(rep) as archive:
            for seed,delay,policy in sorted(rep_keys):
                prefix=f'seed{seed}_{round(delay*50):03d}_{policy}';r=representative[seed,delay,policy];s=cases[seed][0]
                raw=archive[prefix+'__physics'];t=raw[:,0];positions=raw[:,1:]
                np.testing.assert_array_equal(t,np.arange(len(t))/1000)
                assert len(t)==r['physics_samples'] and hashlib.sha256(raw.tobytes()).hexdigest()==r['physics_sha256']
                distances=[]
                for box in s.obstacles:
                    lo,hi=np.array(box.lower),np.array(box.upper)
                    q=np.abs(positions-(lo+hi)/2)-(hi-lo)/2
                    distances.append(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0)-BODY_RADIUS_M)
                distances=np.array(distances)
                np.testing.assert_allclose(distances.min(),r['min_clearance_m'],atol=1e-12,rtol=0)
                bad=np.flatnonzero(distances.min(axis=0)<s.clearance_m)
                assert r['safe']==(len(bad)==0)
                if len(bad):assert bad[0]==r['first_violation']['physics_tick']
                mask=(t>=s.approval_s)&np.all(positions>=s.inspection_lower,axis=1)&np.all(positions<=s.inspection_upper,axis=1)
                edges=np.diff(np.r_[False,mask,False].astype(int))
                dwell=max(np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)-1,default=0)
                assert r['completed']==(dwell>=round(s.inspection_dwell_s*1000))
                step=r['initiation_step'];ref=archive[prefix+'__reference']
                dp,_=displacement(np.arange(len(ref)-step)/50,s,r['mode'])
                np.testing.assert_allclose(ref[step:,1:4]-r['maneuver_anchor_m'],dp,atol=1e-12,rtol=0)
                checked.append(prefix)
    assert journal_metrics(folder,run,truth)==json.loads((folder/'metrics.json').read_text())
    if prior_hashes:
        previous=json.loads(Path(prior_hashes).read_text())
        assert all(historical_result_matches(p,digest) for p,digest in previous.items())
    return dict(passed=True,flights=count,scenarios=len(wanted),
        calibration_d0_physical_hashes_match=no_recovery_matches,
        immediate_sideways_physical_hashes_match=always_matches,
        archived_references_matched=native_references,actual_approval_snapshots_identical=True,
        full_horizon_1khz_monitoring=True,control_memory_and_physical_state_preserved=True,
        only_eastward_sideways_recovery=True,event_order_and_cancellation_valid=True,
        shared_prediction_and_runtime=True,representative_physics_independently_checked=checked,
        paired_staleguard_forecast_and_command_admission_checks=paired_admissions,
        metrics_recomputed_identically=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--ground-truth',type=Path)
    p.add_argument('--prior-hashes',type=Path)
    a=p.parse_args();result=validate(a.run,a.calibration,a.ground_truth,a.prior_hashes)
    write_json(a.run/'validation.json',result);print(json.dumps(result,indent=2))


if __name__=='__main__':main()
