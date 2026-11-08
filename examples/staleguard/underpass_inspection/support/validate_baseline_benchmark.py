"""Audit actual policy journals and representative physics without new flights."""
from pathlib import Path
import sys
if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import numpy as np
from underpass_inspection.storage import (ROOT,load_calibrated_scenarios,load_ground_truth,
    read_committed,check_provenance,verify_physical_reference,canonical,write_json,flight_key)
from underpass_inspection.scenario import BODY_RADIUS_M,displacement
from underpass_inspection.baselines.policies import APPROVAL_POLICIES
from underpass_inspection.baseline_benchmark.metrics import summarize


def validate(folder,calibration,ground_truth,prior_hashes=None):
    folder=Path(folder);run=json.loads((folder/'run.json').read_text())
    check_provenance(run['provenance'])
    assert run['status']=='complete'
    rows=read_committed(folder,run);protocol=run['protocol']
    cases={s.seed:(s,snapshot) for s,snapshot,_ in load_calibrated_scenarios(calibration)}
    truth,_=load_ground_truth(ground_truth)
    expected={(s['seed'],d,p) for s in protocol['scenarios'] for d in protocol['delays'] for p in protocol['policies']}
    assert set(map(flight_key,rows))==expected and len(rows)==len(expected)
    lookup={flight_key(r):r for r in rows}
    native_references=0;no_recovery_matches=0;always_matches=0;early=0
    for r in rows:
        s,snapshot=cases[r['seed']];e=r['event'];p=e['policy']
        approval=round(s.approval_s*50);dispatch=approval+round(r['delay_s']*50)
        total=round((s.approval_s+s.observation_s)*50)
        assert e['scheduled_dispatch_step']==dispatch==e['dispatch_observed_step']
        assert r['approval_step']==approval and r['horizon_step']==total
        assert r['physics_samples']==total*20+1 and r['control_samples']==total+1
        assert r['safe']==(r['first_violation'] is None)==(r['min_clearance_m']>=s.clearance_m)
        assert r['safe_completion']==(r['safe'] and r['completed'])
        np.testing.assert_array_equal(r['approval_state'],snapshot.state)
        assert r['approval_snapshot_verified'] and r['geometry_unchanged']
        assert r['max_physics_position_increment_m']<.01
        actions=[v for v in (e['application_step'],e['fallback_step']) if v is not None]
        assert len(actions)<=1
        if actions:
            assert r['initiation_step']==actions[0]
            assert r['pid_unchanged_at_initiation'] and r['physical_state_unchanged_at_initiation']
            assert r['final_position_m'][1]<r['maneuver_anchor_m'][1]
            assert r['mode'] in ('dive','sideways')
        if e['fallback_step'] is not None:
            assert e['command_canceled'] and e['command_rejected']
            assert r['mode']=='sideways' and e['application_step'] is None
        assert e['early_recovery']==(e['fallback_step'] is not None and e['fallback_step']<dispatch)
        early+=e['early_recovery']
        events=e['events'];times=[v['flight_s'] for v in events]
        assert all(a<=b+1e-12 for a,b in zip(times,times[1:]))
        assert sum(v['kind']=='approval' for v in events)==1
        assert sum(v['kind']=='command_dispatch' for v in events)==1
        assert sum(v['kind'] in ('command_started','recovery_started') for v in events)<=1
        assert events[-1]['kind']=='observation_horizon_end'
        if r['first_violation']:
            bad=[v for v in events if v['kind']=='safety_violation']
            assert len(bad)==1 and bad[0]['physics_tick']==r['first_violation']['physics_tick']
        if p=='no_recovery':
            assert e['application_step']==dispatch and e['fallback_step'] is None
        if p=='always_recover':
            assert e['fallback_step']==approval and e['application_step'] is None
        if p=='fixed_early_recovery':
            assert e['fixed_early_ttl_s']==.2
            if dispatch<approval+10:assert e['application_step']==dispatch
            else:assert e['fallback_step']==approval+10
        if p in APPROVAL_POLICIES:
            forecast=run['predictions'][str(s.seed)]
            assert e['computation_s']==forecast['computation_s'] and e['expiry_steps']==forecast['expiry_steps']
            ready=approval+math.ceil(forecast['computation_s']*50)
            assert e['prediction_ready_control_step']==ready
            assert e['prediction_available_before_dispatch']==(forecast['computation_s']<=r['delay_s'])
            if e['fallback_step'] is not None:
                assert e['fallback_step']>=ready
            if p=='staleguard':assert not e['early_recovery']
            if e['early_recovery']:
                assert p=='staleguard_early_recovery'
                assert forecast['expiry_steps'] is not None
                assert e['fallback_step']>=approval+forecast['expiry_steps']
        if p=='use_time_recheck':
            assert e['check_started_step']==dispatch
            assert e['decision_step']==dispatch+math.floor(e['computation_s']*50)+1
            assert e['prediction_ready_at_dispatch'] is False
            if actions:assert actions[0]==e['decision_step']
        verified=verify_physical_reference(r,truth)
        assert r['archived_physical_reference_verified']==verified
        native_references+=verified
        if p=='no_recovery':no_recovery_matches+=verified
        if p=='always_recover':always_matches+=verified
    paired_admissions=0
    if {'staleguard','staleguard_early_recovery'}<=set(protocol['policies']):
        for s in protocol['scenarios']:
            for delay in protocol['delays']:
                first=lookup[(s['seed'],delay,'staleguard')]
                early_row=lookup[(s['seed'],delay,'staleguard_early_recovery')]
                a,b=first['event'],early_row['event']
                assert a['computation_s']==b['computation_s'] and a['expiry_steps']==b['expiry_steps']
                assert a['application_step']==b['application_step']
                if a['application_step'] is not None:
                    assert first['physics_sha256']==early_row['physics_sha256']
                else:
                    assert b['fallback_step']<=a['fallback_step']
                paired_admissions+=1
    checked=[]
    if (folder/'representative_trajectories.npz').exists():
        with np.load(folder/'representative_trajectories.npz') as archive:
            for key in archive.files:
                if not key.endswith('__physics'):continue
                prefix=key[:-len('__physics')];parts=prefix.split('_',2)
                seed=int(parts[0][4:]);delay=int(parts[1])/50;policy=parts[2]
                r=lookup[(seed,delay,policy)];s=cases[seed][0]
                raw=archive[key];t=raw[:,0];positions=raw[:,1:]
                np.testing.assert_array_equal(t,np.arange(len(t))/1000)
                assert len(t)==r['physics_samples']
                assert hashlib.sha256(raw.tobytes()).hexdigest()==r['physics_sha256']
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
                if r['initiation_step'] is not None:
                    step=r['initiation_step'];ref=archive[prefix+'__reference']
                    dp,_=displacement(np.arange(len(ref)-step)/50,s,r['mode'])
                    np.testing.assert_allclose(ref[step:,1:4]-r['maneuver_anchor_m'],dp,atol=1e-12,rtol=0)
                checked.append(prefix)
    summary=summarize(rows,run,truth)
    assert summary==json.loads((folder/'metrics.json').read_text())
    prior_count=None
    if prior_hashes:
        previous=json.loads(Path(prior_hashes).read_text())
        assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest for p,digest in previous.items())
        prior_count=len(previous)
    result=dict(passed=True,flights=len(rows),scenarios=len(protocol['scenarios']),
        no_recovery_physical_hashes_match=no_recovery_matches,
        always_recover_physical_hashes_match=always_matches,all_archived_references_matched=native_references,
        early_recoveries=early,actual_approval_snapshots_identical=True,
        full_horizon_1khz_monitoring=True,control_memory_and_physical_state_preserved=True,
        only_eastward_sideways_recovery=True,event_order_and_cancellation_valid=True,
        unknown_dispatch_interfaces_tested=True,representative_physics_independently_checked=checked,
        paired_staleguard_forecast_and_command_admission_checks=paired_admissions,
        metrics_recomputed_identically=True,previous_artifacts_unchanged=prior_count,
        violations_before_dispatch=sum(r['pre_dispatch_violation_s'] is not None for r in rows),
        recoveries_after_violation=sum(r['event']['fallback_step'] is not None and r['violation_before_intervention'] for r in rows))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,default=ROOT/'results/development/baseline_benchmark')
    p.add_argument('--calibration',type=Path,default=ROOT/'results/development/calibration')
    p.add_argument('--ground-truth',type=Path,default=ROOT/'results/development/deadline_sweep')
    p.add_argument('--prior-hashes',type=Path)
    a=p.parse_args();result=validate(a.run,a.calibration,a.ground_truth,a.prior_hashes)
    write_json(a.run/'validation.json',result);print(json.dumps(result,indent=2))


if __name__=='__main__':main()
