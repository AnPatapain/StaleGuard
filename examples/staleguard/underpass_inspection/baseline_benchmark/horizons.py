"""Two-policy exploratory extension and metrics-only nine-policy comparison.

No existing policy is run here. Approval forecasts are separately computed and
cached per (new policy, scenario), never reused between policies. Baseline labels
are consumed only after physical execution or for offline metrics/compatibility.
"""
from pathlib import Path
import sys
if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
from dataclasses import asdict
from itertools import groupby
import hashlib
import json
import time
import fcntl
import numpy as np
from model.scg import load_scg_platform_params
from underpass_inspection.storage import (ROOT,canonical,file_hash,write_json,metadata,check_provenance,
    load_calibrated_scenarios,iter_committed,committed_hasher,commit_execution,flight_key)
from underpass_inspection.simulation import execute,check_memory
from underpass_inspection.calibration import release_memory
from underpass_inspection.baselines.staleguard import first_ready_step
from underpass_inspection.baselines.recovery_horizons import predict
from underpass_inspection.baselines.recovery_policies import HorizonCommand,POLICIES
from underpass_inspection.baselines.policies import POLICIES as OLD_POLICIES
from underpass_inspection.baseline_benchmark.run import protocol_hash
from underpass_inspection.baseline_benchmark import metrics


def baseline_context(folder,calibration):
    folder=Path(folder);run=json.loads((folder/'run.json').read_text())
    if run['status']!='complete':raise ValueError('Use completed baseline results')
    committed_hasher(folder,run)
    if run.get('freeze_path'):
        frozen=json.loads(Path(run['freeze_path']).read_text())
        if file_hash(run['freeze_path'])!=run['freeze_sha256'] or frozen['protocol']!=run['protocol']:
            raise ValueError('Original baseline freeze changed')
        if frozen['provenance']!=run['provenance']:raise ValueError('Baseline source provenance differs from its freeze')
    current=metadata();old=run['provenance']
    for key in ('python','numpy','platform','packages','environment_sha256'):
        if old[key]!=current[key]:raise ValueError('Baseline physical environment differs: '+key)
    for key,value in current['shared_sha256'].items():
        if old['shared_sha256'].get(key)!=value:raise ValueError('Shared dynamics/monitor changed: '+key)
    manifest=Path(calibration);manifest=manifest/'calibrated_scenarios.json' if manifest.is_dir() else manifest
    if file_hash(manifest)!=run['protocol']['dataset_sha256']:raise ValueError('Calibration manifest differs from baseline')
    cases=load_calibrated_scenarios(manifest);by_seed={s.seed:(s,snap,p) for s,snap,p in cases}
    selected=[]
    for value in run['protocol']['scenarios']:
        s,snap,p=by_seed[value['seed']]
        if canonical(asdict(s))!=value or file_hash(p)!=run['protocol']['snapshot_sha256'][str(s.seed)]:
            raise ValueError('Frozen scenario/snapshot differs from baseline')
        selected.append((s,snap,p))
    if tuple(run['protocol']['policies'])!=OLD_POLICIES:raise ValueError('Expected the seven-policy source benchmark')
    truth={};physical={};seen=set()
    expected={(s.seed,d,p) for s,_,_ in selected for d in run['protocol']['delays'] for p in OLD_POLICIES}
    for row in iter_committed(folder,run):
        key=flight_key(row)
        if key in seen or key not in expected:raise ValueError('Baseline matrix/duplicate mismatch')
        seen.add(key)
        if row['event']['policy']=='no_recovery':
            truth[row['seed'],'dive',row['delay_s']]=dict(start_s=row['delay_s'],safe=row['safe'],safe_completion=row['safe_completion'])
        physical[row['seed'],row['mode'],row['start_s']]=(row['physics_sha256'],row['safe'],row['safe_completion'],row['min_clearance_m'])
    if seen!=expected:raise ValueError('Baseline matrix incomplete')
    return run,selected,truth,physical


def context(args):
    baseline,cases,truth,physical=baseline_context(args.baseline,args.calibration)
    if args.smoke:
        if baseline['protocol'].get('dataset_split','development')!='development':
            raise ValueError('Smoke tests are development-only')
        wanted=set(args.seeds or [1,3,6]);cases=[c for c in cases if c[0].seed in wanted]
        if len(cases)!=len(wanted):raise ValueError('Missing development smoke seed')
        delays=[0.,.2,1.]
    else:
        if args.seeds:raise ValueError('Full extension uses every frozen baseline scenario')
        if len(cases)!=100 or len(baseline['protocol']['delays'])!=13:raise ValueError('Expected the 100 x 13 baseline')
        delays=baseline['protocol']['delays']
    ids={s.seed for s,_,_ in cases}
    truth={k:v for k,v in truth.items() if k[0] in ids and k[2] in delays}
    protocol=dict(baseline['protocol'],stage='physical_smoke' if args.smoke else 'exploratory_recovery_horizons',
        scenarios=[asdict(s) for s,_,_ in cases],snapshot_sha256={str(s.seed):file_hash(p) for s,_,p in cases},
        delays=delays,policies=list(POLICIES),exploratory=True,
        analysis_subsets=dict(full=[0.,2.],within_prediction_horizon=[0.,1.],beyond_prediction_horizon=[1.2,2.]),
        uncertainty=dict(unit='scenario',method='paired percentile bootstrap',confidence=.95,resamples=2000,seed=2027,complete_scenarios_only=True),
        baseline_run_sha256=file_hash(Path(args.baseline)/'run.json'),baseline_journal_sha256=baseline['journal_sha256'],
        independent_forecasts='One full timed computation per (policy, approval snapshot); no cross-policy result caching',
        horizon_rule='First-invalid command/completion cutoff; recovery trigger is the last safe grid point before first recovery failure; all-safe profiles are censored',
        late_rule='Only recover proactively within verified recovery prefix unless command safety has expired; expired-command fallback is best-effort',
        dispatch_rule='Dispatch wins timer ties. RecoveryAware preserves safety gate; Dual adds completion rejection only while recovery is verified',
        readiness='Atomic profiles/plans, full measured warmed computation; startup compile separate; unavailable predictions fail open',
        representative_seeds=[cases[0][0].seed],representative_delay_s=.2 if args.smoke else 1.)
    return baseline,cases,truth,physical,canonical(protocol)


def verify_frozen(path,protocol):
    f=json.loads(Path(path).read_text());check_provenance(f['provenance'])
    if f['protocol']!=protocol or f['protocol_sha256']!=protocol_hash(protocol):raise ValueError('Extension freeze protocol differs')
    return f


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
        scores['runtime']=metrics.timing_summary(f['computation_s'] for f in cache.values())
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


def write_results(out,run,truth):
    ids={s['seed'] for s in run['protocol']['scenarios']};delays=set(run['protocol']['delays'])
    truth={k:v for k,v in truth.items() if k[0] in ids and k[2] in delays}
    summary=metrics.journal_metrics(out,run,truth)
    summary['prediction_by_policy'],summary['failure_decomposition']=quality(iter_committed(out,run),run,truth)
    summary['predictor']['scope_policy']='staleguard' if 'staleguard' in run['protocol']['policies'] else POLICIES[0]
    metrics.write_outputs(out,run,iter_committed(out,run),truth,metrics=summary)
    with (Path(out)/'REPORT.md').open('a') as stream:
        stream.write('\nNew policy predictor runtime (warmed, independently computed per policy/scenario):\n\n'
            '| Policy | Forecasts | Median ms | p95 ms |\n|---|---:|---:|---:|\n')
        for p,q in summary['prediction_by_policy'].items():
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
    return summary


def execute_run(args):
    baseline,cases,truth,physical,protocol=context(args)
    if args.prepare_only:
        if args.smoke or not args.freeze:raise ValueError('Prepare the full 2,600-flight extension with --freeze PATH')
        frozen=dict(format='underpass_recovery_horizon_freeze_v1',protocol=protocol,protocol_sha256=protocol_hash(protocol),
            provenance=metadata(),expected_flights=len(cases)*len(protocol['delays'])*2,
            baseline_path=str(args.baseline.resolve()),calibration_path=str(args.calibration.resolve()),
            exploratory=True)
        args.freeze.parent.mkdir(parents=True,exist_ok=True)
        if args.freeze.exists():
            if json.loads(args.freeze.read_text())!=frozen:raise ValueError('Preserving an existing different freeze')
        else:write_json(args.freeze,frozen)
        print('Prepared',frozen['expected_flights'],'new-policy flights;',frozen['protocol_sha256']);return
    if not args.smoke:
        if not args.freeze:raise ValueError('Full extension requires a freeze')
        verify_frozen(args.freeze,protocol)
    out=args.out;path=out/'run.json';journal=out/'executions.jsonl'
    if path.exists():
        if not args.resume:raise FileExistsError('Use --resume or a fresh output')
        run=json.loads(path.read_text());check_provenance(run['provenance'])
        if run['protocol']!=protocol:raise ValueError('Resume protocol differs')
        digest=committed_hasher(out,run,repair=True)
    else:
        if args.resume:raise FileNotFoundError(path)
        if out.exists() and any(out.iterdir()):raise FileExistsError(out)
        out.mkdir(parents=True,exist_ok=True);journal.touch();digest=hashlib.sha256()
        run=dict(format='underpass_recovery_horizons_v1',status='partial',protocol=protocol,provenance=metadata(),
            predictions={},predictions_by_policy={p:{} for p in POLICIES},
            baseline_path=str(args.baseline.resolve()),calibration_path=str(args.calibration.resolve()),
            freeze_path=str(args.freeze.resolve()) if args.freeze else None,
            freeze_sha256=file_hash(args.freeze) if args.freeze else None,
            journal_bytes=0,journal_sha256=digest.hexdigest(),completed_flights=0,resumptions=0)
        write_json(path,run)
    expected={(s.seed,d,p) for s,_,_ in cases for d in protocol['delays'] for p in POLICIES};done=set()
    for row in iter_committed(out,run):
        key=flight_key(row)
        if key in done or key not in expected:raise ValueError('Duplicate/unexpected new flight')
        forecast=run['predictions_by_policy'][key[2]][str(key[0])]
        if row['event']['computation_s']!=forecast['computation_s']:raise ValueError('Cached readiness differs')
        done.add(key)
    run['resumptions']+=int(args.resume);platform=None
    if done!=expected:
        check_memory();t=time.perf_counter();platform=load_scg_platform_params()
        # Fixed development-only compilation warmup; no evaluation forecast cache.
        warm=load_calibrated_scenarios(ROOT/'results/development/calibration')[0][1]
        predict(warm,platform)
        run.setdefault('startup_warmups',[]).append(dict(seconds=time.perf_counter()-t,scope='Development-only discarded warmup; not online prediction'))
    write_json(path,run);archive=out/'representative_trajectories.npz';trajectories={}
    if archive.exists():
        with np.load(archive) as a:trajectories={k:a[k] for k in a.files}
    added=0
    try:
        for s,snapshot,_ in cases:
            for policy in POLICIES:
                cache=run['predictions_by_policy'][policy]
                if str(s.seed) not in cache and any((s.seed,d,policy) not in done for d in protocol['delays']):
                    release_memory();cache[str(s.seed)]=predict(snapshot,platform)
                    if policy==POLICIES[0]:run['predictions'][str(s.seed)]=cache[str(s.seed)]
                    write_json(path,run)
            # Scheduling is entirely outside both the predictor and agent.
            for delay in protocol['delays']:
                for policy in POLICIES:
                    key=s.seed,delay,policy
                    if key in done:continue
                    release_memory();forecast=run['predictions_by_policy'][policy][str(s.seed)]
                    row,arrays=execute(s,policy=policy,delay_s=delay,received=forecast,
                        platform=platform,expected_snapshot=snapshot,policy_factory=HorizonCommand)
                    assert row['physics_samples']==row['horizon_step']*20+1 and row['approval_snapshot_verified']
                    ref=physical.get((s.seed,row['mode'],row['start_s']))
                    if ref:assert ref==(row['physics_sha256'],row['safe'],row['safe_completion'],row['min_clearance_m'])
                    row['baseline_physical_reference_verified']=ref is not None
                    duration=s.swerve_start_s+s.swerve_s if row['mode']=='dive' else s.sideways_s
                    row['maneuver_profile_end_approval_s']=None if row['start_s'] is None else row['start_s']+duration
                    row['maneuver_profile_truncated_by_horizon']=row['start_s'] is not None and row['start_s']+duration>s.observation_s
                    row['inspection_observation']='completed' if row['completed'] else 'abandoned_by_recovery' if row['event']['command_canceled'] else 'not_observed_by_fixed_deadline'
                    if s.seed==protocol['representative_seeds'][0] and delay==protocol['representative_delay_s']:
                        prefix=f'seed{s.seed}_{round(delay*50):03d}_{policy}'
                        for kind,array in arrays.items():
                            if kind!='approval_snapshot':trajectories[prefix+'__'+kind]=array
                        temp=archive.with_name(archive.stem+'.tmp.npz');np.savez_compressed(temp,**trajectories);temp.replace(archive)
                    commit_execution(out,run,row,digest);done.add(key);added+=1
                    if added%10==0:print(len(done),'/',len(expected),'new-policy flights',flush=True)
                    if args.max_flights and added>=args.max_flights:return
    finally:
        run=json.loads(path.read_text());committed_hasher(out,run,repair=True)
        run['status']='complete' if run['completed_flights']==len(expected) else 'partial';write_json(path,run)
        write_results(out,run,truth);print('Saved',run['completed_flights'],'new-policy physical flights; added',added)


def merge(args):
    base,cases,truth,physical=baseline_context(args.baseline,args.calibration)
    new=json.loads((args.new_run/'run.json').read_text());check_provenance(new['provenance']);committed_hasher(args.new_run,new)
    if new['status']!='complete':raise ValueError('New-policy run is not complete')
    for key in ('dataset_sha256','ttl_grid_s','control_hz','physics_hz','model_hz','primary_recovery','observation_s'):
        if base['protocol'][key]!=new['protocol'][key]:raise ValueError('Physical protocol mismatch: '+key)
    if not args.smoke:
        for key in ('scenarios','snapshot_sha256','delays'):
            if base['protocol'][key]!=new['protocol'][key]:raise ValueError('Paired matrix differs: '+key)
    elif new['protocol']['stage']!='physical_smoke':raise ValueError('Explicit smoke merge requires a smoke source')
    if tuple(new['protocol']['policies'])!=POLICIES:raise ValueError('Unexpected extension policies')
    if (new['protocol']['baseline_run_sha256']!=file_hash(args.baseline/'run.json') or
            new['protocol']['baseline_journal_sha256']!=base['journal_sha256']):
        raise ValueError('Extension was prepared against a different baseline')
    if new.get('freeze_path'):
        if file_hash(new['freeze_path'])!=new['freeze_sha256']:raise ValueError('Extension freeze changed')
        verify_frozen(new['freeze_path'],new['protocol'])
    protocol=dict(new['protocol'],stage='exploratory_nine_policy_comparison',policies=list(OLD_POLICIES+POLICIES),exploratory=True)
    out=args.out
    if out.exists() and any(out.iterdir()):raise FileExistsError('Use fresh merged output; --metrics-only regenerates an existing merge')
    out.mkdir(parents=True,exist_ok=True);(out/'executions.jsonl').touch()
    selected={str(s['seed']) for s in protocol['scenarios']}
    run=dict(new,protocol=protocol,predictions={s:p for s,p in base['predictions'].items() if s in selected},status='partial',
             merge_sources=dict(new_run_path=str(args.new_run.resolve()),baseline_run_sha256=file_hash(args.baseline/'run.json'),baseline_journal_sha256=base['journal_sha256'],
                                new_run_sha256=file_hash(args.new_run/'run.json'),new_journal_sha256=new['journal_sha256']),
             journal_bytes=0,journal_sha256=hashlib.sha256(b'').hexdigest(),completed_flights=0,
             freeze_path=None,freeze_sha256=None)
    write_json(out/'run.json',run);digest=hashlib.sha256()
    ids=[s['seed'] for s in protocol['scenarios']];delays=protocol['delays']
    expected={(s,d,p) for s in ids for d in delays for p in OLD_POLICIES+POLICIES};seen=set()
    a=iter(groupby(iter_committed(args.baseline,base),key=lambda r:r['seed']));b=iter(groupby(iter_committed(args.new_run,new),key=lambda r:r['seed']))
    ga=next(a,None);gb=next(b,None)
    for seed in ids:
        while ga is not None and ga[0]<seed:ga=next(a,None)
        if ga is None or gb is None or ga[0]!=seed or gb[0]!=seed:raise ValueError('Merge scenario alignment failed')
        rows=[r for r in ga[1] if r['delay_s'] in delays]+list(gb[1]);ga=next(a,None);gb=next(b,None)
        rows.sort(key=lambda r:(r['delay_s'],protocol['policies'].index(r['event']['policy'])))
        for row in rows:
            key=flight_key(row)
            if key in seen or key not in expected:raise ValueError('Duplicate/unexpected merged key')
            seen.add(key);commit_execution(out,run,row,digest)
    if seen!=expected:raise ValueError('Missing paired executions')
    run['status']='complete';write_json(out/'run.json',run)
    write_results(out,run,truth);print('Merged',len(seen),'existing physical records; no simulation')


def validate(args):
    run=json.loads((args.out/'run.json').read_text());check_provenance(run['provenance']);committed_hasher(args.out,run)
    base,cases,truth,physical=baseline_context(args.baseline,args.calibration)
    if run['status']!='complete':raise ValueError('Run incomplete')
    if (run['protocol']['baseline_run_sha256']!=file_hash(args.baseline/'run.json') or
            run['protocol']['baseline_journal_sha256']!=base['journal_sha256']):
        raise ValueError('Baseline source no longer matches')
    if run.get('merge_sources'):
        source=Path(run['merge_sources']['new_run_path'])
        prior=json.loads((source/'run.json').read_text());committed_hasher(source,prior)
        if (file_hash(source/'run.json')!=run['merge_sources']['new_run_sha256'] or
                prior['journal_sha256']!=run['merge_sources']['new_journal_sha256']):
            raise ValueError('Merged extension source changed')
    if run.get('freeze_path'):
        assert file_hash(run['freeze_path'])==run['freeze_sha256'];verify_frozen(run['freeze_path'],run['protocol'])
    cases={s.seed:(s,snap) for s,snap,_ in cases};keys=set();predictions=run['predictions_by_policy']
    for seed in set(predictions[POLICIES[0]])&set(predictions[POLICIES[1]]):
        a,b=(predictions[p][seed] for p in POLICIES)
        for field in ('results','boundaries','policy_plans'):assert a[field]==b[field]
        assert a['wall_finished_unix_s']<=b['wall_started_unix_s'] or b['wall_finished_unix_s']<=a['wall_started_unix_s']
    for r in iter_committed(args.out,run):
        key=flight_key(r);assert key not in keys;keys.add(key)
        s,snapshot=cases[r['seed']];event=r['event']
        np.testing.assert_array_equal(r['approval_state'],snapshot.state)
        assert r['safe']==(r['first_violation'] is None)==(r['min_clearance_m']>=s.clearance_m)
        assert r['safe_completion']==(r['safe'] and r['completed'])
        assert r['physics_samples']==r['horizon_step']*20+1 and r['horizon_step']==round((s.approval_s+s.observation_s)*50)
        assert r['geometry_unchanged'] and r['pid_unchanged_at_initiation'] and r['physical_state_unchanged_at_initiation']
        assert event['scheduled_dispatch_step']==r['approval_step']+round(r['delay_s']*50)
        if event['policy'] in POLICIES:
            forecast=predictions[event['policy']][str(r['seed'])]
            assert event['computation_s']==forecast['computation_s']
            ready=r['approval_step']+first_ready_step(forecast['computation_s'])
            assert event['prediction_ready_control_step']==ready
            if ready<r['horizon_step']:assert event['prediction_ready_step']==ready
            actions=[e for e in event['events'] if e['kind'] in ('recovery_started','command_started')]
            assert len(actions)==1
            if event['application_step'] is not None:
                assert event['application_step']==event['scheduled_dispatch_step']
            if event['fallback_step'] is not None:
                assert event['fallback_step']>=r['approval_step']+first_ready_step(forecast['computation_s'])
                assert event['command_canceled'] and event['application_step'] is None and r['mode']=='sideways'
                assert r['final_position_m'][1]<r['maneuver_anchor_m'][1]
        ref=physical.get((r['seed'],r['mode'],r['start_s']))
        if ref:assert ref==(r['physics_sha256'],r['safe'],r['safe_completion'],r['min_clearance_m'])
    expected={(s['seed'],d,p) for s in run['protocol']['scenarios'] for d in run['protocol']['delays'] for p in run['protocol']['policies']}
    assert keys==expected
    before=file_hash(args.out/'metrics.json');write_results(args.out,run,truth);assert file_hash(args.out/'metrics.json')==before
    result=dict(passed=True,flights=len(keys),full_horizon_1khz=True,common_approval_state=True,physical_reference_compatibility=True,
                prediction_readiness=True,no_duplicate_keys=True,metrics_regeneration_identical=True,
                identical_profiles_independent_computation=True,single_action_cancellation=True)
    write_json(args.out/'validation.json',result);print(result)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,default=ROOT/'results/evaluation/baseline_benchmark')
    p.add_argument('--calibration',type=Path,default=ROOT/'results/evaluation/calibration')
    p.add_argument('--out',type=Path,default=ROOT/'results/evaluation/recovery_horizons_benchmark_v1')
    p.add_argument('--freeze',type=Path);p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--smoke',action='store_true');p.add_argument('--seeds',nargs='+',type=int)
    p.add_argument('--resume',action='store_true');p.add_argument('--max-flights',type=int,default=0)
    p.add_argument('--metrics-only',action='store_true');p.add_argument('--validate',action='store_true')
    p.add_argument('--merge',action='store_true');p.add_argument('--new-run',type=Path)
    args=p.parse_args()
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if args.merge:
            if args.new_run is None:p.error('--merge needs --new-run')
            merge(args)
        elif args.validate:validate(args)
        elif args.metrics_only:
            run=json.loads((args.out/'run.json').read_text());check_provenance(run['provenance']);committed_hasher(args.out,run)
            _,_,truth,_=baseline_context(args.baseline,args.calibration);write_results(args.out,run,truth);print('Metrics regenerated; no simulation')
        else:execute_run(args)


if __name__=='__main__':main()
