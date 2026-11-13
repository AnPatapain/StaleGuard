"""One nine-policy benchmark: prepare, execute/resume, validate and report.

Dispatch scheduling stays in the physical driver. Predictors and policies never
receive the future dispatch delay. Historical analysis uses only this folder.
"""
from pathlib import Path
import sys
from staleguard import underpass_inspection
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import time
import numpy as np
from staleguard.model.scg import load_scg_platform_params
from staleguard.underpass_inspection.simulation import execute,check_memory
from staleguard.underpass_inspection.calibration import release_memory
from staleguard.underpass_inspection.baselines.policies import POLICIES,APPROVAL_POLICIES,HORIZON_POLICIES
from staleguard.underpass_inspection.baselines.staleguard import TTL_GRID,predict_delays
from staleguard.underpass_inspection.baselines.recovery_horizons import predict as predict_horizons
from staleguard.underpass_inspection.baselines import fbs,fixed_early_recovery
from staleguard.underpass_inspection.storage import (ROOT,canonical,file_hash,write_json,metadata,check_provenance,
    load_calibrated_scenarios,save_inputs,load_inputs,save_source_archive,initialize_run,read_run,write_progress,
    iter_committed,committed_hasher,commit_record,flight_key,prediction_for)
from staleguard.underpass_inspection.baseline_benchmark.metrics import write_journal_outputs

DISPATCH_DELAYS=(0.,.04,.08,.12,.16,.20,.40,.60,.80,1.,1.20,1.60,2.)
SAMPLE_SEED=20261109


def protocol_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def select_cases(cases,excluded,count,seed):
    """One outcome-independent uniform draw from sorted eligible accepted IDs."""
    cases=sorted(cases,key=lambda c:c[0].seed);excluded=set(excluded)
    eligible=[c for c in cases if c[0].seed not in excluded]
    if count>len(eligible) or count<1:raise ValueError('Requested cohort exceeds eligible population')
    ids=[c[0].seed for c in eligible]
    chosen=set(np.random.default_rng(seed).choice(ids,count,replace=False).tolist())
    return [c for c in eligible if c[0].seed in chosen]


def prepare(args):
    out=args.out
    if out.exists() and any(out.iterdir()):raise FileExistsError('Use a fresh preparation output')
    calibration=args.calibration or ROOT/'results/development/calibration'
    manifest=calibration/'calibrated_scenarios.json' if calibration.is_dir() else calibration
    data=json.loads(manifest.read_text());cases=load_calibrated_scenarios(manifest)
    split=data.get('dataset_split','development');sampling=None
    policies=tuple(args.policies or POLICIES);delays=tuple(args.delays or DISPATCH_DELAYS)
    if args.sample_count is not None:
        # Selection reads configuration IDs only, never policy outcomes/forecasts.
        history=json.loads((args.exclude_run/'run.json').read_text())
        progress=json.loads((args.exclude_run/'progress.json').read_text())
        if progress['status']!='complete' or file_hash(args.exclude_run/'run.json')!=progress['config_sha256']:
            raise ValueError('Use the verified complete historical configuration for exclusions')
        excluded=[s['seed'] for s in history['protocol']['scenarios']]
        if (split!='evaluation' or len(cases)!=865 or len(excluded)!=100 or args.sample_count!=400
                or policies!=POLICIES or delays!=DISPATCH_DELAYS or args.seeds or args.smoke):
            raise ValueError('v2 requires 400 new of 865 accepted, excluding historical 100, all nine policies and 13 delays')
        population=[s.seed for s,_,_ in cases]
        if not set(excluded)<=set(population):raise ValueError('Historical IDs absent from calibration')
        cases=select_cases(cases,excluded,400,args.sample_seed)
        sampling=dict(population_seeds=population,excluded_seeds=sorted(excluded),rng_seed=args.sample_seed,
            eligible_count=765,sample_count=400,method='NumPy default_rng(seed).choice(sorted eligible IDs, count, replace=False)',
            selection_uses_policy_outcomes=False)
    if args.smoke:
        if split!='development':raise ValueError('Smoke flights are development-only')
        wanted=set(args.seeds or [1,3,6]);cases=[c for c in cases if c[0].seed in wanted]
        if {s.seed for s,_,_ in cases}!=wanted:raise ValueError('Missing development smoke seed')
        delays=tuple(args.delays or (0.,.2,1.))
    elif args.seeds:
        wanted=set(args.seeds);cases=[c for c in cases if c[0].seed in wanted]
        if {s.seed for s,_,_ in cases}!=wanted:raise ValueError('Requested seed not accepted')
    if split=='evaluation' and sampling is None:raise ValueError('Prepare the fresh v2 cohort with --sample-count 400')
    if (not cases or len(set(policies))!=len(policies) or len(set(delays))!=len(delays)
            or not policies or not delays or any(not math.isfinite(d) or not 0<=d<=2 or
                not math.isclose(d*50,round(d*50),abs_tol=1e-9) for d in delays)):
        raise ValueError('Use distinct policies and 20 ms-grid dispatch delays in 0..2 s')
    protocol=canonical(dict(stage='physical_smoke' if args.smoke else split+'_benchmark',dataset_split=split,
        protocol_version='nine_policy_v2',scenarios=[asdict(s) for s,_,_ in cases],
        dataset_sha256=file_hash(manifest),snapshot_sha256={str(s.seed):file_hash(p) for s,_,p in cases},
        calibration_statistics=data['statistics'],scenario_sampling=sampling,policies=policies,delays=delays,
        ttl_grid_s=TTL_GRID,ground_truth_source='no_recovery' if 'no_recovery' in policies else 'not_collected',
        control_hz=50,physics_hz=1000,model_hz=100,observation_s=8.,
        primary_recovery='Existing Eastward sideways maneuver only; no climb selection',
        fixed_early_ttl_s=fixed_early_recovery.FIXED_EARLY_TTL_S,
        fbs=dict(alpha=fbs.ALPHA,epsilon=fbs.EPSILON,max_ttl_steps=fbs.MAX_TTL_STEPS),
        analysis_subsets=dict(full=[0.,2.],within_prediction_horizon=[0.,1.],beyond_prediction_horizon=[1.02,2.]),
        uncertainty=dict(unit='scenario',method='paired percentile bootstrap',confidence=.95,resamples=2000,seed=2027,complete_scenarios_only=True),
        forecast_timing='Original StaleGuard pair shares one command forecast/runtime; each new policy independently computes/times the three-profile forecast',
        readiness='Full measured warmed computation, rounded to first available 50 Hz step; unavailable forecasts fail open',
        ordering='Approval; readiness; actual dispatch; pending timers; maneuver initiation; physical ticks',
        beyond_prediction_horizon='No predicted expiration permits dispatch beyond 1 s; finite ready expiry rejects; no new horizon cutoff',
        representative_seeds=[cases[0][0].seed],representative_delay_s=delays[-1],
        unknown_delay='Only the simulator schedules future dispatch; the policy observes it when it occurs'))
    out.mkdir(parents=True,exist_ok=True);save_inputs(out/'approval_snapshots.npz',cases)
    provenance=metadata();save_source_archive(out/'source_versions.zip',provenance)
    config=dict(format='underpass_nine_policy_benchmark_v2',protocol=protocol,provenance=provenance,
        source_archive_sha256=file_hash(out/'source_versions.zip'),
        inputs_sha256=file_hash(out/'approval_snapshots.npz'),historical=False,
        created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    initialize_run(out,config)
    print('Prepared',len(cases),'scenarios x',len(delays),'delays x',len(policies),'policies =',len(cases)*len(delays)*len(policies),'flights')


def execute_run(args):
    out=args.out
    if not (out/'run.json').exists():prepare(args)
    run=read_run(out,repair=args.resume)
    if run.get('historical'):raise ValueError('Historical benchmark is read-only; use --validate or --metrics-only')
    check_provenance(run['provenance'])
    protocol=run['protocol']
    for argument,field in ((args.policies,'policies'),(args.delays,'delays')):
        if argument is not None and list(argument)!=protocol[field]:raise ValueError('Prepared '+field+' differ')
    if args.sample_count is not None and args.sample_count!=(protocol.get('scenario_sampling') or {}).get('sample_count'):
        raise ValueError('Prepared sample count differs')
    if args.calibration is not None:
        manifest=args.calibration/'calibrated_scenarios.json' if args.calibration.is_dir() else args.calibration
        if file_hash(manifest)!=protocol['dataset_sha256']:raise ValueError('Prepared calibration differs')
    if args.seeds and set(args.seeds)!={s['seed'] for s in protocol['scenarios']}:raise ValueError('Prepared scenario selection differs')
    if run['status']!='prepared' and not args.resume:raise FileExistsError('Use --resume for an existing execution')
    cases=load_inputs(out,run);expected={(s.seed,d,p) for s,_,_ in cases for d in protocol['delays'] for p in protocol['policies']}
    done=set()
    for row in iter_committed(out,run):
        key=flight_key(row)
        if key in done or key not in expected:raise ValueError('Unexpected or duplicate flight')
        done.add(key)
        if key[2] in APPROVAL_POLICIES:
            forecast=prediction_for(run,key[2],key[0])
            if forecast is None or any(row['event'][k]!=forecast[k] for k in ('computation_s','expiry_steps')):
                raise ValueError('Committed forecast/runtime differs')
    if len(done)!=run['completed_flights']:raise ValueError('Flight count differs')
    if done==expected:print('Already complete;',len(done),'flights');return
    run['resumptions']+=int(args.resume);run['status']='running';write_progress(out,run)
    digest=committed_hasher(out,run);prediction_digest=committed_hasher(out,run,kind='prediction')
    platform=None;added=0;archive=out/'representative_trajectories.npz';trajectories={}
    if archive.exists():
        with np.load(archive) as a:trajectories={k:a[k] for k in a.files}
    try:
        check_memory();began=time.perf_counter();platform=load_scg_platform_params()
        warm=load_calibrated_scenarios(ROOT/'results/development/calibration')[0][1]
        if set(protocol['policies'])&{'staleguard','staleguard_early_recovery'}:predict_delays(warm,platform)
        if set(protocol['policies'])&set(HORIZON_POLICIES):predict_horizons(warm,platform)
        run.setdefault('startup_warmups',[]).append(dict(seconds=time.perf_counter()-began,
            scope='Discarded development-only warmup; no evaluation forecast/cache; not online readiness'))
        write_progress(out,run)
        for s,snapshot,_ in cases:
            if all((s.seed,d,p) in done for d in protocol['delays'] for p in protocol['policies']):continue
            for policy in protocol['policies']:
                if policy not in APPROVAL_POLICIES or prediction_for(run,policy,s.seed) is not None:continue
                release_memory();key=policy if policy in HORIZON_POLICIES else 'staleguard'
                forecast=predict_horizons(snapshot,platform) if policy in HORIZON_POLICIES else predict_delays(snapshot,platform)
                commit_record(out,run,dict(policy=key,seed=s.seed,forecast=forecast),prediction_digest,kind='prediction')
                cache=run['predictions'] if key=='staleguard' else run['predictions_by_policy'].setdefault(key,{})
                cache[str(s.seed)]=forecast
            for delay in protocol['delays']:
                for policy in protocol['policies']:
                    key=s.seed,delay,policy
                    if key in done:continue
                    release_memory()
                    row,arrays=execute(s,policy=policy,delay_s=delay,received=prediction_for(run,policy,s.seed),
                                       platform=platform,expected_snapshot=snapshot)
                    assert row['physics_samples']==row['horizon_step']*20+1 and row['approval_snapshot_verified']
                    assert row['geometry_unchanged'] and row['pid_unchanged_at_initiation'] and row['physical_state_unchanged_at_initiation']
                    assert row['final_position_m'][1]<row['maneuver_anchor_m'][1]
                    duration=s.swerve_start_s+s.swerve_s if row['mode']=='dive' else s.sideways_s
                    row.update(maneuver_profile_end_approval_s=row['start_s']+duration,
                        maneuver_profile_truncated_by_horizon=row['start_s']+duration>s.observation_s,
                        inspection_observation='completed' if row['completed'] else 'abandoned_by_recovery' if row['event']['command_canceled'] else 'not_observed_by_fixed_deadline')
                    if s.seed in protocol['representative_seeds'] and delay==protocol['representative_delay_s']:
                        prefix=f'seed{s.seed}_{round(delay*50):03d}_{policy}'
                        for kind,array in arrays.items():
                            if kind!='approval_snapshot':trajectories[prefix+'__'+kind]=array
                        temporary=archive.with_suffix('.tmp.npz');np.savez_compressed(temporary,**trajectories);temporary.replace(archive)
                    commit_record(out,run,row,digest);done.add(key);added+=1
                    if added%10==0:print(f'{len(done)}/{len(expected)} seed={s.seed} D={delay:.2f} {policy}',flush=True)
                    if args.max_flights and added>=args.max_flights:return
    finally:
        saved=read_run(out,repair=True)
        saved['status']='complete' if saved['completed_flights']==len(expected) else 'partial'
        write_progress(out,saved);write_journal_outputs(out,saved)
        print('Saved',saved['completed_flights'],'/',len(expected),'physical flights; added',added,flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--calibration',type=Path)
    p.add_argument('--policies',nargs='+',choices=POLICIES)
    p.add_argument('--delays',nargs='+',type=float);p.add_argument('--seeds',nargs='+',type=int)
    p.add_argument('--sample-count',type=int);p.add_argument('--sample-seed',type=int,default=SAMPLE_SEED)
    p.add_argument('--exclude-run',type=Path,default=ROOT/'results/evaluation/baseline_benchmark')
    p.add_argument('--smoke',action='store_true');p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--resume',action='store_true');p.add_argument('--max-flights',type=int,default=0)
    p.add_argument('--metrics-only',action='store_true');p.add_argument('--validate',action='store_true')
    a=p.parse_args()
    if a.max_flights<0:p.error('--max-flights must be nonnegative')
    if sum((a.metrics_only,a.validate,a.prepare_only))>1:p.error('Choose one operation')
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.prepare_only:prepare(a)
        elif a.metrics_only:
            run=read_run(a.out);write_journal_outputs(a.out,run);print('Metrics regenerated; no simulation')
        elif a.validate:
            from staleguard.underpass_inspection.support.validate_baseline_benchmark import validate
            result=validate(a.out);write_json(a.out/'validation.json',result);print(result)
        else:execute_run(a)


if __name__=='__main__':main()
