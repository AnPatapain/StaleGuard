"""Seven frozen policies, independent physical flights, bounded-memory journals."""
from pathlib import Path
import sys
if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import time
import numpy as np
from model.scg import load_scg_platform_params
from model.lightweight import LightweightPredictiveModel
from underpass_inspection.simulation import check_memory,execute
from underpass_inspection.baselines.staleguard import TTL_GRID,predict_delays,segment_box_clearance
from underpass_inspection.baselines import fbs,fixed_early_recovery
from underpass_inspection.calibration import release_memory
from underpass_inspection.scenario import HZ,POS
from underpass_inspection.baselines.policies import POLICIES,APPROVAL_POLICIES
from underpass_inspection.storage import (ROOT,canonical,load_calibrated_scenarios,metadata,
    check_provenance,write_json,file_hash,iter_committed,committed_hasher,commit_execution,
    flight_key,load_ground_truth,verify_physical_reference)
from underpass_inspection.baseline_benchmark.metrics import write_journal_outputs

DISPATCH_DELAYS=tuple(round(i*.02,2) for i in range(101))
SMOKE_DELAYS=(0.,1.,2.)
REDUCED_DELAYS=(0.,.04,.08,.12,.16,.20,.40,.60,.80,1.,1.20,1.60,2.)
REDUCED_SAMPLE_SEED=20261108


def reduced_cases(cases):
    """Outcome-independent uniform sample from the sorted accepted population."""
    cases=sorted(cases,key=lambda case:case[0].seed)
    if len(cases)!=865:raise ValueError('Reduced evaluation requires the frozen 865 accepted scenarios')
    ids=[s.seed for s,_,_ in cases]
    chosen=set(np.random.default_rng(REDUCED_SAMPLE_SEED).choice(ids,size=100,replace=False).tolist())
    return [case for case in cases if case[0].seed in chosen]


def manifest_path(path):
    return path/'calibrated_scenarios.json' if path.is_dir() else path


def protocol_hash(protocol):
    return hashlib.sha256(json.dumps(protocol,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def verify_freeze(path,protocol,smoke=False):
    frozen=json.loads(Path(path).read_text())
    if frozen['protocol_sha256']!=protocol_hash(frozen['protocol']):
        raise ValueError('Freeze protocol checksum differs')
    if frozen['provenance_sha256']!=protocol_hash(frozen['provenance']):
        raise ValueError('Freeze source/environment checksum differs')
    check_provenance(frozen['provenance'])
    expected=frozen['protocol']
    if smoke:
        expected=dict(expected,stage='physical_smoke',scenarios=expected['scenarios'][:2],
            snapshot_sha256={str(s['seed']):expected['snapshot_sha256'][str(s['seed'])]
                             for s in expected['scenarios'][:2]},
            delays=list(SMOKE_DELAYS),representative_delay_s=2.)
    if protocol!=expected:raise ValueError('Run differs from the frozen protocol')
    return frozen


def run(args):
    cases=load_calibrated_scenarios(args.calibration)
    data_path=manifest_path(args.calibration);dataset=json.loads(data_path.read_text())
    split=dataset.get('dataset_split','development')
    sampling=None
    if args.reduced:
        if split=='evaluation':
            population=[s.seed for s,_,_ in cases];cases=reduced_cases(cases)
            sampling=dict(population_count=len(population),sample_count=100,rng_seed=REDUCED_SAMPLE_SEED,
                method='NumPy default_rng(seed).choice(sorted accepted IDs, 100, replace=False); execute sorted IDs',
                population_ids_sha256=protocol_hash(population),selected_seeds=[s.seed for s,_,_ in cases],
                selection_uses_policy_outcomes=False)
        elif not args.smoke:raise ValueError('Reduced protocol is evaluation-only except development smoke tests')
    if args.smoke and args.seeds is None:args.seeds=[s.seed for s,_,_ in cases[:2]]
    if args.seeds:
        cases=[case for case in cases if case[0].seed in args.seeds]
        if {s.seed for s,_,_ in cases}!=set(args.seeds):
            raise ValueError('Requested seed is not accepted in the frozen manifest')
    if not cases:raise ValueError('No accepted scenarios')
    truth=None
    if args.ground_truth:
        truth,truth_run=load_ground_truth(args.ground_truth)
        archived={s['seed']:s for s in truth_run['protocol']['scenarios']}
        if any(archived.get(s.seed)!=canonical(asdict(s)) for s,_,_ in cases):
            raise ValueError('Archived truth describes a different frozen scenario')
    elif tuple(args.policies)!=POLICIES:
        raise ValueError('No-sweep runs require all seven policies, with No Recovery first')
    if split=='evaluation' and truth is not None:
        raise ValueError('Held-out runs use paired No Recovery labels, not an independent sweep')
    representatives=args.representative_seeds or [cases[0][0].seed]
    if not set(representatives)<={s.seed for s,_,_ in cases}:
        raise ValueError('Representative seed absent from selected scenarios')
    protocol=canonical(dict(stage='physical_smoke' if args.smoke else split+'_benchmark',dataset_split=split,
        scenarios=[asdict(s) for s,_,_ in cases],dataset_sha256=file_hash(data_path),
        snapshot_sha256={str(s.seed):file_hash(p) for s,_,p in cases},
        calibration_statistics=dataset['statistics'],ground_truth_source='external' if truth is not None else 'no_recovery',
        ground_truth_run_sha256=file_hash(args.ground_truth/'run.json') if truth is not None else None,
        ground_truth_journal_sha256=file_hash(args.ground_truth/'executions.jsonl') if truth is not None else None,
        policies=list(args.policies),delays=list(args.delays),ttl_grid_s=list(TTL_GRID),
        protocol_version='reduced_100x13_v3' if args.reduced else 'dispatch_2s_v2',
        scenario_sampling=sampling,
        temporal_resolution=('13 selected delays; 0.04 s through 0.20, then gaps up to 0.40 s; tested thresholds are coarse bounds'
                             if args.reduced else 'Uniform 0.02 s physical dispatch grid'),
        analysis_subsets=dict(full=[0.,2.],within_prediction_horizon=[0.,1.],beyond_prediction_horizon=[1.20 if args.reduced else 1.02,2.]),
        uncertainty=dict(unit='scenario',method='paired percentile bootstrap',confidence=.95,
                         resamples=2000,seed=2027,complete_scenarios_only=True),
        beyond_prediction_horizon='Unchanged gate: ready finite expiry rejects; no predicted expiry permits dispatch; unavailable forecast permits dispatch',
        completion_observation='Fixed 8 s after approval; inspection not observed by deadline is incomplete, not proof of impossibility later',
        control_hz=50,physics_hz=1000,model_hz=100,observation_s='unchanged per frozen scenario',
        primary_recovery='Existing Eastward sideways maneuver only; no climb selection',
        fixed_early_ttl_s=fixed_early_recovery.FIXED_EARLY_TTL_S,
        fbs=dict(alpha=fbs.ALPHA,epsilon=fbs.EPSILON,max_ttl_steps=fbs.MAX_TTL_STEPS,
            feature='required clearance minus minimum actual 3D body clearance; EMA through approval inclusive'),
        readiness='Measured warmed runtime replayed on the control clock; unavailable approval forecasts fail open',
        ordering='Approval first; readiness; dispatch; pending timers; maneuver start; physical ticks. Dispatch wins timer ties.',
        use_time_recheck='One immediate-command rollout from actual dispatch state; act at dispatch + floor(runtime*50)+1',
        representative_seeds=representatives,representative_delay_s=2. if args.smoke else 1. if args.reduced else .92,
        unknown_delay='Only the physical driver knows scheduled dispatch. Policies receive the event when it occurs.'))
    if args.prepare_only:
        expected_delays=REDUCED_DELAYS if args.reduced else DISPATCH_DELAYS
        expected_count=100 if args.reduced else dataset['statistics']['accepted']
        if (args.freeze is None or args.smoke or args.seeds or tuple(args.delays)!=expected_delays
                or tuple(args.policies)!=POLICIES or len(cases)!=expected_count):
            raise ValueError('Prepare the declared cohort, seven policies and complete dispatch grid with --freeze PATH')
        frozen=dict(format='underpass_policy_freeze_v1',protocol=protocol,protocol_sha256=protocol_hash(protocol),
            calibration_path=str(data_path.resolve()),provenance=metadata(),
            expected_flights=len(cases)*len(args.delays)*len(args.policies),
            changes_from_development=[('Random 100-of-865 sample with seed 20261108 and 13 delays; no outcome selection'
                if args.reduced else 'Physical dispatch grid extended to 0..2 s (101 points)')+'; 19-point predictor unchanged',
                'Three analysis subsets of one benchmark; scenario-level paired bootstrap intervals',
                'Mission-local helper copies; identical numerical rules',
                'Readiness timestamp rounding corrected at exact floating-point ties; decisions unchanged',
                'No Recovery supplies command labels; no independent recovery deadline is inferred',
                'Streaming journal commits/metrics and strict rollback of uncommitted tail bytes'])
        frozen['provenance_sha256']=protocol_hash(frozen['provenance'])
        args.freeze.parent.mkdir(parents=True,exist_ok=True)
        if args.freeze.exists():
            if json.loads(args.freeze.read_text())!=frozen:raise ValueError('Preserving a different existing freeze')
        else:write_json(args.freeze,frozen)
        print('Frozen',len(cases),'scenarios;',frozen['expected_flights'],'flights; protocol',frozen['protocol_sha256'])
        return
    if split=='evaluation' and args.freeze is None:
        raise ValueError('Held-out execution requires --freeze PATH')
    if args.freeze:verify_freeze(args.freeze,protocol,args.smoke)
    out=args.out;run_path=out/'run.json';journal=out/'executions.jsonl'
    if run_path.exists():
        if not args.resume:raise FileExistsError('Use --resume or a fresh --out')
        saved=json.loads(run_path.read_text())
        if saved['protocol']!=protocol:raise ValueError('Frozen benchmark protocol differs')
        check_provenance(saved['provenance'])
        if saved.get('freeze_sha256')!=(file_hash(args.freeze) if args.freeze else None):
            raise ValueError('Freeze manifest changed')
        digest=committed_hasher(out,saved,repair=True)
    else:
        if args.resume:raise FileNotFoundError(run_path)
        if out.exists() and any(out.iterdir()):raise FileExistsError('Use an empty benchmark output')
        out.mkdir(parents=True,exist_ok=True);journal.touch();digest=hashlib.sha256()
        saved=dict(format='underpass_seven_policy_benchmark_v1',status='partial',protocol=protocol,
            provenance=metadata(),predictions={},created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            calibration_path=str(data_path.resolve()),freeze_path=str(args.freeze.resolve()) if args.freeze else None,
            freeze_sha256=file_hash(args.freeze) if args.freeze else None,
            journal_bytes=0,journal_sha256=digest.hexdigest(),completed_flights=0,resumptions=0)
        write_json(run_path,saved)
    expected={(s.seed,d,p) for s,_,_ in cases for d in args.delays for p in args.policies};done=set()
    for row in iter_committed(out,saved):
        key=flight_key(row)
        if key in done or key not in expected:raise ValueError('Unexpected or duplicate committed flight')
        done.add(key)
        if row['event']['policy'] in APPROVAL_POLICIES:
            forecast=saved['predictions'][str(row['seed'])]
            if any(row['event'][k]!=forecast[k] for k in ('computation_s','expiry_steps')):
                raise ValueError('Committed flight differs from frozen approval forecast')
    if saved['completed_flights']!=len(done):raise ValueError('Committed flight count differs')
    saved['resumptions']+=int(args.resume);platform=None
    if done!=expected:
        check_memory();began=time.perf_counter();platform=load_scg_platform_params()
        model=LightweightPredictiveModel(*platform)
        try:
            ctrl,obs,info,_=model.start(cases[0][1]);model.step(ctrl.select_action(obs,info),obs,info)
            box=cases[0][0].obstacles[0]
            segment_box_clearance(obs[POS],obs[POS],np.array(box.lower),np.array(box.upper))
        finally:model.close({})
        saved.setdefault('startup_warmups',[]).append(dict(seconds=time.perf_counter()-began,
            description='Platform load; shared PID/RK4 and checker kernel warmup; no full forecast'))
        saved['platform_parameters']=canonical({name:{k:v.tolist() if isinstance(v,np.ndarray) else v
            for k,v in asdict(value).items()} for name,value in zip(('quadrotor','pid'),platform)})
    write_json(run_path,saved)
    trajectories={};rep=out/'representative_trajectories.npz'
    if rep.exists():
        with np.load(rep) as archive:trajectories={k:archive[k].copy() for k in archive.files}
    added=0
    try:
        for s,snapshot,_ in cases:
            if all((s.seed,d,p) in done for d in args.delays for p in args.policies):continue
            received=saved['predictions'].get(str(s.seed))
            if received is None:
                release_memory();received=predict_delays(snapshot,platform,TTL_GRID)
                saved['predictions'][str(s.seed)]=received;write_json(run_path,saved)
            for delay in args.delays:
                for policy in args.policies:
                    key=s.seed,delay,policy
                    if key in done:continue
                    release_memory()
                    row,arrays=execute(s,policy=policy,delay_s=delay,received=received,
                        platform=platform,expected_snapshot=snapshot)
                    assert row['physics_samples']==row['horizon_step']*20+1
                    assert row['geometry_unchanged'] and row['approval_snapshot_verified']
                    if row['initiation_step'] is not None:
                        assert row['pid_unchanged_at_initiation'] and row['physical_state_unchanged_at_initiation']
                        assert row['mode'] in ('dive','sideways') and row['final_position_m'][1]<row['maneuver_anchor_m'][1]
                    row['archived_physical_reference_verified']=verify_physical_reference(row,truth) if truth is not None else False
                    profile_s=s.swerve_start_s+s.swerve_s if row['mode']=='dive' else s.sideways_s
                    row['maneuver_profile_end_approval_s']=None if row['start_s'] is None else row['start_s']+profile_s
                    row['maneuver_profile_truncated_by_horizon']=(row['start_s'] is not None and
                        row['maneuver_profile_end_approval_s']>s.observation_s)
                    row['inspection_observation']=('completed' if row['completed'] else
                        'abandoned_by_recovery' if row['event']['command_canceled'] else 'not_observed_by_fixed_deadline')
                    if s.seed in representatives and delay==protocol['representative_delay_s']:
                        prefix=f'seed{s.seed}_{round(delay*HZ):03d}_{policy}'
                        for kind,array in arrays.items():
                            if kind!='approval_snapshot':trajectories[prefix+'__'+kind]=array
                        temporary=rep.with_name(rep.stem+'.tmp.npz')
                        np.savez_compressed(temporary,**trajectories);temporary.replace(rep)
                    del arrays
                    commit_execution(out,saved,row,digest);done.add(key);added+=1
                    if added%10==0 or len(done)==len(expected):
                        print(f'{len(done)}/{len(expected)} seed={s.seed} D={delay:.2f} {policy}',flush=True)
                    if args.max_flights and added>=args.max_flights:return
    finally:
        # Disk manifest is authoritative even if interruption occurred between
        # journal fsync and metadata replacement. Never commit a stray tail here.
        saved=json.loads(run_path.read_text());committed_hasher(out,saved,repair=True)
        saved.update(status='complete' if saved['completed_flights']==len(expected) else 'partial',
                     updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
        write_json(run_path,saved);write_journal_outputs(out,saved,truth)
        print(f'Saved {saved["completed_flights"]}/{len(expected)} physical flights; new this invocation {added}',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration',type=Path,default=ROOT/'results/development/calibration')
    p.add_argument('--ground-truth',type=Path,help='Optional independent development sweep; never required for held-out')
    p.add_argument('--out',type=Path)
    p.add_argument('--seeds',nargs='+',type=int)
    p.add_argument('--delays',nargs='+',type=float)
    p.add_argument('--policies',nargs='+',choices=POLICIES,default=POLICIES)
    p.add_argument('--representative-seeds',nargs='+',type=int)
    p.add_argument('--smoke',action='store_true')
    p.add_argument('--reduced',action='store_true',help='Frozen 100-of-865 sample (seed 20261108), 13 physical delays')
    p.add_argument('--freeze',type=Path)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--resume',action='store_true')
    p.add_argument('--max-flights',type=int,default=0)
    p.add_argument('--metrics-only',action='store_true')
    a=p.parse_args()
    if a.delays is None:a.delays=SMOKE_DELAYS if a.smoke else REDUCED_DELAYS if a.reduced else DISPATCH_DELAYS
    if a.out is None:
        split=json.loads(manifest_path(a.calibration).read_text()).get('dataset_split','development')
        name=('baseline_benchmark_smoke' if a.smoke else 'baseline_benchmark') if split=='evaluation' else (
            ('baseline_smoke_reduced_v3' if a.smoke else 'baseline_benchmark_reduced_100x13_v3') if a.reduced else
            ('baseline_smoke_2s_v2' if a.smoke else 'baseline_benchmark_2s_v2'))
        a.out=ROOT/'results'/split/name
    if (a.max_flights<0 or not a.policies or not a.delays or len(set(a.policies))!=len(a.policies)
            or len(set(a.delays))!=len(a.delays) or any(not math.isfinite(d) or d<0 or d>=8 or
            not math.isclose(d*HZ,round(d*HZ),abs_tol=1e-9) for d in a.delays)):
        p.error('Use distinct policies and nonnegative control-grid delays below the frozen horizon')
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.metrics_only:
            saved=json.loads((a.out/'run.json').read_text());check_provenance(saved['provenance'])
            committed_hasher(a.out,saved)
            if saved.get('freeze_path'):
                if file_hash(saved['freeze_path'])!=saved['freeze_sha256']:raise ValueError('Freeze manifest changed')
                verify_freeze(saved['freeze_path'],saved['protocol'],saved['protocol']['stage']=='physical_smoke')
            truth,_=load_ground_truth(a.ground_truth) if a.ground_truth else (None,None)
            m=write_journal_outputs(a.out,saved,truth)
            print(m['status'],m['recorded_flights'],'records; no simulation')
        else:run(a)


if __name__=='__main__':main()
