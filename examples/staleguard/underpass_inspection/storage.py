"""Frozen Underpass inputs and simple atomic benchmark I/O.

Historical manifests/snapshots are read-only. Their original hashes are retained.
Inputs and numerical environments are checked without historical path remapping.
New benchmark resumes require exact current source/environment hashes.
"""
from dataclasses import asdict
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import pickle
import platform
import sys
import zipfile
import numpy as np
from .scenario import Scenario,HZ,POS,body_clearances,make_nominal
from .simulation import Snapshot


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as stream:
        stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def append_json(path, value):
    with Path(path).open('a') as stream:
        stream.write(json.dumps(value, allow_nan=False)+'\n')
        stream.flush()
        os.fsync(stream.fileno())


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def flight_key(row):
    return row['seed'], row['delay_s'], row['event']['policy']


ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
CALIBRATION_FORMATS=('underpass_development_calibration_v1','underpass_calibration_v1')
def canonical(value):
    return json.loads(json.dumps(value))


def check_environment(saved):
    """Numerical environment only; old result analysis does not require old drivers."""
    current=metadata()
    for name in ('python','numpy','platform'):
        if saved[name]!=current[name]:raise ValueError('Numerical environment differs: '+name)
    for category in ('packages','shared_sha256','environment_sha256'):
        if category in saved and saved[category]!=current[category]:
            raise ValueError('Numerical environment differs: '+category)
    # Pilot/calibration manifests store shared physics files with project paths.
    # Mission source versions stay recorded in their unchanged input manifests.
    for name,checksum in saved.get('source_sha256',{}).items():
        path=PROJECT/name
        if not path.is_relative_to(ROOT) and (not path.exists() or file_hash(path)!=checksum):
            raise ValueError('Shared physical source changed: '+name)


def check_approval_snapshot(actual, expected):
    def item(value,name):
        return value[name] if isinstance(value,dict) else getattr(value,name)
    for name in ('state','remaining_reference'):
        np.testing.assert_array_equal(item(actual,name),item(expected,name))
    for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
        np.testing.assert_array_equal(getattr(item(actual,'pid_state'),name),
                                      getattr(item(expected,'pid_state'),name))
    for name in ('horizon_steps','approval_time'):
        if item(actual,name)!=item(expected,name):
            raise ValueError(f'Approval snapshot differs: {name}')
    if canonical(asdict(item(actual,'scenario')))!=canonical(asdict(item(expected,'scenario'))):
        raise ValueError('Approval scenario differs')


def load_calibrated_scenarios(dataset):
    path=Path(dataset)
    if path.is_dir():path=path/'calibrated_scenarios.json'
    data=json.loads(path.read_text())
    if data['format'] not in CALIBRATION_FORMATS or data['status']!='complete':
        raise ValueError('Use a completed frozen Underpass calibration manifest')
    if data['format']=='underpass_calibration_v1' and (
            data.get('dataset_split') not in ('development','evaluation') or
            data['dataset_split']!=data['protocol'].get('dataset_split')):
        raise ValueError('Calibration split metadata differs')
    check_environment(data['provenance'])
    cases=[];seen=set()
    for entry in data['scenarios']:
        s=Scenario(**entry['scenario']);p=path.parent/entry['approval_snapshot']
        if s.seed in seen or file_hash(p)!=entry['snapshot_sha256']:
            raise ValueError('Duplicate scenario or changed frozen snapshot')
        raw=pickle.loads(p.read_bytes());seen.add(s.seed)
        if canonical(asdict(raw['scenario']))!=canonical(asdict(s)):
            raise ValueError('Snapshot scenario differs from frozen definition')
        total=round((s.approval_s+s.observation_s)*HZ);approval=round(s.approval_s*HZ)
        if raw['horizon_steps']!=total-approval or raw['approval_time']!=s.approval_s:
            raise ValueError('Frozen approval time/horizon differs')
        np.testing.assert_array_equal(raw['remaining_reference'],make_nominal(total+1,s)[approval:])
        np.testing.assert_array_equal(raw['state'],entry['d0_verification']['approval_state'])
        # Current observed clearance initializes approval prediction. Prior
        # physical violations are retained independently by the real-flight
        # monitor; calibrated approval prefixes are verified on every replay.
        observed=min(body_clearances(raw['state'][POS],s.obstacles).values())
        snapshot=Snapshot(**raw,prefix_min_clearance_m=observed)
        cases.append((s,snapshot,p))
    stats=data['statistics']
    if len(cases)!=stats['accepted'] or stats['attempted']!=stats['accepted']+stats['rejected']:
        raise ValueError('Calibration counts differ')
    return sorted(cases,key=lambda case:case[0].seed)


def metadata():
    local=('__init__.py','scenario.py','simulation.py','calibration.py','pilot.py','storage.py',
        'support/visualization.py','baselines/staleguard.py','baselines/fbs.py',
        'baselines/use_time_recheck.py','baselines/fixed_early_recovery.py','baselines/policies.py',
        'baseline_benchmark/run.py','baseline_benchmark/metrics.py',
        'baselines/recovery_horizons.py','support/validate_baseline_benchmark.py')
    shared=('model/__init__.py','model/common.py','model/lightweight.py','model/scg.py',
        'physics_monitor.py')
    env=('safe_control_gym/controllers/pid/pid.py','safe_control_gym/envs/gym_pybullet_drones/quadrotor.py',
         'safe_control_gym/envs/gym_pybullet_drones/base_aviary.py',
         'safe_control_gym/envs/gym_pybullet_drones/assets/cf2x.urdf')
    return dict(python=sys.version,numpy=np.__version__,platform=platform.platform(),
        packages={name:version(name) for name in ('pybullet','numpy','scipy','gymnasium','numba')},
        simulation_workers=1,numerical_threads=1,control_hz=50,physics_hz=1000,model_hz=100,
        source_sha256={name:file_hash(ROOT/name) for name in local},
        shared_sha256={name:file_hash(ROOT.parent/name) for name in shared},
        environment_sha256={name:file_hash(PROJECT/name) for name in env})


def save_source_archive(path,provenance):
    """Small source snapshot; no result journals or compatibility code."""
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as archive:
        for group,base in (('source_sha256',ROOT),('shared_sha256',ROOT.parent),('environment_sha256',PROJECT)):
            for name,checksum in provenance[group].items():
                source=base/name
                if file_hash(source)!=checksum:raise ValueError('Source changed during preparation')
                archive.write(source,str(source.relative_to(PROJECT)))


def check_provenance(saved):
    if saved!=metadata():raise ValueError('Source/environment changed; use a fresh experiment output')


def prediction_for(run,policy,seed):
    from .baselines.policies import HORIZON_POLICIES
    cache=run.get('predictions_by_policy',{}).get(policy,{}) if policy in HORIZON_POLICIES else run.get('predictions',{})
    return cache.get(str(seed))


def save_inputs(path,cases):
    arrays={}
    for s,snapshot,_ in cases:
        prefix=str(s.seed)
        arrays[prefix+'__state']=snapshot.state
        arrays[prefix+'__reference']=snapshot.remaining_reference
        arrays[prefix+'__pid']=np.array([snapshot.pid_state.integral_pos_e,snapshot.pid_state.last_rpy,snapshot.pid_state.integral_rpy_e])
    temporary=Path(path).with_suffix('.tmp.npz');np.savez_compressed(temporary,**arrays);temporary.replace(path)


def load_inputs(out,run):
    from model.common import PIDState
    path=Path(out)/'approval_snapshots.npz'
    if file_hash(path)!=run['inputs_sha256']:raise ValueError('Frozen approval inputs changed')
    cases=[]
    with np.load(path) as archive:
        for raw in run['protocol']['scenarios']:
            s=Scenario(**raw);key=str(s.seed);memory=archive[key+'__pid']
            snap=Snapshot(state=archive[key+'__state'],remaining_reference=archive[key+'__reference'],
                pid_state=PIDState(*memory),horizon_steps=round(s.observation_s*HZ),approval_time=s.approval_s,
                scenario=s,prefix_min_clearance_m=min(body_clearances(archive[key+'__state'][POS],s.obstacles).values()))
            np.testing.assert_array_equal(snap.remaining_reference,make_nominal(round((s.approval_s+s.observation_s)*HZ)+1,s)[round(s.approval_s*HZ):])
            cases.append((s,snap,None))
    return cases


def _journal(kind):
    return ('executions.jsonl','journal_bytes','journal_sha256','completed_flights') if kind=='flight' else (
        'predictions.jsonl','prediction_bytes','prediction_sha256','completed_predictions')


def committed_hasher(out,run,repair=False,kind='flight'):
    filename,size_key,hash_key,_=_journal(kind)
    path=Path(out)/filename;size=run[size_key];digest=hashlib.sha256()
    with path.open('rb') as stream:
        remaining=size
        while remaining:
            block=stream.read(min(1024*1024,remaining))
            if not block:raise ValueError('Committed journal shortened: '+filename)
            digest.update(block);remaining-=len(block)
        if digest.hexdigest()!=run[hash_key]:raise ValueError('Committed journal changed: '+filename)
        extra=stream.read(1)
    if extra:
        if not repair:raise ValueError('Uncommitted tail; resume first: '+filename)
        with path.open('r+b') as stream:stream.truncate(size);stream.flush();os.fsync(stream.fileno())
    return digest


def iter_committed(out,run,kind='flight'):
    filename,size_key,_,_=_journal(kind)
    with (Path(out)/filename).open('rb') as stream:
        remaining=run[size_key]
        while remaining:
            line=stream.readline(remaining)
            if not line or not line.endswith(b'\n'):raise ValueError('Incomplete committed record')
            remaining-=len(line);yield json.loads(line)


PROGRESS_FIELDS=('status','journal_bytes','journal_sha256','completed_flights','prediction_bytes',
    'prediction_sha256','completed_predictions','config_sha256','resumptions','startup_warmups')


def write_progress(out,run):
    write_json(Path(out)/'progress.json',{k:run[k] for k in PROGRESS_FIELDS if k in run})


def initialize_run(out,config):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    if (out/'run.json').exists():raise FileExistsError(out/'run.json')
    write_json(out/'run.json',config)
    for filename in ('executions.jsonl','predictions.jsonl'):(out/filename).touch()
    empty=hashlib.sha256(b'').hexdigest()
    run=dict(config,status='prepared',journal_bytes=0,journal_sha256=empty,completed_flights=0,
        prediction_bytes=0,prediction_sha256=empty,completed_predictions=0,resumptions=0,
        config_sha256=file_hash(out/'run.json'),predictions={},predictions_by_policy={})
    write_progress(out,run);return run


def read_run(out,repair=False):
    out=Path(out);config=json.loads((out/'run.json').read_text());progress=json.loads((out/'progress.json').read_text())
    if file_hash(out/'run.json')!=progress['config_sha256']:raise ValueError('Experiment configuration changed')
    run=dict(config,**progress,predictions={},predictions_by_policy={})
    committed_hasher(out,run,repair);committed_hasher(out,run,repair,kind='prediction')
    seen=set()
    for row in iter_committed(out,run,kind='prediction'):
        key=row['policy'],row['seed']
        if key in seen:raise ValueError('Duplicate prediction cache entry')
        seen.add(key)
        cache=run['predictions'] if row['policy']=='staleguard' else run['predictions_by_policy'].setdefault(row['policy'],{})
        cache[str(row['seed'])]=row['forecast']
    if len(seen)!=run['completed_predictions']:raise ValueError('Prediction count differs')
    return run


def commit_record(out,run,row,digest,kind='flight'):
    filename,size_key,hash_key,count_key=_journal(kind)
    line=(json.dumps(row,allow_nan=False)+'\n').encode()
    with (Path(out)/filename).open('ab') as stream:stream.write(line);stream.flush();os.fsync(stream.fileno())
    digest.update(line);run.update({size_key:run[size_key]+len(line),hash_key:digest.hexdigest(),count_key:run[count_key]+1})
    write_progress(out,run)


def load_ground_truth(folder):
    folder=Path(folder);run=json.loads((folder/'run.json').read_text())
    if run['status']!='complete':raise ValueError('Use completed physical deadline truth')
    check_environment(run['provenance'])
    raw=(folder/'executions.jsonl').read_bytes()
    if len(raw)!=run['journal_bytes'] or hashlib.sha256(raw).hexdigest()!=run['journal_sha256']:
        raise ValueError('Archived physical truth changed')
    rows=[json.loads(line) for line in raw.splitlines()]
    result={(r['seed'],r['mode'],r['start_s']):r for r in rows}
    if len(result)!=len(rows):raise ValueError('Duplicate archived truth')
    return result,run
