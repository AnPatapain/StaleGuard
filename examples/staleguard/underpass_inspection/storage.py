"""Frozen Underpass inputs, reviewed physical equivalence and benchmark I/O.

Historical manifests/snapshots are read-only. Their original hashes are retained.
Reviewed edits preserve physical mission behavior: policy-guarded additions,
seed-identifier validation, CLI loading and local copies of previously imported
helpers. Syntax hashes pin these exact reviewed versions.
New benchmark resumes require exact current source/environment hashes.
"""
import ast
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


def read_executions(path, repair_tail=False):
    """Read flight journal, rejecting duplicate/inconsistent outcomes."""
    from .baselines.policies import POLICIES
    path = Path(path)
    if not path.exists():
        return []
    raw = path.read_bytes()
    rows, seen, offset = [], set(), 0
    lines = raw.splitlines(keepends=True)
    for index, line in enumerate(lines):
        try:
            row = json.loads(line)
        except (ValueError, UnicodeError) as error:
            if repair_tail and index == len(lines)-1 and not line.endswith(b'\n'):
                with path.open('r+b') as stream:
                    stream.truncate(offset)
                break
            raise ValueError(f'Invalid execution JSON on line {index+1}') from error
        key = flight_key(row)
        if key in seen:
            raise ValueError(f'Duplicate flight in policy journal: {key}')
        if key[2] not in POLICIES or not math.isfinite(key[1]) or key[1] < 0:
            raise ValueError(f'Invalid policy or delay: {key}')
        if any(type(row[name]) is not bool for name in ('safe', 'completed', 'safe_completion')):
            raise ValueError(f'Outcome flags must be Boolean: {key}')
        if row['safe_completion'] != (row['safe'] and row['completed']):
            raise ValueError(f'Inconsistent safety/completion flags: {key}')
        seen.add(key)
        rows.append(row)
        offset += len(line)
    else:
        if repair_tail and raw and not raw.endswith(b'\n'):
            with path.open('ab') as stream:
                stream.write(b'\n')
    return rows


ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
CALIBRATION_FORMATS=('underpass_development_calibration_v1','underpass_calibration_v1')
REVIEWED={
    'calibration.py':('ace91d7fe191bb6ad10b30280ebf91a63f8af1d65a8d66e877c7e9a6db2b7415',
                      '2fb32f630f1a7704abd684aebb235d1ba6dd4b1b7b2d2ee36354f27a2d707c80'),
    'simulation.py':('9413314d21eb8b46e43ce065b4a809bf0b272f7dc1acfee665a504b1554bb787',
                     'e526ddc2c7c8acdbbca5d4efb9fde46eee45478fdf6188f46a0e69bfd010d1fa'),
    'scenario.py':('efec174da7248c4eb15026dea53eb81aaaac884d1733062b1c997a3a2e0aad3d',
                   '44bfdca5637e57569331dafcd433523940e06357e2551ee5b5a3ded036551a74'),
    'pilot.py':('eddadefa80dcaa315b29e44be9f2cf453633022ff559ef007152db78ffc30055',
                '2288161736df74b722983ecc7b8ac3dfe211f38c65499337c968021f59739728')}


def canonical(value):
    return json.loads(json.dumps(value))


def physical_source_hash(source,name):
    class StripPolicy(ast.NodeTransformer):
        def visit_If(self,node):
            expression=ast.unparse(node.test)
            if expression=='policy is not None' or 'isinstance(self.seed, int)' in expression:
                return None
            return self.generic_visit(node)
    tree=ast.parse(source)
    if name=='simulation.py':
        tree=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
        tree.args.args=tree.args.args[:10];tree.args.defaults=tree.args.defaults[:9]
        tree=StripPolicy().visit(tree)
    elif name=='scenario.py':
        tree=StripPolicy().visit(tree)
    elif name=='pilot.py':
        tree.body=[n for n in tree.body if not(isinstance(n,ast.FunctionDef) and n.name=='main')
                   and not(isinstance(n,ast.If) and '__name__' in ast.unparse(n.test))]
    return hashlib.sha256(ast.dump(tree,include_attributes=False).encode()).hexdigest()


def check_frozen_provenance(saved):
    if any(saved[name]!=value for name,value in
           (('python',sys.version),('numpy',np.__version__),('platform',platform.platform()))):
        raise ValueError('Frozen flight numerical environment differs')
    for path,checksum in saved['source_sha256'].items():
        location=PROJECT/path
        if file_hash(location)==checksum:
            continue
        name=location.name
        reviewed=REVIEWED.get(name) if location.parent==ROOT else None
        if (reviewed is None or checksum!=reviewed[0]
                or physical_source_hash(location.read_text(),name)!=reviewed[1]):
            raise ValueError(f'Unreviewed frozen scientific source change: {path}')
    for name,checksum in saved.get('development_sources',{}).items():
        location=ROOT/name
        if file_hash(location)==checksum:
            continue
        reviewed=REVIEWED.get(name)
        if (reviewed is None or checksum!=reviewed[0]
                or physical_source_hash(location.read_text(),name)!=reviewed[1]):
            raise ValueError(f'Frozen development source changed: {name}')


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
    check_frozen_provenance(data['provenance'])
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
        'baseline_benchmark/run.py','baseline_benchmark/metrics.py')
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


def check_provenance(saved):
    if saved!=metadata():
        raise ValueError('Benchmark source/environment changed; use a fresh output')


def read_committed(out,run,repair=False):
    path=Path(out)/'executions.jsonl';raw=path.read_bytes();size=run['journal_bytes']
    if len(raw)<size or hashlib.sha256(raw[:size]).hexdigest()!=run['journal_sha256']:
        raise ValueError('Previously committed flight journal changed')
    return read_executions(path,repair_tail=repair)


def load_ground_truth(folder):
    folder=Path(folder);run=json.loads((folder/'run.json').read_text())
    if run['status']!='complete':raise ValueError('Use completed physical deadline truth')
    check_frozen_provenance(run['provenance'])
    raw=(folder/'executions.jsonl').read_bytes()
    if len(raw)!=run['journal_bytes'] or hashlib.sha256(raw).hexdigest()!=run['journal_sha256']:
        raise ValueError('Archived physical truth changed')
    rows=[json.loads(line) for line in raw.splitlines()]
    result={(r['seed'],r['mode'],r['start_s']):r for r in rows}
    if len(result)!=len(rows):raise ValueError('Duplicate archived truth')
    return result,run


def verify_physical_reference(row,truth):
    policy=row['event']['policy']
    if policy=='no_recovery':key=(row['seed'],'dive',row['delay_s'])
    elif row['event']['fallback_step'] is not None:
        key=(row['seed'],'sideways',row['start_s'])
    elif row['event']['application_step'] is not None:
        key=(row['seed'],'dive',row['start_s'])
    else:return False
    if key not in truth:
        if policy in ('no_recovery','always_recover'):
            raise ValueError(f'Missing required reference flight {key}')
        return False
    expected=truth[key]
    for name in ('safe','inspection_completed','safe_completion','min_clearance_m',
                 'first_violation','physics_sha256'):
        if row[name]!=expected[name]:
            raise ValueError(f'Physical reference mismatch: {key}, {name}')
    return True
