"""Small, explicit frozen-data I/O; historical files are always read-only inputs."""
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace
import json
import os
import pickle
from staleguard._relocation import SnapshotUnpickler
import platform
import sys
import numpy as np
from .scenario import (ApprovalSnapshot, Scenario, configuration, generate_scenario, make_forward_trajectory,
                       CTRL_FREQ_HZ, MEASURE_SECONDS)
from staleguard.geofence_turn.storage import write_json, append_json, file_hash, check_source

ROOT = Path(__file__).resolve().parent
HISTORY = ROOT/'independent_evaluation_20_319'
DEV_BUNDLE = ROOT/'results/development/temporal_safety_correction/prediction_inputs.pkl'
SCIENCE = ('scenario.py','simulation.py','baselines/staleguard.py','baselines/use_time_recheck.py',
           'baselines/fbs.py','baselines/fixed_ttl.py','baselines/always_recover.py',
           'baselines/fixed_early_recovery.py','baselines/ppr.py','calibration.py',
           '../model/__init__.py','../model/common.py','../model/lightweight.py',
           '../model/pybullet.py','../model/scg.py','../physics_monitor.py')


def check_memory():
    """Original 2048 MiB process cap and 2048 MiB available-system reserve."""
    status = Path('/proc/self/status').read_text().splitlines()
    rss = int(next(line.split()[1] for line in status if line.startswith('VmRSS:'))) / 1024
    memory = Path('/proc/meminfo').read_text().splitlines()
    available = int(next(line.split()[1] for line in memory if line.startswith('MemAvailable:'))) / 1024
    if rss > 2048 or available < 2048:
        raise RuntimeError(f'Memory budget reached: process={rss:.0f} MiB, available={available:.0f} MiB')
    return rss, available


def metadata():
    names = SCIENCE + ('__init__.py','storage.py','baseline_benchmark/run.py',
        'baseline_benchmark/metrics.py','ttl_pred_accuracy/run.py','ttl_pred_accuracy/metrics.py',
        '../source_relocation.json')
    env = ('../safe_control_gym/controllers/pid/pid.py',
           '../safe_control_gym/envs/gym_pybullet_drones/quadrotor.py',
           '../safe_control_gym/envs/gym_pybullet_drones/assets/cf2x.urdf')
    return dict(config=configuration(), python=sys.version, numpy=np.__version__,
        platform=platform.platform(), numerical_threads=1, cpu_only=True,
        source_hashes={n:file_hash(ROOT/n) for n in names},
        workflow_hashes={n:file_hash(ROOT.parent/n) for n in
            ('geofence_turn/__init__.py','geofence_turn/storage.py',
             'geofence_turn/baseline_benchmark/metrics.py','geofence_turn/ttl_pred_accuracy/metrics.py')},
        environment_hashes={n:file_hash(ROOT.parent/n) for n in env})


def check_provenance(saved):
    if saved != metadata():
        raise ValueError('Source/configuration/environment changed; use a fresh output directory')


def load_pickle(path):
    """Read unchanged frozen data after moving its two dataclass definitions.

    This is a data-format reader, not an import alias or legacy source module.
    New snapshots serialize the canonical inspection_descent.scenario classes.
    """
    class Reader(SnapshotUnpickler):
        def find_class(self, module, name):
            if module == 'experiment' and name in ('Scenario','ApprovalSnapshot'):
                return {'Scenario':Scenario, 'ApprovalSnapshot':ApprovalSnapshot}[name]
            return super().find_class(module, name)
    with Path(path).open('rb') as stream:
        return Reader(stream).load()


def resolve_input(path):
    """Locate byte-preserved data moved into development/evaluation folders."""
    path = Path(path).resolve()
    moves = json.loads((ROOT.parent/'source_relocation.json').read_text())['data']
    for old,new in moves.items():
        previous = ROOT.parent/old
        if path == previous or previous in path.parents:
            return ROOT.parent/new/path.relative_to(previous)
    return path


def check_scientific_sources(sources):
    for name,checksum in sources.items():
        check_source(ROOT/name,checksum)


def check_approval_snapshot(actual, expected):
    for name in ('state','remaining_reference'):
        np.testing.assert_array_equal(getattr(actual,name),getattr(expected,name))
    for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
        np.testing.assert_array_equal(getattr(actual.pid_state,name),getattr(expected.pid_state,name))
    for name in ('approval_time','horizon_steps','low_height','low_steps',
                 'box_x_min','box_x_max','box_y_limit','unsafe_below_z'):
        if getattr(actual,name) != getattr(expected,name):
            raise ValueError(f'Approval snapshot differs: {name}')


def save_snapshot(path, snapshot):
    path = Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temporary = path.with_suffix('.pkl.tmp')
    with temporary.open('wb') as stream:
        pickle.dump(snapshot,stream,pickle.HIGHEST_PROTOCOL)
        stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)
    return file_hash(path)


def load_calibrated_scenarios(dataset):
    dataset = Path(dataset)
    if dataset.is_dir():
        dataset /= 'calibrated_scenarios.json'
    data = json.loads(dataset.read_text())
    if data.get('format') != 'inspection_calibration_v1' or data['config'] != configuration():
        raise ValueError('Use the standardized calibration manifest; import historical data explicitly')
    check_scientific_sources(data['scientific_sources'])
    for name,checksum in data.get('historical_sources',{}).items():
        if file_hash(resolve_input(dataset.parent/name)) != checksum:
            raise ValueError(f'Historical input changed: {name}')
    rows=[]; seen=set()
    for entry in data['scenarios']:
        seed=entry['seed']; scenario=Scenario(**entry['scenario'])
        if seed in seen:
            raise ValueError(f'Duplicate calibrated seed: {seed}')
        seen.add(seed)
        path=dataset.parent/entry['approval_snapshot']
        if file_hash(path) != entry['snapshot_sha256']:
            raise ValueError(f'Snapshot checksum mismatch: {path}')
        snapshot=load_pickle(path)
        step=round(scenario.approval_time*CTRL_FREQ_HZ)
        expected=make_forward_trajectory(SimpleNamespace(
            CTRL_FREQ=CTRL_FREQ_HZ,X_GOAL=np.zeros((step+len(snapshot.remaining_reference),12))),scenario)[step:]
        if (snapshot.approval_time != scenario.approval_time
                or snapshot.horizon_steps != round(MEASURE_SECONDS*CTRL_FREQ_HZ)-step
                or snapshot.low_steps != round(scenario.low_seconds*CTRL_FREQ_HZ)
                or any(getattr(snapshot,n) != getattr(scenario,n) for n in
                    ('low_height','box_x_min','box_x_max','box_y_limit','unsafe_below_z'))
                or not np.array_equal(snapshot.remaining_reference,expected)
                or not np.all(np.isfinite(snapshot.state))):
            raise ValueError(f'Snapshot scenario/reference differs: seed {seed}')
        rows.append((seed,scenario,snapshot,path))
    stats=data['statistics']
    if stats['accepted'] != len(rows) or stats['attempted'] != stats['accepted']+stats['rejected']:
        raise ValueError('Calibration denominator/counts disagree')
    return sorted(rows,key=lambda row:row[0])


def import_calibration(kind, output):
    """Explicit byte-preserving historical migration, with no physical flights."""
    output=Path(output); target=output/'calibrated_scenarios.json'
    if target.exists():
        load_calibrated_scenarios(target)
        return json.loads(target.read_text())
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Choose an empty calibration output')
    output.mkdir(parents=True,exist_ok=True)
    source=ROOT/'results/development/calibration/generated_scenarios.json' if kind=='development' else HISTORY/'main_result/calibrated_scenarios.json'
    original=json.loads(source.read_text())
    bundle=None
    if kind=='development':
        bundle=load_pickle(DEV_BUNDLE)
        if bundle['calibration_sha256'] != file_hash(source):
            raise ValueError('Development frozen snapshot bundle/calibration mismatch')
    entries=[]
    for seed,item in sorted(original.items(),key=lambda pair:int(pair[0])):
        seed=int(seed)
        if item['generated_scenario'] != asdict(generate_scenario(seed)):
            raise ValueError(f'Historical generator mismatch: {seed}')
        if item['command'] is None:
            continue
        if bundle is None:
            path=HISTORY/f'scenario_checkpoints/{seed}/approval.pkl'
        else:
            path=output/f'snapshots/seed_{seed}.pkl'
            save_snapshot(path,bundle['snapshots'][seed])
        entries.append(dict(seed=seed,scenario=item['command'],
            generated_scenario=item['generated_scenario'],
            approval_snapshot=os.path.relpath(path,output),snapshot_sha256=file_hash(path),
            best_safe_dwell_ticks=item['best_safe_dwell_ticks']))
    sources={os.path.relpath(source,output):file_hash(source)}
    if bundle is not None:
        sources[os.path.relpath(DEV_BUNDLE,output)]=file_hash(DEV_BUNDLE)
    data=dict(format='inspection_calibration_v1',origin='historical_reference',dataset_kind=kind,
        config=configuration(),provenance=metadata(),
        scientific_sources={n:file_hash(ROOT/n) for n in SCIENCE},historical_sources=sources,
        scenarios=entries,attempted_seeds=list(map(int,original)),
        rejected_seeds=[int(k) for k,v in original.items() if v['command'] is None],
        statistics=dict(attempted=len(original),accepted=len(entries),rejected=len(original)-len(entries)))
    write_json(target,data);load_calibrated_scenarios(target)
    return data


HISTORICAL_NAMES={'No Recovery':'no_recovery','Always-Recover':'always_recover',
    'StaleGuard':'staleguard','Use-Time Recheck':'use_time_recheck','FBS':'fbs','Fixed TTL (0.3s)':'fixed_ttl'}


def historical_benchmark():
    """Validate the original aggregate journal against its published core manifest."""
    folder=HISTORY/'main_result';path=folder/'executions.jsonl'
    manifest=json.loads((folder/'core_manifest.json').read_text())
    item=next(r for r in manifest['original_outputs'] if r['file']=='executions.jsonl')
    if file_hash(path) != item['sha256']:
        raise ValueError('Historical benchmark checksum mismatch')
    return path,[json.loads(line) for line in path.read_text().splitlines()]


def normalize_historical(row):
    """Schema conversion only; do not manufacture missing physical measurements."""
    policy=HISTORICAL_NAMES[row['policy']]; complete,unsafe,recover,safe_complete=row['outcome']
    approval,dispatch=row['approval_step'],row['dispatch_step']
    event=dict(row['event'],policy=policy,scheduled_dispatch_step=dispatch,
        computation_s=row['prediction_computation_s'],prediction_ready=row['prediction_ready'])
    fallback=approval if policy=='always_recover' and recover else dispatch if recover else None
    if policy=='use_time_recheck':
        fallback=event.get('fallback_step')
    event.update(fallback_step=fallback,application_step=
        event.get('application_step') if policy=='use_time_recheck' else
        dispatch if row['dispatch_reached'] and not recover else None,
        early_recovery=fallback is not None and fallback<dispatch)
    first=row['first_unsafe_time_s']
    return dict(seed=row['seed'],delay_s=row['delay_steps']/CTRL_FREQ_HZ,
        safe=not unsafe,completed=complete,safe_completion=safe_complete,
        completion_s=None,first_violation_s=first,
        pre_dispatch_violation_s=first if first is not None and first<=dispatch/CTRL_FREQ_HZ else None,
        physics_samples=row['last_physics_tick']+1,monitored_until_s=row['last_physics_tick']/1000,
        horizon_s=row['deadline_step']/CTRL_FREQ_HZ,
        control_sha256=row['control_sha256'],physics_sha256=row['physics_sha256'],
        event=event,runtime_s=None,execution_kind=row['execution_kind'],
        origin='historical_schema_conversion')
