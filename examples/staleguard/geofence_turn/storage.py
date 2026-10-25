"""Shared calibration files and dataset validation; no simulation or selection logic."""
from pathlib import Path
from dataclasses import asdict
import hashlib
import json
import math
import os
import pickle
import platform
import sys
import time
import numpy as np
from .scenario import (SCHEMA, Config, Command, Scenario, HZ, scenario_dict,
                       scenario_from_dict, make_forward_trajectory)

ROOT = Path(__file__).resolve().parent
DEFAULT_DEVELOPMENT = ROOT/'support/logs/northbound_review/development'
DEFAULT_ENVELOPE = ROOT/'support/logs/northbound_review/characterization/envelope.json'

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


def metadata(config):
    # Explicit experiment sources; do not scan the repository or hash archives.
    sources = ('__init__.py', 'scenario.py', 'simulation.py', 'calibration.py', 'storage.py',
               'baselines/staleguard.py', 'baselines/use_time_recheck.py', 'baselines/fixed_ttl.py',
               'baseline_benchmark/run.py', 'baseline_benchmark/metrics.py',
               'ttl_pred_accuracy/run.py', 'ttl_pred_accuracy/metrics.py')
    shared = ('model/common.py', 'model/lightweight.py', 'model/pybullet.py',
              'model/scg.py', 'experiment.py', 'physics_monitor.py')
    return dict(schema=SCHEMA, config=asdict(config), python=sys.version,
                platform=platform.platform(), numpy=np.__version__,
                numerical_threads=1, cpu_only=True, simulation_workers=1,
                source_hashes={name:file_hash(ROOT/name) for name in sources},
                shared_source_hashes={str(ROOT.parent/'inspection_descent'/name):
                    file_hash(ROOT.parent/'inspection_descent'/name) for name in shared},
                started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))


def check_provenance(saved, config):
    current = metadata(config)
    for field in ('schema','config','source_hashes','shared_source_hashes','python','numpy'):
        if saved.get(field) != current[field]:
            raise ValueError(f'Resume source/layout changed or configuration/environment differs: {field}')


def load_snapshot(path):
    """Read frozen snapshots, including class names from the old mission.py.

    The mapping changes only where Python finds the same three data classes;
    snapshot bytes and all dataset compatibility checks stay unchanged.
    """
    class Reader(pickle.Unpickler):
        def find_class(self, module, name):
            if module == 'geofence_turn.mission' and name in ('Command','Config','Scenario'):
                return {'Command':Command, 'Config':Config, 'Scenario':Scenario}[name]
            return super().find_class(module, name)
    with Path(path).open('rb') as stream:
        return Reader(stream).load()


def load_development(development, config=Config()):
    """Read compact or historical calibrated inputs, retaining compatibility checks."""
    development = Path(development)
    if development.is_dir() and (development/'calibrated_scenarios.json').exists():
        development = development/'calibrated_scenarios.json'
    if development.is_file():
        saved = json.loads(development.read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != asdict(config):
            raise ValueError('Calibrated dataset schema/configuration mismatch')
        provenance = saved['provenance']
        if saved.get('format') == 'compact_calibration_v1':
            if provenance.get('schema') != SCHEMA or provenance.get('config') != asdict(config) or not provenance.get('source_hashes') or not provenance.get('shared_source_hashes'):
                raise ValueError('Calibrated dataset provenance is incomplete or incompatible')
        elif file_hash(development.parent/provenance['path']) != provenance['sha256']:
            raise ValueError('Calibrated dataset provenance checksum mismatch')
        entries = saved['scenarios']
        if saved.get('format') == 'compact_calibration_v1':
            statistics = saved['statistics']
            attempted = statistics['attempted']
            if statistics['accepted'] != len(entries) or (attempted is not None and
                    (attempted < len(entries) or statistics['rejected'] != attempted-len(entries))):
                raise ValueError('Calibrated dataset acceptance statistics are inconsistent')
        rows = [(entry['scenario'], development.parent/entry['approval_snapshot'], entry['snapshot_sha256'])
                for entry in entries]
    else:
        saved = json.loads((development/'metadata.json').read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != asdict(config):
            raise ValueError('Development dataset schema/configuration mismatch')
        summary = json.loads((development/'summary.json').read_text())
        rows = [(entry['selected'], development/f"seed_{entry['seed']}/approval.pkl", entry['snapshot_sha256'])
                for entry in summary['scenarios']]
    cases = []
    seen = set()
    for row, path, checksum in rows:
        scenario = scenario_from_dict(row)
        if scenario.seed in seen:
            raise ValueError(f'Duplicate calibrated seed: {scenario.seed}')
        seen.add(scenario.seed)
        if file_hash(path) != checksum:
            raise ValueError(f'Approval snapshot checksum mismatch: {path}')
        snapshot = load_snapshot(path)
        if (snapshot.config != config or snapshot.command != scenario.command
                or snapshot.fence_x != scenario.fence_x):
            raise ValueError(f'Approval snapshot does not match scenario/configuration: {path}')
        approval = round(scenario.approval_s*HZ)
        horizon = math.ceil(scenario.deadline_s*HZ-1e-10)
        expected = make_forward_trajectory(approval+horizon+1, scenario)[approval:]
        if (snapshot.approval_time != scenario.approval_s or snapshot.horizon_steps != horizon
                or not np.array_equal(snapshot.remaining_reference, expected)):
            raise ValueError(f'Approval time/reference does not match scenario: {path}')
        cases.append((scenario, snapshot, path))
    return sorted(cases, key=lambda item:item[0].seed)


def open_calibration(output, seeds, target, envelope_path, config, resume):
    """Start a new output or validate a completed-scenario checkpoint."""
    output = Path(output)
    path = output/'calibrated_scenarios.json'
    settings = dict(seeds=list(seeds), target=target, envelope_sha256=file_hash(envelope_path))
    if resume:
        data = json.loads(path.read_text())
        if data.get('format') != 'compact_calibration_v1' or data['calibration']['settings'] != settings:
            raise ValueError('Resume calibration inputs/settings differ; frozen historical datasets are read-only inputs')
        check_provenance(data['provenance'],config)
        load_development(path,config)
        return data
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Output contains data; use --resume or a fresh directory')
    (output/'snapshots').mkdir(parents=True,exist_ok=True)
    return dict(format='compact_calibration_v1',schema=SCHEMA,config=asdict(config),
                provenance=metadata(config),scenarios=[],statistics={},
                calibration=dict(status='running',settings=settings,attempts=[]))


def save_snapshot(path, snapshot):
    temporary = path.with_suffix('.pkl.tmp')
    with temporary.open('wb') as stream:
        pickle.dump(snapshot,stream)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    return file_hash(path)


def save_progress(output, data):
    attempted = len(data['calibration']['attempts'])
    accepted = len(data['scenarios'])
    data['statistics'] = dict(generated=len(data['calibration']['settings']['seeds']),
        attempted=attempted,accepted=accepted,rejected=attempted-accepted,
        acceptance_rate=accepted/attempted if attempted else None)
    write_json(Path(output)/'calibrated_scenarios.json',data)
