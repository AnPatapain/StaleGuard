"""Shared experiment I/O and scientific input validation; no flight-selection logic."""
from pathlib import Path
import ast
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
from .scenario import (SCHEMA, Config, Command, Scenario, HZ, PHYSICS_HZ, scenario_dict,
                       scenario_from_dict, make_forward_trajectory)

ROOT = Path(__file__).resolve().parent
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
               'baselines/always_recover.py', 'baselines/fbs.py',
               'baseline_benchmark/run.py', 'baseline_benchmark/metrics.py',
               'ttl_pred_accuracy/run.py', 'ttl_pred_accuracy/metrics.py')
    shared = ('model/common.py', 'model/lightweight.py', 'model/pybullet.py',
              'model/scg.py', 'experiment.py', 'physics_monitor.py', 'fbs.py')
    return dict(schema=SCHEMA, config=asdict(config), python=sys.version,
                platform=platform.platform(), numpy=np.__version__,
                numerical_threads=1, cpu_only=True, simulation_workers=1,
                no_recovery_sha256=no_recovery_hash((ROOT/'simulation.py').read_text()),
                environment_source_hashes={name:file_hash(ROOT.parent/name) for name in
                    json.loads((ROOT/'support/logs/provenance.json').read_text())['shared_source_sha256']},
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


def load_calibrated_scenarios(dataset, config=Config()):
    """Load development or evaluation scenarios/snapshots from the supplied path.

    Accept a compact calibration directory/JSON or a historical calibration folder.
    Configuration, provenance, snapshot checksums and references are validated.
    """
    dataset = Path(dataset)
    if not dataset.exists():
        raise FileNotFoundError(f'Calibrated dataset path does not exist: {dataset}')
    if dataset.is_dir() and (dataset/'calibrated_scenarios.json').exists():
        dataset = dataset/'calibrated_scenarios.json'
    if dataset.is_file():
        saved = json.loads(dataset.read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != asdict(config):
            raise ValueError('Calibrated dataset schema/configuration mismatch')
        provenance = saved['provenance']
        if saved.get('format') == 'compact_calibration_v1':
            if provenance.get('schema') != SCHEMA or provenance.get('config') != asdict(config) or not provenance.get('source_hashes') or not provenance.get('shared_source_hashes'):
                raise ValueError('Calibrated dataset provenance is incomplete or incompatible')
        elif file_hash(dataset.parent/provenance['path']) != provenance['sha256']:
            raise ValueError('Calibrated dataset provenance checksum mismatch')
        entries = saved['scenarios']
        if saved.get('format') == 'compact_calibration_v1':
            statistics = saved['statistics']
            attempted = statistics['attempted']
            if statistics['accepted'] != len(entries) or (attempted is not None and
                    (attempted < len(entries) or statistics['rejected'] != attempted-len(entries))):
                raise ValueError('Calibrated dataset acceptance statistics are inconsistent')
        rows = [(entry['scenario'], dataset.parent/entry['approval_snapshot'], entry['snapshot_sha256'])
                for entry in entries]
    else:
        saved = json.loads((dataset/'metadata.json').read_text())
        if saved.get('schema') != SCHEMA or saved['config'] != asdict(config):
            raise ValueError('Calibrated dataset schema/configuration mismatch')
        summary = json.loads((dataset/'summary.json').read_text())
        rows = [(entry['selected'], dataset/f"seed_{entry['seed']}/approval.pkl", entry['snapshot_sha256'])
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
        load_calibrated_scenarios(path,config)
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


# Reviewed original simulator e199ff...: only policy branches were subsequently
# added. Its No-Recovery syntax tree matches the current one exactly. Historical
# source hashes remain untouched; unknown simulator versions are never accepted.
LEGACY_NO_RECOVERY = {
    'e199ff188bfc10f55c5b62b30fac62557b6a4b943a5030818fb3d89e58df9462':
    'da1c66ab5aa28f52520f5add7b7ded29c55e49d836bf39797ae665b6afc3ac7c',
}


def no_recovery_hash(source):
    """Hash the whole simulator with only unreachable policy branches removed."""
    class NoRecovery(ast.NodeTransformer):
        def visit_ImportFrom(self, node):
            policy_imports = {
                'baselines.staleguard': {'staleguard_status'},
                'baselines.fixed_ttl': {'fixed_ttl_expired','FIXED_TTL_S'},
                'baselines.use_time_recheck': {'first_step_after_check'},
                'baselines': {'always_recover','fbs'},
            }
            if node.module in policy_imports and all(
                    alias.name in policy_imports[node.module] and alias.asname is None for alias in node.names):
                return None  # Policy helpers are unused by No-Recovery.
            return node

        def visit_If(self, node):
            self.generic_visit(node)
            test = node.test.values[0] if isinstance(node.test, ast.BoolOp) and isinstance(node.test.op, ast.And) else node.test
            if (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name)
                    and test.left.id == 'policy' and len(test.ops) == 1):
                value, op = ast.literal_eval(test.comparators[0]), test.ops[0]
                truth = ('no_recovery' == value if isinstance(op, ast.Eq) else
                         'no_recovery' in value if isinstance(op, ast.In) else
                         'no_recovery' not in value if isinstance(op, ast.NotIn) else None)
                if truth is False or truth is True and test is node.test:
                    return node.body if truth else node.orelse
            return node
    return hashlib.sha256(ast.dump(NoRecovery().visit(ast.parse(source))).encode()).hexdigest()


def check_ground_truth_source(saved, config):
    """Require the same No-Recovery physics, inputs and numerical environment."""
    current = metadata(config)
    for field in ('schema','config','python','numpy','platform','numerical_threads','cpu_only','simulation_workers'):
        if saved.get(field) != current[field]:
            raise ValueError(f'Ground-truth configuration/environment differs: {field}')
    for name in ('__init__.py','scenario.py'):
        if saved['source_hashes'].get(name) != current['source_hashes'][name]:
            raise ValueError(f'Ground-truth scientific source differs: {name}')
    signature = saved.get('no_recovery_sha256', LEGACY_NO_RECOVERY.get(saved['source_hashes'].get('simulation.py')))
    if signature != current['no_recovery_sha256']:
        raise ValueError('Ground-truth No-Recovery physics/monitor differs: simulation.py')
    shared = saved.get('shared_source_hashes') or json.loads((ROOT/'support/logs/provenance.json').read_text())['shared_source_sha256']
    environment = saved.get('environment_source_hashes') or json.loads((ROOT/'support/logs/provenance.json').read_text())['shared_source_sha256']
    for name, checksum in {**shared, **environment}.items():
        if Path(name).name == 'fbs.py':
            continue  # Another policy's arithmetic cannot affect No-Recovery.
        if file_hash(ROOT.parent/name) != checksum:
            raise ValueError(f'Ground-truth shared model/environment source differs: {name}')


def validate_flight(row, scenario, config):
    """Check actual No-Recovery dispatch and full-horizon 1 kHz safety records."""
    total = round(scenario.approval_s*HZ) + math.ceil(scenario.deadline_s*HZ-1e-10)
    dispatch = round(scenario.approval_s*HZ) + round(row['delay_s']*HZ)
    event, violation = row['event'], row['first_violation_s']
    if (row['seed'] != scenario.seed or not 0 <= row['delay_s'] or dispatch >= total
            or event['policy'] != 'no_recovery'
            or event['fallback_step'] is not None or event['application_step'] != dispatch
            or event['scheduled_dispatch_step'] != dispatch or row['horizon_s'] != total/HZ
            or row['physics_samples'] != total*(PHYSICS_HZ//HZ)+1
            or row['expected_physics_samples'] != row['physics_samples']):
        raise ValueError('Ground truth did not monitor the whole specified No-Recovery flight')
    if (any(type(row[name]) is not bool for name in ('safe','completed','safe_completion'))
            or not math.isfinite(row['min_clearance_m'])
            or row['safe'] != (violation is None)
            or row['safe'] != (row['min_clearance_m'] >= config.d_min_m)
            or row['safe_completion'] != (row['safe'] and row['completed'])
            or row['completed'] != (row['completion_s'] is not None)
            or (row['completion_s'] is not None and not 0 <= row['completion_s'] <= scenario.deadline_s)
            or (violation is not None and not 0 <= violation <= row['horizon_s'])):
        raise ValueError('Ground-truth safety/completion fields disagree')
    predispatch = violation if violation is not None and violation <= dispatch/HZ else None
    if row['pre_dispatch_violation_s'] != predispatch:
        raise ValueError('Ground-truth waiting-period safety differs')
    if row['delay_s'] == 0 and not row['safe_completion']:
        raise ValueError('Calibrated D=0 mission did not complete safely')


def check_approval_snapshot(actual, expected):
    for name in ('state','remaining_reference'):
        np.testing.assert_array_equal(getattr(actual,name),getattr(expected,name))
    for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
        np.testing.assert_array_equal(getattr(actual.pid_state,name),getattr(expected.pid_state,name))
    for name in ('horizon_steps','approval_time','command','fence_x','config','prefix_min_clearance_m'):
        if getattr(actual,name) != getattr(expected,name):
            raise ValueError(f'Approval snapshot differs: {name}')


def load_ground_truth(dataset, cases, config, paths=()):
    """Reuse verified calibration D=0 and saved No-Recovery journals; leave gaps."""
    dataset = Path(dataset)
    if dataset.is_dir():
        dataset = dataset/'calibrated_scenarios.json'
    saved = json.loads(dataset.read_text())
    scenarios = {s.seed:s for s,_,_ in cases}
    snapshots = {s.seed:file_hash(path) for s,_,path in cases}
    actual = {}
    calibration_rows = [entry['d0_verification'] for entry in saved['scenarios'] if 'd0_verification' in entry]
    if calibration_rows:
        check_ground_truth_source(saved['provenance'],config)
        for row in calibration_rows:
            if row['delay_s'] != 0:
                raise ValueError('Calibration verification must be a D=0 flight')
            validate_flight(row,scenarios[row['seed']],config)
            actual[row['seed'],row['delay_s']] = row
    candidates = [Path(path) for path in paths]
    if not candidates:
        candidates = [dataset.parent/'flights.jsonl', dataset.parent.parent/'baseline_benchmark/executions.jsonl']
        if 'source_dataset' in saved:
            source = dataset.parent/saved['source_dataset']['path']
            if file_hash(source) != saved['source_dataset']['sha256']:
                raise ValueError('Source calibration dataset checksum differs')
            candidates.append(source.parent/'flights.jsonl')
    for journal in dict.fromkeys(candidates):
        if not journal.exists():
            if paths:
                raise FileNotFoundError(journal)
            continue
        folder = journal.parent
        rows = [json.loads(line) for line in journal.read_text().splitlines()]
        if not any(row['event']['policy'] == 'no_recovery' for row in rows):
            continue
        if (folder/'run.json').exists():
            run = json.loads((folder/'run.json').read_text())
            check_ground_truth_source(run['provenance'],config)
            protocol = run['protocol']
            if protocol['control_hz'] != HZ:
                raise ValueError('Ground-truth control frequency differs')
            source_scenarios = {row['seed']:scenario_from_dict(row) for row in protocol['scenarios']}
            source_snapshots = protocol['snapshot_sha256']
            data = journal.read_bytes()
            if len(data) != run['journal_bytes'] or hashlib.sha256(data).hexdigest() != run['journal_sha256']:
                raise ValueError('Ground-truth journal differs from its committed results')
        else:
            provenance = json.loads((folder/'metadata.json').read_text())
            check_ground_truth_source(provenance,config)
            protocol = json.loads((folder/'protocol.json').read_text())
            finished = json.loads((folder/'finished.json').read_text())
            original = load_calibrated_scenarios(folder/'calibrated_scenarios.json',config)
            source_scenarios = {s.seed:s for s,_,_ in original}
            source_snapshots = {str(s.seed):file_hash(path) for s,_,path in original}
            if (finished['status'] != 'SUCCESS' or finished['delay_sweep_flights'] != len(rows)
                    or finished['journal_sha256'] != file_hash(journal)
                    or protocol['policy'] != 'no_recovery' or protocol['control_hz'] != HZ
                    or protocol['observation_hz'] != PHYSICS_HZ
                    or protocol['delay_steps'] != [round(d*HZ) for d in protocol['delays_s']]):
                raise ValueError('Ground-truth journal/protocol differs from its completed experiment')
            for name, checksum in protocol['shared_model_sha256'].items():
                if file_hash(ROOT.parent/name) != checksum:
                    raise ValueError(f'Ground-truth shared model source differs: {name}')
        seen = set()
        for row in rows:
            if row['event']['policy'] != 'no_recovery':
                continue
            key = row['seed'],row['delay_s']
            if key in seen or key[0] not in source_scenarios or key[1] not in protocol.get('delays_s',protocol.get('delays',[])):
                raise ValueError(f'Duplicate or unexpected ground-truth flight: {key}')
            seen.add(key)
            if key[0] not in scenarios:
                continue
            if scenarios[key[0]] != source_scenarios[key[0]] or snapshots[key[0]] != source_snapshots[str(key[0])]:
                raise ValueError(f'Ground-truth scenario/command/snapshot differs: {key[0]}')
            validate_flight(row,scenarios[key[0]],config)
            # Historical analysis may add optional diagnostics; compare every
            # shared flight field, without requiring those extras in fresh runs.
            if key in actual and any(row[name] != actual[key][name]
                    for name in row.keys() & actual[key].keys() if name != 'runtime_s'):
                raise ValueError(f'Conflicting ground-truth flights: {key}')
            actual[key] = row
    return actual
