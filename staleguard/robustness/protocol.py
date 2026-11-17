"""Immutable E01 design, deterministic uncertainty, and atomic primary-record I/O."""

from dataclasses import asdict
from pathlib import Path
import hashlib
import json
import os

import numpy as np

from staleguard.io import file_hash
from staleguard.reproduce.common import Data
from staleguard.underpass_inspection.baselines.staleguard import TTL_GRID

REPO = Path(__file__).resolve().parents[2]
ROOT = REPO / 'outputs/robustness_v1'
MISSION = 'underpass_inspection'
CONDITIONS = ('N0', 'A1', 'A2', 'B1', 'B2')
POLICIES = (
    'no_recovery',
    'use_time_recheck',
    'fixed_early_recovery',
    'staleguard_early_recovery',
    'staleguard_recovery_aware',
    'staleguard_dual_horizon',
)
from staleguard.policies import LABELS as PAPER_LABELS

LABELS = {name: PAPER_LABELS[name] for name in POLICIES}

SEVERITY = {
    'N0': (0.0, 0.0, 0.0),
    'A1': (0.05, 0.0, 0.0),
    'A2': (0.10, 0.0, 0.0),
    'B1': (0.0, 0.01, 0.02),
    'B2': (0.0, 0.03, 0.05),
}
BOOTSTRAP_SEED = 2030


def digest(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    ).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def save_record(path, payload):
    wrapped = dict(payload=payload, sha256=digest(payload))
    if Path(path).exists():
        if load_record(path) != payload:
            raise ValueError(f'Refusing to overwrite completed record: {path}')
        return
    atomic_json(path, wrapped)


def load_record(path):
    wrapped = json.loads(Path(path).read_text())
    if wrapped['sha256'] != digest(wrapped['payload']):
        raise ValueError(f'Corrupted E01 record: {path}')
    return wrapped['payload']


def environment():
    from staleguard.io import metadata

    return metadata()


def freeze(data_root):
    """Selection and specification are written before any evaluation flight."""
    from staleguard.model.scg import load_scg_platform_params

    data = Data(data_root)
    run = data.run(MISSION + '/benchmark')
    scenarios = run['protocol']['scenarios']
    population = sorted(s['seed'] for s in scenarios)
    if len(population) != 400 or len(set(population)) != 400:
        raise ValueError('E01 requires the original complete 400-scenario evaluation cohort')
    selected = np.random.default_rng(2027).choice(population, size=64, replace=False).tolist()
    ordered = sorted(selected)
    signs = np.random.default_rng(2028).permutation([1] * 32 + [-1] * 32).tolist()
    vectors = np.random.default_rng(2029).standard_normal((64, 2, 3)).tolist()
    selection = dict(
        selection_seed=2027,
        selected_seeds=selected,
        execution_order=ordered,
        mass_sign_seed=2028,
        observation_noise_seed=2029,
        assignment_order='sorted selected scenario IDs',
        mass_signs=dict(zip(map(str, ordered), signs)),
        standardized_errors=dict(zip(map(str, ordered), vectors)),
    )
    path = ROOT / 'selection.json'
    if path.exists() and json.loads(path.read_text()) != selection:
        raise ValueError('Frozen E01 selection differs')
    if not path.exists():
        atomic_json(path, selection)
    platform = load_scg_platform_params()
    physical = dict(
        mass=platform[0].mass,
        inertia=platform[0].inertia.tolist(),
        gravity=platform[0].gravity,
        arm_length=platform[0].arm_length,
        kf=platform[0].kf,
        km=platform[0].km,
    )
    spec = dict(
        experiment='E01',
        mission=MISSION,
        **selection,
        conditions=list(CONDITIONS),
        policies=list(POLICIES),
        severity={
            k: dict(mass_fraction=v[0], position_sigma_m=v[1], velocity_sigma_mps=v[2])
            for k, v in SEVERITY.items()
        },
        scenarios=[s for s in scenarios if s['seed'] in selected],
        policy_delays=run['protocol']['delays'],
        prediction_delays=list(TTL_GRID),
        control_hz=50,
        physics_hz=1000,
        model_hz=100,
        prediction_horizon_s=1.0,
        expected_h3_outcomes=24960,
        expected_full_profiles=320,
        expected_mass_feasibility_evaluations=4864,
        nominal_physical_parameters=physical,
        bootstrap=dict(
            unit='whole scenario',
            seed=BOOTSTRAP_SEED,
            resamples=2000,
            confidence=0.95,
            shared_indices_across_conditions=True,
        ),
        assumptions=[
            'Mass changes only PyBullet base-link mass; inertia and nominal PID/model stay fixed.',
            'Bias affects supervisory snapshots only; PID sees exact true states.',
            'Orientation, angular rates, PID memory, reference and prefix safety remain exact.',
            'One warmed full forecast per scenario-condition is shared by three StaleGuard ablations.',
            'Use-Time Recheck is independently timed on the biased dispatch snapshot.',
            'No dispatch delay or physical feasibility labels enter approval-time prediction.',
            'Original command/recovery first-loss and completion later-success rules remain unchanged.',
            'Offline measured computation is replayed on the 50 Hz simulation clock.',
            'Nominal archive comparisons separate fresh timing changes from physical discrepancies.',
        ],
        data_manifest_sha256=file_hash(data.root / 'manifest.json'),
        original_inputs={
            str(p.relative_to(data.root)): file_hash(p)
            for p in (
                data.file(data.entry(MISSION + '/benchmark')['files']['run.json']),
                data.file(data.entry(MISSION + '/benchmark')['files']['executions.jsonl']),
                data.file(data.entry(MISSION + '/benchmark')['files']['predictions.jsonl']),
                data.file(data.entry(MISSION + '/feasibility')['files']['executions.jsonl']),
                data.file(
                    data.entry(MISSION + '/calibration')['files']['calibrated_scenarios.json']
                ),
            )
        },
        environment=environment(),
    )
    assert len(spec['policy_delays']) == 13 and len(TTL_GRID) == 19
    path = ROOT / 'specification.json'
    if path.exists():
        existing = json.loads(path.read_text())
        # The protocol is immutable. Source changes belong to execution manifests.
        left = {k: v for k, v in spec.items() if k != 'environment'}
        right = {k: v for k, v in existing.items() if k != 'environment'}
        if left != right:
            raise ValueError('E01 specification differs from its frozen protocol')
        if file_hash(path) != (ROOT / 'specification.sha256').read_text().strip():
            raise ValueError('Frozen specification checksum differs')
        return existing
    atomic_json(path, spec)
    (ROOT / 'specification.sha256').write_text(file_hash(path) + '\n')
    return spec


def perturbation(spec, condition, seed):
    mass, position, velocity = SEVERITY[condition]
    vectors = np.asarray(spec['standardized_errors'][str(seed)])
    return dict(
        condition=condition,
        seed=int(seed),
        mass_factor=1.0 + mass * spec['mass_signs'][str(seed)],
        mass_sign=spec['mass_signs'][str(seed)],
        position_bias_m=(position * vectors[0]).tolist(),
        velocity_bias_mps=(velocity * vectors[1]).tolist(),
    )


def plant_name(condition):
    return condition if condition in ('A1', 'A2') else 'N0'
