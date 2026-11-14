"""Original calibration search and acceptance criteria; fresh outputs only."""

from pathlib import Path
import sys
from staleguard import underpass_inspection
import argparse
from dataclasses import asdict
import ctypes
import fcntl
import gc
import json
import pickle
import time
import numpy as np
from staleguard.underpass_inspection.storage import write_json, append_json, file_hash, metadata
from staleguard.underpass_inspection.scenario import Scenario, generate_scenario, DEVELOPMENT_RANGES
from staleguard.underpass_inspection.simulation import execute

ROOT = Path(__file__).resolve().parent
experiment_provenance = metadata


def release_memory():
    gc.collect()
    ctypes.CDLL(None).malloc_trim(0)


def verify_snapshot(a, b):
    for name in ('state', 'remaining_reference'):
        np.testing.assert_array_equal(a[name], b[name])
    for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
        np.testing.assert_array_equal(getattr(a['pid_state'], name), getattr(b['pid_state'], name))
    for name in ('scenario', 'approval_time', 'horizon_steps'):
        assert a[name] == b[name]


def calibrate(seeds, out, resume=False, max_scenarios=0, split='development'):
    seeds = list(seeds)
    out = Path(out)
    path = out / 'calibrated_scenarios.json'
    if split not in ('development', 'evaluation'):
        raise ValueError('Use the development or evaluation split')
    if (
        not seeds
        or any(not isinstance(s, int) or isinstance(s, bool) for s in seeds)
        or len(set(seeds)) != len(seeds)
    ):
        raise ValueError('Use distinct integer seeds')
    if split == 'development' and any(s not in range(1, 21) for s in seeds):
        raise ValueError('Use development seeds from 1..20')
    if split == 'evaluation' and any(s <= 20 for s in seeds):
        raise ValueError(
            'Evaluation seeds must be >=21; pilot/development seeds 0..20 are reserved'
        )
    other = 'development' if split == 'evaluation' else 'evaluation'
    if out.resolve().is_relative_to((ROOT / 'results' / other).resolve()):
        raise ValueError(f'The {split} split cannot write inside results/{other}')
    if max_scenarios < 0:
        raise ValueError('max_scenarios must be nonnegative')
    protocol = dict(
        dataset_split=split,
        seeds=seeds,
        generation_ranges=DEVELOPMENT_RANGES,
        sampling=(
            'NumPy default_rng(seed); draw speed, approval, deck front, '
            'absolute pier X independently'
        ),
        approval_quantization_s=0.02,
        bridge_length_m=2.5,
        inspection_rule='Translate fixed seed-0 inspection region by deck-front displacement',
        calibration_rule=(
            'Single draw; nominal unsafe, D=0 safe inspection, immediate sideways safe'
        ),
        allowed_starts_s=[0.0],
        candidate_count=1,
        replacement=False,
        climb='Immediate diagnostic only; never affects acceptance',
    )
    if path.exists() or resume:
        raise FileExistsError('Use a fresh output directory')
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(out)
    out.mkdir(parents=True, exist_ok=True)
    data = dict(
        format='underpass_calibration_v1',
        dataset_split=split,
        protocol=protocol,
        provenance=experiment_provenance(),
        created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        status='partial',
        attempts=[],
        scenarios=[],
        statistics=dict(attempted=0, accepted=0, rejected=0),
    )
    write_json(path, data)
    done = {r['seed'] for r in data['attempts']}
    added = 0
    for seed in seeds:
        if seed in done:
            continue
        s = generate_scenario(seed)
        flights = {}
        snapshot = None
        for mode in ('nominal', 'dive', 'sideways', 'climb'):
            release_memory()
            row, arrays = execute(s, mode, 0.0, capture_snapshot=mode == 'dive')
            flights[mode] = row
            if mode == 'dive':
                snapshot = arrays.pop('approval_snapshot')
            del arrays
        reasons = []
        if flights['nominal']['safe']:
            reasons.append('nominal_safe_through_horizon')
        if not flights['dive']['safe']:
            reasons.append('d0_unsafe:' + flights['dive']['first_violation']['part'])
        if not flights['dive']['inspection_completed']:
            reasons.append('d0_inspection_incomplete')
        if not flights['sideways']['safe']:
            reasons.append(
                'initial_sideways_unsafe:' + flights['sideways']['first_violation']['part']
            )
        entry = dict(
            seed=seed,
            scenario=asdict(s),
            accepted=not reasons,
            rejection_reasons=reasons,
            immediate_flights=flights,
            initial_climb_safe=flights['climb']['safe'],
        )
        if not reasons:
            # Repeat only D=0, independently checking physical and controller state.
            release_memory()
            replay, arrays = execute(s, 'dive', 0.0, capture_snapshot=True)
            assert replay['safe_completion']
            assert replay['physics_sha256'] == flights['dive']['physics_sha256']
            verify_snapshot(snapshot, arrays.pop('approval_snapshot'))
            del arrays
            location = out / f'snapshots/seed_{seed}.pkl'
            location.parent.mkdir(exist_ok=True)
            temporary = location.with_suffix('.tmp')
            temporary.write_bytes(pickle.dumps(snapshot, protocol=pickle.HIGHEST_PROTOCOL))
            temporary.replace(location)
            entry['d0_replay_physics_sha256'] = replay['physics_sha256']
            data['scenarios'].append(
                dict(
                    seed=seed,
                    scenario=asdict(s),
                    approval_snapshot=str(location.relative_to(out)),
                    snapshot_sha256=file_hash(location),
                    d0_verification=flights['dive'],
                    initial_sideways=flights['sideways'],
                    initial_climb_safe=flights['climb']['safe'],
                )
            )
        data['attempts'].append(entry)
        done.add(seed)
        added += 1
        data['statistics'] = dict(
            attempted=len(done),
            accepted=len(data['scenarios']),
            rejected=len(done) - len(data['scenarios']),
        )
        data['status'] = 'complete' if len(done) == len(seeds) else 'partial'
        write_json(path, data)
        print(
            f'seed={seed}: {"accepted" if not reasons else reasons}; {data["statistics"]}',
            flush=True,
        )
        if max_scenarios and added >= max_scenarios:
            break
    return data
