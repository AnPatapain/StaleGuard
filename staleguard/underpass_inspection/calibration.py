"""Development or held-out feasibility screening: one draw/seed, D=0 only."""
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

from staleguard.underpass_inspection.storage import write_json, append_json, file_hash, CALIBRATION_FORMATS
from staleguard.underpass_inspection.scenario import Scenario, generate_scenario, DEVELOPMENT_RANGES
from staleguard.underpass_inspection.simulation import execute
from staleguard.underpass_inspection.pilot import provenance

ROOT = Path(__file__).resolve().parent


def experiment_provenance():
    data = provenance(Scenario())
    data['development_sources'] = {name: file_hash(ROOT/name) for name in
                                   ('calibration.py', 'support/deadline_sweep.py')}
    # JSON is the frozen format: tuple/list differences must not break resume.
    return json.loads(json.dumps(data))


def check_provenance(saved):
    if saved != experiment_provenance():
        raise ValueError('Source/environment changed; use a fresh experiment output')


def release_memory():
    gc.collect()
    ctypes.CDLL(None).malloc_trim(0)


def manifest_path(path):
    path = Path(path)
    return path/'calibrated_scenarios.json' if path.is_dir() else path


def load_calibrated(path, verify_sources=True):
    path = manifest_path(path)
    data = json.loads(path.read_text())
    if data.get('format') not in CALIBRATION_FORMATS:
        raise ValueError('Not an Underpass calibration')
    if data['format']=='underpass_calibration_v1' and (
            data.get('dataset_split') not in ('development','evaluation') or
            data['dataset_split']!=data['protocol'].get('dataset_split')):
        raise ValueError('Calibration split metadata differs')
    if verify_sources:
        check_provenance(data['provenance'])
    cases = []
    seen = set()
    for row in data['scenarios']:
        s = Scenario(**row['scenario'])
        if s.seed in seen or file_hash(path.parent/row['approval_snapshot']) != row['snapshot_sha256']:
            raise ValueError('Duplicate seed or changed approval snapshot')
        seen.add(s.seed)
        cases.append(s)
    return sorted(cases, key=lambda s:s.seed), data


def verify_snapshot(a, b):
    for name in ('state', 'remaining_reference'):
        np.testing.assert_array_equal(a[name], b[name])
    for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
        np.testing.assert_array_equal(getattr(a['pid_state'], name), getattr(b['pid_state'], name))
    for name in ('scenario', 'approval_time', 'horizon_steps'):
        assert a[name] == b[name]


def calibrate(seeds, out, resume=False, max_scenarios=0, split='development'):
    seeds = list(seeds); out = Path(out); path = out/'calibrated_scenarios.json'
    if split not in ('development','evaluation'):
        raise ValueError('Use the development or evaluation split')
    if (not seeds or any(not isinstance(s,int) or isinstance(s,bool) for s in seeds)
            or len(set(seeds)) != len(seeds)):
        raise ValueError('Use distinct integer seeds')
    if split=='development' and any(s not in range(1,21) for s in seeds):
        raise ValueError('Use development seeds from 1..20')
    if split=='evaluation' and any(s<=20 for s in seeds):
        raise ValueError('Evaluation seeds must be >=21; pilot/development seeds 0..20 are reserved')
    other='development' if split=='evaluation' else 'evaluation'
    if out.resolve().is_relative_to((ROOT/'results'/other).resolve()):
        raise ValueError(f'The {split} split cannot write inside results/{other}')
    if max_scenarios<0:
        raise ValueError('max_scenarios must be nonnegative')
    protocol = dict(dataset_split=split,seeds=seeds, generation_ranges=DEVELOPMENT_RANGES,
        sampling='NumPy default_rng(seed); draw speed, approval, deck front, absolute pier X independently',
        approval_quantization_s=.02, bridge_length_m=2.5,
        inspection_rule='Translate fixed seed-0 inspection region by deck-front displacement',
        calibration_rule='Single draw; nominal unsafe, D=0 safe inspection, immediate sideways safe',
        allowed_starts_s=[0.], candidate_count=1, replacement=False,
        climb='Immediate diagnostic only; never affects acceptance')
    if path.exists():
        if not resume:
            raise FileExistsError('Calibration exists; use --resume or a fresh --out')
        data = json.loads(path.read_text())
        if data['protocol'] != protocol or data.get('dataset_split')!=split:
            raise ValueError('Calibration resume protocol differs')
        check_provenance(data['provenance'])
        load_calibrated(path)
    else:
        if resume:
            raise FileNotFoundError(path)
        if out.exists() and any(out.iterdir()):
            raise FileExistsError(out)
        out.mkdir(parents=True, exist_ok=True)
        data = dict(format='underpass_calibration_v1', dataset_split=split, protocol=protocol,
                    provenance=experiment_provenance(), created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                    status='partial', attempts=[], scenarios=[],
                    statistics=dict(attempted=0,accepted=0,rejected=0))
        write_json(path,data)
    done = {r['seed'] for r in data['attempts']}
    added = 0
    for seed in seeds:
        if seed in done:
            continue
        s = generate_scenario(seed)
        flights = {}; snapshot = None
        for mode in ('nominal','dive','sideways','climb'):
            release_memory()
            row, arrays = execute(s, mode, 0., capture_snapshot=mode=='dive')
            flights[mode] = row
            if mode == 'dive':
                snapshot = arrays.pop('approval_snapshot')
            del arrays
        reasons = []
        if flights['nominal']['safe']:
            reasons.append('nominal_safe_through_horizon')
        if not flights['dive']['safe']:
            reasons.append('d0_unsafe:'+flights['dive']['first_violation']['part'])
        if not flights['dive']['inspection_completed']:
            reasons.append('d0_inspection_incomplete')
        if not flights['sideways']['safe']:
            reasons.append('initial_sideways_unsafe:'+flights['sideways']['first_violation']['part'])
        entry = dict(seed=seed, scenario=asdict(s), accepted=not reasons,
                     rejection_reasons=reasons, immediate_flights=flights,
                     initial_climb_safe=flights['climb']['safe'])
        if not reasons:
            # Repeat only D=0, independently checking physical and controller state.
            release_memory()
            replay, arrays = execute(s, 'dive', 0., capture_snapshot=True)
            assert replay['safe_completion']
            assert replay['physics_sha256'] == flights['dive']['physics_sha256']
            verify_snapshot(snapshot, arrays.pop('approval_snapshot'))
            del arrays
            location = out/f'snapshots/seed_{seed}.pkl'
            location.parent.mkdir(exist_ok=True)
            temporary = location.with_suffix('.tmp')
            temporary.write_bytes(pickle.dumps(snapshot, protocol=pickle.HIGHEST_PROTOCOL))
            temporary.replace(location)
            entry['d0_replay_physics_sha256'] = replay['physics_sha256']
            data['scenarios'].append(dict(seed=seed,scenario=asdict(s),
                approval_snapshot=str(location.relative_to(out)),snapshot_sha256=file_hash(location),
                d0_verification=flights['dive'], initial_sideways=flights['sideways'],
                initial_climb_safe=flights['climb']['safe']))
        data['attempts'].append(entry); done.add(seed); added += 1
        data['statistics'] = dict(attempted=len(done), accepted=len(data['scenarios']),
                                 rejected=len(done)-len(data['scenarios']))
        data['status'] = 'complete' if len(done)==len(seeds) else 'partial'
        write_json(path,data)
        print(f'seed={seed}: {"accepted" if not reasons else reasons}; {data["statistics"]}',flush=True)
        if max_scenarios and added >= max_scenarios:
            break
    return data


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--split',choices=('development','evaluation'),default='development')
    p.add_argument('--seeds', nargs='+',type=int,help='Explicit attempted seed list; cannot combine with range options')
    p.add_argument('--start-seed',type=int,help='First attempted seed (development: 1; evaluation: 1001)')
    p.add_argument('--count',type=int,help='Number of attempted seeds, not accepted scenarios (development: 20; evaluation: 100)')
    p.add_argument('--out',type=Path,help='Defaults to results/<split>/calibration')
    p.add_argument('--resume',action='store_true')
    p.add_argument('--max-scenarios',type=int,default=0)
    a = p.parse_args()
    if a.max_scenarios < 0:
        p.error('--max-scenarios must be nonnegative')
    if a.seeds is not None and (a.start_seed is not None or a.count is not None):
        p.error('Use --seeds or --start-seed/--count, not both')
    if a.seeds is None:
        start=a.start_seed if a.start_seed is not None else (1 if a.split=='development' else 1001)
        count=a.count if a.count is not None else (20 if a.split=='development' else 100)
        if count<=0:
            p.error('--count must be positive')
        a.seeds=list(range(start,start+count))
    if a.out is None:
        a.out=ROOT/'results'/a.split/'calibration'
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result = calibrate(a.seeds,a.out,a.resume,a.max_scenarios,split=a.split)
    print(result['status'],result['statistics'])


if __name__ == '__main__':
    main()
