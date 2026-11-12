
"""Seed-0 physical feasibility pilot. Times are always relative to approval."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import underpass_inspection  # Set numerical thread limits first.
import argparse
from dataclasses import asdict
import csv
import ctypes
import fcntl
import gc
import hashlib
import json
import os
import platform
import time
import numpy as np

from underpass_inspection.scenario import (Scenario, HZ, PHYSICS_HZ, BODY_RADIUS_M,
    POS, MODES, displacement, SCENARIO_VERSION, WORLD_TO_NED, world_to_ned)
from underpass_inspection.simulation import execute

ROOT = Path(__file__).resolve().parent
GRID = tuple(i/HZ for i in range(51))


def intervals(rows, field='safe'):
    """Inclusive tested endpoints only, not interpolated continuous intervals."""
    groups = []
    for row in sorted(rows, key=lambda r: r['start_s']):
        value = bool(row[field])
        if groups and value == groups[-1]['value']:
            groups[-1]['last_tested_s'] = row['start_s']
        else:
            groups.append(dict(value=value, first_tested_s=row['start_s'],
                               last_tested_s=row['start_s']))
    return groups


def deadline_summary(rows):
    output = {}
    for mode in ('dive', 'climb', 'sideways'):
        selected = sorted([r for r in rows if r['mode'] == mode], key=lambda r: r['start_s'])
        bad = [r['start_s'] for r in selected if not r['safe']]
        good = [r['start_s'] for r in selected if r['safe']]
        nonmonotonic = any(not a['safe'] and b['safe'] for a, b in zip(selected, selected[1:]))
        complete = [r['start_s'] for r in selected if r['inspection_completed']]
        lost = [r['start_s'] for r in selected if not r['inspection_completed']]
        status = ('non_monotonic' if nonmonotonic else 'no_safe_start_observed' if not good
                  else 'no_unsafe_start_observed_right_censored' if not bad else 'bracketed_on_grid')
        output[mode] = dict(first_tested_unsafe_s=min(bad) if bad else None,
            latest_tested_safe_s=max(good) if good else None,
            non_monotonic=nonmonotonic, status=status,
            safety_intervals=intervals(selected),
            inspection_intervals=intervals(selected, 'inspection_completed'),
            first_tested_inspection_failure_s=min(lost) if lost else None,
            latest_tested_inspection_completion_s=max(complete) if complete else None)
    cmd = output['dive']['first_tested_unsafe_s']
    result = dict(by_mode=output, t_cmd_s=cmd)
    for mode in ('climb', 'sideways'):
        # A fully safe grid provides a lower bound, not an observed expiration.
        rec = (output[mode]['latest_tested_safe_s']
               if output[mode]['status'] == 'bracketed_on_grid' else None)
        result['t_rec_'+mode+'_s'] = rec
        result['gap_'+mode+'_s'] = (round(rec-cmd, 10) if cmd is not None and rec is not None
                                  and not output['dive']['non_monotonic'] else None)
    return result


def validate_smoke(rows, traces, s):
    base = {r['mode']: r for r in rows if r['start_s'] in (0., None)}
    checks = dict(nominal_unsafe=not base['nominal']['safe'],
                  d0_safe_completion=base['dive']['safe_completion'],
                  immediate_climb_safe=base['climb']['safe'],
                  immediate_sideways_safe=base['sideways']['safe'])
    for row in rows:
        key = row['trajectory_key']
        data = traces[key]
        expected = row['horizon_step']*(PHYSICS_HZ//HZ)+1
        checks[key+'_full_1khz_horizon'] = len(data['physics']) == expected
        checks[key+'_regular_1khz_ticks'] = bool(np.allclose(np.diff(data['physics'][:, 0]), 1/PHYSICS_HZ,
                                                                     atol=1e-12, rtol=0))
        checks[key+'_geometry_fixed'] = row['geometry_unchanged']
        # 1 cm per millisecond is 10 m/s, a diagnostic guard far above the
        # prescribed 0.6 m/s flight; this is NOT the scientific safety threshold.
        checks[key+'_no_position_jump'] = row['max_physics_position_increment_m'] < .01
        if row['mode'] != 'nominal':
            checks[key+'_pid_preserved'] = row['pid_unchanged_at_initiation']
            checks[key+'_state_preserved'] = row['physical_state_unchanged_at_initiation']
            step = row['initiation_step']
            times = np.arange(len(data['reference'])-step)/HZ
            dp, _ = displacement(times, s, row['mode'])
            relative = data['reference'][step:, 1:4]-row['maneuver_anchor_m']
            checks[key+'_fixed_relative_shape'] = bool(np.allclose(relative, dp, atol=1e-12, rtol=0))
    d0 = next(r for r in rows if r['mode'] == 'dive' and r['start_s'] == 0.)
    late = next(r for r in rows if r['mode'] == 'dive' and r['start_s'] == .4)
    a = traces[d0['trajectory_key']]['physics']
    b = traces[late['trajectory_key']]['physics']
    approval_tick = round(s.approval_s*PHYSICS_HZ)
    checks['identical_preapproval_physics'] = bool(np.array_equal(a[:approval_tick+1], b[:approval_tick+1]))
    shift = np.asarray(late['maneuver_anchor_m'])-d0['maneuver_anchor_m']
    checks['delayed_dispatch_shifts_north'] = bool(shift[0] > 0)
    physics = traces[d0['trajectory_key']]['physics']
    control = traces[d0['trajectory_key']]['control']
    first = d0['initiation_step']*(PHYSICS_HZ//HZ)+round(s.swerve_start_s*PHYSICS_HZ)
    last = first+round(s.swerve_s*PHYSICS_HZ)
    checks['d0_actual_swerve_negative_world_y'] = bool(physics[last, 2] < physics[first, 2])
    checks['d0_actual_swerve_positive_ned_east'] = bool(-physics[last, 2] > -physics[first, 2])
    checks['d0_final_ned_east_positive_relative_to_anchor'] = bool(physics[-1, 2] < d0['maneuver_anchor_m'][1])
    checks['d0_negative_actual_world_vy_observed'] = bool(control[:, 4].min() < 0)
    side = next(r for r in rows if r['mode'] == 'sideways' and r['start_s'] == 0.)
    side_positions = traces[side['trajectory_key']]['physics']
    side_first = side['initiation_step']*(PHYSICS_HZ//HZ)
    side_last = side_first+round(s.sideways_s*PHYSICS_HZ)
    checks['sideways_actual_recovery_positive_ned_east'] = bool(side_positions[side_last, 2] < side_positions[side_first, 2])
    return dict(passed=all(checks.values()), checks=checks, delayed_anchor_shift_m=shift.tolist(),
        d0_world_y=dict(at_swerve_start_m=float(physics[first, 2]), at_swerve_end_m=float(physics[last, 2]),
            final_m=float(physics[-1, 2]), min_m=float(physics[:, 2].min()), max_m=float(physics[:, 2].max()),
            min_vy_mps=float(control[:, 4].min()), max_vy_mps=float(control[:, 4].max())),
        d0_ned_east=dict(at_swerve_start_m=float(-physics[first, 2]), at_swerve_end_m=float(-physics[last, 2]),
            final_m=float(-physics[-1, 2]), max_m=float(-physics[:, 2].min())))


def provenance(s):
    paths = [ROOT/'__init__.py', ROOT/'scenario.py', ROOT/'simulation.py', ROOT/'pilot.py',
             ROOT/'support/visualization.py', ROOT.parent/'physics_monitor.py',
             ROOT.parent/'model/common.py',
             ROOT.parent.parent.parent/'safe_control_gym/controllers/pid/pid.py',
             ROOT.parent.parent.parent/'safe_control_gym/envs/gym_pybullet_drones/quadrotor.py',
             ROOT.parent.parent.parent/'safe_control_gym/envs/gym_pybullet_drones/base_aviary.py',
             ROOT.parent.parent.parent/'safe_control_gym/envs/gym_pybullet_drones/assets/cf2x.urdf']
    return dict(verification_protocol=SCENARIO_VERSION, scenario=asdict(s), body_radius_m=BODY_RADIUS_M,
        mission_coordinate_frame='North-East-Down (NED)', world_to_ned=WORLD_TO_NED.tolist(),
        raw_trajectory_coordinate_frame='PyBullet world Z-up; N=x_world, E=-y_world, D=-z_world',
        obstacle_boxes=[asdict(b) for b in s.obstacles],
        scientific_assumptions='All new geometry, maneuver and inspection parameters are provisional pilot assumptions.',
        command_semantics='Measured initiation position plus fixed scenario displacement; no replanning.',
        reference_smoothness='C2 internal joins; actual-position reanchoring at dispatch retains measured nominal tracking mismatch.',
        obstacle_contact_forces=False, safety='signed Euclidean point-to-box distance minus body radius >= clearance_m',
        inspection='Inclusive fixed XYZ bounds, elapsed continuous dwell; safe_completion also requires whole-flight safety.',
        monitoring='From reset through approval + observation_s, even after violation or completion.',
        control_hz=HZ, physics_hz=PHYSICS_HZ, integration='SCG/PyBullet; no predictive model',
        python=sys.version, numpy=np.__version__, platform=platform.platform(),
        simulation_workers=1, numerical_threads=1,
        source_sha256={str(path.relative_to(ROOT.parent.parent.parent)): hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in paths})


def write_bundle(out, metadata, rows, traces, validation=None, sweep=False, status='complete'):
    summary = dict(**metadata, status=status, flight_count=len(rows), validation=validation,
                   sweep_grid_s=list(GRID) if sweep else None,
                   evidence='Actual deterministic seed-0 SCG/PyBullet flights, not model predictions.',
                   gui_inspected=False,
                   deadline_interpretation='20 ms tested grid; no interpolation or monotonicity assumption.')
    if sweep:
        summary['deadlines'] = deadline_summary(rows)
    (out/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    arrays = {key+'__'+kind: data for key, values in traces.items() for kind, data in values.items()}
    np.savez_compressed(out/'trajectories.npz', **arrays)
    fields = ('mode', 'start_s', 'approval_step', 'initiation_step', 'safe', 'inspection_completed',
              'safe_completion', 'min_clearance_m', 'first_violation_approval_s', 'obstacle', 'part')
    with (out/'per_delay.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for row in rows:
            entry = {key: row[key] for key in fields if key in row}
            hit = row['first_violation']
            entry.update(first_violation_approval_s=None if hit is None else hit['time_approval_s'],
                         obstacle=None if hit is None else hit['obstacle'], part=None if hit is None else hit['part'])
            writer.writerow(entry)


def prepare_output(out, numbered=False):
    """Preserve runs; repeated default GUI playback gets a new numbered folder."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if not any(out.iterdir()):
        return out
    if not numbered:
        raise FileExistsError(f'Preserving existing results at {out}; choose a fresh --out')
    number = 2
    while True:
        candidate = out.with_name(f'{out.name}_{number:03d}')
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            number += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=MODES, default='dive')
    parser.add_argument('--delay', type=float, default=0., help='Dive dispatch delay after approval, seconds')
    parser.add_argument('--start', type=float, default=0., help='Recovery start after approval, seconds')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--overlay-all', action='store_true',
                        help='GUI only: compare saved actual nominal/dive/climb/sideways flights from the frozen pilot')
    parser.add_argument('--paper-figure', action='store_true',
                        help='GUI-only clean four-trajectory publication view')
    parser.add_argument('--paper-at', type=float, default=1.8,
                        help='Paper screenshot time in seconds after approval')
    parser.add_argument('--playback-speed', type=float, default=.5, help='Simulated seconds per wall second; .5 = slow motion')
    parser.add_argument('--hold', type=float, default=0., help='Keep final GUI scene open for this many seconds')
    parser.add_argument('--capture', type=Path, help='Optional final GUI camera PNG')
    parser.add_argument('--sweep', action='store_true', help='Validate basics, then sweep all three modes at 0..1 s / 20 ms')
    parser.add_argument('--smoke', action='store_true', help='Four basic flights and one delayed dive; no sweep')
    parser.add_argument('--scenario', type=Path, help='Optional full Scenario JSON for a separate replay')
    parser.add_argument('--calibration', type=Path, help='Frozen development calibration directory/manifest')
    parser.add_argument('--seed', type=int, help='Accepted development seed to visualize/replay')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    if args.sweep and args.smoke or (args.sweep or args.smoke) and args.gui:
        parser.error('Smoke/sweep are headless and mutually exclusive; GUI replays one selected flight')
    if args.capture and not args.gui:
        parser.error('--capture requires --gui')
    if args.overlay_all and not args.gui:
        parser.error('--overlay-all requires --gui')
    if args.paper_figure:
        if (not args.gui or args.sweep or args.smoke or args.calibration
                or args.scenario or args.seed is not None
                or args.mode != 'dive' or args.delay not in (0., .4)):
            parser.error('--paper-figure requires --gui, frozen pilot scenario and dive delay 0 or 0.4')
        args.overlay_all = True
    if args.mode == 'dive' and args.start != 0. or args.mode != 'dive' and args.delay != 0.:
        parser.error('Use --delay for dive and --start for recovery')
    if args.capture:
        args.capture.parent.mkdir(parents=True, exist_ok=True)
    if args.calibration:
        if args.scenario or args.seed is None or args.sweep or args.smoke:
            parser.error('Use --calibration with --seed for a single frozen flight; use deadline_sweep.py for its sweeps')
        from underpass_inspection.storage import load_calibrated_scenarios
        cases = [case[0] for case in load_calibrated_scenarios(args.calibration)]
        s = next((case for case in cases if case.seed==args.seed), None)
        if s is None:
            parser.error('Requested seed was not accepted by the supplied calibration')
    else:
        if args.seed is not None:
            parser.error('--seed requires --calibration')
        s = Scenario(**json.loads(args.scenario.read_text())) if args.scenario else Scenario()

    if args.paper_figure:
        if not 0 <= args.paper_at < s.observation_s:
            parser.error('--paper-at must be within the observation horizon after approval')
        os.environ['STALEGUARD_PAPER_FIGURE'] = '1'
        os.environ['STALEGUARD_PAPER_AT'] = str(s.approval_s+args.paper_at)
        if args.capture:
            os.environ['STALEGUARD_PAPER_CAPTURE'] = str(args.capture.resolve())
        else:
            os.environ.pop('STALEGUARD_PAPER_CAPTURE', None)

    name = f'pilot_{SCENARIO_VERSION}' if args.sweep else 'smoke' if args.smoke else f'playback_{args.mode}_{args.delay if args.mode == "dive" else args.start:.2f}'
    try:
        out = prepare_output(args.out or ROOT/'results/development'/name,
                             numbered=args.gui and args.out is None)
    except FileExistsError as error:
        parser.error(str(error))
    print(f'Output: {out}', flush=True)
    metadata = provenance(s)
    metadata.update(launched_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                    argv=sys.argv, trajectory_columns=dict(physics=['flight_s', 'x', 'y', 'z'],
                    control=['flight_s', 'x', 'vx', 'y', 'vy', 'z', 'vz', 'roll', 'pitch', 'yaw', 'p', 'q', 'r'],
                    reference=['flight_s', 'x', 'y', 'z', 'vx', 'vy', 'vz']))
    rows, traces = [], {}
    validation = None
    status = 'partial'
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            overlay_paths = None
            if args.calibration and args.gui and args.overlay_all:
                from underpass_inspection.support.visualization import replay_paths
                overlay_paths = replay_paths(s,args.mode,args.delay if args.mode=='dive' else args.start)
            jobs = ([('nominal', 0.), ('dive', 0.), ('climb', 0.), ('sideways', 0.), ('dive', .4)]
                    if args.sweep or args.smoke else
                    [(args.mode, args.delay if args.mode == 'dive' else args.start)])
            if args.sweep:
                jobs += [(mode, d) for mode in ('dive', 'climb', 'sideways') for d in GRID
                         if (mode, d) not in jobs]
            for index, (mode, start) in enumerate(jobs):
                if index == 5:
                    validation = validate_smoke(rows, traces, s)
                    if not validation['passed']:
                        status = 'basic_feasibility_failed'
                        raise RuntimeError('Basic feasibility failed; timing sweep was NOT started')
                gc.collect()
                if index % 8 == 0:
                    ctypes.CDLL(None).malloc_trim(0)
                record, data = execute(s, mode, start, args.gui, args.playback_speed,
                                       args.hold, None if args.paper_figure else args.capture,
                                       overlay_all=args.overlay_all, overlay_paths=overlay_paths)
                key = f'{mode}_{round(start*HZ):03d}'
                record['trajectory_key'] = key
                rows.append(record); traces[key] = data
                with (out/'flights.jsonl').open('a') as stream:
                    stream.write(json.dumps(record, allow_nan=False)+'\n'); stream.flush()
                print(f'{index+1}/{len(jobs)} {mode} +{start:.2f}s safe={record["safe"]} '
                      f'inspection={record["inspection_completed"]} clearance={record["min_clearance_m"]:.4f} m '
                      f'violation={record["first_violation"]}', flush=True)
            if args.smoke:
                validation = validate_smoke(rows, traces, s)
                if not validation['passed']:
                    raise RuntimeError('Basic feasibility failed')
            status = 'complete'
        finally:
            write_bundle(out, metadata, rows, traces, validation, args.sweep, status)
    print(f'Saved {len(rows)} actual flights: {out}', flush=True)
    if args.sweep:
        print(json.dumps(deadline_summary(rows), indent=2))


if __name__ == '__main__':
    main()
