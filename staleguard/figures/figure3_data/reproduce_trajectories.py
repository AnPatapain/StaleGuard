"""Export selected actual PyBullet flights; do not draw or alter the flights.

Run from the repository root with the project's scg Python environment.
Only seed 22, D=0.50 s is reproduced. Historical experiments remain read-only.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
from staleguard import geofence_turn # Set single-thread numerical limits before NumPy imports.

import argparse
from dataclasses import asdict
import fcntl
import json
import platform
import numpy as np

import staleguard.geofence_turn.simulation as simulation
from staleguard.geofence_turn.scenario import (
    Config, HZ, PHYSICS_HZ, POS, BODY_RADIUS_M, clearance, scenario_dict,
)
from staleguard.geofence_turn.storage import (
    check_source, check_approval_snapshot, file_hash, load_calibrated_scenarios,
    write_json,
)
from staleguard.geofence_turn.baselines.staleguard import LightweightPredictiveModel
from staleguard.model.scg import load_scg_platform_params

SEED, DELAY = 22, 0.50
EVALUATION = ROOT / 'geofence_turn/results/evaluation'
POLICIES = ('staleguard_early_recovery', 'use_time_recheck', 'no_recovery')
EXPERIMENTS = dict(zip(POLICIES, ('fixed_early_benchmark', 'baseline_benchmark', 'baseline_benchmark')))
CSV_NAMES = ('staleguard_early_trajectory.csv', 'use_time_recheck_trajectory.csv', 'no_recovery_trajectory.csv')
# Reviewed simulator versions: the current version adds unrelated PPR branches.
# The historical diff preserves physics/monitoring and Use-Time's control loop.
# Pin the reviewed current source, rather than accepting later edits silently.
REVIEWED_SIMULATOR = 'c80921f470e62bca9f24cc1177b9b78fd332c5043ff0f5d401dd7b4f32449758'
ARCHIVED_SIMULATORS = dict(zip(POLICIES, (
    '4fde431f8fd6312cf50ddf3aa9210d6c1e66264840e2b57025f3c49f1f28459e',
    '24e1a677ee5de40d340ee8ba65a9c38ba9cb01648562051140515d018119493c',
    '24e1a677ee5de40d340ee8ba65a9c38ba9cb01648562051140515d018119493c',
)))


def archive(policy, scenario, snapshot_path, calibration):
    """Check journal bytes, frozen inputs, and reviewed scientific sources."""
    folder = EVALUATION / EXPERIMENTS[policy]
    run = json.loads((folder / 'run.json').read_text())
    provenance, protocol = run['provenance'], run['protocol']
    assert run['status'] == 'complete'
    journal = folder / 'executions.jsonl'
    assert journal.stat().st_size == run['journal_bytes']
    assert file_hash(journal) == run['journal_sha256']
    assert protocol['dataset_sha256'] == file_hash(calibration)
    assert protocol['snapshot_sha256'][str(SEED)] == file_hash(snapshot_path)
    assert json.loads(json.dumps(scenario_dict(scenario))) in protocol['scenarios']
    assert protocol['config'] == asdict(Config()) and protocol['control_hz'] == HZ
    assert DELAY in protocol['delays'] and policy in protocol['policies']
    assert provenance['config'] == protocol['config']
    assert provenance['numpy'] == np.__version__
    assert provenance['python'].split()[0].split('.')[:2] == [str(x) for x in sys.version_info[:2]]
    assert provenance['numerical_threads'] == 1 and provenance['cpu_only']
    assert provenance['simulation_workers'] == 1
    assert provenance['source_hashes']['simulation.py'] == ARCHIVED_SIMULATORS[policy]
    for name in ('__init__.py', 'scenario.py', 'baselines/staleguard.py',
                 'baselines/use_time_recheck.py'):
        check_source(ROOT / 'geofence_turn' / name, provenance['source_hashes'][name])
    for name, digest in {**provenance['shared_source_hashes'],
                         **provenance['environment_source_hashes']}.items():
        # Historical runs used two different absolute checkout prefixes.
        if 'examples/staleguard/' in name:
            name = name.split('examples/staleguard/', 1)[1]
        check_source(ROOT / name, digest)
    rows = [json.loads(line) for line in journal.open()]
    selected = [row for row in rows if row['seed'] == SEED and row['delay_s'] == DELAY
                and row['event']['policy'] == policy]
    assert len(selected) == 1, 'Missing or duplicate archived case'
    return run, selected[0], dict(directory=str(folder.relative_to(ROOT)),
        checksums={name: file_hash(folder / name)
                   for name in ('run.json', 'executions.jsonl', 'metrics.json')})


def replay(scenario, snapshot, policy, prediction, platform_parameters):
    """Observe the exact same 1 kHz positions as the existing safety monitor."""
    first = None
    original = simulation.monitor_physics_steps

    def record_monitor(env, observe):
        def observe_and_record(position, tick):
            nonlocal first
            observe(position, tick)  # Original safety and mission monitoring.
            if first is None and clearance(position[0], scenario.fence_x) < snapshot.config.d_min_m:
                first = dict(time_flight_s=tick / PHYSICS_HZ,
                             physics_tick=tick, position_m=list(position))
        return original(env, observe_and_record)

    simulation.monitor_physics_steps = record_monitor
    try:
        result, approval, trace = simulation.execute(
            scenario, DELAY, config=snapshot.config, policy=policy,
            received=prediction if policy == POLICIES[0] else None,
            platform=platform_parameters, save_trace=True)
    finally:
        simulation.monitor_physics_steps = original
    check_approval_snapshot(approval, snapshot)
    total = round(result['horizon_s'] * HZ)
    assert trace.shape == (total + 1, 25) and np.isfinite(trace).all()
    np.testing.assert_array_equal(trace[:, 0], np.arange(total + 1))
    np.testing.assert_array_equal(trace[round(scenario.approval_s * HZ), 1:13], snapshot.state)
    assert result['physics_samples'] == result['expected_physics_samples'] == total * 20 + 1
    assert (first is None) == result['safe']
    if first is not None:
        assert first['time_flight_s'] == result['first_violation_s']
    return result, trace, first


def event_metadata(scenario, result, trace, first, archived, source, prediction):
    event = result['event']
    step = event['fallback_step']
    approval_step = round(scenario.approval_s * HZ)
    if event['policy'] == 'no_recovery':
        application = event['application_step']
        return dict(seed=SEED, delay_s=DELAY, source_experiment=source,
            approval_time_flight_s=scenario.approval_s,
            approval_position_m=trace[approval_step, 1 + np.array(POS)].tolist(),
            scheduled_dispatch_time_flight_s=event['scheduled_dispatch_step'] / HZ,
            command_application_time_flight_s=application / HZ,
            command_application_position_m=trace[application, 1 + np.array(POS)].tolist(),
            prediction_start_time_flight_s=None, prediction_computation_s=None,
            prediction_finish_time_flight_s=None, prediction_availability_time_flight_s=None,
            predicted_first_unsafe_dispatch_delay_s=None, predicted_decision=None,
            recovery_time_flight_s=None, recovery_position_m=None,
            first_violation=first, min_clearance_m=result['min_clearance_m'],
            completed=result['completed'], completion_delay_after_approval_s=result['completion_s'],
            safe=result['safe'], safe_completion=result['safe_completion'],
            final_position_m=result['final_position'], final_velocity_mps=result['final_velocity'],
            archived_execution=archived, replayed_execution=result)
    start = (scenario.approval_s if event['policy'] == POLICIES[0]
             else event['scheduled_dispatch_step'] / HZ)
    duration = event['computation_s']
    ready_step = (approval_step + int(np.ceil(duration * HZ))
                  if event['policy'] == POLICIES[0] else event['decision_step'])
    expiry = event.get('expiry_steps')
    return dict(seed=SEED, delay_s=DELAY, source_experiment=source,
        approval_time_flight_s=scenario.approval_s,
        approval_position_m=trace[approval_step, 1 + np.array(POS)].tolist(),
        scheduled_dispatch_time_flight_s=event['scheduled_dispatch_step'] / HZ,
        command_application_time_flight_s=(None if event['application_step'] is None
                                          else event['application_step'] / HZ),
        prediction_start_time_flight_s=start, prediction_computation_s=duration,
        prediction_timing_source=('archived approval forecast' if event['policy'] == POLICIES[0]
                                  else 'new measured dispatch prediction'),
        prediction_finish_time_flight_s=start + duration,
        prediction_availability_time_flight_s=ready_step / HZ,
        prediction_availability_semantics=('derived first ready control tick; inclusive readiness'
            if event['policy'] == POLICIES[0] else 'stored decision application tick; strictly after finish'),
        predicted_first_unsafe_dispatch_delay_s=None if expiry is None else expiry / HZ,
        predicted_expiration_time_flight_s=None if expiry is None else scenario.approval_s + expiry / HZ,
        predicted_decision=('UNSAFE' if not event['check']['safe'] else 'SAFE')
            if event['policy'] == POLICIES[1] else
            ('UNSAFE' if expiry is not None and round(DELAY * HZ) >= expiry else 'SAFE'),
        predicted_decision_basis=('immediate turn from dispatch state' if event['policy'] == POLICIES[1]
                                  else 'frozen TTL rule evaluated at scheduled dispatch'),
        approval_forecast=prediction if event['policy'] == POLICIES[0] else None,
        recovery_time_flight_s=None if step is None else step / HZ,
        recovery_position_m=None if step is None else trace[step, 1 + np.array(POS)].tolist(),
        first_violation=first, min_clearance_m=result['min_clearance_m'],
        completed=result['completed'], completion_delay_after_approval_s=result['completion_s'],
        safe=result['safe'], safe_completion=result['safe_completion'],
        archived_execution=archived, replayed_execution=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--overwrite', action='store_true', help='Replace this utility\'s exports only')
    parser.add_argument('--policies', nargs='+', choices=POLICIES, default=POLICIES,
                        help='Replay only these policies; preserve other existing policy exports')
    args = parser.parse_args()
    if len(set(args.policies)) != len(args.policies):
        parser.error('Duplicate policy selection')
    outputs = [CSV_NAMES[POLICIES.index(p)] for p in args.policies]
    if not args.overwrite and any((args.out / name).exists() for name in outputs):
        parser.error('Exports already exist; choose a fresh --out or use --overwrite')
    existing_path = args.out / 'figure3_events.json'
    existing = json.loads(existing_path.read_text()) if existing_path.exists() else None
    preserved_files = {name:file_hash(args.out / name) for name in (*CSV_NAMES, 'figure3_geometry.json')
                       if (args.out / name).exists() and name not in outputs}
    preserved_policies = {p:json.dumps(item, sort_keys=True) for p,item in existing['policies'].items()
                          if p not in args.policies} if existing else {}
    if existing:
        assert all((args.out / CSV_NAMES[POLICIES.index(p)]).exists() for p in existing['policies'])
    assert file_hash(ROOT / 'geofence_turn/simulation.py') == REVIEWED_SIMULATOR, 'Simulator changed; review before replay'
    calibration = EVALUATION / 'calibration/calibrated_scenarios.json'
    scenario, snapshot, snapshot_path = next(c for c in load_calibrated_scenarios(calibration)
                                            if c[0].seed == SEED)
    assert scenario.command.direction == 'TURN_LEFT_90'
    assert scenario.command.radius == .4 and scenario.command.speed == .8
    archives = {p: archive(p, scenario, snapshot_path, calibration) for p in args.policies}
    prediction = None
    if POLICIES[0] in archives:
        prediction = archives[POLICIES[0]][0]['predictions'][str(SEED)]
        assert all(prediction[k] == archives[POLICIES[0]][1]['event'][k]
                   for k in ('expiry_steps', 'computation_s'))
    protocols = [r[0]['protocol'] for r in archives.values()]
    assert all(r['delays'] == protocols[0]['delays'] for r in protocols)
    if existing:
        assert existing['calibration']['sha256'] == file_hash(calibration)
        assert existing['calibration']['snapshot_sha256'] == file_hash(snapshot_path)
    metadata, trajectories, comparisons = {}, {}, []
    # Reuse the simulator lock and its unchanged memory safeguards.
    with (ROOT / 'geofence_turn/support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        simulation.check_memory(snapshot.config)
        parameters = load_scg_platform_params()
        if 'use_time_recheck' in args.policies:
            model = LightweightPredictiveModel(*parameters)
            try:  # Warm kernels, not a new approval forecast or a physical flight.
                controller, obs, info, _ = model.start(snapshot)
                model.step(controller.select_action(obs, info), obs, info)
            finally:
                model.close({})
        for policy in args.policies:
            run, old, source = archives[policy]
            actual, trace, first = replay(scenario, snapshot, policy, prediction, parameters)
            fields = ('safe', 'completed', 'safe_completion', 'first_violation_s',
                      'completion_s', 'pre_dispatch_violation_s', 'physics_samples', 'horizon_s')
            differences = [k for k in fields if actual[k] != old[k]]
            differences += ['event.' + k for k in ('fallback_step', 'application_step', 'scheduled_dispatch_step')
                            if actual['event'][k] != old['event'][k]]
            if policy == POLICIES[1] and actual['event']['check']['safe'] != old['event']['check']['safe']:
                differences.append('event.check.safe')
            clearance_error = abs(actual['min_clearance_m'] - old['min_clearance_m'])
            if clearance_error > 1e-10:
                differences.append('min_clearance_m')
            final_errors = {k:float(np.max(np.abs(np.asarray(actual[k]) - old[k])))
                            for k in ('final_position', 'final_velocity')}
            differences += [k for k, error in final_errors.items() if error > 1e-10]
            comparisons.append(dict(policy=policy, differences=differences,
                clearance_absolute_error_m=clearance_error, approval_snapshot_exact=True,
                final_state_max_absolute_errors=final_errors,
                samples=len(trace), archived_computation_s=old['event'].get('computation_s'),
                replay_computation_s=actual['event'].get('computation_s')))
            metadata[policy] = event_metadata(scenario, actual, trace, first, old, source, prediction)
            trajectories[policy] = trace
            print(f'{policy}: safe={actual["safe"]}, completed={actual["completed"]}, '
                  f'application_step={actual["event"]["application_step"]}, '
                  f'recovery_step={actual["event"]["fallback_step"]}, '
                  f'clearance={actual["min_clearance_m"]:.12f}m, differences={differences}', flush=True)
    approval_step = round(scenario.approval_s * HZ)
    for trace in trajectories.values():
        for policy, filename in zip(POLICIES, CSV_NAMES):
            if policy in trajectories:
                prefix = trajectories[policy][:approval_step+1, 1:13]
            elif (args.out / filename).exists():
                saved = np.loadtxt(args.out / filename, delimiter=',', skiprows=1)
                prefix = saved[:approval_step+1, [1,4,2,5,3,6,7,8,9,10,11,12]]
            else:
                continue
            np.testing.assert_array_equal(trace[:approval_step+1, 1:13], prefix)
    passed = all(not c['differences'] for c in comparisons)
    args.out.mkdir(parents=True, exist_ok=True)
    geometry = dict(seed=SEED, delay_s=DELAY, units='meters, seconds, radians',
        fence_x_m=scenario.fence_x, body_radius_m=BODY_RADIUS_M,
        required_clearance_m=snapshot.config.d_min_m,
        maximum_safe_center_x_m=scenario.fence_x - BODY_RADIUS_M - snapshot.config.d_min_m,
        checkpoint_min_xyz_m=list(scenario.command.checkpoint_lower),
        checkpoint_max_xyz_m=list(scenario.command.checkpoint_upper),
        approval_position_m=snapshot.state[POS].tolist(), approval_time_flight_s=scenario.approval_s,
        turn_radius_m=scenario.command.radius, turn_speed_mps=scenario.command.speed,
        mission_deadline_after_approval_s=scenario.deadline_s,
        monitoring_deadline_flight_s=scenario.approval_s + scenario.deadline_s,
        frozen_scenario=scenario_dict(scenario))
    geometry = json.loads(json.dumps(geometry))
    events = dict(ready_for_visualization=passed, control_hz=HZ, safety_monitor_hz=PHYSICS_HZ,
        timestamp_note='All *_flight_s are seconds from reset. TTL is an approval-relative dispatch delay. '
                       'Prediction finish/availability are simulation timing-model values derived from measured compute duration, not concurrent onboard timestamps.',
        csv_units=dict(time_s='s', position='m', velocity='m/s', angles='rad', body_rates='rad/s'),
        calibration=dict(path=str(calibration.relative_to(ROOT)), sha256=file_hash(calibration),
                         snapshot=str(snapshot_path.relative_to(ROOT)), snapshot_sha256=file_hash(snapshot_path)),
        source_validation=dict(reviewed_current_simulation_sha256=REVIEWED_SIMULATOR,
            source_relocation_sha256=file_hash(ROOT / 'source_relocation.json'),
            exporter_sha256=file_hash(Path(__file__)),
            note='Mission/predictor/controller/environment sources checked directly or against pinned source_relocation.json entries. '
                 'Simulator versions pinned after reviewing policy-only changes; historical hashes are not rewritten.'),
        replay_environment=dict(python=sys.version, platform=platform.platform(), numpy=np.__version__),
        archived_environments={p:{k:archives[p][0]['provenance'][k] for k in ('python','platform','numpy')}
                               for p in archives},
        comparisons=comparisons, policies=metadata)
    if not passed:
        raise RuntimeError(f'Replay differs: {comparisons}. Existing data preserved; new exports withheld.')
    if existing:
        # Keep earlier policy records and their original measurements intact.
        existing['policies'].update(metadata)
        existing['comparisons'] = [c for c in existing['comparisons'] if c['policy'] not in args.policies] + comparisons
        existing['archived_environments'].update(events['archived_environments'])
        existing.setdefault('data_extensions', []).append(dict(
            policies=list(args.policies), exporter_sha256=file_hash(Path(__file__)),
            replay_environment=events['replay_environment']))
        events = existing
    geometry_path = args.out / 'figure3_geometry.json'
    if geometry_path.exists():
        assert json.loads(geometry_path.read_text()) == geometry, 'Existing geometry differs'
    else:
        write_json(geometry_path, geometry)
    for policy in args.policies:
        filename = CSV_NAMES[POLICIES.index(policy)]
        trace = trajectories[policy]
        # Physical state only: step + [x,vx,y,vy,z,vz,roll,pitch,yaw,p,q,r].
        data = np.column_stack((trace[:, 0] / HZ,
                                trace[:, [1,3,5,2,4,6,7,8,9,10,11,12]]))
        np.savetxt(args.out / filename, data, delimiter=',', fmt='%.17g', comments='',
                   header='time_s,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r')
        np.testing.assert_array_equal(np.loadtxt(args.out / filename, delimiter=',', skiprows=1), data)
        if policy == 'no_recovery':
            before = data[data[:,0] < metadata[policy]['first_violation']['time_flight_s']][-1]
            application = data[round(metadata[policy]['command_application_time_flight_s']*HZ)]
            shape = dict(xyz_ranges_m={name:[float(data[:,i].min()),float(data[:,i].max())]
                                      for i,name in enumerate(('x','y','z'),1)},
                max_altitude_error_m=float(np.max(np.abs(data[:,3] - scenario.height))),
                final_velocity_heading_deg=float(np.degrees(np.arctan2(data[-1,5],data[-1,4]))),
                last_previolation_sample_time_flight_s=float(before[0]),
                last_previolation_velocity_heading_deg=float(np.degrees(np.arctan2(before[5],before[4]))),
                north_displacement_from_application_to_last_previolation_sample_m=float(before[2]-application[2]),
                samples_after_first_violation=int(np.count_nonzero(data[:,0] > metadata[policy]['first_violation']['time_flight_s'])))
            events['policies'][policy]['physical_trajectory_measurements'] = shape
    write_json(args.out / 'figure3_events.json', events)
    write_validation(args.out, events)
    assert all(file_hash(args.out / name) == digest for name,digest in preserved_files.items())
    assert all(json.dumps(events['policies'][p], sort_keys=True) == item
               for p,item in preserved_policies.items())
    print(f'PASS: {len(args.policies)} physical flight(s) exported to {args.out.resolve()}')


def write_validation(output, events):
    lines = ['# Figure 3 trajectory validation', '',
        'Actual SCG/PyBullet flights reproduce the eastbound patrol, the two safety-only braking/hover '
        'recoveries, and (when exported) the unmodified No-Recovery turn. '
        'Neither recovery executes the northbound turn or completes the checkpoint. '
        'No plotting, interpolation, or coordinate smoothing was performed.', '',
        'Status: **' + ('READY' if events['ready_for_visualization'] else 'NOT READY: replay discrepancy') + '**.', '',
        '## Recorded versus replayed physical outcomes', '',
        '| Policy | Quantity | Archived | Replay |', '|---|---|---:|---:|']
    for policy, item in events['policies'].items():
        old, new = item['archived_execution'], item['replayed_execution']
        for name, before, now in [
            ('Recovery (flight seconds)', None if old['event']['fallback_step'] is None else old['event']['fallback_step']/HZ,
             None if new['event']['fallback_step'] is None else new['event']['fallback_step']/HZ),
            ('Command application (flight seconds)', None if old['event']['application_step'] is None else old['event']['application_step']/HZ,
             None if new['event']['application_step'] is None else new['event']['application_step']/HZ),
            ('First violation (flight seconds)', old['first_violation_s'], new['first_violation_s']),
            ('Minimum body clearance (m)', old['min_clearance_m'], new['min_clearance_m']),
            ('Completed', old['completed'], new['completed']), ('Safe', old['safe'], new['safe']),
            ('Prediction computation (ms)', None if old['event'].get('computation_s') is None else 1000*old['event']['computation_s'],
             None if new['event'].get('computation_s') is None else 1000*new['event']['computation_s'])]:
            lines.append(f'| {policy} | {name} | {before} | {now} |')
    lines += ['', '## Configuration, sources, and checks', '',
        '- Frozen seed 22, dispatch delay 0.50 s, TURN_LEFT_90 radius 0.40 m, speed 0.80 m/s.',
        '- Controller 50 Hz; unchanged physics/safety monitor 1,000 Hz. The full monitored flight includes pre-approval flight and continues after a violation until the original deadline.',
        '- Every approval snapshot matches exactly: physical state, PID memory, remaining reference, command, configuration, and prior clearance.',
        '- All exported physical-state prefixes through approval match exactly.',
        '- First-violation XYZ is captured from the existing monitor callback, at its actual physics tick; it is not interpolated from the 50 Hz CSV.',
        '- The observation wrapper does not change actions, physics steps, prediction timing measurement, or policy decisions. The original monitor is restored after each flight.',
        '- Memory guards, the simulation lock, CPU-only execution, and single numerical threads remain active.',
        '- Journal byte counts/checksums, calibration checksum, snapshot checksum, configuration, and scientific sources were verified before replay. Source relocations require the existing pinned relocation checks.',
        '- Simulator hash differences were reviewed against Git: new policy branches preserve the common physical execution and Use-Time decision scheduling. The exporter pins the reviewed current and archived versions.', '']
    for policy, item in events['policies'].items():
        lines.append(f'- **{policy} source:** `{item["source_experiment"]["directory"]}/` (`run.json`, `executions.jsonl`, `metrics.json`).')
    for check in events['comparisons']:
        lines.append(f'- **{check["policy"]}:** {check["samples"]} physical samples; clearance error '
                     f'{check["clearance_absolute_error_m"]:.17g} m; discrepancies: {check["differences"]}.')
        if 'final_state_max_absolute_errors' in check:
            lines.append(f'  Final-state maximum absolute errors: {check["final_state_max_absolute_errors"]}.')
    if 'no_recovery' in events['policies']:
        nr = events['policies']['no_recovery']
        shape = nr['physical_trajectory_measurements']
        lines += ['', '## No-Recovery physical trajectory', '',
            f'- Command executes at {nr["command_application_time_flight_s"]:.3f} s, XYZ {nr["command_application_position_m"]} m.',
            f'- First violation: {nr["first_violation"]}. XYZ is the actual center position, sampled at 1,000 Hz.',
            f'- Final position: {nr["final_position_m"]} m; final velocity: {nr["final_velocity_mps"]} m/s.',
            '- Full flight: 323 physical states at 50 Hz, from 0.00 through 6.44 s; no samples after the violation are removed.',
            '', '| Coordinate | Minimum (m) | Maximum (m) |', '|---|---:|---:|']
        for coordinate, bounds in shape['xyz_ranges_m'].items():
            lines.append(f'| {coordinate.upper()} | {bounds[0]:.12f} | {bounds[1]:.12f} |')
        lines += ['',
            f'At the last 50 Hz sample before the violation ({shape["last_previolation_sample_time_flight_s"]:.3f} s), '
            f'the velocity heading is {shape["last_previolation_velocity_heading_deg"]:.6f} degrees north of east '
            f'and the UAV has moved {shape["north_displacement_from_application_to_last_previolation_sample_m"]:.9f} m north '
            'since command application. These are physical-state measurements, not reference-path estimates.', '',
            f'The final velocity heading is {shape["final_velocity_heading_deg"]:.6f} degrees north of east. '
            f'The maximum altitude departure from the frozen nominal height is {shape["max_altitude_error_m"]:.9f} m. '
            f'The CSV retains {shape["samples_after_first_violation"]} control samples after the first violation.', '',
            'The coordinate and velocity changes show a meaningful left-curving segment that starts before the first '
            'violation and continues northward afterward. They do not establish an exact 90-degree physical turn '
            'or a perfect circular arc. No-Recovery has negative body clearance; its intrusion must be distinguished '
            'from Use-Time\'s positive-clearance breach of the 5 cm requirement.', '',
            'The No-Recovery extension replayed exactly one physical flight. The previous two CSVs, their policy '
            'metadata and prediction timings, and the geometry were preserved. Historical benchmark artifacts were not modified.']
    lines += ['', '## Interpretation and limitations', '',
        'The two source experiments were separate runs. Their Python patch versions and Linux kernels differ '
        'and are retained in figure3_events.json. Use-Time computation is measured again; its duration may differ '
        'without changing the first subsequent 50 Hz application tick. EarlyRecovery uses its archived forecast '
        'and archived 65.922 ms duration, without recomputing TTL.', '',
        'Prediction finish and availability times are derived from the simulator timing model, not timestamps '
        'of concurrent onboard computation. Prediction expiry is a dispatch-delay threshold, not the physical '
        'violation time. Angular rates p/q/r are rad/s; roll/pitch/yaw are radians.', '',
        'The physical fence and the safe-center boundary are different planes. Use-Time violates the required '
        '5 cm clearance while retaining positive minimum body clearance. No-Recovery, when included, '
        'has negative body clearance. The figure must distinguish these outcomes. This comparison illustrates recovery timing; '
        'it does not isolate computation latency as the sole cause of the unsafe flight.', '',
        '## Regenerate', '', 'From the repository root, in the `scg` environment:', '',
        '```bash', 'python -m staleguard.figures.figure3_data.reproduce_trajectories --overwrite', '```', '',
        '`--overwrite` replaces only the exporter outputs. Omit it for a fresh output, or supply `--out PATH`. '
        'The default regenerates all three policies. To add only No-Recovery while preserving the other exports, use '
        '`python -m staleguard.figures.figure3_data.reproduce_trajectories --policies no_recovery` '
        '(add `--overwrite` only to replace an existing No-Recovery export). Only selected policies are replayed; '
        'no benchmark is launched. Each CSV uses full-precision numeric '
        'values and can be loaded directly by PGFPlots with `col sep=comma, x=x, y=y, z=z`.', '']
    (output / 'VALIDATION.md').write_text('\n'.join(lines))


if __name__ == '__main__':
    main()
