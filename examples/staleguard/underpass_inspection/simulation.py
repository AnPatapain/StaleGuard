"""One serial SCG/PyBullet flight, monitored from reset through a fixed horizon."""
from functools import partial
import hashlib
import time
import numpy as np
import pybullet as p
from safe_control_gym.utils.registration import get_config, make

from model.common import capture_pid_state
from physics_monitor import monitor_physics_steps
from geofence_turn.simulation import check_memory
from geofence_turn.scenario import Config
from underpass_inspection.scenario import (HZ, PHYSICS_HZ, POS, VEL, MODES,
    make_nominal, apply_maneuver, body_clearances, in_inspection, violation_part, world_to_ned)


def execute(s, mode='nominal', start_s=0., gui=False, playback_speed=.5,
            hold_s=0., capture=None, overlay_all=False, capture_snapshot=False,
            overlay_paths=None):
    if mode not in MODES:
        raise ValueError(mode)
    if (not np.isfinite(start_s) or not 0 <= start_s < s.observation_s
            or not np.isclose(start_s*HZ, round(start_s*HZ), atol=1e-9)
            or not np.isfinite(playback_speed) or playback_speed <= 0
            or not np.isfinite(hold_s) or hold_s < 0):
        raise ValueError('Use nonnegative 20 ms start times and positive playback speed')
    check_memory(Config())
    approval = round(s.approval_s*HZ)
    total = round((s.approval_s+s.observation_s)*HZ)
    initiation = None if mode == 'nominal' else approval+round(start_s*HZ)
    config = get_config('quadrotor')
    config.update(quad_type=3, task='traj_tracking', cost='quadratic', ctrl_freq=HZ,
                  pyb_freq=PHYSICS_HZ, episode_len_sec=(total+5)/HZ,
                  init_state={'init_x': 0., 'init_y': 0., 'init_z': s.cruise_z},
                  randomized_init=False, randomized_inertial_prop=False, disturbances=None,
                  adversary_disturbance=None, done_on_out_of_bound=False, gui=gui)
    factory = partial(make, 'quadrotor', **config)
    # One environment, owned by the stock controller. No duplicate GUI clients.
    ctrl = make('pid', factory)
    env = ctrl.env
    trace, physics, references = [], [], []
    minimum = {'deck': float('inf'), 'pier': float('inf')}
    violation = None
    entered = None
    completion = None
    longest_dwell = 0
    pid_unchanged = None
    state_unchanged = None
    anchor = None
    splice = None
    approval_state = None
    approval_snapshot = None
    geometry_before = repr(s.obstacles)
    began = time.perf_counter()
    try:
        obs, info = env.reset(seed=s.seed)
        ctrl.reset_before_run(obs=obs, info=info, env=env)
        reference = make_nominal(total+1, s)
        ctrl.reference = reference
        env.X_GOAL = reference
        obstacles = s.obstacles
        if gui:
            from .support.visualization import FlightView, draw_reference, save_camera
            view = FlightView(s, env, mode, start_s, overlay_all, overlay_paths)
            draw_reference(reference, env.PYB_CLIENT, 'nominal')

        def observe(position, tick):
            nonlocal violation, entered, completion, longest_dwell
            values = body_clearances(position, obstacles)
            physics.append((tick/PHYSICS_HZ, *position))
            for name, value in values.items():
                minimum[name] = min(minimum[name], value)
            if violation is None and min(values.values()) < s.clearance_m:
                name = min(values, key=values.get)
                box = next(b for b in obstacles if b.name == name)
                violation = dict(time_flight_s=tick/PHYSICS_HZ,
                    time_approval_s=tick/PHYSICS_HZ-s.approval_s,
                    physics_tick=tick, position_m=list(position), obstacle=name,
                    part=violation_part(position, box), clearance_m=values[name])
            if tick >= approval*(PHYSICS_HZ//HZ) and in_inspection(position, s):
                if entered is None:
                    entered = tick
                dwell = tick-entered
                longest_dwell = max(longest_dwell, dwell)
                if completion is None and dwell >= round(s.inspection_dwell_s*PHYSICS_HZ):
                    completion = tick/PHYSICS_HZ-s.approval_s
            else:
                entered = None

        sample = monitor_physics_steps(env, observe)
        sample()
        for step in range(total):
            if step == approval:
                approval_state = obs.copy()
                if capture_snapshot:
                    approval_snapshot = dict(state=obs.copy(), pid_state=capture_pid_state(ctrl),
                        remaining_reference=reference[step:].copy(), horizon_steps=total-step,
                        approval_time=s.approval_s, scenario=s)
                if gui:
                    view.approval(obs[POS])
            if step == initiation:
                anchor = obs[POS].copy()
                physical_before = obs.copy()
                pid_before = capture_pid_state(ctrl)
                previous_row = reference[step].copy()
                reference = apply_maneuver(reference, step, obs, s, mode)
                ctrl.reference = reference
                env.X_GOAL = reference
                pid_after = capture_pid_state(ctrl)
                pid_unchanged = all(np.array_equal(getattr(pid_before, name), getattr(pid_after, name))
                                    for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'))
                state_unchanged = np.array_equal(physical_before, env.state)
                splice = dict(position_m=(reference[step, POS]-previous_row[POS]).tolist(),
                              velocity_mps=(reference[step, VEL]-previous_row[VEL]).tolist(),
                              measured_velocity_mps=obs[VEL].tolist())
                if gui:
                    view.initiation(anchor, reference[step:])
            trace.append((step/HZ, *obs))
            references.append((step/HZ, *reference[step, POS], *reference[step, VEL]))
            old_position = obs[POS].copy()
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            sample()
            if gui:
                view.update(old_position, obs[POS], step,
                            initiation is not None and step >= initiation, violation is not None,
                            world_y=obs[2], world_vy=obs[3])
                time.sleep(1/(HZ*playback_speed))
            if done:
                raise RuntimeError(f'Environment terminated early: {step+1}/{total}')
        trace.append((total/HZ, *obs))
        references.append((total/HZ, *reference[total, POS], *reference[total, VEL]))
        assert len(physics) == total*(PHYSICS_HZ//HZ)+1
        assert geometry_before == repr(s.obstacles)
        if initiation is not None:
            assert pid_unchanged and state_unchanged
        if gui:
            view.refresh(total/HZ, initiation is not None, violation is not None)
            if capture:
                save_camera(capture, env.PYB_CLIENT)
            deadline = time.monotonic()+hold_s
            while time.monotonic() < deadline and p.isConnected(env.PYB_CLIENT):
                view.refresh(total/HZ, initiation is not None, violation is not None)
                time.sleep(.05)
        positions = np.asarray(physics)
        record = dict(seed=s.seed, mode=mode, start_s=None if initiation is None else start_s,
            approval_step=approval, initiation_step=initiation, horizon_step=total,
            horizon_flight_s=total/HZ, observation_s=s.observation_s,
            safe=violation is None, inspection_completed=completion is not None,
            safe_completion=violation is None and completion is not None,
            completion_approval_s=completion, longest_inspection_dwell_s=longest_dwell/PHYSICS_HZ,
            min_clearance_m=min(minimum.values()), min_clearance_by_obstacle_m=minimum,
            first_violation=violation, physics_samples=len(physics), control_samples=len(trace),
            maneuver_anchor_m=None if anchor is None else anchor.tolist(),
            approval_state=approval_state.tolist(), pid_unchanged_at_initiation=pid_unchanged,
            physical_state_unchanged_at_initiation=state_unchanged,
            nominal_to_measured_reference_splice=splice, geometry_unchanged=True,
            max_physics_position_increment_m=float(np.linalg.norm(np.diff(positions[:, 1:], axis=0), axis=1).max()),
            final_position_m=obs[POS].tolist(), final_velocity_mps=obs[VEL].tolist(),
            final_position_ned_m=world_to_ned(obs[POS]).tolist(),
            final_velocity_ned_mps=world_to_ned(obs[VEL]).tolist(),
            physics_sha256=hashlib.sha256(positions.tobytes()).hexdigest(),
            runtime_s=time.perf_counter()-began, gui=gui)
        data = dict(physics=positions, control=np.asarray(trace), reference=np.asarray(references))
        if capture_snapshot:
            data['approval_snapshot'] = approval_snapshot
        return record, data
    finally:
        ctrl.close()
