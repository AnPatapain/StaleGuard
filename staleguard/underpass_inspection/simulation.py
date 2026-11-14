"""One serial SCG/PyBullet flight, monitored from reset through a fixed horizon."""

from functools import partial
from pathlib import Path
from dataclasses import dataclass
import hashlib
import time
import numpy as np
import pybullet as p
from safe_control_gym.utils.registration import get_config, make
from staleguard.model.common import capture_pid_state
from staleguard.physics_monitor import monitor_physics_steps
from staleguard.underpass_inspection.scenario import (
    HZ,
    PHYSICS_HZ,
    POS,
    VEL,
    MODES,
    make_nominal,
    apply_maneuver,
    body_clearances,
    in_inspection,
    violation_part,
    world_to_ned,
)


def check_memory(max_rss_mb=2048, min_available_mb=2048):
    status = Path('/proc/self/status').read_text().splitlines()
    rss = int(next((x.split()[1] for x in status if x.startswith('VmRSS:')))) / 1024
    mem = Path('/proc/meminfo').read_text().splitlines()
    available = int(next((x.split()[1] for x in mem if x.startswith('MemAvailable:')))) / 1024
    if rss > max_rss_mb or available < min_available_mb:
        raise RuntimeError(f'Memory guard: RSS={rss:.0f} MiB available={available:.0f} MiB')
    return (rss, available)


@dataclass(frozen=True)
class Snapshot:
    state: np.ndarray
    pid_state: object
    remaining_reference: np.ndarray
    horizon_steps: int
    approval_time: float
    scenario: object
    prefix_min_clearance_m: float


def execute(
    s,
    mode='nominal',
    start_s=0.0,
    gui=False,
    playback_speed=0.5,
    hold_s=0.0,
    capture=None,
    overlay_all=False,
    capture_snapshot=False,
    overlay_paths=None,
    policy=None,
    delay_s=0.0,
    received=None,
    platform=None,
    expected_snapshot=None,
):
    if policy is not None:
        from .baselines.policies import make_policy

        if (
            not np.isfinite(delay_s)
            or delay_s < 0
            or delay_s >= s.observation_s
            or (not np.isclose(delay_s * HZ, round(delay_s * HZ), atol=1e-09))
        ):
            raise ValueError('Use a nonnegative control-grid dispatch delay inside the horizon')
        if mode != 'nominal' or start_s != 0.0:
            raise ValueError('Policies start on nominal control; the policy selects the maneuver')
        agent = make_policy(policy, s, received)
        scheduled_dispatch = round(s.approval_s * HZ) + round(delay_s * HZ)
        capture_snapshot = True
    if mode not in MODES:
        raise ValueError(mode)
    if (
        not np.isfinite(start_s)
        or not 0 <= start_s < s.observation_s
        or (not np.isclose(start_s * HZ, round(start_s * HZ), atol=1e-09))
        or (not np.isfinite(playback_speed))
        or (playback_speed <= 0)
        or (not np.isfinite(hold_s))
        or (hold_s < 0)
    ):
        raise ValueError('Use nonnegative 20 ms start times and positive playback speed')
    check_memory()
    approval = round(s.approval_s * HZ)
    total = round((s.approval_s + s.observation_s) * HZ)
    initiation = None if mode == 'nominal' else approval + round(start_s * HZ)
    config = get_config('quadrotor')
    config.update(
        quad_type=3,
        task='traj_tracking',
        cost='quadratic',
        ctrl_freq=HZ,
        pyb_freq=PHYSICS_HZ,
        episode_len_sec=(total + 5) / HZ,
        init_state={'init_x': 0.0, 'init_y': 0.0, 'init_z': s.cruise_z},
        randomized_init=False,
        randomized_inertial_prop=False,
        disturbances=None,
        adversary_disturbance=None,
        done_on_out_of_bound=False,
        gui=gui,
    )
    factory = partial(make, 'quadrotor', **config)
    ctrl = make('pid', factory)
    env = ctrl.env
    trace, physics, references = ([], [], [])
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
        reference = make_nominal(total + 1, s)
        ctrl.reference = reference
        env.X_GOAL = reference
        obstacles = s.obstacles
        if gui:
            from staleguard.visualize import FlightView, draw_reference, save_camera

            view = FlightView(s, env, mode, start_s, overlay_all, overlay_paths)
            draw_reference(reference, env.PYB_CLIENT, 'nominal')

        def observe(position, tick):
            nonlocal violation, entered, completion, longest_dwell
            values = body_clearances(position, obstacles)
            physics.append((tick / PHYSICS_HZ, *position))
            for name, value in values.items():
                minimum[name] = min(minimum[name], value)
            if violation is None and min(values.values()) < s.clearance_m:
                name = min(values, key=values.get)
                box = next((b for b in obstacles if b.name == name))
                violation = dict(
                    time_flight_s=tick / PHYSICS_HZ,
                    time_approval_s=tick / PHYSICS_HZ - s.approval_s,
                    physics_tick=tick,
                    position_m=list(position),
                    obstacle=name,
                    part=violation_part(position, box),
                    clearance_m=values[name],
                )
                if policy is not None:
                    agent.events.append(
                        dict(
                            kind='safety_violation',
                            physics_tick=tick,
                            flight_s=tick / PHYSICS_HZ,
                            approval_s=tick / PHYSICS_HZ - s.approval_s,
                            obstacle=name,
                            part=violation['part'],
                        )
                    )
            if tick >= approval * (PHYSICS_HZ // HZ) and in_inspection(position, s):
                if entered is None:
                    entered = tick
                dwell = tick - entered
                longest_dwell = max(longest_dwell, dwell)
                if completion is None and dwell >= round(s.inspection_dwell_s * PHYSICS_HZ):
                    completion = tick / PHYSICS_HZ - s.approval_s
                    if policy is not None:
                        agent.events.append(
                            dict(
                                kind='inspection_completed',
                                physics_tick=tick,
                                flight_s=tick / PHYSICS_HZ,
                                approval_s=completion,
                            )
                        )
            else:
                entered = None

        sample = monitor_physics_steps(env, observe)
        sample()
        for step in range(total):
            if step == approval:
                approval_state = obs.copy()
                if capture_snapshot:
                    approval_snapshot = dict(
                        state=obs.copy(),
                        pid_state=capture_pid_state(ctrl),
                        remaining_reference=reference[step:].copy(),
                        horizon_steps=total - step,
                        approval_time=s.approval_s,
                        scenario=s,
                    )
                if gui:
                    view.approval(obs[POS])
            if policy is not None:
                agent.observe(step, obs)
                if step == approval:
                    if expected_snapshot is not None:
                        from .storage import check_approval_snapshot

                        check_approval_snapshot(approval_snapshot, expected_snapshot)
                    agent.approve()
                if step >= approval:
                    agent.announce_readiness(step)
                    if step == scheduled_dispatch:
                        current = Snapshot(
                            obs.copy(),
                            capture_pid_state(ctrl),
                            reference[step:].copy(),
                            total - step,
                            step / HZ,
                            s,
                            min(minimum.values()),
                        )
                        agent.dispatch(step, current, platform)
                    agent.tick(step)
                    selected = agent.take_action()
                    if selected is not None:
                        mode = selected
                        initiation = step
                        start_s = (step - approval) / HZ
                        if gui:
                            view.mode = mode
                            view.start_s = start_s
                            view.camera_key = None
            if step == initiation:
                anchor = obs[POS].copy()
                physical_before = obs.copy()
                pid_before = capture_pid_state(ctrl)
                previous_row = reference[step].copy()
                reference = apply_maneuver(reference, step, obs, s, mode)
                ctrl.reference = reference
                env.X_GOAL = reference
                pid_after = capture_pid_state(ctrl)
                pid_unchanged = all(
                    (
                        np.array_equal(getattr(pid_before, name), getattr(pid_after, name))
                        for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e')
                    )
                )
                state_unchanged = np.array_equal(physical_before, env.state)
                splice = dict(
                    position_m=(reference[step, POS] - previous_row[POS]).tolist(),
                    velocity_mps=(reference[step, VEL] - previous_row[VEL]).tolist(),
                    measured_velocity_mps=obs[VEL].tolist(),
                )
                if gui:
                    view.initiation(anchor, reference[step:])
            trace.append((step / HZ, *obs))
            references.append((step / HZ, *reference[step, POS], *reference[step, VEL]))
            old_position = obs[POS].copy()
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            sample()
            if gui:
                view.update(
                    old_position,
                    obs[POS],
                    step,
                    initiation is not None and step >= initiation,
                    violation is not None,
                    world_y=obs[2],
                    world_vy=obs[3],
                )
                time.sleep(1 / (HZ * playback_speed))
            if done:
                raise RuntimeError(f'Environment terminated early: {step + 1}/{total}')
        trace.append((total / HZ, *obs))
        references.append((total / HZ, *reference[total, POS], *reference[total, VEL]))
        assert len(physics) == total * (PHYSICS_HZ // HZ) + 1
        assert geometry_before == repr(s.obstacles)
        if initiation is not None:
            assert pid_unchanged and state_unchanged
        if gui:
            view.refresh(total / HZ, initiation is not None, violation is not None)
            if capture:
                save_camera(capture, env.PYB_CLIENT)
            deadline = time.monotonic() + hold_s
            while time.monotonic() < deadline and p.isConnected(env.PYB_CLIENT):
                view.refresh(total / HZ, initiation is not None, violation is not None)
                time.sleep(0.05)
        positions = np.asarray(physics)
        record = dict(
            seed=s.seed,
            mode=mode,
            start_s=None if initiation is None else start_s,
            approval_step=approval,
            initiation_step=initiation,
            horizon_step=total,
            horizon_flight_s=total / HZ,
            observation_s=s.observation_s,
            safe=violation is None,
            inspection_completed=completion is not None,
            safe_completion=violation is None and completion is not None,
            completion_approval_s=completion,
            longest_inspection_dwell_s=longest_dwell / PHYSICS_HZ,
            min_clearance_m=min(minimum.values()),
            min_clearance_by_obstacle_m=minimum,
            first_violation=violation,
            physics_samples=len(physics),
            control_samples=len(trace),
            maneuver_anchor_m=None if anchor is None else anchor.tolist(),
            approval_state=approval_state.tolist(),
            pid_unchanged_at_initiation=pid_unchanged,
            physical_state_unchanged_at_initiation=state_unchanged,
            nominal_to_measured_reference_splice=splice,
            geometry_unchanged=True,
            max_physics_position_increment_m=float(
                np.linalg.norm(np.diff(positions[:, 1:], axis=0), axis=1).max()
            ),
            final_position_m=obs[POS].tolist(),
            final_velocity_mps=obs[VEL].tolist(),
            final_position_ned_m=world_to_ned(obs[POS]).tolist(),
            final_velocity_ned_mps=world_to_ned(obs[VEL]).tolist(),
            physics_sha256=hashlib.sha256(positions.tobytes()).hexdigest(),
            runtime_s=time.perf_counter() - began,
            gui=gui,
        )
        if policy is not None:
            event = agent.finish(scheduled_dispatch, total)
            action = (
                event['fallback_step']
                if event['fallback_step'] is not None
                else event['application_step']
            )
            record.update(
                delay_s=delay_s,
                completed=completion is not None,
                completion_s=completion,
                event=event,
                first_violation_s=None if violation is None else violation['time_flight_s'],
                pre_dispatch_violation_s=(
                    violation['time_flight_s']
                    if violation is not None
                    and violation['time_flight_s'] <= scheduled_dispatch / HZ
                    else None
                ),
                violation_before_intervention=violation is not None
                and action is not None
                and (violation['time_flight_s'] <= action / HZ),
                approval_snapshot_verified=expected_snapshot is not None,
            )
        data = dict(physics=positions, control=np.asarray(trace), reference=np.asarray(references))
        if capture_snapshot:
            data['approval_snapshot'] = approval_snapshot
        return (record, data)
    finally:
        ctrl.close()
