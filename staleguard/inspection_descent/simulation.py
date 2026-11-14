"""Shared original physical flight engine and standardized execution records.

Safety-only recovery retains forward motion and requests usual altitude.
The 1000 Hz monitor preserves first-unsafe termination and the original traces.
"""

from dataclasses import replace
import hashlib
import numpy as np
import time
import pybullet as p
from .scenario import (
    ApprovalSnapshot,
    apply_descent_command,
    unsafe,
    CTRL_FREQ_HZ,
    PYB_FREQ_HZ,
    MEASURE_SECONDS,
    SIM_SEED,
    make_forward_trajectory,
    inspection_x,
)
from safe_control_gym.utils.registration import make
from staleguard.model.common import STATE_X, STATE_Y, STATE_Z, capture_pid_state
from staleguard.model.lightweight import LightweightPredictiveModel
from staleguard.model.scg import load_scg_platform_params, make_scg_env_factory
from staleguard.physics_monitor import monitor_physics_steps
from .baselines.use_time_recheck import predict_immediate, first_step_after_check
from .baselines import fbs
from .scenario import MISSION_DEADLINE_S, SCAN_DWELL_S, in_inspection_region, safety_only_reference
from .baselines.staleguard import ttl_expired_at_dispatch


def run_actual(
    s,
    snapshot,
    dispatch_step,
    recover_step,
    deadline_step,
    recover,
    gui=False,
    trace=None,
    physics_trace=None,
    ttl_prediction=None,
    use_time_check=None,
    recheck_log=None,
    fbs_log=None,
    event=None,
    early_policy=None,
    truth_only=False,
):
    """Execute one flight; recovery cancels descent and keeps forward flight."""
    if use_time_check is not None and (ttl_prediction is not None or recover):
        raise ValueError('Use-Time Recheck requires its own dispatch decision')
    if fbs_log is not None and (
        ttl_prediction is not None or use_time_check is not None or recover
    ):
        raise ValueError('FBS requires its own dispatch decision')
    env_factory = make_scg_env_factory(
        init_state={'init_x': 0.0, 'init_y': 0.0, 'init_z': s.usual_height}, pyb_freq=PYB_FREQ_HZ
    )
    env = env_factory(gui=gui)
    ctrl = make('pid', env_factory)
    obs, info = env.reset(seed=SIM_SEED)
    ctrl.reset_before_run(obs=obs, info=info, env=env)
    traj = make_forward_trajectory(env, s)
    assert deadline_step <= len(traj)
    env.X_GOAL = traj.copy()
    ctrl.reference = traj.copy()
    if trace is not None:
        trace.append((0, obs.copy(), traj[0].copy()))
    if event is not None:
        control_digest, physics_digest = (hashlib.sha256(), hashlib.sha256())
        for value in (0, obs, traj[0]):
            control_digest.update(np.asarray(value).tobytes())
        event.update(physics_samples=0, first_violation_s=None, completion_s=None)
    scan_x = inspection_x(s)
    scan_start_tick = None
    completed = False
    collision = False
    recovery_started = False
    command_step = dispatch_step
    check_result = None
    if fbs_log is not None:
        approval_step = round(snapshot.approval_time * CTRL_FREQ_HZ)
        if dispatch_step < approval_step:
            raise ValueError('FBS dispatch must not precede approval')
        previous_feature, volatility = (None, 0.0)
        fbs_log.update(observation_steps=[], features_m=[], observation_update_s=[])
    if early_policy is not None:
        if event is None:
            raise ValueError('Early policies require standardized safety-only execution')
        approval_step = round(snapshot.approval_time * CTRL_FREQ_HZ)
    truth = dict(hit_nominal=None, hit_while_waiting=None, hit_after_command=None)
    approval_tick = round(snapshot.approval_time * CTRL_FREQ_HZ) * (PYB_FREQ_HZ // CTRL_FREQ_HZ)
    dispatch_tick = None if dispatch_step is None else dispatch_step * (PYB_FREQ_HZ // CTRL_FREQ_HZ)

    def monitor(state, tick):
        nonlocal scan_start_tick, completed, collision
        if truth_only:
            if tick >= approval_tick and unsafe(state, snapshot):
                key = (
                    'hit_nominal'
                    if dispatch_tick is None
                    else 'hit_while_waiting' if tick <= dispatch_tick else 'hit_after_command'
                )
                if truth[key] is None:
                    truth[key] = tick / PYB_FREQ_HZ
            return
        collision = collision or unsafe(state, snapshot)
        if event is not None and event['first_violation_s'] is None and unsafe(state, snapshot):
            event['first_violation_s'] = tick / PYB_FREQ_HZ
        if in_inspection_region(state, s, scan_x):
            if scan_start_tick is None:
                scan_start_tick = tick
            elapsed_ticks = tick - scan_start_tick
            if elapsed_ticks >= round(SCAN_DWELL_S * PYB_FREQ_HZ):
                completed = True
                if event is not None and event['completion_s'] is None:
                    event['completion_s'] = tick / PYB_FREQ_HZ - s.approval_time
        else:
            scan_start_tick = None

    def monitor_position(position, tick):
        state = obs.copy()
        state[[STATE_X, STATE_Y, STATE_Z]] = position
        monitor(state, tick)
        if event is not None:
            event['physics_samples'] += 1
            event['last_physics_tick'] = tick
            physics_digest.update(np.asarray(tick).tobytes())
            physics_digest.update(np.asarray(position).tobytes())
            if tick and tick % PYB_FREQ_HZ == 0:
                from .storage import check_memory

                check_memory()
        if physics_trace is not None:
            physics_trace.append((tick, position))

    sample_physics = monitor_physics_steps(env, monitor_position)
    if gui:
        p.addUserDebugText(
            'SCAN TARGET',
            [scan_x, 0, s.low_height],
            textColorRGB=[0, 1, 0],
            physicsClientId=env.PYB_CLIENT,
        )
        p.addUserDebugText(
            'UNSAFE START',
            [s.box_x_min, 0, s.unsafe_below_z],
            textColorRGB=[1, 0, 0],
            physicsClientId=env.PYB_CLIENT,
        )
    try:
        for step in range(deadline_step):
            if event is not None and step == round(snapshot.approval_time * CTRL_FREQ_HZ):
                event['approval_snapshot'] = replace(
                    snapshot,
                    state=obs.copy(),
                    pid_state=capture_pid_state(ctrl),
                    remaining_reference=traj[step:].copy(),
                )
            if fbs_log is not None and step <= approval_step:
                started = time.perf_counter()
                feature = fbs.clearance(obs, snapshot)
                volatility = fbs.update_volatility(feature, previous_feature, volatility)
                previous_feature = feature
                if step == approval_step:
                    frozen = fbs.freeze_prediction(feature, volatility)
                elapsed = time.perf_counter() - started
                fbs_log['observation_steps'].append(step)
                fbs_log['features_m'].append(feature)
                fbs_log['observation_update_s'].append(elapsed)
                if step == approval_step:
                    fbs_log.update(
                        prediction=frozen,
                        computation_s=elapsed,
                        observation_step=step,
                        observation_state=obs.copy(),
                        cold_start=step == 0,
                    )
            if fbs_log is not None and step == dispatch_step:
                waited_steps = step - approval_step
                status = fbs.dispatch_status(frozen, fbs_log['computation_s'], waited_steps)
                recover = status['recover']
                recover_step = step
                fbs_log.update(
                    status,
                    dispatch_step=step,
                    waited_steps=waited_steps,
                    application_step=None if recover else step,
                    fallback_step=step if recover else None,
                )
            if use_time_check is not None and step == dispatch_step:
                started = time.perf_counter()
                current = replace(
                    snapshot,
                    state=obs.copy(),
                    pid_state=capture_pid_state(ctrl),
                    remaining_reference=traj[step:].copy(),
                    approval_time=step / CTRL_FREQ_HZ,
                    horizon_steps=round(MEASURE_SECONDS * CTRL_FREQ_HZ) - step,
                )
                check_result = use_time_check(current)
                elapsed = time.perf_counter() - started
                command_step = first_step_after_check(dispatch_step, elapsed)
                if recheck_log is not None:
                    recheck_log.update(
                        snapshot=current,
                        prediction=check_result,
                        computation_s=elapsed,
                        scheduled_dispatch_step=dispatch_step,
                        decision_step=command_step,
                        decision_applied_step=None,
                        application_step=None,
                        fallback_step=None,
                    )
            if use_time_check is not None and step == command_step and (check_result is not None):
                recover = check_result != 'SAFE'
                recover_step = step
                if recheck_log is not None:
                    recheck_log.update(
                        decision_applied_step=step,
                        state_at_decision=obs.copy(),
                        application_step=None if recover else step,
                        fallback_step=step if recover else None,
                    )
            if (
                early_policy is not None
                and approval_step <= step < dispatch_step
                and (not recovery_started)
            ):
                trigger = False
                if early_policy == 'staleguard_early_recovery':
                    prediction, duration = ttl_prediction
                    trigger = duration <= (
                        step - approval_step
                    ) / CTRL_FREQ_HZ and ttl_expired_at_dispatch(prediction, step - approval_step)
                if trigger:
                    recover = True
                    recover_step = step
                    command_step = None
                    if early_policy == 'staleguard_early_recovery':
                        event['prediction_ready'] = True
            if step == dispatch_step and ttl_prediction is not None:
                prediction, computation_s = ttl_prediction
                waited_steps = step - round(snapshot.approval_time * CTRL_FREQ_HZ)
                ready = computation_s <= waited_steps / CTRL_FREQ_HZ
                if not recovery_started:
                    recover = ready and ttl_expired_at_dispatch(prediction, waited_steps)
                if event is not None:
                    event.update(
                        prediction_ready=ready,
                        computation_s=computation_s,
                        expiry_steps=(
                            0
                            if prediction['classification']
                            in ('INVALID_AT_APPROVAL', 'INVALID_NOMINAL')
                            else (
                                None
                                if prediction['first_bad_delay'] is None
                                else round(prediction['first_bad_delay'] * CTRL_FREQ_HZ)
                            )
                        ),
                    )
                if not recovery_started:
                    recover_step = step
            if recover and step == recover_step:
                recovery_started = True
                if event is not None:
                    event.update(fallback_step=step, early_recovery=step < dispatch_step)
                traj = safety_only_reference(traj, step, s.usual_height)
                env.X_GOAL = traj
                ctrl.reference = traj
            if not recover and step == command_step:
                if event is not None:
                    event['application_step'] = step
                traj = apply_descent_command(traj, step, snapshot.low_steps, snapshot.low_height)
                env.X_GOAL = traj
                ctrl.reference = traj
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            if trace is not None:
                trace.append((step + 1, obs.copy(), traj[step].copy()))
            sample_physics()
            if event is not None:
                for value in (step + 1, obs, traj[step]):
                    control_digest.update(np.asarray(value).tobytes())
                event['last_control_step'] = step + 1
            if gui:
                time.sleep(1.0 / CTRL_FREQ_HZ)
            if collision and (not truth_only):
                break
            if done:
                break
        if gui:
            print(f'completed={completed}, collision={collision}')
            pass
        if event is not None:
            event.update(
                control_sha256=control_digest.hexdigest(), physics_sha256=physics_digest.hexdigest()
            )
        if truth_only:
            return truth
        return (
            bool(completed),
            bool(collision),
            recovery_started,
            bool(completed and (not collision)),
        )
    finally:
        ctrl.close()
        env.close()


def execute(
    s,
    delay=0.0,
    policy='no_recovery',
    snapshot=None,
    seed=None,
    received=None,
    platform=None,
    save_trace=False,
    forced_action=None,
    gui=False,
):
    """One original physical flight; approval forecasts remain frozen inputs."""
    from .storage import check_memory, check_approval_snapshot

    check_memory()
    if policy in ('use_time_recheck',) and platform is None:
        platform = load_scg_platform_params()
    if snapshot is None:
        snapshot = capture_approval(s)
    approval = round(snapshot.approval_time * CTRL_FREQ_HZ)
    dispatch = approval + round(delay * CTRL_FREQ_HZ)
    deadline = round((snapshot.approval_time + MISSION_DEADLINE_S) * CTRL_FREQ_HZ)
    if delay < 0 or dispatch >= deadline:
        raise ValueError('Dispatch outside original mission horizon')
    event = dict(
        policy=policy,
        scheduled_dispatch_step=dispatch,
        application_step=None,
        fallback_step=None,
        early_recovery=False,
        prediction_ready=None,
    )
    kwargs, log = ({}, {})
    recover, recover_step = (False, dispatch)
    if policy == 'fixed_early_recovery' and forced_action is None:
        from staleguard.model.mission_evaluation import clock_action

        forced_action, _ = clock_action(
            policy, snapshot, delay, horizon_steps=round(MISSION_DEADLINE_S * CTRL_FREQ_HZ)
        )
    if forced_action is not None:
        kind, action_step = forced_action
        if kind not in ('command', 'recovery') or not approval <= action_step < deadline:
            raise ValueError('Invalid recorded clock-policy action')
        if kind == 'command' and action_step != dispatch:
            raise ValueError('Approved command starts only at dispatch')
        recover, recover_step = (kind == 'recovery', action_step)
    elif policy in ('staleguard', 'staleguard_early_recovery'):
        if received is None:
            raise ValueError('StaleGuard requires a frozen approval forecast')
        kwargs['ttl_prediction'] = (received['prediction'], received['computation_s'])
        if policy == 'staleguard_early_recovery':
            kwargs['early_policy'] = policy
        event.update(
            computation_s=received['computation_s'],
            expiry_steps=(
                0
                if received['prediction']['classification']
                in ('INVALID_NOMINAL', 'INVALID_AT_APPROVAL')
                else (
                    None
                    if received['prediction']['first_bad_delay'] is None
                    else round(received['prediction']['first_bad_delay'] * CTRL_FREQ_HZ)
                )
            ),
        )
    elif policy == 'use_time_recheck':
        kwargs.update(
            use_time_check=lambda current: predict_immediate(
                current, LightweightPredictiveModel(*platform)
            ),
            recheck_log=log,
        )
    elif policy == 'fbs':
        kwargs['fbs_log'] = log
    elif policy != 'no_recovery':
        raise ValueError(f'Unsupported policy: {policy}')
    trace = [] if save_trace else None
    began = time.perf_counter()
    outcome = run_actual(
        s,
        snapshot,
        dispatch,
        recover_step,
        deadline,
        recover,
        trace=trace,
        event=event,
        gui=gui,
        **kwargs,
    )
    duration = time.perf_counter() - began
    replayed = event.pop('approval_snapshot')
    check_approval_snapshot(replayed, snapshot)
    current = log.pop('snapshot', None)
    if current is not None:
        event.update(
            snapshot_state=current.state.tolist(),
            snapshot_time_s=current.approval_time,
            snapshot_horizon_steps=current.horizon_steps,
            snapshot_low_steps=current.low_steps,
        )
    for name, value in log.items():
        if name in ('application_step', 'fallback_step') and value is None:
            continue
        event[name] = value.tolist() if isinstance(value, np.ndarray) else value
    event['policy'] = policy
    event['early_recovery'] = (
        event['fallback_step'] is not None and event['fallback_step'] < dispatch
    )
    first = event.pop('first_violation_s')
    completion = event.pop('completion_s')
    result = dict(
        seed=seed,
        delay_s=float(round(delay * CTRL_FREQ_HZ) / CTRL_FREQ_HZ),
        safe=not outcome[1],
        completed=outcome[0],
        safe_completion=outcome[3],
        completion_s=completion,
        first_violation_s=first,
        pre_dispatch_violation_s=(
            first if first is not None and first <= dispatch / CTRL_FREQ_HZ else None
        ),
        physics_samples=event['physics_samples'],
        monitored_until_s=event['last_physics_tick'] / PYB_FREQ_HZ,
        horizon_s=deadline / CTRL_FREQ_HZ,
        monitoring_stopped_unsafe=outcome[1],
        control_sha256=event.pop('control_sha256'),
        physics_sha256=event.pop('physics_sha256'),
        event=event,
        runtime_s=duration,
    )
    return (result, replayed, trace)


def capture_approval(scenario, seed=SIM_SEED):
    """Fly the mission to approval and capture the information available then."""
    env_factory = make_scg_env_factory(
        init_state={'init_x': 0.0, 'init_y': 0.0, 'init_z': scenario.usual_height},
        pyb_freq=PYB_FREQ_HZ,
    )
    env = env_factory()
    ctrl = None
    try:
        ctrl = make('pid', env_factory)
        assert ctrl is not None
        obs, info = env.reset(seed=seed)
        ctrl.reset_before_run(obs=obs, info=info, env=env)
        trajectory = make_forward_trajectory(env, scenario)
        env.X_GOAL = trajectory.copy()
        ctrl.reference = trajectory.copy()
        approval_step = round(scenario.approval_time * CTRL_FREQ_HZ)
        total_steps = round(MEASURE_SECONDS * CTRL_FREQ_HZ)
        for _ in range(approval_step):
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            if done:
                raise RuntimeError('Mission terminated before approval')
        return ApprovalSnapshot(
            state=np.asarray(obs).copy(),
            pid_state=capture_pid_state(ctrl),
            remaining_reference=np.asarray(trajectory[approval_step:]).copy(),
            low_height=float(scenario.low_height),
            low_steps=round(scenario.low_seconds * CTRL_FREQ_HZ),
            box_x_min=float(scenario.box_x_min),
            box_x_max=float(scenario.box_x_max),
            box_y_limit=float(scenario.box_y_limit),
            unsafe_below_z=float(scenario.unsafe_below_z),
            approval_time=float(scenario.approval_time),
            horizon_steps=total_steps - approval_step,
        )
    finally:
        if ctrl is not None:
            ctrl.close()
        env.close()
