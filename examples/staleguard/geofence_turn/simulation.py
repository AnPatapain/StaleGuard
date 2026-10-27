"""Full SCG/PyBullet execution, constant-memory 1 kHz safety/dwell monitor."""
from dataclasses import dataclass
from functools import partial
import math
import ctypes
import gc
from pathlib import Path
import time
import numpy as np
import pybullet as p
from safe_control_gym.utils.registration import get_config, make
from model.common import capture_pid_state, PIDState
from physics_monitor import monitor_physics_steps
from .baselines.staleguard import staleguard_status
from .baselines.fixed_ttl import fixed_ttl_expired, FIXED_TTL_S
from .baselines.use_time_recheck import first_step_after_check
from .baselines import always_recover, fbs
from .scenario import (HZ, PHYSICS_HZ, POS, VEL, Config, make_forward_trajectory,
                      apply_turn_command, clearance, in_checkpoint, safety_only_reference)


@dataclass(frozen=True)
class Snapshot:
    state: np.ndarray
    pid_state: PIDState
    remaining_reference: np.ndarray
    horizon_steps: int
    approval_time: float
    command: object
    fence_x: float
    config: Config
    prefix_min_clearance_m: float


def check_memory(config):
    status = Path('/proc/self/status').read_text().splitlines()
    rss = int(next(x.split()[1] for x in status if x.startswith('VmRSS:'))) / 1024
    mem = Path('/proc/meminfo').read_text().splitlines()
    available = int(next(x.split()[1] for x in mem if x.startswith('MemAvailable:'))) / 1024
    if rss > config.max_rss_mb or available < config.min_available_mb:
        raise RuntimeError(f'Memory guard: RSS={rss:.0f} MiB available={available:.0f} MiB')
    return rss, available


def start_work(max_trials=160, max_seconds=900):
    """A plain counter shared by a script's flights or diagnostic forecasts."""
    if not 1 <= max_trials <= 200 or not 0 < max_seconds <= 1800:
        raise ValueError('Development limit: 1..200 trials and at most 1800 seconds')
    return dict(trials=0, start=time.monotonic(), max_trials=max_trials, max_seconds=max_seconds)


def check_work(work, config, release=False):
    if work['trials'] >= work['max_trials'] or time.monotonic()-work['start'] >= work['max_seconds']:
        raise RuntimeError('Explicit development trial/time budget reached; partial logs preserved')
    if release:
        gc.collect()
        ctypes.CDLL(None).malloc_trim(0)
    check_memory(config)
    work['trials'] += 1


def env_factory(s, total_steps):
    config = get_config('quadrotor')
    config.update(quad_type=3, task='traj_tracking', cost='quadratic', ctrl_freq=HZ,
                  pyb_freq=PHYSICS_HZ, episode_len_sec=(total_steps + 5)/HZ,
                  init_state={'init_x': s.initial_x, 'init_y': s.initial_y, 'init_z': s.height},
                  randomized_init=False, randomized_inertial_prop=False, disturbances=None,
                  adversary_disturbance=None, done_on_out_of_bound=False, gui=False)
    return partial(make, 'quadrotor', **config)


def execute(s, delay=0., config=Config(), policy='no_recovery', received=None,
            platform=None, save_trace=False, work=None):
    """Replay from initial state, with no simulator reset/teleport at approval.

    received is computed from approval information before the caller enumerates
    actual D. Use-Time Recheck instead captures the actual dispatch state/PID.
    Every run monitors through approval+T even after completion or a violation.
    """
    if work is not None:
        check_work(work, config, release=True)
    check_memory(config)
    approval = round(s.approval_s * HZ)
    total = approval + math.ceil(s.deadline_s * HZ - 1e-10)
    dispatch = None if delay is None else approval + round(delay * HZ)
    if delay is not None and (delay < 0 or dispatch >= total):
        raise ValueError('Delay outside monitored horizon')

    if policy not in (
            'no_recovery', 'staleguard', 'staleguard_early_recovery',
            'use_time_recheck', 'fixed_ttl', 'always_recover', 'fbs'
    ):
        raise ValueError('Unadapted policy')

    if policy in ('staleguard', 'staleguard_early_recovery') and received is None:
        raise ValueError('StaleGuard requires approval-only prediction')

    factory = env_factory(s, total)
    env, ctrl = factory(), None
    snapshot = None
    min_clear = math.inf
    first_violation = None
    first_wait_violation = None
    entered = None
    completion = None
    samples = 0
    max_error = 0.
    max_arc_error = 0.
    max_tilt = 0.
    trace = []
    event = {'policy': policy, 'scheduled_dispatch_step': dispatch,
             'application_step': None, 'fallback_step': None, 'prediction_ready': None}
    if policy == 'staleguard_early_recovery':
        event.update(
            early_recovery=False,
            computation_s=received['computation_s'],
            expiry_steps=received['expiry_steps'],
        )

    if policy == 'fbs':
        previous_feature, volatility = None, 0.0
        # Diagnostics are optional; the FBS decision always uses each 50 Hz observation.
        if save_trace:
            event.update(observation_steps=[], features_m=[], observation_update_s=[])
    started = time.perf_counter()
    try:
        ctrl = make('pid', factory)
        obs, info = env.reset(seed=s.seed)
        ctrl.reset_before_run(obs=obs, info=info, env=env)
        ref = make_forward_trajectory(total + 1, s)
        ctrl.reference = ref
        env.X_GOAL = ref
        action_step = dispatch
        recover = False

        def observe(position, tick):
            nonlocal min_clear, first_violation, first_wait_violation
            nonlocal entered, completion, samples
            samples += 1
            current = float(clearance(position[0], s.fence_x))
            min_clear = min(min_clear, current)
            if current < config.d_min_m:
                if first_violation is None:
                    first_violation = tick / PHYSICS_HZ
                if dispatch is None or tick <= dispatch * (PHYSICS_HZ//HZ):
                    if first_wait_violation is None:
                        first_wait_violation = tick / PHYSICS_HZ
            velocity, _ = p.getBaseVelocity(env.DRONE_ID, physicsClientId=env.PYB_CLIENT)
            elapsed = tick / PHYSICS_HZ - approval/HZ
            if (0 <= elapsed <= s.deadline_s
                    and in_checkpoint(position, velocity, s.command, config)):
                if entered is None:
                    entered = tick
                if completion is None and tick-entered >= round(config.dwell_s*PHYSICS_HZ):
                    completion = elapsed
            else:
                entered = None

        sample = monitor_physics_steps(env, observe)
        sample()
        for step in range(total):
            if step == approval:
                snapshot = Snapshot(obs.copy(), capture_pid_state(ctrl), ref[step:].copy(),
                                    total-step, s.approval_s, s.command, s.fence_x,
                                    config, min_clear)
            if policy == 'fbs' and step <= approval:
                began = time.perf_counter()
                feature = fbs.safety_feature(obs, s, config)
                volatility = fbs.update_volatility(feature, previous_feature, volatility)
                previous_feature = feature
                if step == approval:
                    frozen = fbs.freeze_prediction(feature, volatility)
                elapsed = time.perf_counter()-began
                if save_trace:
                    event['observation_steps'].append(step)
                    event['features_m'].append(feature)
                    event['observation_update_s'].append(elapsed)
                if step == approval:
                    event.update(prediction=frozen, computation_s=elapsed,
                                 observation_step=step, cold_start=step == 0)
                    if save_trace:
                        event['observation_state'] = obs.tolist()

            # EarlyRecovery: act when TTL expires while the command is pending.
            if (
                    policy == 'staleguard_early_recovery'
                    and dispatch is not None
                    and approval <= step < dispatch
                    and not event['early_recovery']
            ):
                ready, expired = staleguard_status(
                    received, step - approval
                )
                if ready and expired:
                    ref = safety_only_reference(ref, step, obs)
                    ctrl.reference = ref
                    env.X_GOAL = ref
                    event.update(
                        fallback_step=step,
                        prediction_ready=True,
                        early_recovery=True,
                    )
                    # Cancel the pending turn permanently.
                    action_step = None

            if step == dispatch:
                if policy in ('staleguard', 'staleguard_early_recovery'):
                    ready, expired = staleguard_status(
                        received, step - approval
                    )
                    # Do not override a recovery already started before dispatch.
                    if event['fallback_step'] is None:
                        recover = expired
                    event.update(
                        prediction_ready=ready,
                        computation_s=received['computation_s'],
                        expiry_steps=received['expiry_steps'],
                    )
                elif policy == 'fixed_ttl':
                    # Preregistered development control, not tuned against outcomes.
                    recover = fixed_ttl_expired(step-approval)
                    event['fixed_ttl_s'] = FIXED_TTL_S
                elif policy == 'always_recover':
                    recover = always_recover.should_recover()
                elif policy == 'fbs':
                    status = fbs.dispatch_status(frozen, event['computation_s'], step-approval)
                    recover = status['recover']
                    event.update(status, prediction_ready=status['ready'],
                                 dispatch_step=step, waited_steps=step-approval)
                elif policy == 'use_time_recheck':
                    from .baselines.use_time_recheck import predict_immediate
                    current = Snapshot(obs.copy(), capture_pid_state(ctrl), ref[step:].copy(),
                                       total-step, step/HZ, s.command, s.fence_x, config, min_clear)
                    began = time.perf_counter()
                    result = predict_immediate(current, platform)
                    elapsed = time.perf_counter()-began
                    action_step = first_step_after_check(step, elapsed)
                    recover = not result['safe']
                    event.update(check=result, computation_s=elapsed, decision_step=action_step,
                                 check_state=obs.tolist(), prediction_ready=False)
            if step == action_step:
                if recover:
                    ref = safety_only_reference(ref, step, obs)
                    event['fallback_step'] = step
                else:
                    ref = apply_turn_command(ref, step, s.command)
                    event['application_step'] = step
                if policy == 'use_time_recheck':
                    event['prediction_ready'] = True
                ctrl.reference = ref
                env.X_GOAL = ref
            if save_trace:
                trace.append((step, *obs, *ref[step]))
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            sample()
            error = float(np.linalg.norm(obs[POS]-ref[step+1, POS]))
            if step >= approval:
                max_error = max(max_error, error)
                if dispatch is not None and dispatch <= step < dispatch + math.ceil(s.command.arc_s*HZ):
                    max_arc_error = max(max_arc_error, error)
                max_tilt = max(max_tilt, float(np.max(np.abs(obs[6:8]))))
            if done:
                raise RuntimeError(f'SCG terminated early at {step+1}/{total}; info={info}')
        if save_trace:
            trace.append((total, *obs, *ref[total]))
        safe = first_violation is None
        # Flight-start seconds: first_violation_s, pre_dispatch_violation_s, horizon_s.
        # Approval-relative seconds: completion_s. Keep these existing journal fields.
        result = dict(seed=s.seed, delay_s=delay, min_clearance_m=min_clear, safe=safe,
                      completed=completion is not None, completion_s=completion,
                      safe_completion=safe and completion is not None,
                      actual_patrol_distance_m=float(snapshot.state[0]-s.initial_x),
                      first_violation_s=first_violation, pre_dispatch_violation_s=first_wait_violation,
                      max_tracking_error_m=max_error, max_arc_tracking_error_m=max_arc_error,
                      max_tilt_rad=max_tilt, physics_samples=samples,
                      expected_physics_samples=total*(PHYSICS_HZ//HZ)+1,
                      final_position=obs[POS].tolist(), final_velocity=obs[VEL].tolist(),
                      horizon_s=total/HZ, event=event, runtime_s=time.perf_counter()-started)
        assert samples == result['expected_physics_samples']
        return result, snapshot, np.asarray(trace)
    finally:
        if ctrl is not None:
            ctrl.close()
        env.close()
