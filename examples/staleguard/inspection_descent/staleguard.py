"""StaleGuard T0.
Predicts validity using approval-time information only.

T0 assumptions:
- perfect UAV state
- perfect obstacle geometry
- same SCG dynamics as ground truth
- same PID controller as ground truth

StaleGuard DOES NOT replay from mission start.
Every rollout starts directly from approval time.
"""
from copy import deepcopy
from dataclasses import dataclass
from functools import partial
import numpy as np
import pybullet as p
from safe_control_gym.utils.registration import (
    get_config,
    make,
)

FREQ = 50
EPISODE_SECONDS = 6
MEASURE_SECONDS = 5.5
DELAYS = tuple(
    np.arange(
        0.0,
        1.51,
        0.1,
    )
)


@dataclass(frozen=True)
class PIDState:
    integral_pos_e: np.ndarray
    last_rpy: np.ndarray
    integral_rpy_e: np.ndarray


@dataclass(frozen=True)
class ObstacleMap:
    x_min: float
    x_max: float
    y_limit: float
    unsafe_below_z: float


@dataclass(frozen=True)
class ApprovedCommand:
    low_height: float
    low_steps: int


@dataclass(frozen=True)
class ApprovalSnapshot:
    # [x, vx, y, vy, z, vz,
    #  roll, pitch, yaw,
    #  p_body, q_body, r_body]
    state: np.ndarray
    pid_state: PIDState

    # Index 0 = approval time.
    remaining_reference: np.ndarray

    command: ApprovedCommand
    obstacle: ObstacleMap

    approval_step: int
    horizon_steps: int


@dataclass(frozen=True)
class RolloutCheckpoint:
    pyb_state_id: int
    obs: np.ndarray
    info: dict
    pid_state: PIDState
    ctrl_step_counter: int
    pyb_step_counter: int


def unsafe(state, obstacle):
    return (
        obstacle.x_min
        <= state[0]
        <= obstacle.x_max

        and abs(state[2])
        <= obstacle.y_limit

        and state[4]
        <= obstacle.unsafe_below_z
    )


def predictor_config(state):
    config = get_config(
        "quadrotor"
    )
    config.update({
        "quad_type": 3,
        "task": "traj_tracking",
        "cost": "quadratic",

        "ctrl_freq": FREQ,
        "pyb_freq": 1000,

        "episode_len_sec":
            EPISODE_SECONDS,

        # Start directly at approval.
        "init_state": np.asarray(
            state,
            dtype=float,
        ).copy(),

        "randomized_init": False,
        "randomized_inertial_prop": False,

        "disturbances": None,
        "adversary_disturbance": None,

        "done_on_out_of_bound": False,
        "gui": False,
    })
    return config


def capture_pid_state(ctrl):
    return PIDState(
        integral_pos_e=np.asarray(
            ctrl.integral_pos_e
        ).copy(),
        last_rpy=np.asarray(
            ctrl.last_rpy
        ).copy(),
        integral_rpy_e=np.asarray(
            ctrl.integral_rpy_e
        ).copy(),
    )


def restore_pid_state(ctrl, pid_state):
    ctrl.integral_pos_e = (
        np.asarray(
            pid_state.integral_pos_e
        ).copy()
    )
    ctrl.last_rpy = (
        np.asarray(
            pid_state.last_rpy
        ).copy()
    )
    ctrl.integral_rpy_e = (
        np.asarray(
            pid_state.integral_rpy_e
        ).copy()
    )


def restore_exact_state(
    env,
    state,
):
    """
    SCG observation stores angular rates as body-frame p,q,r.

    PyBullet resetBaseVelocity expects world-frame angular velocity,
    so convert explicitly.
    """
    state = np.asarray(state, dtype=float)
    position = np.array([
        state[0],
        state[2],
        state[4],
    ])
    velocity_world = np.array([
        state[1],
        state[3],
        state[5],
    ])
    rpy = state[6:9]
    body_rates = state[9:12]
    quat = p.getQuaternionFromEuler(
        rpy
    )
    R_world_from_body = np.asarray(
        p.getMatrixFromQuaternion(
            quat
        )
    ).reshape(3, 3)

    angular_velocity_world = R_world_from_body @ body_rates

    p.resetBasePositionAndOrientation(
        env.DRONE_ID,
        position,
        quat,
        physicsClientId=env.PYB_CLIENT,
    )
    p.resetBaseVelocity(
        env.DRONE_ID,
        linearVelocity=velocity_world,
        angularVelocity=(
            angular_velocity_world
        ),
        physicsClientId=env.PYB_CLIENT,
    )

    # Refresh SCG cached state.
    env._update_and_store_kinematic_information()
    obs = env._get_observation()

    return np.asarray(obs).copy()


def save_checkpoint(env, ctrl, obs, info):
    return RolloutCheckpoint(
        pyb_state_id=p.saveState(
            physicsClientId=env.PYB_CLIENT
        ),
        obs=np.asarray(obs).copy(),
        info=deepcopy(info),
        pid_state=capture_pid_state(ctrl),
        ctrl_step_counter=env.ctrl_step_counter,
        pyb_step_counter=env.pyb_step_counter,
    )


def restore_checkpoint(env, ctrl, checkpoint):
    p.restoreState(
        stateId=checkpoint.pyb_state_id,
        physicsClientId=env.PYB_CLIENT,
    )
    env.ctrl_step_counter = checkpoint.ctrl_step_counter
    env.pyb_step_counter = checkpoint.pyb_step_counter
    env._update_and_store_kinematic_information()
    restore_pid_state(
        ctrl,
        checkpoint.pid_state,
    )
    obs = checkpoint.obs.copy()
    info = deepcopy(checkpoint.info)
    return obs, info


def start_from_approval(
    snapshot,
):
    """
    Create predictive simulator with local t=0 = approval time.
    """
    config = predictor_config(
        snapshot.state
    )
    env_factory = partial(
        make,
        "quadrotor",
        **config,
    )

    ctrl = None
    try:
        ctrl = make(
            "pid",
            env_factory,
        )
        # pid.py creates env, use it as predictive environment.
        env = ctrl.env
        obs, info = env.reset(
            seed=0
        )
        # Restore exact physical state.
        obs = restore_exact_state(
            env,
            snapshot.state,
        )
        # Reference is already rebased:
        # local step 0 == approval time.
        reference = np.asarray(
            snapshot.remaining_reference
        ).copy()

        env.X_GOAL = reference.copy()
        ctrl.reference = reference.copy()

        # This resets PID memory.
        ctrl.reset_before_run(
            obs=obs,
            info=info,
            env=env,
        )
        # Restore PID state from approval.
        restore_pid_state(ctrl, pid_state=snapshot.pid_state)
        if not np.allclose(
            obs,
            snapshot.state,
            rtol=1e-7,
            atol=1e-8,
        ):
            raise RuntimeError(
                "Could not reconstruct approval-time state"
            )
        return (
            env,
            ctrl,
            obs,
            info,
            reference,
        )
    except Exception:
        if ctrl is not None:
            ctrl.close()  # PID.close will call env.close() also.
        raise

def rollout_from_checkpoint(
    env,
    ctrl,
    checkpoint,
    reference,
    snapshot,
    dispatch_step,
):
    """
    Restore the nominal state at dispatch time,
    dispatch immediately, and simulate only
    the post-dispatch consequence.
    """

    obs, info = restore_checkpoint(env, ctrl, checkpoint)

    # Dispatch command
    changed = reference.copy()
    end = min(dispatch_step + snapshot.command.low_steps, len(changed))
    changed[
        dispatch_step:end,
        4,
    ] = snapshot.command.low_height
    env.X_GOAL = changed
    ctrl.reference = changed

    # Simulate only AFTER dispatch
    for local_step in range(
        dispatch_step,
        snapshot.horizon_steps,
    ):
        action = ctrl.select_action(obs, info)
        obs, _, done, info = env.step(action)
        # EARLY TERMINATION.
        # Once unsafe has happened, emit UNSAFE immediately even UAV could be SAFE after.
        if unsafe(
            obs,
            snapshot.obstacle,
        ):
            return "UNSAFE"
        if done:
            break

    return "SAFE"


def build_nominal_checkpoints(
    env, ctrl, obs, info, reference, snapshot, delays,
):
    """
    From approval-time snapshot, run nominal mission once.
    Save checkpoint at every candidate dispatch time.

    Later, we can restore a checkpoint and simulate a command.
    """

    checkpoints = {}
    if unsafe(
        obs,
        snapshot.obstacle,
    ):
        return False, checkpoints

    dispatch_steps = {round(float(d) * FREQ) for d in delays} # dispatch set.
    for local_step in range(snapshot.horizon_steps):
        if local_step in dispatch_steps:
            # Save checkpoint
            checkpoints[local_step] = save_checkpoint(env, ctrl, obs, info)
        action = ctrl.select_action(obs, info)
        obs, _, done, info = env.step(action)

        # Early return as UNSAFE if the nominal trajectory before "command dispatch" is unsafe.
        if unsafe(obs, snapshot.obstacle):
            return False, checkpoints
        if done: break
    return True, checkpoints


def predict(
    snapshot,
    delays=DELAYS,
):
    env, ctrl, checkpoints = None, None, {}
    try:
        # ====================================
        # 1. ONE nominal forward trajectory
        # ====================================
        # Init quad, control, trajectory reference from approval-time snapshot
        env, ctrl, obs, info, reference = start_from_approval(snapshot)
        nominal_safe, checkpoints = build_nominal_checkpoints(
            env=env,
            ctrl=ctrl,
            obs=obs,
            info=info,
            reference=reference,
            snapshot=snapshot,
            delays=delays,
        )

        # Nominal mission must itself be safe.
        if not nominal_safe:
            return {
                "classification":
                    "INVALID_NOMINAL",
                "results": None,
                "first_bad_delay": None,
                "last_safe_delay": None,
            }

        # ===============================================
        # 2. Restore checkpoint and simulate the command
        # ===============================================
        results = []
        for delay in delays:
            dispatch_step = round(float(delay) * FREQ)
            checkpoint = checkpoints[dispatch_step]
            status = (
                rollout_from_checkpoint(
                    env=env,
                    ctrl=ctrl,
                    checkpoint=checkpoint,
                    reference=reference,
                    snapshot=snapshot,
                    dispatch_step=dispatch_step,
                )
            )
            results.append(status)

        # ====================================
        # 3. Classification
        # ====================================

        # Immediate command must be safe.
        if results[0] != "SAFE":
            return {
                "classification": "INVALID_AT_APPROVAL",
                "results": results,
                "first_bad_delay": 0.0,
                "last_safe_delay": None,
            }
        first_bad = next(
            (
                i
                for i, status
                in enumerate(results)
                if status != "SAFE"
            ),
            None,
        )
        if first_bad is None:
            return {
                "classification":
                    "ALWAYS_SAFE",
                "results": results,
                "first_bad_delay": None,
                "last_safe_delay":
                    delays[-1],
            }
        monotonic = all(
            status != "SAFE"
            for status
            in results[first_bad:]
        )
        return {
            "classification": (
                "FINITE_TTL"
                if monotonic
                else "NON_MONOTONIC"
            ),
            "results": results,
            "first_bad_delay":
                delays[first_bad],
            "last_safe_delay":
                delays[first_bad - 1],
        }
    finally:
        # Must remove states BEFORE closing PyBullet.
        if env is not None:
            for checkpoint in checkpoints.values():
                try:
                    p.removeState(
                        checkpoint.pyb_state_id,
                        physicsClientId=env.PYB_CLIENT,
                    )
                except Exception:
                    pass

        if ctrl is not None:
            ctrl.close()
