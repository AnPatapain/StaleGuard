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


def start_from_approval(
    snapshot,
):
    """
    Create predictive simulator with local t=0 = approval time.
    """
    if snapshot.horizon_steps <= 0:
        raise ValueError(
            "Invalid prediction horizon"
        )
    if (
        len(snapshot.remaining_reference)
        < snapshot.horizon_steps
    ):
        raise ValueError(
            "Reference shorter than prediction horizon"
        )
    config = predictor_config(
        snapshot.state
    )
    env_factory = partial(
        make,
        "quadrotor",
        **config,
    )

    env = env_factory()
    ctrl = None

    try:
        ctrl = make(
            "pid",
            env_factory,
        )
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
        ctrl.integral_pos_e = (
            np.asarray(
                snapshot.pid_state.integral_pos_e
            ).copy()
        )
        ctrl.last_rpy = (
            np.asarray(
                snapshot.pid_state.last_rpy
            ).copy()
        )
        ctrl.integral_rpy_e = (
            np.asarray(
                snapshot.pid_state.integral_rpy_e
            ).copy()
        )
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
            ctrl.close()
        env.close()
        raise


def rollout_delay(
    snapshot,
    delay,
):
    """
    Two-stage rollout:
    1. Continue old mission for delay d.
    2. Dispatch approved command and simulate consequence.
    delay=None:
        never execute the command.
    """
    env, ctrl, obs, info, reference = start_from_approval(snapshot)

    try:
        if delay is None:
            dispatch_step = None
        else:
            dispatch_step = round(float(delay) * FREQ)
            if dispatch_step >= snapshot.horizon_steps:
                raise ValueError("Dispatch beyond horizon")

        states = [
            np.asarray(
                obs
            ).copy()
        ]
        for local_step in range(snapshot.horizon_steps):
            # -----------------------
            # Stage 1 -> Stage 2
            # -----------------------
            if dispatch_step is not None and local_step == dispatch_step:
                changed = (
                    reference.copy()
                )
                end = min(
                    local_step
                    + snapshot.command.low_steps,
                    len(changed),
                )
                changed[
                    local_step:end,
                    4,
                ] = (
                    snapshot.command.low_height
                )

                env.X_GOAL = changed
                ctrl.reference = changed

            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            states.append(np.asarray(obs).copy())

            if done: break

        # ---------------------------
        # Nominal mission
        # ---------------------------

        if dispatch_step is None:
            if any(unsafe(state, snapshot.obstacle) for state in states):
                return "NOMINAL_UNSAFE"
            return "SAFE"

        boundary = (
            dispatch_step + 1
        )
        # Stage 1 safety.
        if any(
            unsafe(state, snapshot.obstacle) for state in states[:boundary]
        ):
            return "WAIT_UNSAFE"

        # Stage 2 safety.
        if any(
            unsafe(state, snapshot.obstacle) for state in states[boundary:]
        ):
            return "UNSAFE"
        return "SAFE"

    finally:
        ctrl.close()
        env.close()


def predict(
    snapshot,
    delays=DELAYS,
):
    # Nominal mission must itself be safe.
    if (
        rollout_delay(
            snapshot,
            None,
        )
        != "SAFE"
    ):
        return {
            "classification":
                "INVALID_NOMINAL",
            "results": None,
            "first_bad_delay": None,
            "last_safe_delay": None,
        }

    results = [
        rollout_delay(
            snapshot,
            delay,
        )
        for delay in delays
    ]

    # Immediate command must be safe.
    if results[0] != "SAFE":
        return {
            "classification":
                "INVALID_AT_APPROVAL",
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
