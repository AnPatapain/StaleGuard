"""StaleGuard T0.
Predicts validity using approval-time information only.
Every rollout starts directly from approval time.
"""
from dataclasses import dataclass
import numpy as np
from model.common import (
    FREQ,
    PIDState,
)
from model.pybullet import PyBullet1000HzPredictiveModel


MEASURE_SECONDS = 5.5
DELAYS = tuple(
    np.arange(
        0.0,
        1.51,
        0.1,
    )
)


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


def rollout_from_checkpoint(
    predictive_model,
    ctrl,
    checkpoint,
    reference,
    snapshot,
    dispatch_step,
):
    obs, info = predictive_model.restore_checkpoint(
        ctrl,
        checkpoint,
    )

    # Immediately dispatch command.
    changed = reference.copy()

    end = min(
        dispatch_step + snapshot.command.low_steps,
        len(changed),
    )

    changed[
        dispatch_step:end,
        4,
    ] = snapshot.command.low_height

    predictive_model.set_reference(
        ctrl,
        changed,
    )

    # Simulate after dispatch.
    for local_step in range(
        dispatch_step,
        snapshot.horizon_steps,
    ):
        action = ctrl.select_action(
            obs,
            info,
        )

        obs, done, info = predictive_model.step(
            action,
            obs,
            info,
        )

        if unsafe(
            obs,
            snapshot.obstacle,
        ):
            return "UNSAFE"

        if done:
            break

    return "SAFE"


def build_nominal_checkpoints(
    predictive_model,
    ctrl,
    obs,
    info,
    reference,
    snapshot,
    delays,
):
    """
    Simulate waiting once and save a predictive-model checkpoint
    at every hypothetical dispatch delay.
    """

    checkpoints = {}

    if unsafe(
        obs,
        snapshot.obstacle,
    ):
        return False, checkpoints

    dispatch_steps = {
        round(float(delay) * FREQ)
        for delay in delays
    }

    for local_step in range(
        snapshot.horizon_steps
    ):
        if local_step in dispatch_steps:
            checkpoints[local_step] = (
                predictive_model.save_checkpoint(
                    ctrl,
                    obs,
                    info,
                )
            )

        # Normal mission while command waits.
        action = ctrl.select_action(
            obs,
            info,
        )

        obs, done, info = predictive_model.step(
            action,
            obs,
            info,
        )

        if unsafe(
            obs,
            snapshot.obstacle,
        ):
            return False, checkpoints

        if done:
            break

    return True, checkpoints


def predict(
    snapshot,
    delays=DELAYS,
    predictive_model=None,
):
    if predictive_model is None:
        predictive_model = PyBullet1000HzPredictiveModel()

    checkpoints = {}

    try:
        (
            ctrl,
            obs,
            info,
            reference,
        ) = predictive_model.start(
            snapshot
        )

        nominal_safe, checkpoints = (
            build_nominal_checkpoints(
                predictive_model=predictive_model,
                ctrl=ctrl,
                obs=obs,
                info=info,
                reference=reference,
                snapshot=snapshot,
                delays=delays,
            )
        )

        if not nominal_safe:
            return {
                "classification":
                    "INVALID_NOMINAL",
                "results": None,
                "first_bad_delay": None,
                "last_safe_delay": None,
            }

        results = []

        for delay in delays:
            dispatch_step = round(
                float(delay) * FREQ
            )

            checkpoint = checkpoints[
                dispatch_step
            ]

            status = rollout_from_checkpoint(
                predictive_model=predictive_model,
                ctrl=ctrl,
                checkpoint=checkpoint,
                reference=reference,
                snapshot=snapshot,
                dispatch_step=dispatch_step,
            )

            results.append(status)

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

    finally:
        predictive_model.close(
            checkpoints
        )