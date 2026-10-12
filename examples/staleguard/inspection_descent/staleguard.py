"""StaleGuard T0.
Predicts validity using approval-time information only.
Every rollout starts directly from approval time.
"""
from dataclasses import dataclass
import numpy as np
from classification import classify_delay_results
from config import CTRL_FREQ_HZ, DELAY_GRID
from model.common import (
    PIDState,
)
from scenario import unsafe, apply_descent_command


@dataclass(frozen=True)
class ObstacleMap:
    box_x_min: float
    box_x_max: float
    box_y_limit: float
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
    changed_traj = apply_descent_command(
        reference=reference,
        start_step=dispatch_step,
        low_steps=snapshot.command.low_steps,
        low_height=snapshot.command.low_height,
    )

    predictive_model.set_reference(
        ctrl,
        changed_traj,
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
        round(float(delay) * CTRL_FREQ_HZ)
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
        predictive_model,
        delays=DELAY_GRID,
):
    checkpoints = {}
    try:
        (
            ctrl,
            obs,
            info,
            reference,
        ) = predictive_model.start(snapshot)
        # 1. Simulate nominal trajectory and save checkpoints.
        nominal_safe, checkpoints = build_nominal_checkpoints(
            predictive_model=predictive_model,
            ctrl=ctrl,
            obs=obs,
            info=info,
            snapshot=snapshot,
            delays=delays,
        )

        if not nominal_safe:
            return classify_delay_results(
                delays,
                results=None,
                nominal_safe=False,
            )

        # 2. Predict each dispatch delay.
        results = []
        for delay in delays:
            dispatch_step = round(
                float(delay) * CTRL_FREQ_HZ
            )
            checkpoint = checkpoints[dispatch_step]
            status = rollout_from_checkpoint(
                predictive_model=predictive_model,
                ctrl=ctrl,
                checkpoint=checkpoint,
                reference=reference,
                snapshot=snapshot,
                dispatch_step=dispatch_step,
            )
            results.append(status)

        # 3. Shared classification.
        return classify_delay_results(delays, results)
    finally:
        predictive_model.close(checkpoints)
