"""Command-safety prediction for the two command-safety ablations.
Predicts validity using approval-time information only.
Every rollout starts directly from approval time.
Safety covers straight position segments between consecutive control states.
"""

import time
from staleguard.model.lightweight import LightweightPredictiveModel
from ..scenario import (
    CTRL_FREQ_HZ,
    DELAY_GRID,
    apply_descent_command,
    classify_delay_results,
    unsafe,
    unsafe_segment,
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
    changed_traj = apply_descent_command(
        reference=reference,
        start_step=dispatch_step,
        low_steps=snapshot.low_steps,
        low_height=snapshot.low_height,
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

        previous_obs = obs
        obs, done, info = predictive_model.step(
            action,
            obs,
            info,
        )

        if unsafe_segment(previous_obs, obs, snapshot):
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
        snapshot,
    ):
        return False, checkpoints

    dispatch_steps = {round(float(delay) * CTRL_FREQ_HZ) for delay in delays}

    for local_step in range(snapshot.horizon_steps):
        if local_step in dispatch_steps:
            checkpoints[local_step] = predictive_model.save_checkpoint(
                ctrl,
                obs,
                info,
            )

        # Normal mission while command waits.
        action = ctrl.select_action(
            obs,
            info,
        )

        previous_obs = obs
        obs, done, info = predictive_model.step(
            action,
            obs,
            info,
        )

        if unsafe_segment(previous_obs, obs, snapshot):
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
            dispatch_step = round(float(delay) * CTRL_FREQ_HZ)
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


def predict_at_reception(snapshot, quad_params, pid_params):
    """Experiment B receives no actual dispatch delay or dispatch timestamp."""
    started = time.perf_counter()
    prediction = predict(
        snapshot, LightweightPredictiveModel(quad_params, pid_params), delays=DELAY_GRID
    )
    return prediction, time.perf_counter() - started


def ttl_expired_at_dispatch(prediction, waited_steps):
    """Apply the existing first-unsafe cutoff on the scheduler's control grid."""
    if prediction["classification"] in {"INVALID_AT_APPROVAL", "INVALID_NOMINAL"}:
        return True
    first_unsafe = prediction["first_bad_delay"]
    # Integer steps preserve equality at e.g. 0.6 versus np.arange's 0.600...1.
    return first_unsafe is not None and waited_steps >= round(float(first_unsafe) * CTRL_FREQ_HZ)
