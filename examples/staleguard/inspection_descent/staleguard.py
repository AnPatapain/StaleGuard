"""StaleGuard T0.
Predicts validity using approval-time information only.
Every rollout starts directly from approval time.
Safety covers straight position segments between consecutive control states.
"""
from experiment import (
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
