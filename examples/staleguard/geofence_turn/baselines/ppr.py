"""Periodic Predictive Recovery: predict a delayed brake, never a queued turn.

Every 10 control ticks (0.20 s), predict nominal patrol for another 10 ticks
PLUS the next check's latency allowance, then the unchanged braking/hover
controller through the original remaining horizon. The initial PID memory is
restored by the existing lightweight model; it is not reset at the brake.

The frozen warmed-computation allowance is 20 ms. Strictly-after-finish 50 Hz
scheduling adds up to two ticks (40 ms), so recovery is predicted 12 ticks
(240 ms) ahead. Before evaluation, 25 warmed checks on development seeds 0--4
measured 4.04 ms mean / 6.12 ms maximum on the development host. The allowance
is over three times that observed maximum. It is a timing assumption, not a certified
worst-case bound. Actual check times and overruns must always be recorded;
an overrun never becomes an artificially early action.

Model integration is 100 Hz; safety observes its physical states at 50 Hz,
using the existing body-clearance predicate. Actual PyBullet safety stays 1 kHz.
"""
import math
from model.lightweight import LightweightPredictiveModel
from ..scenario import HZ, clearance, safety_only_reference

INTERVAL_STEPS = 10
COMPUTATION_BOUND_S = .020
LATENCY_BOUND_STEPS = math.floor(COMPUTATION_BOUND_S*HZ)+1


def predict_recovery(snapshot, platform):
    """Roll out patrol, then brake at the predicted physical position.

    Inputs contain only the current observation/PID/reference and original
    mission horizon. No future dispatch delay or PyBullet outcome is supplied.
    """
    model = LightweightPredictiveModel(*platform)
    try:
        ctrl, state, info, reference = model.start(snapshot)
        wait = INTERVAL_STEPS+LATENCY_BOUND_STEPS
        minimum = min(snapshot.prefix_min_clearance_m,
                      float(clearance(state[0], snapshot.fence_x)))
        predicted_steps = 0
        for step in range(snapshot.horizon_steps):
            if minimum < snapshot.config.d_min_m:
                break  # A safety violation cannot be undone by later braking.
            if step == wait:
                model.set_reference(ctrl, safety_only_reference(reference, step, state))
            previous = state
            state, _, info = model.step(ctrl.select_action(state, info), state, info)
            predicted_steps += 1
            current = float(clearance(state[0], snapshot.fence_x))
            if not math.isfinite(current):
                raise FloatingPointError('Nonfinite PPR predicted clearance')
            minimum = min(minimum,
                          float(clearance(previous[0], snapshot.fence_x)),
                          current)
        return dict(safe=bool(minimum >= snapshot.config.d_min_m),
                    min_clearance_m=minimum,
                    recovery_after_steps=wait if wait < snapshot.horizon_steps else None,
                    horizon_steps=snapshot.horizon_steps, predicted_steps=predicted_steps)
    finally:
        model.close({})
