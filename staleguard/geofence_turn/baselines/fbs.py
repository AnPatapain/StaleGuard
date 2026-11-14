"""Model-free FBS freshness gate for Geofence Turn.

At each 50 Hz observation through approval (inclusive), update the EMA of
absolute feature changes, starting from zero. At approval, freeze the TTL:
safe-side margin / volatility, rounded down and capped at 210 control steps.
At dispatch, recover only if this frozen prediction is ready and expired.
An unavailable prediction permits the original turn without later cancellation.
The simulator supplies observations, measures computation time and applies
the decision at scheduled dispatch; this module does not simulate future motion.
"""

import math

from ..scenario import HZ, clearance

ALPHA = 0.1
EPSILON = 1e-12  # Numerical floor, in metres per control sample.
MAX_TTL_STEPS = 210  # 4.2 seconds at 50 Hz.


def safety_feature(state, scenario, config):
    return float(config.d_min_m - clearance(state[0], scenario.fence_x))


def update_volatility(feature, previous_feature, volatility):
    if previous_feature is None:
        return 0.0
    return ALPHA * abs(feature - previous_feature) + (1 - ALPHA) * volatility


def freeze_prediction(feature, volatility, epsilon=EPSILON):
    margin = -feature  # f = d_min - clearance; m = -f is positive when safe.
    if epsilon <= 0:
        raise ValueError("FBS epsilon must be positive")
    delta = min(MAX_TTL_STEPS, max(margin, 0) / max(volatility, epsilon))
    return dict(
        margin_m=margin,
        volatility_m_per_sample=volatility,
        epsilon_m_per_sample=epsilon,
        delta_steps=delta,
        ttl_steps=math.floor(delta),
    )


def dispatch_status(prediction, computation_s, waited_steps):
    # Equality is ready; a positive-margin TTL expires only strictly after its end.
    ready = computation_s <= waited_steps / HZ
    expired = prediction["margin_m"] <= 0 or waited_steps > prediction["ttl_steps"]
    return dict(
        ready=ready,
        expired=expired,
        recover=ready and expired,
        reason="unavailable" if not ready else "expired" if expired else "valid",
    )
