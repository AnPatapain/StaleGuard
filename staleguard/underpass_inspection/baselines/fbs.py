"""Underpass FBS feature, EMA and freshness gate (alpha 0.1)."""

import math
from ..scenario import HZ, POS, body_clearances


def safety_feature(state, scenario):
    # Same sign/units as Geofence: positive means a clearance violation.
    return scenario.clearance_m - min(body_clearances(state[POS], scenario.obstacles).values())


ALPHA = 0.1
EPSILON = 1e-12  # Numerical floor, metres per control sample.
MAX_TTL_STEPS = 210  # 4.2 seconds at 50 Hz.


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
