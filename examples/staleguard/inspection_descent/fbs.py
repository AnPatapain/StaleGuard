"""Model-free FBS freshness gate, adapted from Shraga et al., Section III-D.

Volatility is metres/control sample; the margin/volatility ratio is in steps.
Epsilon is a numerical floor, not a sensor-noise or physical safety bound.
The fixed protocol and sensitivity values are in results/fbs/epsilon_protocol.json.
"""
import math

from model.common import STATE_X, STATE_Y, STATE_Z
from experiment import CTRL_FREQ_HZ

ALPHA = 0.1
EPSILON = 1e-12  # metres per 50 Hz sample; frozen before policy evaluation.
MAX_TTL_STEPS = 210  # 4.2 seconds at 50 Hz.


def clearance(state, geometry):
    return float(max(geometry.box_x_min - state[STATE_X],
                     state[STATE_X] - geometry.box_x_max,
                     abs(state[STATE_Y]) - geometry.box_y_limit,
                     state[STATE_Z] - geometry.unsafe_below_z))


def update_volatility(feature, previous_feature, volatility):
    if previous_feature is None:
        return 0.0
    return ALPHA * abs(feature - previous_feature) + (1 - ALPHA) * volatility


def freeze_prediction(margin, volatility, epsilon=EPSILON):
    if epsilon <= 0:
        raise ValueError("FBS epsilon must be positive")
    delta = min(MAX_TTL_STEPS, max(margin, 0) / max(volatility, epsilon))
    return dict(margin_m=margin, volatility_m_per_sample=volatility,
                epsilon_m_per_sample=epsilon, delta_steps=delta,
                ttl_steps=math.floor(delta))


def dispatch_status(prediction, computation_s, waited_steps):
    # Match Experiment B's fail-open availability rule. Equality is ready.
    ready = computation_s <= waited_steps / CTRL_FREQ_HZ
    expired = prediction["margin_m"] <= 0 or waited_steps > prediction["ttl_steps"]
    return dict(ready=ready, expired=expired, recover=ready and expired,
                reason="unavailable" if not ready else "expired" if expired else "valid")
