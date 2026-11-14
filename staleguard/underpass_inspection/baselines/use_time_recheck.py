"""One immediate-command rollout at dispatch, without an approval TTL sweep."""

import math
from ..scenario import HZ
from .staleguard import predict_delays


def predict_immediate(snapshot, platform):
    return predict_delays(snapshot, platform, (0.0,))['results'][0]


def first_step_after_check(dispatch_step, computation_s):
    # Match existing use_time_recheck.first_step_after_check exactly.
    return dispatch_step + math.floor(computation_s * HZ) + 1
