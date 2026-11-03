"""One dispatch-time rollout, inspired by verification before commitment.

Uses StaleGuard's model and swept checker, without its nominal/TTL sweep.
This is not a complete implementation of Gatekeeper or its safety guarantees.
"""
import math

from ..scenario import CTRL_FREQ_HZ, unsafe
from .staleguard import rollout_from_checkpoint


def predict_immediate(snapshot, predictive_model):
    checkpoints = {}
    try:
        ctrl, obs, info, reference = predictive_model.start(snapshot)
        if unsafe(obs, snapshot):
            return "UNSAFE"
        checkpoints[0] = predictive_model.save_checkpoint(ctrl, obs, info)
        return rollout_from_checkpoint(predictive_model, ctrl, checkpoints[0],
                                       reference, snapshot, dispatch_step=0)
    finally:
        predictive_model.close(checkpoints)


def first_step_after_check(dispatch_step, computation_s):
    """Strictly after completion, on the existing 50 Hz control grid."""
    return dispatch_step + math.floor(computation_s * CTRL_FREQ_HZ) + 1
