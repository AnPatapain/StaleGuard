"""Small scheduler adapters, preserving the existing timing conventions."""
import math
from .mission import HZ

FIXED_TTL_S = .4  # Provisional smoke control, preregistered; not tuned.


def staleguard_status(received, waited_steps):
    ready = received['computation_s'] <= waited_steps/HZ
    expired = received['expiry_steps'] is not None and waited_steps >= received['expiry_steps']
    return ready, ready and expired


def fixed_ttl_expired(waited_steps):
    # The existing fixed TTL protocol permits equality.
    return waited_steps > round(FIXED_TTL_S*HZ)


def first_step_after_check(dispatch_step, computation_s):
    # Match existing use_time_recheck.first_step_after_check exactly.
    return dispatch_step + math.floor(computation_s*HZ) + 1
