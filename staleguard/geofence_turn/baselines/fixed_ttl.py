"""The existing fixed 0.4 s lifetime; equality still permits dispatch."""
from ..scenario import HZ

FIXED_TTL_S = .4  # Provisional smoke control, preregistered; not tuned.


def fixed_ttl_expired(waited_steps):
    # The existing fixed TTL protocol permits equality.
    return waited_steps > round(FIXED_TTL_S*HZ)
