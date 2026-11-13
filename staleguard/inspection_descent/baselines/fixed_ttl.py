"""Historical development-fixed lifetime: 0.30 s; equality permits dispatch."""
FIXED_TTL_S = .30
FIXED_TTL_STEPS = 15

def fixed_ttl_expired_at_dispatch(ttl_steps, waited_steps):
    """A constant approval lifetime; equality still permits the command."""
    return waited_steps > ttl_steps
