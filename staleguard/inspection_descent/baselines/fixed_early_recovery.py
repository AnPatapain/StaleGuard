"""Use the original development-fixed 0.30 s lifetime, with StaleGuard's >= expiry.

The threshold is retained from Inspection-Descent's development Fixed-TTL
protocol, not selected from seeds 20--319. Only this extension's enforcement
time/boundary changes: the original Fixed TTL still expires strictly after .30 s.
"""
FIXED_EARLY_TTL_S=.30
FIXED_EARLY_TTL_STEPS=15


def expired(waited_steps):
    return waited_steps>=FIXED_EARLY_TTL_STEPS
