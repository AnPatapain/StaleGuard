"""Fixed 0.20-second early-recovery baseline; no predictive model."""
from ..scenario import HZ

FIXED_EARLY_TTL_S = 0.20
FIXED_EARLY_TTL_STEPS = round(FIXED_EARLY_TTL_S * HZ)


def pending_recovery_due(waited_steps):
    return waited_steps >= FIXED_EARLY_TTL_STEPS


def dispatch_recovery_due(waited_steps):
    return waited_steps >= FIXED_EARLY_TTL_STEPS