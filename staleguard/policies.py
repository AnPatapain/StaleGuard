"""Identifiers and display names of the eight published policies."""

POLICIES = (
    'no_recovery',
    'fbs',
    'use_time_recheck',
    'fixed_early_recovery',
    'staleguard',
    'staleguard_early_recovery',
    'staleguard_recovery_aware',
    'staleguard_dual_horizon',
)

LABELS = dict(
    zip(
        POLICIES,
        (
            'No Recovery',
            'FBS',
            'Use-Time Recheck',
            'Fixed EarlyRecovery (0.20 s)',
            'StaleGuard (Dispatch)',
            'StaleGuard (Command-safety-aware)',
            'StaleGuard (Recovery-aware)',
            'StaleGuard (Three-opportunity-aware)',
        ),
    )
)
