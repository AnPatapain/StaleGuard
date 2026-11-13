# Underpass seven-policy development benchmark

Implemented seven policies on 3 frozen development scenarios. The UAV flies North toward a bridge; its approved command dives underneath to inspect and swerves East around a pier. Every recovery instead abandons inspection and uses the same independent Eastward sideways maneuver. This journal contains 189/189 actual PyBullet executions. StaleGuard has 12 unsafe flights and its EarlyRecovery variant has 7. Prediction readiness, false-safe decisions and abandoned inspections are reported separately. This 3-scenario development cohort does not establish held-out or real-world performance; inspect the timing/model failures before freezing a later evaluation.

All policies use the same physical scenario, PID and eight-second horizon after approval. Safety is monitored at 1,000 Hz from reset, including before intervention and after a violation. Unsafe inspection dwell is never counted as safe completion. Dispatch delays are unknown to the policy until the event occurs. No offline deadline or other policy outcome is supplied to a policy.

**Complete paired matrix.**

| Policy | Flights | Unsafe | Safe inspection | Recovery | Early recovery | Ready false-safe decisions |
|---|---:|---:|---:|---:|---:|---:|
| no_recovery | 27 | 16 | 11 | 0 | 0 | None |
| always_recover | 27 | 0 | 0 | 27 | 24 | None |
| fbs | 27 | 16 | 11 | 0 | 0 | 16 |
| use_time_recheck | 27 | 11 | 10 | 17 | 0 | 0 |
| staleguard | 27 | 12 | 11 | 12 | 0 | 4 |
| staleguard_early_recovery | 27 | 7 | 11 | 12 | 10 | 4 |
| fixed_early_recovery | 27 | 0 | 3 | 24 | 21 | None |

Early means recovery starts strictly before the scheduled dispatch, including Always Recover at approval for positive delays. At D=0 Always Recover still cancels immediately. Fixed EarlyRecovery uses 0.20 s with >= equality. Climb is never selected. A rejected command can still lead to an unsafe flight when sideways recovery starts too late.

| Policy versus No Recovery | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |
|---|---:|---:|---:|---:|
| always_recover | 16 | 0 | 0 | 11 |
| fbs | 0 | 0 | 11 | 0 |
| use_time_recheck | 5 | 0 | 10 | 1 |
| staleguard | 4 | 0 | 11 | 0 |
| staleguard_early_recovery | 9 | 0 | 11 | 0 |
| fixed_early_recovery | 16 | 0 | 3 | 8 |

The StaleGuard variants share one measured approval forecast per scenario and the same runtime. The shared 100 Hz lightweight model preserves PID memory, predicts nominal waiting and then anchors the same dive to each predicted dispatch state. A swept 3D Euclidean box/body checker applies the unchanged clearance threshold. The fixed prediction grid follows Geofence (19 candidate delays); actual dispatch uses 51 delays. Expiration is the first predicted unsafe grid point; an all-safe grid has unobserved expiration.

Approval forecast runtime: 3 scenario samples, median 0.14668072800850496 s, p95 0.15267783997114748 s. Finite comparable TTL errors: 3; median absolute error 0.040000000000000036 s. Unobserved boundaries are excluded from scalar TTL errors, not from flight outcomes. First-forecast and subsequent timing, startup warmup and per-scenario forecasts are stored separately.

The TTL admission rule has 4 false-safe and 0 false-reject scenario/delay labels. Direct model labels at sampled prediction-grid points have 0 false-safe and 0 false-reject labels. The distinction exposes errors between coarse prediction-grid points as well as dynamic-model mismatch. 0 forecasts finish after their predicted expiry; unavailable dispatches remain separate from ready false-safe decisions.

Computation is measured offline, then its duration is replayed on the physical control clock, matching Geofence. Nominal flight continues until the result becomes actionable. An unavailable approval forecast permits dispatch and cannot cancel an already started command later. EarlyRecovery waits for both readiness and expiration; a late result acts at its actual ready control step while still pending, never retroactively. Use-Time Recheck performs one dispatch-state rollout and acts strictly after measured computation on the control grid; that movement during checking can change its eventual outcome.

Seeds with negative physical recovery gaps and unobserved command expiration remain in the cohort. Scenario is the independent experimental unit; aggregate flight counts are repeated-case descriptions. Per-scenario results, all cell outcomes and paired EarlyRecovery comparisons are in the CSVs and metrics.json. The [technical appendix](TECHNICAL_APPENDIX.md) documents policy rules, source equivalence, tests, timing limitations and reproduction. No held-out scenarios were generated or evaluated.
