# Underpass seven-policy development benchmark

Implemented seven policies on 2 frozen development scenarios. The UAV flies North toward a bridge; its approved command dives underneath to inspect and swerves East around a pier. Every recovery instead abandons inspection and uses the same independent Eastward sideways maneuver. This journal contains 42/42 actual PyBullet executions. StaleGuard has 3 unsafe flights and its EarlyRecovery variant has 3. Prediction readiness, false-safe decisions and abandoned inspections are reported separately. The scenario is the independent experimental unit; simulation results do not establish real-world safety.

All policies use the same physical scenario, PID and eight-second horizon after approval. Safety is monitored at 1,000 Hz from reset, including before intervention and after a violation. Unsafe inspection dwell is never counted as safe completion. Dispatch delays are unknown to the policy until the event occurs. No offline deadline or other policy outcome is supplied to a policy.

**Complete paired matrix.**

| Policy | Flights | Unsafe | Safe inspection | Recovery | Early recovery | Ready false-safe decisions |
|---|---:|---:|---:|---:|---:|---:|
| no_recovery | 6 | 3 | 3 | 0 | 0 | None |
| always_recover | 6 | 0 | 0 | 6 | 4 | None |
| fbs | 6 | 3 | 3 | 2 | 0 | 1 |
| use_time_recheck | 6 | 3 | 3 | 3 | 0 | 0 |
| staleguard | 6 | 3 | 3 | 2 | 0 | 1 |
| staleguard_early_recovery | 6 | 3 | 3 | 2 | 2 | 1 |
| fixed_early_recovery | 6 | 0 | 2 | 4 | 4 | None |

Early means recovery starts strictly before the scheduled dispatch, including Always Recover at approval for positive delays. At D=0 Always Recover still cancels immediately. Fixed EarlyRecovery uses 0.20 s with >= equality. Climb is never selected. A rejected command can still lead to an unsafe flight when sideways recovery starts too late.

| Policy versus No Recovery | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |
|---|---:|---:|---:|---:|
| always_recover | 3 | 0 | 0 | 3 |
| fbs | 0 | 0 | 3 | 0 |
| use_time_recheck | 0 | 0 | 3 | 0 |
| staleguard | 0 | 0 | 3 | 0 |
| staleguard_early_recovery | 0 | 0 | 3 | 0 |
| fixed_early_recovery | 3 | 0 | 2 | 1 |

The physical grid covers 0–2.00 s; the predictor still tests only its original 19 delays through 1.00 s. These windows are subsets of one benchmark. With no predicted expiration, the unchanged StaleGuard gate permits dispatch even beyond 1 s; this is not a safety certificate for those delays. A ready finite TTL still rejects at/after expiry, and unavailable predictions still permit dispatch.

The fixed eight-second horizon is unchanged. Missing inspection dwell means incomplete by that deadline, not proof it could never complete later. Truncated maneuver profiles are recorded separately.

Analysis subset: **full**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 6 | 3 | 3 | 0.500 [0.333–0.667] |
| always_recover | 6 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 6 | 3 | 3 | 0.500 [0.333–0.667] |
| use_time_recheck | 6 | 3 | 3 | 0.500 [0.333–0.667] |
| staleguard | 6 | 3 | 3 | 0.500 [0.333–0.667] |
| staleguard_early_recovery | 6 | 3 | 3 | 0.500 [0.333–0.667] |
| fixed_early_recovery | 6 | 0 | 2 | 0.000 [0.000–0.000] |

Analysis subset: **within_prediction_horizon**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 4 | 1 | 3 | 0.250 [0.000–0.500] |
| always_recover | 4 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 4 | 1 | 3 | 0.250 [0.000–0.500] |
| use_time_recheck | 4 | 1 | 3 | 0.250 [0.000–0.500] |
| staleguard | 4 | 1 | 3 | 0.250 [0.000–0.500] |
| staleguard_early_recovery | 4 | 1 | 3 | 0.250 [0.000–0.500] |
| fixed_early_recovery | 4 | 0 | 2 | 0.000 [0.000–0.000] |

Analysis subset: **beyond_prediction_horizon**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 2 | 2 | 0 | 1.000 [1.000–1.000] |
| always_recover | 2 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 2 | 2 | 0 | 1.000 [1.000–1.000] |
| use_time_recheck | 2 | 2 | 0 | 1.000 [1.000–1.000] |
| staleguard | 2 | 2 | 0 | 1.000 [1.000–1.000] |
| staleguard_early_recovery | 2 | 2 | 0 | 1.000 [1.000–1.000] |
| fixed_early_recovery | 2 | 0 | 0 | 0.000 [0.000–0.000] |

95% percentile bootstrap intervals resample whole scenarios (2,000 resamples, fixed seed 2027). Complete scenario grids contribute; fewer than two complete scenarios gives no interval. Paired net safety/completion differences use the same scenario resamples and are in metrics.json; correlated delay trials are never treated as independent scenarios.

Prediction availability means the approval-time forecast has finished by the scheduled dispatch time (including equality). For EarlyRecovery, the denominator also includes commands already canceled by an earlier recovery.

| Policy | Available by scheduled dispatch | Availability rate |
|---|---:|---:|
| StaleGuard Dispatch | 4/6 cases | 66.7% |
| StaleGuard EarlyRecovery | 4/6 cases | 66.7% |

Both variants share the same forecasts and computation times, so their availability rates are identical for the same scenario-delay cases. For proactive recovery, a second measure is readiness by predicted command expiration: **1/1 scenario forecasts (100.0%)** finish by that time; 0/1 finish later. Scenarios with no predicted expiration within the prediction grid are excluded only from this second denominator (1 here). These rates use measured warmed computation times replayed on the simulation clock; they do not describe cold-start availability. Availability alone does not establish prediction correctness or safe recovery.

An unavailable forecast permits dispatch under the established policy rule. StaleGuard Dispatch: 2 cases without a forecast by dispatch, 0 unsafe dispatches; StaleGuard EarlyRecovery: 2 cases without a forecast by dispatch, 0 unsafe dispatches. These counts overlap across variants and must not be added together.

The StaleGuard variants share one measured approval forecast per scenario and the same runtime. The shared 100 Hz lightweight model preserves PID memory, predicts nominal waiting and then anchors the same dive to each predicted dispatch state. A swept 3D Euclidean box/body checker applies the unchanged clearance threshold. The fixed prediction grid follows Geofence (19 candidate delays); actual dispatch uses 3 delays. Expiration is the first predicted unsafe grid point; an all-safe grid has unobserved expiration.

Approval forecast runtime: 2 scenario samples, median 0.23733017698395997 s, p95 0.26863945997320116 s. Finite comparable TTL errors: 1; median absolute error 0.09999999999999998 s. Unobserved boundaries are excluded from scalar TTL errors, not from flight outcomes. First-forecast and subsequent timing, startup warmup and per-scenario forecasts are stored separately.

The TTL admission rule has 1 false-safe and 0 false-reject scenario/delay labels. Direct model labels at sampled prediction-grid points have 0 false-safe and 0 false-reject labels. The distinction exposes errors between coarse prediction-grid points as well as dynamic-model mismatch. 0 forecasts finish after their predicted expiry; unavailable dispatches remain separate from ready false-safe decisions.

Computation is measured offline, then its duration is replayed on the physical control clock, matching Geofence. Nominal flight continues until the result becomes actionable. An unavailable approval forecast permits dispatch and cannot cancel an already started command later. EarlyRecovery waits for both readiness and expiration; a late result acts at its actual ready control step while still pending, never retroactively. Use-Time Recheck performs one dispatch-state rollout and acts strictly after measured computation on the control grid; that movement during checking can change its eventual outcome.

Seeds with negative physical recovery gaps and unobserved command expiration remain in the cohort. Scenario is the independent experimental unit; aggregate flight counts are repeated-case descriptions. Per-scenario results, all cell outcomes and paired EarlyRecovery comparisons are in the CSVs and metrics.json. The [technical appendix](TECHNICAL_APPENDIX.md) documents policy rules, source equivalence, tests, timing limitations and reproduction.
