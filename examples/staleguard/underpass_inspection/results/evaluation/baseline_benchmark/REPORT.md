# Underpass seven-policy evaluation benchmark

Implemented seven policies on 100 frozen evaluation scenarios. The UAV flies North toward a bridge; its approved command dives underneath to inspect and swerves East around a pier. Every recovery instead abandons inspection and uses the same independent Eastward sideways maneuver. This journal contains 9100/9100 actual PyBullet executions. StaleGuard has 374 unsafe flights and its EarlyRecovery variant has 142. Prediction readiness, false-safe decisions and abandoned inspections are reported separately. The scenario is the independent experimental unit; simulation results do not establish real-world safety.

All policies use the same physical scenario, PID and eight-second horizon after approval. Safety is monitored at 1,000 Hz from reset, including before intervention and after a violation. Unsafe inspection dwell is never counted as safe completion. Dispatch delays are unknown to the policy until the event occurs. No offline deadline or other policy outcome is supplied to a policy.

**Complete paired matrix.**

| Policy | Flights | Unsafe | Safe inspection | Recovery | Early recovery | Ready false-safe decisions |
|---|---:|---:|---:|---:|---:|---:|
| no_recovery | 1300 | 539 | 761 | 0 | 0 | None |
| always_recover | 1300 | 0 | 0 | 1300 | 1200 | None |
| fbs | 1300 | 539 | 761 | 178 | 0 | 361 |
| use_time_recheck | 1300 | 385 | 742 | 540 | 0 | 4 |
| staleguard | 1300 | 374 | 759 | 460 | 0 | 70 |
| staleguard_early_recovery | 1300 | 142 | 759 | 460 | 428 | 70 |
| fixed_early_recovery | 1300 | 21 | 479 | 800 | 700 | None |

Early means recovery starts strictly before the scheduled dispatch, including Always Recover at approval for positive delays. At D=0 Always Recover still cancels immediately. Fixed EarlyRecovery uses 0.20 s with >= equality. Climb is never selected. A rejected command can still lead to an unsafe flight when sideways recovery starts too late.

| Policy versus No Recovery | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |
|---|---:|---:|---:|---:|
| always_recover | 539 | 0 | 0 | 761 |
| fbs | 0 | 0 | 761 | 0 |
| use_time_recheck | 169 | 15 | 742 | 19 |
| staleguard | 165 | 0 | 759 | 2 |
| staleguard_early_recovery | 397 | 0 | 759 | 2 |
| fixed_early_recovery | 518 | 0 | 479 | 282 |

**Reduced protocol:** 100 scenarios uniformly sampled without replacement from 865 accepted, using RNG seed 20261108. The full reduced grid has 13 delays: 0, 0.04, 0.08, 0.12, 0.16, 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.60, 2.00 s. Gaps range from 0.04 to 0.40 s. Observed expiration thresholds are first tested unsafe points, not exact deadlines. All ten selected delays through 1 s are prediction-grid points, so this reduced design does not test between-grid admissions within 1 s. Rates average the selected points equally; they do not assume a uniform continuous dispatch-delay distribution.

The physical grid covers 0–2.00 s; the predictor still tests only its original 19 delays through 1.00 s. These windows are subsets of one benchmark. With no predicted expiration, the unchanged StaleGuard gate permits dispatch even beyond 1 s; this is not a safety certificate for those delays. A ready finite TTL still rejects at/after expiry, and unavailable predictions still permit dispatch.

The fixed eight-second horizon is unchanged. Missing inspection dwell means incomplete by that deadline, not proof it could never complete later. Truncated maneuver profiles are recorded separately.

Analysis subset: **full**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 1300 | 539 | 761 | 0.415 [0.380–0.449] |
| always_recover | 1300 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 1300 | 539 | 761 | 0.415 [0.380–0.449] |
| use_time_recheck | 1300 | 385 | 742 | 0.296 [0.280–0.312] |
| staleguard | 1300 | 374 | 759 | 0.288 [0.272–0.305] |
| staleguard_early_recovery | 1300 | 142 | 759 | 0.109 [0.085–0.136] |
| fixed_early_recovery | 1300 | 21 | 479 | 0.016 [0.007–0.028] |

Analysis subset: **within_prediction_horizon**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 1000 | 247 | 753 | 0.247 [0.204–0.291] |
| always_recover | 1000 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 1000 | 247 | 753 | 0.247 [0.204–0.291] |
| use_time_recheck | 1000 | 95 | 735 | 0.095 [0.077–0.113] |
| staleguard | 1000 | 84 | 751 | 0.084 [0.065–0.105] |
| staleguard_early_recovery | 1000 | 33 | 751 | 0.033 [0.021–0.047] |
| fixed_early_recovery | 1000 | 21 | 479 | 0.021 [0.009–0.036] |

Analysis subset: **beyond_prediction_horizon**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 300 | 292 | 8 | 0.973 [0.953–0.990] |
| always_recover | 300 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 300 | 292 | 8 | 0.973 [0.953–0.990] |
| use_time_recheck | 300 | 290 | 7 | 0.967 [0.947–0.983] |
| staleguard | 300 | 290 | 8 | 0.967 [0.947–0.983] |
| staleguard_early_recovery | 300 | 109 | 8 | 0.363 [0.277–0.460] |
| fixed_early_recovery | 300 | 0 | 0 | 0.000 [0.000–0.000] |

95% percentile bootstrap intervals resample whole scenarios (2,000 resamples, fixed seed 2027). Complete scenario grids contribute; fewer than two complete scenarios gives no interval. Paired net safety/completion differences use the same scenario resamples and are in metrics.json; correlated delay trials are never treated as independent scenarios.

Prediction availability means the approval-time forecast has finished by the scheduled dispatch time (including equality). For EarlyRecovery, the denominator also includes commands already canceled by an earlier recovery.

| Policy | Available by scheduled dispatch | Availability rate |
|---|---:|---:|
| StaleGuard Dispatch | 898/1300 cases | 69.1% |
| StaleGuard EarlyRecovery | 898/1300 cases | 69.1% |

Both variants share the same forecasts and computation times, so their availability rates are identical for the same scenario-delay cases. For proactive recovery, a second measure is readiness by predicted command expiration: **69/75 scenario forecasts (92.0%)** finish by that time; 6/75 finish later. Scenarios with no predicted expiration within the prediction grid are excluded only from this second denominator (25 here). These rates use measured warmed computation times replayed on the simulation clock; they do not describe cold-start availability. Availability alone does not establish prediction correctness or safe recovery.

An unavailable forecast permits dispatch under the established policy rule. StaleGuard Dispatch: 402 cases without a forecast by dispatch, 11 unsafe dispatches; StaleGuard EarlyRecovery: 402 cases without a forecast by dispatch, 11 unsafe dispatches. These counts overlap across variants and must not be added together.

The StaleGuard variants share one measured approval forecast per scenario and the same runtime. The shared 100 Hz lightweight model preserves PID memory, predicts nominal waiting and then anchors the same dive to each predicted dispatch state. A swept 3D Euclidean box/body checker applies the unchanged clearance threshold. The fixed prediction grid follows Geofence (19 candidate delays); actual dispatch uses 13 delays. Expiration is the first predicted unsafe grid point; an all-safe grid has unobserved expiration.

Approval forecast runtime: 100 scenario samples, median 0.1395598025410436 s, p95 0.1443251259624958 s. Finite comparable TTL errors: 75; median absolute error 0.020000000000000004 s. Unobserved boundaries are excluded from scalar TTL errors, not from flight outcomes. First-forecast and subsequent timing, startup warmup and per-scenario forecasts are stored separately.

The TTL admission rule has 70 false-safe and 2 false-reject scenario/delay labels. Direct model labels at sampled prediction-grid points have 3 false-safe and 2 false-reject labels. The distinction exposes errors between coarse prediction-grid points as well as dynamic-model mismatch. 6 forecasts finish after their predicted expiry; unavailable dispatches remain separate from ready false-safe decisions.

Computation is measured offline, then its duration is replayed on the physical control clock, matching Geofence. Nominal flight continues until the result becomes actionable. An unavailable approval forecast permits dispatch and cannot cancel an already started command later. EarlyRecovery waits for both readiness and expiration; a late result acts at its actual ready control step while still pending, never retroactively. Use-Time Recheck performs one dispatch-state rollout and acts strictly after measured computation on the control grid; that movement during checking can change its eventual outcome.

Seeds with negative physical recovery gaps and unobserved command expiration remain in the cohort. Scenario is the independent experimental unit; aggregate flight counts are repeated-case descriptions. Per-scenario results, all cell outcomes and paired EarlyRecovery comparisons are in the CSVs and metrics.json. The [technical appendix](TECHNICAL_APPENDIX.md) documents policy rules, source equivalence, tests, timing limitations and reproduction.
