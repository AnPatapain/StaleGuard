# Underpass 9-policy evaluation benchmark

Implemented 9 policies on 400 frozen evaluation scenarios. The UAV flies North toward a bridge; its approved command dives underneath to inspect and swerves East around a pier. Every recovery instead abandons inspection and uses the same independent Eastward sideways maneuver. This journal contains 46800/46800 actual PyBullet executions. StaleGuard has 1515 unsafe flights and its EarlyRecovery variant has 563. Prediction readiness, false-safe decisions and abandoned inspections are reported separately. The scenario is the independent experimental unit; simulation results do not establish real-world safety.

All policies use the same physical scenario, PID and eight-second horizon after approval. Safety is monitored at 1,000 Hz from reset, including before intervention and after a violation. Unsafe inspection dwell is never counted as safe completion. Dispatch delays are unknown to the policy until the event occurs. No offline deadline or other policy outcome is supplied to a policy.

**Complete paired matrix.**

| Policy | Flights | Unsafe | Safe inspection | Recovery | Early recovery | Ready false-safe decisions |
|---|---:|---:|---:|---:|---:|---:|
| no_recovery | 5200 | 2230 | 2970 | 0 | 0 | None |
| always_recover | 5200 | 0 | 0 | 5200 | 4800 | None |
| fbs | 5200 | 2230 | 2970 | 687 | 0 | 1543 |
| use_time_recheck | 5200 | 1528 | 2897 | 2237 | 0 | 9 |
| staleguard | 5200 | 1515 | 2958 | 1957 | 0 | 217 |
| staleguard_early_recovery | 5200 | 563 | 2958 | 1957 | 1797 | 217 |
| fixed_early_recovery | 5200 | 117 | 1883 | 3200 | 2800 | None |
| staleguard_recovery_aware | 5200 | 182 | 2900 | 2131 | 2012 | 126 |
| staleguard_dual_horizon | 5200 | 182 | 2900 | 2131 | 2012 | 126 |

Early means recovery starts strictly before the scheduled dispatch, including Always Recover at approval for positive delays. At D=0 Always Recover still cancels immediately. Fixed EarlyRecovery uses 0.20 s with >= equality. Climb is never selected. A rejected command can still lead to an unsafe flight when sideways recovery starts too late.

| Policy versus No Recovery | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |
|---|---:|---:|---:|---:|
| always_recover | 2230 | 0 | 0 | 2970 |
| fbs | 0 | 0 | 2970 | 0 |
| use_time_recheck | 763 | 61 | 2897 | 73 |
| staleguard | 719 | 4 | 2958 | 12 |
| staleguard_early_recovery | 1671 | 4 | 2958 | 12 |
| fixed_early_recovery | 2113 | 0 | 1883 | 1087 |
| staleguard_recovery_aware | 2050 | 2 | 2900 | 70 |
| staleguard_dual_horizon | 2050 | 2 | 2900 | 70 |

**Fresh evaluation:** 400 scenarios sampled without replacement from 765 eligible accepted scenarios, with RNG seed 20261109. The 100 previously evaluated scenarios are excluded. Selection used accepted IDs only, without policy outcomes.

**Temporal resolution:** 13 selected dispatch delays; gaps range from 0.04 to 0.40 s. Tested expiration boundaries are coarse observations. Rates weight the selected delays equally, not a uniform continuous delay distribution. The 19-point prediction grid is unchanged.

The physical grid covers 0–2.00 s; the predictor still tests only its original 19 delays through 1.00 s. These windows are subsets of one benchmark. With no predicted expiration, the unchanged StaleGuard gate permits dispatch even beyond 1 s; this is not a safety certificate for those delays. A ready finite TTL still rejects at/after expiry, and unavailable predictions still permit dispatch.

The fixed eight-second horizon is unchanged. Missing inspection dwell means incomplete by that deadline, not proof it could never complete later. Truncated maneuver profiles are recorded separately.

Analysis subset: **full**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 5200 | 2230 | 2970 | 0.429 [0.411–0.448] |
| always_recover | 5200 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 5200 | 2230 | 2970 | 0.429 [0.411–0.448] |
| use_time_recheck | 5200 | 1528 | 2897 | 0.294 [0.286–0.302] |
| staleguard | 5200 | 1515 | 2958 | 0.291 [0.282–0.300] |
| staleguard_early_recovery | 5200 | 563 | 2958 | 0.108 [0.096–0.122] |
| fixed_early_recovery | 5200 | 117 | 1883 | 0.022 [0.016–0.029] |
| staleguard_recovery_aware | 5200 | 182 | 2900 | 0.035 [0.027–0.043] |
| staleguard_dual_horizon | 5200 | 182 | 2900 | 0.035 [0.027–0.043] |

Analysis subset: **within_prediction_horizon**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 4000 | 1058 | 2942 | 0.265 [0.242–0.288] |
| always_recover | 4000 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 4000 | 1058 | 2942 | 0.265 [0.242–0.288] |
| use_time_recheck | 4000 | 372 | 2875 | 0.093 [0.085–0.102] |
| staleguard | 4000 | 365 | 2930 | 0.091 [0.081–0.102] |
| staleguard_early_recovery | 4000 | 156 | 2930 | 0.039 [0.032–0.046] |
| fixed_early_recovery | 4000 | 117 | 1883 | 0.029 [0.021–0.038] |
| staleguard_recovery_aware | 4000 | 54 | 2872 | 0.014 [0.009–0.018] |
| staleguard_dual_horizon | 4000 | 54 | 2872 | 0.014 [0.009–0.018] |

Analysis subset: **beyond_prediction_horizon**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| no_recovery | 1200 | 1172 | 28 | 0.977 [0.968–0.984] |
| always_recover | 1200 | 0 | 0 | 0.000 [0.000–0.000] |
| fbs | 1200 | 1172 | 28 | 0.977 [0.968–0.984] |
| use_time_recheck | 1200 | 1156 | 22 | 0.963 [0.953–0.973] |
| staleguard | 1200 | 1150 | 28 | 0.958 [0.947–0.969] |
| staleguard_early_recovery | 1200 | 407 | 28 | 0.339 [0.293–0.386] |
| fixed_early_recovery | 1200 | 0 | 0 | 0.000 [0.000–0.000] |
| staleguard_recovery_aware | 1200 | 128 | 28 | 0.107 [0.078–0.137] |
| staleguard_dual_horizon | 1200 | 128 | 28 | 0.107 [0.078–0.137] |

95% percentile bootstrap intervals resample whole scenarios (2,000 resamples, fixed seed 2027). Complete scenario grids contribute; fewer than two complete scenarios gives no interval. Paired net safety/completion differences use the same scenario resamples and are in metrics.json; correlated delay trials are never treated as independent scenarios.

Prediction availability means the approval-time forecast has finished by the scheduled dispatch time (including equality). For EarlyRecovery, the denominator also includes commands already canceled by an earlier recovery.

| Policy | Available by scheduled dispatch | Availability rate |
|---|---:|---:|
| StaleGuard Dispatch | 3600/5200 cases | 69.2% |
| StaleGuard EarlyRecovery | 3600/5200 cases | 69.2% |
| StaleGuard RecoveryAware | 3537/5200 cases | 68.0% |
| StaleGuard DualHorizon | 3547/5200 cases | 68.2% |

The original StaleGuard pair shares forecasts/runtime. Each new policy has an independently timed run of the same three-profile predictor; no cross-policy computation cache advances readiness. For the root predictor summary (scope_policy in metrics.json), a second measure is readiness by predicted command expiration: **283/321 scenario forecasts (88.2%)** finish by that time; 38/321 finish later. Scenarios with no predicted expiration within the prediction grid are excluded only from this second denominator (79 here). These rates use measured warmed computation times replayed on the simulation clock; they do not describe cold-start availability. Availability alone does not establish prediction correctness or safe recovery.

An unavailable forecast permits dispatch under the established policy rule. StaleGuard Dispatch: 1600 cases without a forecast by dispatch, 68 unsafe dispatches; StaleGuard EarlyRecovery: 1600 cases without a forecast by dispatch, 68 unsafe dispatches; StaleGuard RecoveryAware: 1663 cases without a forecast by dispatch, 43 unsafe dispatches; StaleGuard DualHorizon: 1653 cases without a forecast by dispatch, 43 unsafe dispatches. These counts overlap across variants and must not be added together.

The original StaleGuard pair shares one measured approval forecast per scenario and runtime. Each new policy independently computes/times the shared three-profile predictor. The shared 100 Hz lightweight model preserves PID memory, predicts nominal waiting and then anchors the same dive to each predicted dispatch state. A swept 3D Euclidean box/body checker applies the unchanged clearance threshold. The fixed prediction grid follows Geofence (19 candidate delays); actual dispatch uses 13 delays. Expiration is the first predicted unsafe grid point; an all-safe grid has unobserved expiration.

Approval forecast runtime: 400 scenario samples, median 0.14236335147870705 s, p95 0.14671246602665633 s. Finite comparable TTL errors: 321; median absolute error 0.01999999999999999 s. Unobserved boundaries are excluded from scalar TTL errors, not from flight outcomes. First-forecast and subsequent timing, startup warmup and per-scenario forecasts are stored separately.

The TTL admission rule has 219 false-safe and 14 false-reject scenario/delay labels. Direct model labels at sampled prediction-grid points have 11 false-safe and 14 false-reject labels. The distinction exposes errors between coarse prediction-grid points as well as dynamic-model mismatch. 38 forecasts finish after their predicted expiry; unavailable dispatches remain separate from ready false-safe decisions.

Computation is measured offline, then its duration is replayed on the physical control clock, matching Geofence. Nominal flight continues until the result becomes actionable. An unavailable approval forecast permits dispatch and cannot cancel an already started command later. EarlyRecovery waits for both readiness and expiration; a late result acts at its actual ready control step while still pending, never retroactively. Use-Time Recheck performs one dispatch-state rollout and acts strictly after measured computation on the control grid; that movement during checking can change its eventual outcome.

Seeds with negative physical recovery gaps and unobserved command expiration remain in the cohort. Scenario is the independent experimental unit; aggregate flight counts are repeated-case descriptions. Per-scenario results, all cell outcomes and paired EarlyRecovery comparisons are in the CSVs and metrics.json. The [technical appendix](TECHNICAL_APPENDIX.md) documents policy rules, source equivalence, tests, timing limitations and reproduction.

New policy predictor runtime (warmed, independently computed per policy/scenario):

| Policy | Forecasts | Median ms | p95 ms |
|---|---:|---:|---:|
| staleguard_recovery_aware | 400 | 149.1 | 167.0 |
| staleguard_dual_horizon | 400 | 149.0 | 167.4 |

RecoveryAware proactively uses the earlier of command expiry and the last safe recovery-prefix grid point. DualHorizon also considers completion expiry. A late prediction does not retroactively act. A missed recovery prefix does not invalidate an otherwise safe command; known-expired commands use the existing recovery as best-effort. Completion-only abandonment requires verified recovery. All-safe grids are censored; no horizon-end rejection is invented.

Whole-flight failure decomposition and command/completion/recovery prediction labels are in metrics.json. Recovery-label comparisons include only actual starts that coincide with predicted grid points. The root predictor summary refers to the policy named by predictor.scope_policy.
