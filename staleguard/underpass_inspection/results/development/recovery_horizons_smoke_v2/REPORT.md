# Underpass 2-policy development benchmark

Implemented 2 policies on 3 frozen development scenarios. The UAV flies North toward a bridge; its approved command dives underneath to inspect and swerves East around a pier. Every recovery instead abandons inspection and uses the same independent Eastward sideways maneuver. This journal contains 18/18 actual PyBullet executions. The two new policies are physically executed; original policy outcomes are not rerun. Prediction readiness, false-safe decisions and abandoned inspections are reported separately. The scenario is the independent experimental unit; simulation results do not establish real-world safety.

All policies use the same physical scenario, PID and eight-second horizon after approval. Safety is monitored at 1,000 Hz from reset, including before intervention and after a violation. Unsafe inspection dwell is never counted as safe completion. Dispatch delays are unknown to the policy until the event occurs. No offline deadline or other policy outcome is supplied to a policy.

**Complete paired matrix.**

| Policy | Flights | Unsafe | Safe inspection | Recovery | Early recovery | Ready false-safe decisions |
|---|---:|---:|---:|---:|---:|---:|
| staleguard_recovery_aware | 9 | 0 | 7 | 2 | 2 | 0 |
| staleguard_dual_horizon | 9 | 0 | 7 | 2 | 2 | 0 |

Early means recovery starts strictly before the scheduled dispatch, including Always Recover at approval for positive delays. At D=0 Always Recover still cancels immediately. Fixed EarlyRecovery uses 0.20 s with >= equality. Climb is never selected. A rejected command can still lead to an unsafe flight when sideways recovery starts too late.

| Policy versus No Recovery | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |
|---|---:|---:|---:|---:|

The physical grid covers 0–2.00 s; the predictor still tests only its original 19 delays through 1.00 s. These windows are subsets of one benchmark. With no predicted expiration, the unchanged StaleGuard gate permits dispatch even beyond 1 s; this is not a safety certificate for those delays. A ready finite TTL still rejects at/after expiry, and unavailable predictions still permit dispatch.

The fixed eight-second horizon is unchanged. Missing inspection dwell means incomplete by that deadline, not proof it could never complete later. Truncated maneuver profiles are recorded separately.

Analysis subset: **full**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| staleguard_recovery_aware | 9 | 0 | 7 | 0.000 [0.000–0.000] |
| staleguard_dual_horizon | 9 | 0 | 7 | 0.000 [0.000–0.000] |

Analysis subset: **within_prediction_horizon**

| Policy | Flights | Unsafe | Safe inspections | Unsafe rate, scenario mean [95% CI] |
|---|---:|---:|---:|---|
| staleguard_recovery_aware | 9 | 0 | 7 | 0.000 [0.000–0.000] |
| staleguard_dual_horizon | 9 | 0 | 7 | 0.000 [0.000–0.000] |

95% percentile bootstrap intervals resample whole scenarios (2,000 resamples, fixed seed 2027). Complete scenario grids contribute; fewer than two complete scenarios gives no interval. Paired net safety/completion differences use the same scenario resamples and are in metrics.json; correlated delay trials are never treated as independent scenarios.

**Exploratory extension:** these scenarios were examined in the earlier seven-policy study. New policies were developed/profiler-tested only on development scenarios, but this is not a fresh held-out superiority test. Existing flights were not rerun. New policy forecasts are independently computed/timed; all online waiting, both continuations, completion checks and boundary extraction count toward their atomic readiness.

Prediction availability means the approval-time forecast has finished by the scheduled dispatch time (including equality). For EarlyRecovery, the denominator also includes commands already canceled by an earlier recovery.

| Policy | Available by scheduled dispatch | Availability rate |
|---|---:|---:|
| StaleGuard RecoveryAware | 6/9 cases | 66.7% |
| StaleGuard DualHorizon | 6/9 cases | 66.7% |

The original StaleGuard pair shares forecasts/runtime. Each new policy has an independently timed run of the same three-profile predictor; no cross-policy computation cache advances readiness. For the root predictor summary (scope_policy in metrics.json), a second measure is readiness by predicted command expiration: **2/2 scenario forecasts (100.0%)** finish by that time; 0/2 finish later. Scenarios with no predicted expiration within the prediction grid are excluded only from this second denominator (1 here). These rates use measured warmed computation times replayed on the simulation clock; they do not describe cold-start availability. Availability alone does not establish prediction correctness or safe recovery.

An unavailable forecast permits dispatch under the established policy rule. StaleGuard RecoveryAware: 3 cases without a forecast by dispatch, 0 unsafe dispatches; StaleGuard DualHorizon: 3 cases without a forecast by dispatch, 0 unsafe dispatches. These counts overlap across variants and must not be added together.

The original StaleGuard pair shares one measured approval forecast per scenario and runtime. Each new policy independently computes/times the shared three-profile predictor. The shared 100 Hz lightweight model preserves PID memory, predicts nominal waiting and then anchors the same dive to each predicted dispatch state. A swept 3D Euclidean box/body checker applies the unchanged clearance threshold. The fixed prediction grid follows Geofence (19 candidate delays); actual dispatch uses 3 delays. Expiration is the first predicted unsafe grid point; an all-safe grid has unobserved expiration.

Approval forecast runtime: 3 scenario samples, median 0.14738325495272875 s, p95 0.1720479050418362 s. Finite comparable TTL errors: 2; median absolute error 0.35 s. Unobserved boundaries are excluded from scalar TTL errors, not from flight outcomes. First-forecast and subsequent timing, startup warmup and per-scenario forecasts are stored separately.

The TTL admission rule has 0 false-safe and 0 false-reject scenario/delay labels. Direct model labels at sampled prediction-grid points have 0 false-safe and 0 false-reject labels. The distinction exposes errors between coarse prediction-grid points as well as dynamic-model mismatch. 0 forecasts finish after their predicted expiry; unavailable dispatches remain separate from ready false-safe decisions.

Computation is measured offline, then its duration is replayed on the physical control clock, matching Geofence. Nominal flight continues until the result becomes actionable. An unavailable approval forecast permits dispatch and cannot cancel an already started command later. EarlyRecovery waits for both readiness and expiration; a late result acts at its actual ready control step while still pending, never retroactively. Use-Time Recheck performs one dispatch-state rollout and acts strictly after measured computation on the control grid; that movement during checking can change its eventual outcome.

Seeds with negative physical recovery gaps and unobserved command expiration remain in the cohort. Scenario is the independent experimental unit; aggregate flight counts are repeated-case descriptions. Per-scenario results, all cell outcomes and paired EarlyRecovery comparisons are in the CSVs and metrics.json. The [technical appendix](TECHNICAL_APPENDIX.md) documents policy rules, source equivalence, tests, timing limitations and reproduction.

New policy predictor runtime (warmed, independently computed per policy/scenario):

| Policy | Forecasts | Median ms | p95 ms |
|---|---:|---:|---:|
| staleguard_recovery_aware | 3 | 147.4 | 172.0 |
| staleguard_dual_horizon | 3 | 143.6 | 162.6 |

RecoveryAware proactively uses the earlier of command expiry and the last safe recovery-prefix grid point. DualHorizon also considers completion expiry. A late prediction does not retroactively act. A missed recovery prefix does not invalidate an otherwise safe command; known-expired commands use the existing recovery as best-effort. Completion-only abandonment requires verified recovery. All-safe grids are censored; no horizon-end rejection is invented.

Whole-flight failure decomposition and command/completion/recovery prediction labels are in metrics.json. Recovery-label comparisons include only actual starts that coincide with predicted grid points. The root predictor summary refers to the policy named by predictor.scope_policy.
