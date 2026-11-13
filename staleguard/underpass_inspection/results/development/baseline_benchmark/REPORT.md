# Underpass seven-policy development benchmark

Implemented and completed **4,284 actual PyBullet policy flights** on all 12 frozen development scenarios. The UAV flies North toward a bridge, dives underneath to inspect and swerves East around a pier. Every recovery instead abandons inspection and uses the same independent Eastward sideways maneuver. StaleGuard EarlyRecovery has **33 unsafe flights**, versus **90** for dispatch-time StaleGuard; both retain **318 safe inspections**. Fixed EarlyRecovery has 12 unsafe flights but retains only 108 safe inspections. Model/grid errors and late forecast availability remain visible; neither negative-gap scenario was removed. The implementation passes physical and scheduling validation. Review the warmed-runtime assumption and observed failure modes before freezing a separate held-out evaluation. No held-out results were generated.

Each policy receives the same initial physical state, PID history, fixed geometry and eight-second observation period after approval. Actual dispatch delay is unknown to it until dispatch occurs. Recovery is always sideways, never climb. Safety is monitored from reset at 1,000 Hz, including before intervention and after violations. A later inspection dwell cannot turn an unsafe flight into a successful mission.

| Policy | Unsafe | Safe inspections | Recoveries | Before dispatch | Ready false-safe decisions |
|---|---:|---:|---:|---:|---:|
| No Recovery | 294 | 318 | 0 | 0 | — |
| Always Recover | 0 | 0 | 612 | 600 | — |
| FBS | 294 | 318 | 0 | 0 | 294 |
| Use-Time Recheck | 87 | 307 | 294 | 0 | 2 |
| StaleGuard Dispatch | 90 | 318 | 268 | 0 | 17 |
| StaleGuard EarlyRecovery | 33 | 318 | 268 | 257 | 17 |
| Fixed EarlyRecovery | 12 | 108 | 492 | 480 | — |

Each row contains **612 flights: 12 scenarios × 51 shared delays**. “Before dispatch” means strictly earlier, including Always Recover at approval for positive delays. At D=0 it still cancels immediately. Fixed EarlyRecovery uses the previously declared 0.20 s timer; it was not fitted to these outcomes.

Ready false-safe decisions compare an accepted command against its original scheduled-delay No Recovery safety label. They exclude unavailable forecasts. Actual unsafe accepted outcomes, including movement during checking, are retained separately. Use-Time Recheck has 11 actual unsafe accepted flights; StaleGuard has 9 unsafe dispatches made before forecast readiness.

| EarlyRecovery versus | Unsafe prevented | Unsafe introduced | Additional safe inspections |
|---|---:|---:|---:|
| StaleGuard Dispatch | 57 | 0 | 0 |
| Use-Time Recheck | 73 | 19 | 11 |
| Fixed EarlyRecovery | 4 | 25 | 210 |

Relative to No Recovery, EarlyRecovery prevents 261 unsafe flights and introduces none, while preserving all 318 safe inspections. Use-Time Recheck prevents 216 unsafe flights but introduces nine and loses eleven safe inspections. The complete paired No Recovery comparisons are retained in the appendix and metrics.
EarlyRecovery prevents **57** unsafe flights relative to dispatch-time StaleGuard and introduces **0**. Their command-admission decisions and forecasts are identical; the change is physical recovery timing. The per-scenario table and explicit comparisons against Use-Time Recheck and Fixed EarlyRecovery are retained in metrics.json and per_scenario.csv. The scenario is the independent experimental unit; 4,284 cells are not 4,284 independent sampled scenarios.

For a measured example, seed 6 at D=0.92 s recovers at the predicted 0.40 s trigger under EarlyRecovery and remains safe; dispatch-time StaleGuard starts recovery at 0.92 s and violates clearance. Both abandon inspection. In negative-gap seeds 1 and 8, the recovery opportunity expires before command safety does; waiting for predicted command expiry does not rescue those cases. Seed 3, with unobserved command expiration, remains in every denominator.

Prediction availability means the approval-time forecast has finished by the scheduled dispatch time (including equality). For EarlyRecovery, the denominator also includes commands already canceled by an earlier recovery.

| Policy | Available by scheduled dispatch | Availability rate |
|---|---:|---:|
| StaleGuard Dispatch | 511/612 cases | 83.5% |
| StaleGuard EarlyRecovery | 511/612 cases | 83.5% |

Both variants share the same forecasts and computation times, so their availability rates are identical for the same scenario-delay cases. For proactive recovery, a second measure is readiness by predicted command expiration: **8/11 scenario forecasts (72.7%)** finish by that time; 3/11 finish later. Scenarios with no predicted expiration within the prediction grid are excluded only from this second denominator (1 here). These rates use measured warmed computation times replayed on the simulation clock; they do not describe cold-start availability. Availability alone does not establish prediction correctness or safe recovery.

An unavailable forecast permits dispatch under the established policy rule. StaleGuard Dispatch: 101 cases without a forecast by dispatch, 9 unsafe dispatches; StaleGuard EarlyRecovery: 101 cases without a forecast by dispatch, 9 unsafe dispatches. These counts overlap across variants and must not be added together.

The shared approval predictor uses the existing lightweight PID/RK4 model and a waiting-then-relative-dive rollout. Across **12 scenario forecasts**, warmed runtime p50 is **150.8 ms** and p95 **221.9 ms**. 3 forecasts finish after their predicted expiry. Use-Time Recheck p50/p95 are 7.9/10.4 ms per actual dispatch check. Startup warmup and first versus subsequent forecasts are recorded separately. These are warmed computation measurements replayed on the control clock, not cold-start or total flight runtimes.

Finite comparable command-expiration errors cover **11 scenarios**; median absolute error is **0.020 s**. Unobserved boundaries are not assigned artificial finite errors. Direct model checks on prediction-grid points have 1 false-safe and 1 false-reject labels. The TTL admission rule across all 51 delays has 17 false-safe and 1 false-reject labels. The inherited 19-point prediction grid is coarser than the execution grid after 0.20 s; sampled accuracy does not guarantee safe admission between points. Boundaries are discrete-grid observations.

All 27 focused tests pass, including two actual waiting-violation stress flights. The 189-flight physical smoke and its torn-tail resumption audit pass. The final audit verifies all 612 No Recovery trajectories against frozen dive truth, all 612 Always Recover trajectories against immediate sideways truth, actual approval snapshots, event ordering/cancellation, full horizons and representative 1,000 Hz clearance/dwell. All 110 pre-existing result artifacts remain unchanged.

These results support a recovery-timing benefit in this development cohort, with explicit safety/inspection trade-offs. They do not establish superiority on held-out scenarios or real-world safety. The policy semantics are now explicit and physically validated; the remaining protocol choices to review before held-out freezing are coarse-grid admission, model mismatch, warmed readiness and movement during rechecking. No policy was tuned to hide these failures. Details, complete file inventory, policy definitions and commands are in the [technical appendix](TECHNICAL_APPENDIX.md).
