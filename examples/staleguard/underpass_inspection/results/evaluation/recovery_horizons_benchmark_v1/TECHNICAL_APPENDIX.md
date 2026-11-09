# Underpass frozen benchmark protocol

Each recorded outcome is an actual PID/PyBullet flight. Safety is monitored at 1,000 Hz over the whole fixed horizon; earlier violations persist. All recovery is the same Eastward sideways maneuver.

StaleGuard uses the fixed 19-point grid and first-unsafe cutoff; equality rejects only when ready. The original two variants share one warmed forecast/runtime per scenario. New variants independently time the same three-profile predictor; neither consumes the other policy forecast. The early variant can recover while pending. Unavailable forecasts permit dispatch. Fixed EarlyRecovery uses 0.20 s; FBS uses alpha 0.1, epsilon 1e-12 and a 210-step cap, with strict age > TTL expiration. Use-Time Recheck observes at dispatch, continues nominal motion while computing and acts at dispatch + floor(runtime*50) + 1.

No future dispatch time is supplied to policies. Counterfactual command labels come from paired No Recovery flights when no external truth is configured. Command expiration is the first tested unsafe delay; all-safe complete grids remain censored. No independent recovery deadline is inferred. Full intervals expose non-monotonic command safety. Scenario-level rates use complete scenario grids only.

Each journal append is fsynced before its offset/hash is atomically committed in run.json. Resume checks that prefix and rolls back only uncommitted tail bytes. Metrics are regenerated one scenario at a time, without simulation. Source/environment, frozen protocol and cached forecast runtimes must match on resume.

Freeze SHA-256: 92366f9b7fa28b2d0bd3c95ab42d0b456062492318b6812662d0e4d710affa94

Protocol and source provenance are recorded verbatim in run.json.

For the 2 s protocol the predictor candidate grid is still 19 points through 1 s. No predicted expiration continues to permit dispatch beyond 1 s; finite TTL and readiness gates are unchanged. The displacement profile ends 3.8 s after a dive starts, so D=2 leaves 2.2 s after that profile before the unchanged 8 s deadline. Observed completion and horizon truncation are reported separately. One journal is analyzed over the full grid, within <=1 s and beyond >1 s (sampled delays only). Confidence intervals resample entire paired scenario vectors, not individual flights.

## Paired scenario uncertainty by analysis window

Positive net unsafe-rate reduction favors EarlyRecovery. Intervals resample whole scenarios with all paired delay trials retained; no interval is estimated from fewer than two complete scenarios. These are analyses of one physical journal.

| Window | EarlyRecovery versus | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |
|---|---|---:|---:|---|

## Exploratory new-policy paired comparisons

| Window | Comparison | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |
|---|---|---:|---:|---|
| full | staleguard_dual_horizon__vs__staleguard_recovery_aware | 1 | 0 | 0.0008 [0.0000–0.0023] |
| within_prediction_horizon | staleguard_dual_horizon__vs__staleguard_recovery_aware | 1 | 0 | 0.0010 [0.0000–0.0030] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__staleguard_recovery_aware | 0 | 0 | 0.0000 [0.0000–0.0000] |

Paired completion-rate intervals and all scenario-level values are retained in metrics.json. Inspection abandonment is separate from an inspection not observed by the fixed deadline.
