# Underpass frozen benchmark protocol

Each recorded outcome is an actual PID/PyBullet flight. Safety is monitored at 1,000 Hz over the whole fixed horizon; earlier violations persist. All recovery is the same Eastward sideways maneuver.

StaleGuard uses the fixed 19-point grid and first-unsafe cutoff; equality rejects only when ready. Both variants share one warmed forecast/runtime per scenario. The early variant can recover while pending. Unavailable forecasts permit dispatch. Fixed EarlyRecovery uses 0.20 s; FBS uses alpha 0.1, epsilon 1e-12 and a 210-step cap, with strict age > TTL expiration. Use-Time Recheck observes at dispatch, continues nominal motion while computing and acts at dispatch + floor(runtime*50) + 1.

No future dispatch time is supplied to policies. Counterfactual command labels come from paired No Recovery flights when no external truth is configured. Command expiration is the first tested unsafe delay; all-safe complete grids remain censored. No independent recovery deadline is inferred. Full intervals expose non-monotonic command safety. Scenario-level rates use complete scenario grids only.

Each journal append is fsynced before its offset/hash is atomically committed in run.json. Resume checks that prefix and rolls back only uncommitted tail bytes. Metrics are regenerated one scenario at a time, without simulation. Source/environment, frozen protocol and cached forecast runtimes must match on resume.

Freeze SHA-256: 785ac2ed73046227b4b1ad41799dc342d97cff6a33d40c6bd000e0192b624799

Protocol and source provenance are recorded verbatim in run.json.

For the 2 s protocol the predictor candidate grid is still 19 points through 1 s. No predicted expiration continues to permit dispatch beyond 1 s; finite TTL and readiness gates are unchanged. The displacement profile ends 3.8 s after a dive starts, so D=2 leaves 2.2 s after that profile before the unchanged 8 s deadline. Observed completion and horizon truncation are reported separately. One journal is analyzed over the full grid, within <=1 s and beyond >1 s (sampled delays only). Confidence intervals resample entire paired scenario vectors, not individual flights.

## Paired scenario uncertainty by analysis window

Positive net unsafe-rate reduction favors EarlyRecovery. Intervals resample whole scenarios with all paired delay trials retained; no interval is estimated from fewer than two complete scenarios. These are analyses of one physical journal.

| Window | EarlyRecovery versus | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |
|---|---|---:|---:|---|
| full | staleguard | 232 | 0 | 0.1785 [0.1485–0.2077] |
| full | use_time_recheck | 255 | 12 | 0.1869 [0.1546–0.2169] |
| full | fixed_early_recovery | 7 | 128 | -0.0931 [-0.1215–-0.0669] |
| within_prediction_horizon | staleguard | 51 | 0 | 0.0510 [0.0370–0.0650] |
| within_prediction_horizon | use_time_recheck | 73 | 11 | 0.0620 [0.0450–0.0790] |
| within_prediction_horizon | fixed_early_recovery | 7 | 19 | -0.0120 [-0.0240–-0.0000] |
| beyond_prediction_horizon | staleguard | 181 | 0 | 0.6033 [0.5033–0.6934] |
| beyond_prediction_horizon | use_time_recheck | 182 | 1 | 0.6033 [0.5033–0.6933] |
| beyond_prediction_horizon | fixed_early_recovery | 0 | 109 | -0.3633 [-0.4600–-0.2767] |

Paired completion-rate intervals and all scenario-level values are retained in metrics.json. Inspection abandonment is separate from an inspection not observed by the fixed deadline.
