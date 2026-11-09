# Underpass frozen benchmark protocol

Each recorded outcome is an actual PID/PyBullet flight. Safety is monitored at 1,000 Hz over the whole fixed horizon; earlier violations persist. All recovery is the same Eastward sideways maneuver.

StaleGuard uses the fixed 19-point grid and first-unsafe cutoff; equality rejects only when ready. The original two variants share one warmed forecast/runtime per scenario. New variants independently time the same three-profile predictor; neither consumes the other policy forecast. The early variant can recover while pending. Unavailable forecasts permit dispatch. Fixed EarlyRecovery uses 0.20 s; FBS uses alpha 0.1, epsilon 1e-12 and a 210-step cap, with strict age > TTL expiration. Use-Time Recheck observes at dispatch, continues nominal motion while computing and acts at dispatch + floor(runtime*50) + 1.

No future dispatch time is supplied to policies. Counterfactual command labels come from paired No Recovery flights when no external truth is configured. Command expiration is the first tested unsafe delay; all-safe complete grids remain censored. No independent recovery deadline is inferred. Full intervals expose non-monotonic command safety. Scenario-level rates use complete scenario grids only.

Each flight/forecast append is fsynced before its offset/hash is atomically committed in progress.json. Resume checks those prefixes and rolls back only uncommitted tail bytes. Metrics are regenerated one scenario at a time, without simulation. Source/environment, frozen protocol and cached forecast runtimes must match on resume.

Frozen configuration SHA-256: fc1af39b9de82c883eb725863745da5cce69c44b844c009bc8209bbb62184e3e

Protocol and source/environment information are recorded in the immutable run.json. Actual approval state, PID memory and references are bundled in approval_snapshots.npz. Validation and metrics use only this folder; no original benchmark directories or calibration folder are required. Historical experiment provenance remains in original_experiments, with source/preparation evidence in source_versions.zip.

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

## Exploratory new-policy paired comparisons

| Window | Comparison | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |
|---|---|---:|---:|---|
| full | staleguard_recovery_aware__vs__staleguard_early_recovery | 94 | 1 | 0.0715 [0.0477–0.0985] |
| full | staleguard_recovery_aware__vs__fixed_early_recovery | 10 | 38 | -0.0215 [-0.0377–-0.0069] |
| full | staleguard_recovery_aware__vs__use_time_recheck | 346 | 10 | 0.2585 [0.2323–0.2823] |
| full | staleguard_dual_horizon__vs__staleguard_early_recovery | 95 | 1 | 0.0723 [0.0485–0.0985] |
| full | staleguard_dual_horizon__vs__fixed_early_recovery | 11 | 38 | -0.0208 [-0.0377–-0.0054] |
| full | staleguard_dual_horizon__vs__use_time_recheck | 346 | 9 | 0.2592 [0.2331–0.2831] |
| full | staleguard_dual_horizon__vs__staleguard_recovery_aware | 1 | 0 | 0.0008 [0.0000–0.0023] |
| within_prediction_horizon | staleguard_recovery_aware__vs__staleguard_early_recovery | 22 | 1 | 0.0210 [0.0110–0.0330] |
| within_prediction_horizon | staleguard_recovery_aware__vs__fixed_early_recovery | 10 | 1 | 0.0090 [0.0020–0.0170] |
| within_prediction_horizon | staleguard_recovery_aware__vs__use_time_recheck | 92 | 9 | 0.0830 [0.0650–0.1000] |
| within_prediction_horizon | staleguard_dual_horizon__vs__staleguard_early_recovery | 23 | 1 | 0.0220 [0.0120–0.0330] |
| within_prediction_horizon | staleguard_dual_horizon__vs__fixed_early_recovery | 11 | 1 | 0.0100 [0.0020–0.0190] |
| within_prediction_horizon | staleguard_dual_horizon__vs__use_time_recheck | 92 | 8 | 0.0840 [0.0660–0.1010] |
| within_prediction_horizon | staleguard_dual_horizon__vs__staleguard_recovery_aware | 1 | 0 | 0.0010 [0.0000–0.0030] |
| beyond_prediction_horizon | staleguard_recovery_aware__vs__staleguard_early_recovery | 72 | 0 | 0.2400 [0.1600–0.3300] |
| beyond_prediction_horizon | staleguard_recovery_aware__vs__fixed_early_recovery | 0 | 37 | -0.1233 [-0.1867–-0.0699] |
| beyond_prediction_horizon | staleguard_recovery_aware__vs__use_time_recheck | 254 | 1 | 0.8433 [0.7667–0.9067] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__staleguard_early_recovery | 72 | 0 | 0.2400 [0.1600–0.3300] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__fixed_early_recovery | 0 | 37 | -0.1233 [-0.1867–-0.0699] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__use_time_recheck | 254 | 1 | 0.8433 [0.7667–0.9067] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__staleguard_recovery_aware | 0 | 0 | 0.0000 [0.0000–0.0000] |

Paired completion-rate intervals and all scenario-level values are retained in metrics.json. Inspection abandonment is separate from an inspection not observed by the fixed deadline.
