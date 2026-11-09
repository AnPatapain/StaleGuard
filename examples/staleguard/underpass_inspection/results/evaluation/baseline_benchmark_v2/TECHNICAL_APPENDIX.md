# Underpass frozen benchmark protocol

Each recorded outcome is an actual PID/PyBullet flight. Safety is monitored at 1,000 Hz over the whole fixed horizon; earlier violations persist. All recovery is the same Eastward sideways maneuver.

StaleGuard uses the fixed 19-point grid and first-unsafe cutoff; equality rejects only when ready. The original two variants share one warmed forecast/runtime per scenario. New variants independently time the same three-profile predictor; neither consumes the other policy forecast. The early variant can recover while pending. Unavailable forecasts permit dispatch. Fixed EarlyRecovery uses 0.20 s; FBS uses alpha 0.1, epsilon 1e-12 and a 210-step cap, with strict age > TTL expiration. Use-Time Recheck observes at dispatch, continues nominal motion while computing and acts at dispatch + floor(runtime*50) + 1.

No future dispatch time is supplied to policies. Counterfactual command labels come from paired No Recovery flights when no external truth is configured. Command expiration is the first tested unsafe delay; all-safe complete grids remain censored. No independent recovery deadline is inferred. Full intervals expose non-monotonic command safety. Scenario-level rates use complete scenario grids only.

Each flight/forecast append is fsynced before its offset/hash is atomically committed in progress.json. Resume checks those prefixes and rolls back only uncommitted tail bytes. Metrics are regenerated one scenario at a time, without simulation. Source/environment, frozen protocol and cached forecast runtimes must match on resume.

Frozen configuration SHA-256: 22f6808ff24f3e668022fcc1043594c2630354af846c20b949d65535c9a8281e

Protocol and source/environment information are recorded in the immutable run.json. Actual approval state, PID memory and references are bundled in approval_snapshots.npz. Validation and metrics use only this folder; no original benchmark directories or calibration folder are required. Historical experiment provenance remains in original_experiments, with source/preparation evidence in source_versions.zip.

For the 2 s protocol the predictor candidate grid is still 19 points through 1 s. No predicted expiration continues to permit dispatch beyond 1 s; finite TTL and readiness gates are unchanged. The displacement profile ends 3.8 s after a dive starts, so D=2 leaves 2.2 s after that profile before the unchanged 8 s deadline. Observed completion and horizon truncation are reported separately. One journal is analyzed over the full grid, within <=1 s and beyond >1 s (sampled delays only). Confidence intervals resample entire paired scenario vectors, not individual flights.

## Paired scenario uncertainty by analysis window

Positive net unsafe-rate reduction favors EarlyRecovery. Intervals resample whole scenarios with all paired delay trials retained; no interval is estimated from fewer than two complete scenarios. These are analyses of one physical journal.

| Window | EarlyRecovery versus | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |
|---|---|---:|---:|---|
| full | staleguard | 952 | 0 | 0.1831 [0.1683–0.1979] |
| full | use_time_recheck | 1036 | 71 | 0.1856 [0.1700–0.2008] |
| full | fixed_early_recovery | 48 | 494 | -0.0858 [-0.1004–-0.0723] |
| within_prediction_horizon | staleguard | 209 | 0 | 0.0522 [0.0450–0.0597] |
| within_prediction_horizon | use_time_recheck | 285 | 69 | 0.0540 [0.0440–0.0635] |
| within_prediction_horizon | fixed_early_recovery | 48 | 87 | -0.0098 [-0.0162–-0.0038] |
| beyond_prediction_horizon | staleguard | 743 | 0 | 0.6192 [0.5708–0.6667] |
| beyond_prediction_horizon | use_time_recheck | 751 | 2 | 0.6242 [0.5767–0.6725] |
| beyond_prediction_horizon | fixed_early_recovery | 0 | 407 | -0.3392 [-0.3858–-0.2933] |

## Exploratory new-policy paired comparisons

| Window | Comparison | Unsafe prevented | Unsafe introduced | Net unsafe-rate reduction [95% CI] |
|---|---|---:|---:|---|
| full | staleguard_recovery_aware__vs__staleguard_early_recovery | 383 | 2 | 0.0733 [0.0612–0.0863] |
| full | staleguard_recovery_aware__vs__fixed_early_recovery | 73 | 138 | -0.0125 [-0.0208–-0.0040] |
| full | staleguard_recovery_aware__vs__use_time_recheck | 1395 | 49 | 0.2588 [0.2463–0.2713] |
| full | staleguard_dual_horizon__vs__staleguard_early_recovery | 383 | 2 | 0.0733 [0.0612–0.0863] |
| full | staleguard_dual_horizon__vs__fixed_early_recovery | 73 | 138 | -0.0125 [-0.0208–-0.0040] |
| full | staleguard_dual_horizon__vs__use_time_recheck | 1395 | 49 | 0.2588 [0.2463–0.2713] |
| full | staleguard_dual_horizon__vs__staleguard_recovery_aware | 0 | 0 | 0.0000 [0.0000–0.0000] |
| within_prediction_horizon | staleguard_recovery_aware__vs__staleguard_early_recovery | 104 | 2 | 0.0255 [0.0207–0.0310] |
| within_prediction_horizon | staleguard_recovery_aware__vs__fixed_early_recovery | 73 | 10 | 0.0158 [0.0105–0.0213] |
| within_prediction_horizon | staleguard_recovery_aware__vs__use_time_recheck | 365 | 47 | 0.0795 [0.0700–0.0888] |
| within_prediction_horizon | staleguard_dual_horizon__vs__staleguard_early_recovery | 104 | 2 | 0.0255 [0.0207–0.0310] |
| within_prediction_horizon | staleguard_dual_horizon__vs__fixed_early_recovery | 73 | 10 | 0.0158 [0.0105–0.0213] |
| within_prediction_horizon | staleguard_dual_horizon__vs__use_time_recheck | 365 | 47 | 0.0795 [0.0700–0.0888] |
| within_prediction_horizon | staleguard_dual_horizon__vs__staleguard_recovery_aware | 0 | 0 | 0.0000 [0.0000–0.0000] |
| beyond_prediction_horizon | staleguard_recovery_aware__vs__staleguard_early_recovery | 279 | 0 | 0.2325 [0.1900–0.2775] |
| beyond_prediction_horizon | staleguard_recovery_aware__vs__fixed_early_recovery | 0 | 128 | -0.1067 [-0.1367–-0.0783] |
| beyond_prediction_horizon | staleguard_recovery_aware__vs__use_time_recheck | 1030 | 2 | 0.8567 [0.8233–0.8892] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__staleguard_early_recovery | 279 | 0 | 0.2325 [0.1900–0.2775] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__fixed_early_recovery | 0 | 128 | -0.1067 [-0.1367–-0.0783] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__use_time_recheck | 1030 | 2 | 0.8567 [0.8233–0.8892] |
| beyond_prediction_horizon | staleguard_dual_horizon__vs__staleguard_recovery_aware | 0 | 0 | 0.0000 [0.0000–0.0000] |

Paired completion-rate intervals and all scenario-level values are retained in metrics.json. Inspection abandonment is separate from an inspection not observed by the fixed deadline.
