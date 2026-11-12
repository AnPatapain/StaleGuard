# Table II — Safety and safe mission completion

Status: COMPLETE. Each mission uses its own frozen scenario/delay grid.

| Policy | Inspection unsafe / safe completion (%) | Geofence unsafe / safe completion (%) | Underpass unsafe / safe completion (%) |
|---|---:|---:|---:|
| No Recovery | 60.64 / 30.63 | 36.94 / 24.52 | 42.88 / 57.12 |
| Always Recover (approval) | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 |
| FBS | 60.64 / 30.63 | 36.94 / 24.52 | 42.88 / 57.12 |
| Fixed TTL (legacy) | 4.58 / 26.98 | 17.96 / 24.52 | Not evaluated |
| Use-Time Recheck | 2.08 / 28.46 | 9.72 / 20.19 | 29.38 / 55.71 |
| StaleGuard (dispatch) | 0.72 / 30.20 | 8.70 / 24.43 | 29.13 / 56.88 |
| Command-only EarlyRecovery | 0.72 / 30.20 | 1.42 / 24.41 | 10.83 / 56.88 |
| Fixed EarlyRecovery (0.20 s) | 0.33 / 17.06 | 4.57 / 24.52 | 2.25 / 36.21 |
| RecoveryAware (ablation) | 0.72 / 30.20 | 1.32 / 24.38 | 3.50 / 55.77 |
| StaleGuard (full) | 0.26 / 30.07 | 0.64 / 24.00 | 3.50 / 55.77 |

Always Recover cancels at approval for every D, including D=0. Corrected Inspection/Geofence cells reuse physically equivalent existing approval recoveries; represented delay cases are not claimed as new physical flights. Source reuse and new-flight counts are separate columns in the CSV. Legacy Fixed TTL is retained where already evaluated, not introduced for Underpass. Rates do not pool missions or assume a uniform continuous dispatch-delay distribution.

| Mission: full versus | Unsafe prevented / introduced | Completion preserved / sacrificed / gained | Unsafe-rate reduction pp [95% CI] | Completion change pp [95% CI] | Different actions / scenarios |
|---|---:|---:|---:|---:|---:|
| Inspection Descent: Command-only EarlyRecovery | 14 / 0 | 913 / 4 / 0 | 0.46 [0.23, 0.69] | -0.13 [-0.26, -0.03] | 997 / 124 |
| Inspection Descent: RecoveryAware (ablation) | 14 / 0 | 913 / 4 / 0 | 0.46 [0.23, 0.69] | -0.13 [-0.26, -0.03] | 997 / 124 |
| Inspection Descent: Fixed EarlyRecovery (0.20 s) | 9 / 7 | 516 / 2 / 397 | 0.07 [-0.20, 0.33] | 13.01 [11.50, 14.59] | 1924 / 210 |
| Inspection Descent: Use-Time Recheck | 60 / 5 | 859 / 5 / 54 | 1.81 [1.35, 2.31] | 1.61 [1.15, 2.08] | 3036 / 276 |
| Geofence Turn: Command-only EarlyRecovery | 59 / 5 | 1656 / 28 / 0 | 0.78 [0.54, 1.01] | -0.41 [-0.57, -0.26] | 4146 / 256 |
| Geofence Turn: RecoveryAware (ablation) | 50 / 3 | 1656 / 26 / 0 | 0.68 [0.48, 0.90] | -0.38 [-0.52, -0.25] | 3899 / 243 |
| Geofence Turn: Fixed EarlyRecovery (0.20 s) | 274 / 3 | 1656 / 36 / 0 | 3.93 [3.06, 4.84] | -0.52 [-0.70, -0.36] | 4599 / 282 |
| Geofence Turn: Use-Time Recheck | 664 / 37 | 1382 / 11 / 274 | 9.09 [7.77, 10.48] | 3.81 [3.64, 3.97] | 6900 / 300 |
| Underpass: Command-only EarlyRecovery | 383 / 2 | 2900 / 58 / 0 | 7.33 [6.12, 8.63] | -1.12 [-1.38, -0.87] | 780 / 133 |
| Underpass: RecoveryAware (ablation) | 0 / 0 | 2900 / 0 / 0 | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0 / 0 |
| Underpass: Fixed EarlyRecovery (0.20 s) | 73 / 138 | 1882 / 1 / 1018 | -1.25 [-2.08, -0.40] | 19.56 [18.15, 20.89] | 3218 / 393 |
| Underpass: Use-Time Recheck | 1395 / 49 | 2845 / 52 / 55 | 25.88 [24.63, 27.13] | 0.06 [-0.33, 0.44] | 5200 / 400 |

Intervals resample whole paired scenario vectors (2000 resamples, seed 2027). Full/within-prediction-horizon/beyond-horizon comparisons are in metrics.json. RecoveryAware/full forecasts are computed independently; actual decision differences may involve readiness as well as the completion condition. Raw trigger/profile and event data remain archived for attribution.
