# Table I — Prediction accuracy and computational readiness

Status: COMPLETE. Evaluation cohorts only.

| Mission / property | Scenarios | Physical pairs | Accuracy (%) | FF n (%) | FR n (%) | Boundary MAE / median (ms) | Boundary pairs |
|---|---:|---:|---:|---:|---:|---:|---:|
| Inspection Descent / command | 276 | 3036 | 98.06 | 22 (1.20) | 37 (3.10) | 21.38 / 0.00 | 276 |
| Inspection Descent / completion | 276 | 3036 | 99.01 | 12 (0.57) | 18 (1.94) | 10.87 / 0.00 | 276 |
| Inspection Descent / recovery | 276 | 3036 | 100.00 | 0 (N/A) | 0 (0.00) | unobserved / unobserved | 0 |
| Geofence Turn / command | 300 | 5700 | 99.02 | 27 (1.43) | 29 (0.76) | 8.83 / 0.00 | 247 |
| Geofence Turn / completion | 300 | 5700 | 97.09 | 87 (2.17) | 79 (4.67) | 12.47 / 0.00 | 300 |
| Geofence Turn / recovery | 300 | 5700 | 99.67 | 5 (1.32) | 14 (0.26) | 14.63 / 0.00 | 123 |
| Underpass / command | 400 | 7600 | 99.29 | 25 (1.28) | 29 (0.51) | 11.45 / 0.00 | 318 |
| Underpass / completion | 400 | 7600 | 99.29 | 25 (1.28) | 29 (0.51) | 11.45 / 0.00 | 318 |
| Underpass / recovery | 400 | 7600 | 99.61 | 12 (2.18) | 18 (0.26) | 7.17 / 0.00 | 237 |

| Full predictor | Timed forecasts | Median / p95 (ms) | Ready by dispatch | Ready by last safe recovery sample | Inside unresolved bracket | Late / observed recovery losses | Recovery loss unobserved |
|---|---:|---:|---:|---:|---:|---:|---:|
| Inspection Descent | 276 | 41.24 / 55.28 | 2760/3036 | 0 | 0 | N/A | 276 |
| Geofence Turn | 300 | 96.38 / 141.10 | 5273/6900 | 124 | 0 | 0/124 | 176 |
| Underpass | 400 | 149.00 / 167.44 | 3547/5200 | 243 | 0 | 0/243 | 157 |

FF divides by physically infeasible labels; FR by physically feasible labels. N/A means the conditional denominator is zero. Boundary errors are sampled-grid errors, not continuous-time guarantees. Initially infeasible, censored, and non-monotonic cases are retained in CSV/JSON. Right-censored cases have no artificial finite error. Runtime is warmed offline computation replayed on the simulator clock, not an onboard timing validation. No development scenarios enter these tables.
