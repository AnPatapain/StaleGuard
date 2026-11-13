# Underpass development deadline analysis

Implemented and ran a bounded actual-PyBullet study: 20 generated scenarios, 12 accepted and 8 rejected. The UAV flies North toward a bridge, dives to inspect underneath, then swerves East around a pier. Independent sideways recovery abandons inspection and goes beside the deck. 9/12 scenarios have positive tested sideways gaps, 2/12 negative, 0/12 zero and 1/12 undefined. Grid endpoints can leave expiration unobserved. This small simulated development cohort does not establish real-world prevalence or predictive-policy performance. Next, evaluate approval-time command predictions against the frozen physical outcomes, retaining unfavorable and unobserved cases.

## What was tested

Calibration uses only nominal flight and immediate maneuvers: nominal must become unsafe, dive must complete inspection safely, and initial sideways recovery must be safe. Initial climb is a separate diagnostic. Each seed gets one draw with no replacement or parameter search. Speed, approval time and deck front vary around seed 0; absolute pier North position is drawn independently. These are provisional design assumptions (ranges in [README](../../../README.md)), not sampled real-world frequencies.

After calibration, every accepted scenario is replayed independently for dive and sideways starts from 0–1.00 s at 0.02 s spacing. The climb diagnostic extends to 1.50 s on that grid. All times are after approval. Safety is measured from reset through approval plus the unchanged eight-second observation period at 1,000 Hz, including before intervention and after violations. Obstacles are visual-only in PyBullet; the unchanged 3D body-clearance predicate detects violations. Safe completion requires inspection dwell and safety throughout the flight. No model prediction or recovery selector was used.

## Observed results

Completed physical sweep flights: **2136/2136**. Counts below use all accepted scenarios; repeated delay trials are not additional independent scenarios.

| Measurement | Observed count / accepted scenarios |
|---|---:|
| Finite command expiration | 11 / 12 |
| Finite sideways expiration | 9 / 12 |
| Positive tested sideways gap | 9 / 12 |
| Zero tested sideways gap | 0 / 12 |
| Negative tested sideways gap | 2 / 12 |
| Undefined sideways gap | 1 / 12 |
| Finite climb expiration | 11 / 12 |
| Non-monotonic safety scenarios | 0 / 12 |
| Climb safe at first sideways failure | 9 / 12 |

The grid gap is the last tested safe sideways start minus the first tested unsafe dive delay. Only 7/12 positive gaps also have an observed sideways expiration; 3/12 sideways profiles never fail on the tested grid. For those, the grid difference records a demonstrated safe-start separation, not an observed expiry gap. 1/12 command profiles never fail through 1.00 s and have no finite measured command lifetime or scalar gap. The seven positive gaps with both expirations observed range from **0.04–0.70 s (median 0.44 s)**. Seeds 2 and 11 instead have positive demonstrated-start differences of 0.86 and 0.90 s, with sideways expiration unobserved. Seeds 1 and 8 have negative gaps of −0.16 and −0.18 s; seed 3 has no observed command expiration. No boundary is interpolated between samples.

| Gap distribution (s) | Cases | Minimum | Median | Mean | Maximum |
|---|---:|---:|---:|---:|---:|
| All defined sideways grid gaps | 11 | -0.18 | 0.44 | 0.36 | 0.90 |
| Positive sideways grid gaps | 9 | 0.04 | 0.52 | 0.48 | 0.90 |
| Sideways gaps with observed recovery expiration | 9 | -0.18 | 0.16 | 0.25 | 0.70 |
| Climb gaps with observed recovery expiration | 11 | 0.06 | 0.68 | 0.64 | 1.34 |

| First unsafe sample location | Dive scenarios | Sideways scenarios | Climb scenarios |
|---|---:|---:|---:|
| deck front | 0 | 7 | 0 |
| deck front side | 0 | 2 | 0 |
| deck front top | 0 | 0 | 11 |
| deck front underside | 2 | 0 | 0 |
| pier | 9 | 0 | 0 |

Locations describe geometry at the first violating sample; they are not a controlled causal isolation. Climb remains safe at the first unsafe sideways start in 9/12 scenarios; 3/12 have no observed sideways failure to compare. The dive is still safe when sideways first fails in 2/12 cases. Climb is always reported separately and never selected automatically.

Safe commands miss inspection in **0 flights across 0/12 scenarios**. All 318 safe dives complete inspection. Thus this cohort does not empirically demonstrate safe-but-incomplete inspection, even though the two criteria are recorded separately. Per-scenario last safe and last safe-completable delays are in [per_scenario.csv](per_scenario.csv); every trial is in [per_delay.csv](per_delay.csv).

## Interpretation and next decision

Positive gaps demonstrate that a command can become unsafe while a specified recovery remains physically available. Negative gaps or a safe command at sideways failure show the opposite ordering; neither is removed. These separately replayed deadlines can arise from different structure encounters, but this study does not establish statistical independence. A positive gap alone does not guarantee successful early intervention: prediction error, computation time and readiness have not been evaluated.

The frozen cohort and approval snapshots are suitable for a bounded approval-time prediction-accuracy study, provided it treats grid resolution, unobserved boundaries, whole-flight safety and inspection failure explicitly. Inspect these findings before implementing any policy benchmark. Geometry remains idealized, obstacles exert no contact forces, and only one simulator/controller family is tested. No StaleGuard or other predictive policy was implemented. Validation, rejected draws, source provenance and reproduction are documented in the [technical appendix](TECHNICAL_APPENDIX.md).

Non-monotonic tested safety intervals (inclusive tested endpoints; scalar gaps do not imply continuous windows):
None observed on the tested grids.
