# Underpass development calibration

The UAV flies North toward a bridge, dives underneath to inspect, then swerves East around a pier. One frozen draw was attempted for each seed 1–20: 12 passed and 8 were rejected. All eight rejections violate pier clearance during the immediate approved command. No rejected seed was replaced or adjusted. Accepted cases safely complete immediate inspection and immediate sideways recovery while nominal flight is unsafe. Initial climb was safe in all 20 attempts but did not affect acceptance. These are actual PyBullet observations, not model predictions. The fixed ranges do not guarantee immediate feasibility. Next, independently sweep start times for the accepted cases without changing their geometry or selecting by recovery-window size.

| Screening observation | Accepted 12 | Rejected 8 |
|---|---:|---:|
| Nominal unsafe | 12 | 8 |
| Immediate dive safe inspection | 12 | 0 |
| Immediate sideways safe | 12 | 8 |
| Immediate climb safe (diagnostic) | 12 | 8 |

Accepted seeds: [1, 2, 3, 6, 8, 9, 11, 12, 14, 16, 17, 20].

| Rejected seed | Reason | Immediate minimum body clearance (m) |
|---|---|---:|
| 4 | d0_unsafe:pier | -0.102347 |
| 5 | d0_unsafe:pier | 0.036676 |
| 7 | d0_unsafe:pier | 0.037529 |
| 10 | d0_unsafe:pier | -0.020721 |
| 13 | d0_unsafe:pier | -0.017503 |
| 15 | d0_unsafe:pier | -0.099557 |
| 18 | d0_unsafe:pier | 0.019073 |
| 19 | d0_unsafe:pier | -0.031873 |

## Frozen design assumptions

| Parameter | Independent uniform draw | Units |
|---|---|---|
| Nominal North speed | 0.55–0.65 | m/s |
| Approval after flight start | 0.70–0.90, rounded to 0.02 | s |
| Deck North front | 1.35–1.65 | m |
| Absolute pier North center | 2.35–3.25 | m |

Draw order is speed, approval, front, pier using NumPy default_rng(seed). The first three ranges are small neighborhoods around seed 0; the pier range occupies a common interior segment behind every inspection box and fits every possible deck. Pier North is independent of deck North front. The deck remains 2.5 m long; the inspection box translates with its front. No maneuver parameters were sampled or searched. All remaining parameters, thresholds, NED signs and physics/controller settings stay at their validated values.

Calibration executed 92 actual flights: four immediate-mode checks per attempt and a second D=0 dive for each accepted case. Every accepted replay matched physical trajectory, approval state, PID memory and remaining nominal reference exactly. No positive-delay execution entered calibration. Full attempted definitions, immediate records and reproducibility metadata are in [calibrated_scenarios.json](calibrated_scenarios.json); 12 verified snapshots are stored in snapshots/. Read the [deadline report](../deadline_sweep/REPORT.md) for the separate delayed-flight experiment.
