# Development study — technical appendix

The [main report](REPORT.md) explains the flight experiment and measured findings.
This appendix records implementation, validation and reproduction. All deadline
measurements are actual controlled SCG/PyBullet flights, never lightweight-model
predictions. No predictive policy or benchmark was implemented.

## Code inventory

All paths below are within `underpass_inspection/`.

| File | Change |
|---|---|
| `calibration.py` (new) | One draw per seed; four immediate feasibility checks; repeated D=0 verification; frozen accepted/rejected definitions and approval snapshots; interruption/resumption. |
| `support/deadline_sweep.py` (new) | Independent physical sweeps, committed journal, intervals, aggregate/per-scenario metrics, CSVs and report; metrics-only mode. |
| `support/test_development.py` (new) | Deterministic sampling, immediate-only calibration and resume, torn-tail/duplicate checks, negative/undefined/non-monotonic analysis examples. |
| `scenario.py` | Added bounded generator for seeds 1–20. Seed-0 parameters, displacement functions, geometry predicates and completion criteria are unchanged. |
| `simulation.py` | Optional read-only approval-state/PID/reference capture, seed in records, optional supplied GUI overlay paths. Physics, controller calls, safety sampling and maneuvers are unchanged. |
| `pilot.py` | Load an accepted calibrated seed for single-flight replay; pass same-scenario actual overlays to the existing viewer. |
| `support/visualization.py` | Calibrated comparison overlays replay the same frozen scenario physically. Existing camera, colors, floor, NED projection and drawing routines are preserved. |
| `README.md` | Development protocol, ranges, artifact semantics, calibrated GUI and reproduction. |

Geofence-Turn supplies existing atomic JSON, journal append and file-hash helpers.
Its mission, calibration logic, recovery algorithms and results are not modified.
There is no new simulation framework or scenario replacement/search mechanism.

## Frozen inputs and selection

The cohort consists of exactly one attempt for each seed 1–20. Seed 0 remains the
reference pilot and is excluded from all development denominators. Independent
NumPy `default_rng(seed)` draws occur in this order:

| Parameter | Distribution | Why this range was used |
|---|---|---|
| Nominal North speed | Uniform [0.55, 0.65) m/s | Small neighborhood around validated 0.60 m/s. |
| Approval after reset | Uniform [0.70, 0.90) s, rounded to 0.02 s | Small neighborhood around validated 0.80 s; already beyond the 0.40 s startup ramp. |
| Deck North front | Uniform [1.35, 1.65) m | ±0.15 m around the reference; changes nominal approach distance. |
| Absolute pier North center | Independent uniform [2.35, 3.25) m | Common interior segment behind all inspection regions, with the full pier fitting even the shortest deck. |

The deck remains 2.50 m long. Its back and inspection-box North coordinates shift
by the front displacement. Inspection-box size, altitude, dwell, deck width and
height, pier dimensions, maneuver shape/timing and every clearance threshold are
unchanged. Pier North is drawn independently, not at a fixed distance behind the
front. These provisional design ranges were frozen before any positive-delay
development flight. They do not represent a measured real-world distribution.

Accept iff nominal is unsafe within the horizon, D=0 dive is safe and completes
inspection, and immediate sideways recovery is safe. Initial climb is measured
but cannot affect acceptance. There is no approval-time grid search or adaptive
parameter choice. Calibration's only maneuver start is 0.00 s after approval.

The [calibration report](../calibration/REPORT.md) lists all rejected seeds.
Calibration attempted 20 scenarios, accepted 12, and rejected 8 for immediate
pier-clearance violations. The accepted seeds are 1, 2, 3, 6, 8, 9, 11, 12, 14,
16, 17 and 20. Its 92 physical flights comprise four checks per attempt plus an
independent D=0 repeat per accepted seed. Repeated flights verify identical actual
positions, approval-state/PID memory and remaining nominal reference. No delayed
flight record is an input to calibration or acceptance.

## Sweeps and deadline interpretation

For each accepted frozen scenario, start from the same original reset and fly the
nominal reference until the chosen intervention. Dive and sideways each use 51
starts from 0 to 1.00 s at 0.02 s spacing. Climb independently uses 76 starts from
0 to 1.50 s. This optional diagnostic extension was frozen before the study; it
does not extend the eight-second observation period after approval. The complete
matrix has 12 × (51 + 51 + 76) = **2,136 actual flights**.

The body sphere radius remains 0.06128825336065631 m and required body clearance
0.05 m. Both 3D boxes remain fixed during every flight. Obstacles have no PyBullet
contact forces; safety is the existing signed Euclidean box-distance predicate.
Safety is sampled at 1,000 Hz from reset through approval plus eight seconds,
including before intervention and after a violation. The PID remains at 50 Hz.
The first violation is irreversible in a record. Inspection dwell can occur in
an unsafe flight; `safe_completion` requires both inspection and whole-flight
safety. A safe recovery need not complete inspection.

`t_cmd` is the first tested unsafe dive delay. `t_rec_side_last_tested_safe_s` is
the last tested safe sideways start; its observed-expiration flag is separate.
Their difference is always called a **grid gap**. If recovery never fails through
1.00 s, that difference is a demonstrated-start separation, not a measured
recovery-expiration gap. If no dive fails, command expiration and its gap are
undefined. A finite climb deadline requires both safe/unsafe samples and no
unsafe-to-safe return. No interpolation or monotonicity assumption is applied.
Full tested intervals appear under `metrics.json → by_scenario → profiles`.

The body state and PID memory are never overwritten at initiation. The existing
measured-position anchor can differ slightly from the nominal reference due to
tracking error. That reference splice is recorded; it is not a physical jump.
The unchanged trajectory has smooth internal joins and keeps flying forward.
NED East remains negative world Y for both lateral maneuvers.

## Validation evidence

Before the 20-attempt study, seeds 1–3 were calibrated and all 534 timing flights
were completed. The sweep was stopped after seven flights, an unfinished final
JSON line was deliberately appended **only to that temporary validation journal**,
and execution resumed. The seven committed records remained byte-for-byte
unchanged; the tail was repaired and all 534 unique flight keys were recorded.
The calibration was also stopped after seed 1 and resumed without replaying its
committed scenario. Production artifacts were not corrupted for this test.

Additional actual smoke flights for seeds 1–3 checked fixed geometry, dispatch-
relative references, negative-world-Y/positive-NED-East motion, identical approval
prefixes, 1,000 Hz timestamps, full horizons, no position jumps and unchanged PID
memory. Four seed-0 immediate flights matched the archived physical hashes,
clearance, safety, inspection and first-violation records exactly.

Evidence is retained in [validation/](validation/): three-scenario calibration,
run, execution journal, metrics, physical smoke/resume checks, and checksums for
all 78 pre-existing mission result artifacts. Ten focused unit tests pass,
including synthetic unfavorable and non-monotonic outcomes explicitly labelled
as analysis tests. Synthetic records never enter scientific results.

The final artifact audit and calibrated GUI check are recorded separately in
`validation/final_audit.json` and `validation/gui_validation.json`.

## Artifacts, provenance and resumption

`../calibration/calibrated_scenarios.json` freezes all attempted definitions,
immediate outcomes, rejection reasons, actual settings, numerical environment and
source hashes. Twelve approval snapshots include measured state, PID memory,
remaining nominal reference, approval time, horizon and scenario, with checksums.
Read these trusted local pickle files only within the recorded environment.

`run.json` freezes the accepted configurations, calibration-manifest hash, start
grids, observation convention, source/environment provenance, committed journal
size/hash and execution status. `executions.jsonl` stores full actual flight
outcomes, physics hashes, approval/initiation control steps, monitoring counts,
minimum clearances and first violation including physics tick and obstacle part.
`metrics.json`, both CSVs and `REPORT.md` derive from that journal. Only seed 1's
representative actual trajectories at 0, 0.5, 1.0 s (plus climb at 1.5 s) are
retained in one compressed `representative_trajectories.npz`.

Each committed row is flushed and synced. Resume verifies settings, source/
environment, frozen snapshots and previously committed journal bytes, then skips
existing flight keys. A valid completed row written just before interruption is
also retained. Only an unfinished trailing line can be discarded; changed
committed data or duplicate keys are rejected. An uncommitted interrupted flight
may be rerun, but does not create a duplicate record. Partial reports explicitly
identify pending cells. `--max-flights`/`--max-scenarios` control invocation budgets,
not protocol or selection. One process lock prevents concurrent CLI simulations;
one worker, one numerical thread and the existing memory guards remain active.

## Reproduction from examples/staleguard

Use the existing `scg` conda environment. These use fresh output folders and the
same frozen seed range and grids. Existing nonempty outputs require `--resume`.

```bash
conda run --no-capture-output -n scg python underpass_inspection/calibration.py --out /tmp/underpass_dev_calibration_repro
conda run --no-capture-output -n scg python underpass_inspection/support/deadline_sweep.py --calibration /tmp/underpass_dev_calibration_repro --out /tmp/underpass_dev_deadlines_repro
```

Resume the original completed or interrupted study without duplicating flights:

```bash
conda run --no-capture-output -n scg python underpass_inspection/calibration.py --resume
conda run --no-capture-output -n scg python underpass_inspection/support/deadline_sweep.py --resume
```

Recompute derived results from the original journal without running physics:

```bash
conda run --no-capture-output -n scg python underpass_inspection/support/deadline_sweep.py --metrics-only
```

Display a selected accepted scenario; substitute `--mode nominal`, `--mode climb
--start 0.5` or `--mode sideways --start 0.5` as desired:

```bash
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --calibration underpass_inspection/results/development/calibration --seed 1 --mode dive --delay 0.2 --gui --overlay-all --hold 10
```

The comparison overlay uses fresh actual replays of this same accepted scenario,
never the seed-0 archive or invented reference curves. It does not write into the
deadline journal. The selected aircraft flies live under the existing controller.
Default half-speed playback affects wall time only. The validated camera remains
distance 3.4 m, yaw −35°, pitch −20°, target [2.2, 0.25, 1.0] m; the SCG checkerboard
floor is untouched. Source and calibration checks reject a rejected or mismatched
seed. Actual per-case approval time is shown by the GUI.

Run the focused code checks without new physical flights:

```bash
conda run --no-capture-output -n scg python -m unittest underpass_inspection.support.test_development underpass_inspection.support.test_pilot -v
```

## Final observed per-scenario deadlines

All entries are approval-relative seconds on tested grids. “Unobserved” is not an endpoint deadline. The side column includes the last safe tested endpoint even when expiration is unobserved.

| Seed | First unsafe dive | Last safe side | First unsafe side | Last safe climb boundary | Side grid gap |
|---|---:|---:|---:|---:|---:|
| 1 | 0.82 | 0.66 | 0.68 | 0.88 | -0.16 |
| 2 | 0.14 | 1.00 | unobserved | 1.48 | 0.86 |
| 3 | unobserved | 1.00 | unobserved | unobserved | unobserved |
| 6 | 0.36 | 0.88 | 0.90 | 1.12 | 0.52 |
| 8 | 0.98 | 0.80 | 0.82 | 1.04 | -0.18 |
| 9 | 0.72 | 0.88 | 0.90 | 1.12 | 0.16 |
| 11 | 0.10 | 1.00 | unobserved | 1.38 | 0.90 |
| 12 | 0.14 | 0.78 | 0.80 | 1.02 | 0.64 |
| 14 | 0.86 | 0.90 | 0.92 | 1.16 | 0.04 |
| 16 | 0.28 | 0.72 | 0.74 | 0.96 | 0.44 |
| 17 | 0.18 | 0.88 | 0.90 | 1.14 | 0.70 |
| 20 | 0.76 | 0.82 | 0.84 | 1.06 | 0.06 |

The completed audit checks all 2,136 unique physical executions and all 12 approval snapshots. The 534 preliminary validation flights match their final-study counterparts exactly. All 78 pre-existing result files remain unchanged. 6 extended/late-start flights have a recorded violation before initiation; these outcomes remain unsafe. Ten representative flight arrays were independently checked for timestamps, body clearance, first violation, inspection dwell and dispatch-relative reference shape.

The native calibrated seed-1 GUI with all four actual overlays was captured and visually inspected: [GUI screenshot](validation/calibrated_gui.png). The selected dive at +0.20 s matched the corresponding headless physical hash, safety, inspection, minimum clearance, initiation step and monitoring count. Camera and checkerboard floor are unchanged. GUI simulation/rendering overhead means playback-speed is a pacing request; it does not change simulated velocities or delays.

Additional installed package versions are retained in `run.json` and [environment.json](validation/environment.json). Metric recomputation was executed without new physical flights. The generated report has been editorially clarified with the finite-positive gap distribution and the absence of safe-but-incomplete inspections; `--metrics-only` regenerates the base report and the same numerical tables.
