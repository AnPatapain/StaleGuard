# Geofence: 90-degree left turn and northbound checkpoint

The approved command freezes radius, speed and a world-frame checkpoint box.
At dispatch the nominal reference anchors a quarter-circle from +X to +Y.
The reference then continues north at the approved speed, with constant X/Z.
The checkpoint never steers the path. A delay changes the turn's starting lane
and can cause a miss; it never produces a connector, new goal, or return west.

Completion is a northbound pass through the fixed box: actual position inside
all three inclusive bounds, actual velocity within 0.12 m/s of `(0,speed,0)`,
and 0.20 s continuous dwell before T. There is no stop/hover requirement. The
0.60 m east-west box width, 0.10 m north/altitude half-extents, and north distance
`radius + speed*2 s` are provisional physical mission settings, fixed before
characterization/calibration. The sampled box lies wholly inside the body-safe
geofence. Incompatible geometry is rejected, not repaired during calibration.

The existing SCG 50 Hz PID, 1000 Hz physics/body-clearance monitor, and shared
Numba model are retained. Calibration samples one measured feasible radius and
one fixed checkpoint, then searches descending approval times at D=0. It
maximizes nominal pre-turn patrol distance on the 0.2 s grid, recording actual
progress too. Every later approval is checked or ruled out by the observed
nominal pre-dispatch violation. Safety is monitored through approval+T even
when completion occurred earlier.

Run from `examples/staleguard` using the existing `scg` environment, serially.
The development cap remains 20 attempted seeds, 200 flights per command, one
worker, one numerical thread, and the existing 2 GiB memory guardrails. No
baseline matrix is needed for this correction.

```sh
SCG_PYTHON=/home/kean/miniconda3/envs/scg/bin/python
$SCG_PYTHON -m geofence_turn.cli characterize --out geofence_turn/logs/northbound_review/characterization --max-trials 9
$SCG_PYTHON -m geofence_turn.cli calibrate --out geofence_turn/logs/northbound_review/development --attempts 10 --target 5 --max-trials 100
$SCG_PYTHON -m geofence_turn.cli replay --out geofence_turn/logs/northbound_review/d0_replay --max-trials 5
$SCG_PYTHON -m unittest geofence_turn.tests.test_mission geofence_turn.tests.test_prediction -v
$SCG_PYTHON -m unittest geofence_turn.tests.test_integration -v
$SCG_PYTHON -m geofence_turn.validation
$SCG_PYTHON -m geofence_turn.diagnostics
```

Use `--resume` to continue interrupted characterization/calibration without
repeating completed trials. Output directories must otherwise be new; use explicit `--envelope` and `--development`
when changing their locations. The integration tests use the current run under
`logs/northbound_review/` (or `GEOFENCE_DEVELOPMENT` for the development path).

The schema is **northbound_checkpoint_v2**. Previous connector-based scenarios,
snapshots, characterization and baseline results are historical and incompatible;
they are preserved under the old log directories and are not current evidence.
The previous report is retained at
`logs/northbound_review/original_PHASE1_REPORT.md`. Read [PHASE1_REPORT.md](PHASE1_REPORT.md)
for the correction's actual validation status. Load only local trusted pickles.
