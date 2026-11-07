# Underpass Inspection — North-East-Down, version 2

This is a single deterministic physical-feasibility pilot, with seed 0 only.
There are no predictive policies. The aircraft approaches a finite bridge deck
at deck height. Its approved relative maneuver dives beneath the deck, spends
time in a fixed inspection region, then swerves around one pier. Alternatively,
it can abandon inspection by climbing above the bridge or passing beside it.
Every maneuver keeps moving North. Both lateral maneuvers now move East in NED,
which is **negative PyBullet world Y**. Late starts retain the same prescribed shape.

Read the [NED pilot report](results/development/pilot_ned_east_v2/REPORT.md)
and its [technical appendix](results/development/pilot_ned_east_v2/TECHNICAL_APPENDIX.md).
The original [pilot](results/development/pilot/REPORT.md) and earlier
[positive-world-Y verification](results/development/pilot_positive_y_verified/REPORT.md)
remain unchanged. Those versions used the opposite physical lateral direction.

The locked mission frame is right-handed North-East-Down:

```text
North = x_world
East  = -y_world
Down  = -z_world
```

PyBullet physics, obstacle geometry and requested references remain in Z-up world
coordinates. Altitude parameters such as `cruise_z` remain world heights.
`swerve_y` and `sideways_y` are positive **NED East displacement magnitudes**;
the generator converts them to negative world-Y displacement/velocity. The
separate climb maneuver is unchanged. No obstacle was moved: the deck, centered
pier and inspection region are already symmetric about world Y=0.

## Run from examples/staleguard

Use the existing `scg` environment. These open the real PyBullet GUI with the
actual controlled UAV. Default playback is half speed. Mouse controls in the
PyBullet window allow rotating and zooming the camera.

The default view places the aircraft on the left, looking toward the bridge on
the right in a front-oblique view: distance **3.4 m**, yaw **−35°**, pitch **−20°**,
target **[2.2, 0.25, 1.0] m**. The translucent deck exposes the underpass and
lateral turn. Geometry and all flight coordinates stay in the
physical Z-up world. The corner inset is labelled **NED coordinate frame**:
**+X North (forward), +Y East (right), +Z Down**. Its arrows follow the actual
camera orientation. In the default view, North is upper-right, East lower-right
and Down vertical downward. No mission geometry is rotated to achieve this.

```bash
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode nominal --gui --hold 10
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode dive --delay 0.0 --gui --hold 10
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode climb --start 0.5 --gui --hold 10
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode sideways --start 0.5 --gui --hold 10
```

To see the command move toward the pier, replay `--mode dive --delay 0.4`.
`--playback-speed 0.25` is quarter speed; `1` requests real-time pacing.
Simulation/rendering cost adds to that pacing, so achieved playback may be slower.
`--hold` keeps the final scene open for the specified wall seconds.
Omit `--gui` for headless execution. All start/delay values are relative to
approval at flight time 0.8 s and must align with the 20 ms control clock.

Thin colored lines show the requested path; thick lines show physical flight.
Gray is nominal, green the approved maneuver, blue climb, cyan sideways, and
red the actual path after a safety violation. The inspection region is green.
The deck's translucent front, underside and top use distinct materials; the pier
has a darker material. The default SCG/PyBullet checkerboard floor is visible;
its texture, color and geometry are untouched. The live readout reports actual
**NED East and East velocity**, alongside actual world Y for verification.
Small actual-position event points are explained in the screen
margins; labels do not sit on the flight paths. Faint expanded box outlines are
conservative visual guides; the actual Euclidean clearance envelope
has rounded corners. Safety is measured by the predicate, not contact forces.

To compare all four **recorded physical flights** from the same frozen scenario:

```bash
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode dive --delay 0 --gui --overlay-all --hold 10
```

Comparison uses the saved nominal flight, dive at 0 s, climb at 0.5 s and sideways
recovery at 0.5 s. For the selected mode it uses the selected start/delay instead.
Every legend entry gives its approval-relative start time. These are direct joins
of actual 50 Hz samples from the fresh `pilot_ned_east_v2` 1 kHz positions, with no smoothing or
invented alternatives. Gray nominal lines are dashed. The currently selected
aircraft still flies under the real PID/PyBullet dynamics; the other paths are
explanatory saved-flight overlays, not simultaneously simulated aircraft or
predictive policies. A custom scenario or unrecorded start cannot use this overlay.
The original pilot is preserved. Freshly rerun thresholds and direction evidence
are recorded separately in `results/development/pilot_ned_east_v2/`.

Each invocation saves physical trajectories. Default single-flight outputs are
under `results/development/playback_<mode>_<start>/`. Repeating a GUI command
automatically saves to a new numbered folder (`_002`, `_003`, ...), preserving
earlier flights. Explicit `--out` paths and headless smoke/sweep outputs still
require an empty directory. `--capture path.png` captures the actual GUI window
including overlays on X11 systems with `xwininfo` and ImageMagick `import`.
Otherwise it falls back to a camera render and reports that debug overlays are
absent. Current checkerboard-floor screenshots are under
[`results/development/pilot_ned_east_v2/gui/`](results/development/pilot_ned_east_v2/gui/).
The earlier [`support/gui_review/`](support/gui_review/README.md) and original pilot
screenshots are preserved historical evidence.

## Reproduce the headless pilot

The completed original and positive-world-Y verification are preserved. The new
physical version is `ned_east_v2`, with data in `results/development/pilot_ned_east_v2/`.
Reproduce its single-scenario experiment in a fresh directory:

```bash
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --sweep --out /tmp/underpass_ned_east_reproduction
```

The sweep first verifies nominal flight, immediate dive, both immediate recoveries,
and a delayed dive. It stops before sweeping if a basic requirement fails. It then
tests 51 starts from 0 to 1 s at 20 ms spacing independently for dive/climb/sideways.
The initial five flights are reused inside the 154-flight matrix; no extra
scenario is generated. Geometry is never recalibrated. `--smoke` executes only
the five basic flights; give it a fresh output path as well.

The four primary outputs are `summary.json`, `flights.jsonl`, `per_delay.csv`,
and `trajectories.npz`. The summary includes the full scenario, obstacle bounds,
assumptions, software/source provenance, column definitions and observation
horizon. Each flight records approval/initiation control steps, first violation
at its physics tick, obstacle and geometric part, safety, raw inspection dwell
completion, whole-flight safe completion and minimum body clearance.

The NPZ contains `<mode>_<start_tick>__physics` arrays of flight time and actual
world XYZ at 1,000 Hz, `__control` arrays of actual world-state observations at
50 Hz, and `__reference` arrays of requested world position/velocity at 50 Hz.
Use `scenario.world_to_ned()` for three-component positions or linear velocities;
full attitude states remain in the simulator convention. Each flight also records
`final_position_ned_m` and `final_velocity_ned_mps`. Time zero is flight
start in those arrays; subtract scenario approval time for approval-relative time.
All flights run through flight time 8.8 s, including after safety loss.

Geometry, timings, inspection bounds and displacements are explicit provisional
design assumptions in `scenario.py`. A full scenario JSON can be supplied with
`--scenario` for an explicitly separate seed-0 experiment; no randomization or
parameter search is implemented. Moving a pier never changes a maneuver shape.

Maneuver joins have continuous position, velocity and acceleration. At initiation,
the required measured-position anchor differs slightly from the old nominal
reference. This small reference splice is recorded, and is not a physical state
jump or controller reset. Exact continuity to the old nominal reference and exact
measured-position anchoring cannot both hold when nominal tracking error is nonzero.

## Validation without new flights

```bash
conda run --no-capture-output -n scg python -m unittest underpass_inspection.support.test_pilot -v
conda run --no-capture-output -n scg python underpass_inspection/support/validate_results.py underpass_inspection/results/development/pilot_ned_east_v2
```

One simulation worker and one numerical thread are used. The shared memory guard
retains a 2 GiB process limit and 2 GiB available-host-memory reserve. A mission lock
prevents overlapping pilot CLI runs. Existing mission code and results are untouched.
