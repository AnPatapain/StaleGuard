# NED East version 2: technical appendix

## Exact file changes

| Modified file, relative to underpass_inspection | Change |
|---|---|
| `scenario.py` | Add `ned_east_v2` identity and the world-to-NED rotation; negate only the two lateral displacement/velocity profiles; document their positive NED East magnitudes |
| `simulation.py` | Add converted final NED position/linear velocity to records; underlying control, physics and monitoring unchanged |
| `support/visualization.py` | Use the new NED dataset, correct inset and East telemetry; retain camera, floor, colors and geometry |
| `pilot.py` | NED metadata, versioned output, actual negative-world-Y/positive-East smoke checks and North terminology |
| `support/test_pilot.py` | Test both independent lateral profiles and the proper right-handed coordinate transform |
| `README.md` | Current NED definitions, data interpretation, GUI usage and reproduction |

The two physical changes are `world_y = -swerve_y * phase` for the approved
command and `world_y = -sideways_y * phase` for the independent recovery, with
the corresponding velocity sign changes. Their phases remain independent:
approved swerve starts at elapsed 2.60 s, lasts 1.20 s and moves 0.85 m East;
sideways recovery starts immediately, lasts 1.30 s and moves 1.00 m East. Forward
and vertical references are unchanged. No general planner or new policy exists.

## Coordinate and geometry definitions

The conversion is R=diag(1,−1,−1): N=x_world, E=−y_world, D=−z_world.
It has determinant +1, satisfies R²=I, and the North/East basis cross product
equals Down. `world_to_ned()` converts three-component position or linear velocity
arrays without modifying its inputs. Raw physics, PID, body states, reference
arrays and safety volumes stay in the original Z-up world. Attitude arrays are
not relabelled as NED attitude. Positive Down is therefore negative world Z;
altitude above the world ground plane has negative NED Down coordinate.

Numeric geometry and timing parameters are unchanged. World bounds remain:
deck X=[1.50,4.00], Y=[−0.55,0.55], Z=[1.15,1.55] m; pier X=[2.43,2.67],
Y=[−0.15,0.15], Z=[0,1.15] m; inspection X=[1.55,1.85], Y=[−0.18,0.18],
Z=[0.65,0.85] m. Since every lateral bound is symmetric about zero, mirroring
their Y placement would produce identical geometry. No adjustment was made.

The existing body sphere and 0.05 m clearance threshold, 50 Hz controller,
1,000 Hz physics observation, 0.18 s inspection dwell and approval+8 s observation
horizon are unchanged. The original nominal ramp, dive, level segment and climb
are unchanged. Only the two physical lateral signs change.

## Executed checks

Six focused unit tests passed. The five-flight smoke test passed all **43 checks**,
including safe immediate inspection, safe immediate recoveries, unsafe nominal
flight, actual negative-world-Y swerve/positive NED East, independent East recovery,
fixed relative shapes and geometry, no state/PID reset at activation and regular
1 kHz observations through the full horizon.

The new serial matrix contains **154 actual flights**: one nominal and 51 starts
for each maneuver on 0–1 s in 20 ms steps. Its initial five validation flights
are not duplicated inside that count. Every flight has 8,801 physical positions
and 441 full-state observations. Independent vectorized clearance/dwell and
first-violation recomputation passed for all flights, as did grid completeness,
record digests and current-source verification. See `validation.json`.

`direction_audit.json` verifies that all 154 North/vertical reference profiles
match the preceding version, and that nominal/climb actual positions are unchanged.
All 102 dive/sideways flights finish at negative world Y relative to their
dispatch anchors. Every stored final NED position and velocity was independently
checked against the raw world values. Actual state and PID memory remain unchanged
at maneuver activation; the largest physical 1 ms position increment is
0.002081135 m. The original 1 cm/ms jump diagnostic is not a safety threshold.

All previous result files and protected shared sources were compared with their
pre-edit checksums and preserved. Their checksum inventory is retained in
`direction_audit.json`; no historical result, report, screenshot or source hash
was rewritten. The new scenario has explicit identity `ned_east_v2` and uses
`results/development/pilot_ned_east_v2/`.

## GUI and actual projection checks

Final camera: **distance=3.4, yaw=−35°, pitch=−20°, target=[2.2,0.25,1.0]**.
No camera adjustment was needed. The stock checkerboard floor's visual/material/
texture data remain identical before and after scene styling in every GUI case.
The inset projects the NED basis through the actual camera rotation and is labelled
“NED coordinate frame”, “+X North (forward)”, “+Y East (right)”, “+Z Down”.
Transformation equations are kept out of the GUI.

The five actual GUI replays cover dive D=0, dive D=0.4, climb start=0.5, sideways
start=0.5 and live D=0 dive with saved-path overlays. Their full physical,
control-state and reference arrays match their fresh headless records exactly;
safety, completion, minima and first violations match too. Their camera bases
show North upper-right, East lower-right and Down vertical downward. For D=0,
the actual lateral-phase displacement is [0.70176,−1.04569,0.00702] m in world
coordinates. Projecting those actual endpoints gives a normalized screen change
of **[+0.16102,+0.03663]**, where positive means right/down. The D=0.4 s flight
also passes physical and screen-direction checks. See `gui_validation.json`.

Screenshots were captured directly from actual PyBullet windows and personally
inspected. Comparison curves join the version-2 recorded physical samples;
there are no old opposite-direction overlays, invented trajectories or smoothing.

- [Actual Eastward motion at flight 4.00 s](gui/eastward_motion.png)
- [Immediate safe dive](gui/dive_0.png)
- [Delayed unsafe dive](gui/dive_delayed.png)
- [Climb](gui/climb.png)
- [Independent East recovery](gui/sideways.png)
- [All four paths with the UAV at flight start](gui/overlay_start.png)
- [All paths after the selected flight](gui/overlay.png)

## Timing interpretation and reproduction

The fresh first unsafe dive delay is 0.12 s, after safe delay 0.10 s. The last
safe East recovery start is 0.92 s, before unsafe start 0.94 s. Their grid-resolved
gap is +0.80 s. Climb is safe at all 51 starts, including 1 s; its actual
expiration and exact gap are null. Full tested intervals and any non-monotonic
flags are retained in `summary.json`; no interpolation manufactures a deadline.

With `scg` active, run from examples/staleguard:

```bash
python underpass_inspection/pilot.py --mode dive --delay 0 --gui --overlay-all --hold 10
python underpass_inspection/pilot.py --mode dive --delay 0.4 --gui --hold 10
```

Reproduce the NED seed-0 sweep in a fresh output, preserving the completed one:

```bash
python underpass_inspection/pilot.py --sweep --out /tmp/underpass_ned_east_reproduction
```

Validation without new flights:

```bash
python -m unittest underpass_inspection.support.test_pilot -v
python underpass_inspection/support/validate_results.py underpass_inspection/results/development/pilot_ned_east_v2
```

The four primary outputs are `summary.json`, `flights.jsonl`, `per_delay.csv` and
`trajectories.npz`. Raw trajectory arrays remain in world coordinates, explicitly
declared in summary metadata. Flight records additionally contain converted final
NED position/velocity. Source/environment provenance, direction audit, independent
validation, GUI verification and these reports accompany them. No shared model,
existing mission, PID, safety predicate, completion criterion or frequency changed.
