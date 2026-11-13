# Underpass GUI redesign — visual review

The GUI now shows the UAV approaching from the left toward the bridge on the
right. A front-oblique camera, translucent deck faces and a quiet background make
the under-deck flight and lateral pier avoidance visible. Blue climb and cyan
sideways recovery are available together through `--overlay-all`, using recorded
physical flights from the unchanged scenario. Labels and the sketch-axis inset
stay in screen margins. Six requested replay cases matched the original actual
positions, full states, references, safety, completion and minimum clearance
exactly. This is a visualization validation, not a new deadline experiment.
No scenario parameter, maneuver, controller, safety predicate or archived pilot
file changed. Mouse orbit and zoom remain available.

## Selected view and coordinate convention

| PyBullet camera parameter | Final value |
|---|---:|
| Distance | 3.4 m |
| Yaw | −35° |
| Pitch | −20° |
| Target, world XYZ | [2.2, 0.25, 1.0] m |

Thirteen camera candidates were captured and inspected. Low views exposed the
underside directly, but compressed the aircraft approach and made the lateral
bend less clear. The selected higher front-oblique view shows that bend clearly;
separate translucent materials expose the underside through the deck while
retaining the exact original faces. Camera candidate images and parameters are
kept here as visual review evidence.

The physical scene remains Z-up. No state, reference or obstacle is reflected or
rotated. The separate inset reproduces the sketch's display axes, explicitly
labelled `Xd=X_world`, `Yd=-Y_world`, `Zd=-Z_world`, with “WORLD +Z IS UP”. It is
a sketch-coordinate inset rather than a claim that physical +Z points down.

## Screenshots inspected

- [All recorded paths at flight start](comparison_start.png)
- [All recorded paths after the selected flight](comparison.png)
- [Nominal](nominal.png)
- [Immediate dive](dive_0.png)
- [Dive delayed by 0.4 s](dive_delayed.png)
- [Climb started at 0.5 s](climb.png)
- [Sideways recovery started at 0.5 s](sideways.png)

These are actual PyBullet window captures, including debug overlays. No path was
smoothed and no alternative flight was invented. Comparison lines directly join
actual archived positions sampled at 50 Hz from their original 1 kHz records.
Comparison is explanatory: complete saved alternatives are visible before the
selected live aircraft has finished its flight.

## Physical-equivalence checks

| Requested GUI replay | Whole-flight safe? | Inspection dwell completed? | Minimum clearance, unchanged |
|---|---|---|---:|
| Nominal | No | No | −0.2604802855 m |
| Dive, D=0 | Yes | Yes | 0.1465430594 m |
| Dive, D=0.4 s | No | Yes | −0.0885227033 m |
| Climb, start=0.5 s | Yes | No | 0.4138377419 m |
| Sideways, start=0.5 s | Yes | No | 0.2999388255 m |
| Comparison overlay with live D=0 dive | Yes | Yes | 0.1465430594 m |

For every case, the 1 kHz physical arrays, 50 Hz full-state arrays and reference
arrays matched the original arrays exactly. First violation, safety/completion,
dwell completion time and per-obstacle minima also matched exactly. See
[`replay_validation.json`](replay_validation.json). All original pilot files were
checked against pre-edit checksums and are unchanged. The scenario file is also
unchanged. Removing GUI-only blocks and the added GUI option from the old/current
simulation code produces identical syntax trees. Four existing mission unit
tests passed. The complete 154-flight sweep was not rerun.

The original observed grid thresholds remain: first unsafe dive delay 0.14 s;
last safe sideways start 0.94 s and first unsafe 0.96 s; climb safe through 1 s
with no measured expiration. This redesign adds no deadline markers or new claims
about those thresholds. Event points use the actual approval/initiation positions.

## Changed implementation files

- `support/visualization.py`: camera, per-face visual materials, floor appearance,
  thin clearance guides, physical-path overlay, screen-anchored labels, inset and
  GUI-window capture.
- `simulation.py`: only GUI drawing calls replaced by the drawing helper, plus
  an optional GUI overlay argument; all physical code unchanged.
- `pilot.py`: `--overlay-all` option and its GUI-only validation/forwarding.
- `README.md`: updated GUI instructions.

All new screenshots and validation notes are here, separate from the existing
experimental outputs. Their historical source checksums were not rewritten.

## Commands, with scg active, from examples/staleguard

```bash
python underpass_inspection/pilot.py --mode nominal --gui --hold 10
python underpass_inspection/pilot.py --mode dive --delay 0 --gui --hold 10
python underpass_inspection/pilot.py --mode dive --delay 0.4 --gui --hold 10
python underpass_inspection/pilot.py --mode climb --start 0.5 --gui --hold 10
python underpass_inspection/pilot.py --mode sideways --start 0.5 --gui --hold 10
python underpass_inspection/pilot.py --mode dive --delay 0 --gui --overlay-all --hold 10
```

Default playback is half speed, without changing simulated timing. Comparison
uses nominal, the selected dive delay, climb at +0.5 s and sideways at +0.5 s.
When another mode is selected, its requested start replaces the corresponding
comparison start. Only recorded starts in the same frozen scenario are accepted.
