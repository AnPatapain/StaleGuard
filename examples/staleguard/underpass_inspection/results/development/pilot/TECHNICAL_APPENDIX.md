# Underpass pilot: implementation and reproduction

## Created files and scope

| File | Responsibility |
|---|---|
| `underpass_inspection/__init__.py` | CPU-only, one numerical thread |
| `underpass_inspection/scenario.py` | Fixed dataclasses, geometry, body clearance, inspection predicate, nominal and relative profiles |
| `underpass_inspection/simulation.py` | One stock SCG PID-owned environment; actual flights, 1 kHz monitoring and optional GUI |
| `underpass_inspection/pilot.py` | Single-flight CLI, basic-feasibility gate, three independent sweeps, four primary artifacts |
| `underpass_inspection/support/visualization.py` | Visual-only deck/pier, inspection region, references, markers, camera capture |
| `underpass_inspection/support/test_pilot.py` | Four focused geometry/reference/reporting tests |
| `underpass_inspection/support/validate_results.py` | Independent vectorized verification of saved physics and outcomes; no flights |
| `underpass_inspection/README.md` | Usage, assumptions and artifact interpretation |

No existing mission/model/monitor source was modified. There is no predictive
policy or new dynamics model. Body radius is imported from Geofence; memory
checking uses its existing guard; actual physics observation uses the shared
`physics_monitor.monitor_physics_steps`. Stock controller parameters and CF2X
physics are retained. Full scenario and source hashes are in `summary.json`.

## Fixed physical settings

| Setting | Value / interpretation |
|---|---|
| Seed / randomization | 0; initialization/inertial randomization and disturbances disabled |
| Controller / actual physics monitor | 50 Hz / 1,000 Hz |
| Nominal reference | 0.4 s ramp to 0.6 m/s east; cruising altitude 1.35 m |
| Approval / fixed endpoint | Flight 0.8 s / flight 8.8 s; steps 40 / 440 |
| Body radius / required clearance | sqrt(0.06² + 0.0125²) = 0.0612882534 m / 0.05 m |
| Deck | X=[1.50,4.00], Y=[−0.55,0.55], Z=[1.15,1.55] m |
| Pier | X=[2.43,2.67], Y=[−0.15,0.15], Z=[0,1.15] m |
| Inspection region | X=[1.55,1.85], Y=[−0.18,0.18], Z=[0.65,0.85] m; inclusive bounds |
| Required dwell | 180 elapsed physics ticks, or 0.18 s |
| Dive | Fixed 0.60 m drop over 1.30 s; level thereafter |
| Pier swerve | Starts 2.60 s after dispatch; shifts +Y by 0.85 m over 1.20 s |
| Climb | Fixed 0.70 m rise over 1.20 s; continues above deck |
| Sideways recovery | Fixed +Y displacement of 1.00 m over 1.30 s; continues beside deck |

All new dimensions, maneuver parameters and inspection requirements are explicit
unvalidated design assumptions. They were chosen for this small-UAV simulator
pilot and were not obtained by optimizing any policy comparison. Only seed 0
with the displayed values was physically evaluated. No geometry search or pier
position sweep was performed.

For a normalized phase u in [0,1], each changing lateral/vertical displacement
uses 10u³−15u⁴+6u⁵ and its analytic velocity. Displacement and its first two
derivatives join continuously to level/straight segments. X displacement is
always speed × elapsed time. At initiation, the actual measured XYZ becomes the
anchor; this never changes the scenario-fixed profile. Obstacles are not read
when constructing the displacement profile. Actual velocity is not forcibly
matched to requested velocity: the stock PID produces the physical response.

The maximum initiation reference repositioning across the matrix was
0.0193525 m. Desired velocity remains continuous with nominal forward velocity,
but position reanchoring is not an exact continuous splice to the old nominal
reference when tracking error is nonzero. Both the measured anchor and splice
are saved. No physical position/velocity or PID memory is changed at initiation.

Safety is `signed_point_distance_to_box − body_radius >= 0.05 m`, separately for
deck and pier. Outside a box the point distance is Euclidean, including corners;
inside it is negative nearest-face depth. Equality is safe. Visual-only bodies
have collision shape index −1 and exert no forces. The red wireframe expands
each box axis by body radius + margin; this is a conservative visual guide at
corners, not the exact rounded clearance surface used for decisions. First-hit
part labels describe the first sample's geometry (including edge combinations),
not a counterfactual claim about which maneuver phase caused an event.

## Executed validation and evidence

1. A five-flight smoke run in `/tmp/underpass_smoke_v2` passed all 38 recorded
   checks before the full sweep was launched. The preceding startup attempt only
   exposed a missing argument to the reused memory guard; no flight or scenario
   parameter changed to fix that integration error.
2. Four unit tests passed: signed 3D box distances, smooth-profile derivatives
   and joins, fixed relative translation/no input mutation, and non-monotonic/
   censored deadline reporting.
3. The completed matrix contains 154 distinct actual flights: one nominal and
   51 starts for each of dive, climb and sideways. Its initial five flights were
   used as the gate, not duplicated inside the matrix. All 154 flights have
   8,801 actual physics positions and 441 actual control observations. Flights
   continue after first violation and inspection completion to the fixed endpoint.
4. The saved 154-flight bundle passed independent vectorized distance and dwell
   recomputation, first-violation tick/obstacle checks, complete grid/key checks,
   per-trajectory checksums and source-provenance checks. See `validation.json`.
   All flights finish beyond X=4 m, past the deck. The largest actual position
   increment over 1 ms is 0.00208114 m; initiation preserves the complete physical
   state and all captured PID memory. The 1 cm/ms diagnostic jump guard is not a
   safety margin and does not alter outcomes.
5. Six GUI replay executions were performed serially for the same seed: immediate
   dive, delayed dive, nominal, climb, sideways, and a second delayed dive to
   capture full window overlays. None entered the timing matrix. Representative
   GUI physics hashes for all four modes and delayed dive match their headless
   records exactly. The five actual GUI window screenshots were personally
   inspected, including the green early path around the pier and the red late
   path violating its clearance. `gui_validation.json` records the comparisons.
   A root-window capture attempt failed on this display; capturing the identified
   PyBullet client windows succeeded. Some sequential GUI reconnects emitted
   debug-draw warnings; the captured windows contain the required paths/labels,
   and exact physical replay checks passed.

The preliminary five smoke flights and six GUI flights are separate validation
work, not extra scenarios or independent statistical samples. The four primary
artifacts describe only the 154-flight matrix. This pilot has no confidence
intervals or scenario-population claims.

## Results and timing interpretation

The delay/start grid contains 0,0.02,...,1.00 s. Initiation is control step
40 + round(50 × start_s). First unsafe dive delay 0.14 s is initiation step 47;
its later physical violation occurs at flight time 4.038 s, physics tick 4038.
These are different quantities. The latest safe sideways start 0.94 s is step
87; at start 0.96 s (step 88), the first violation is flight time 2.510 s.
Climb at 1.00 s (step 90) remains safe with 0.153117 m minimum body clearance.

All three tested safety profiles are monotonic on the grid, but code does not
assume this: full tested safe/unsafe intervals are reported. A non-monotonic
recovery suppresses a single scalar expiration/gap; a wholly safe grid records
a latest tested safe start but leaves the unobserved recovery expiration null.
Thus climb expiration and its exact gap are null, while the known safe start
at 1.00 s is retained. All dive starts complete raw inspection dwell; only
0–0.12 s starts are safe completions. Loss of inspection completion was not
observed for dive within the grid.

The sum of matrix per-flight wall runtimes was 65.264 s on this host. It excludes
some startup, garbage collection, output compression and GUI validation. It is
neither total task elapsed time nor a predictor timing measurement. No forecast
was computed. Full Python/NumPy/platform/source provenance is in `summary.json`.

## Artifacts

| File | Contents |
|---|---|
| `summary.json` | Full configuration, provenance, smoke gate, measured grid intervals/deadlines and GUI-inspection annotation |
| `flights.jsonl` | Authoritative one-row-per-flight actual outcomes and event measurements |
| `per_delay.csv` | Mode/start, steps, safety, raw inspection completion, safe completion, minimum clearance and first-hit diagnostics |
| `trajectories.npz` | Float64 actual physical XYZ/time at 1 kHz, actual states at 50 Hz, and requested references; about 41 MB |
| `validation.json` | Independent checks of all saved flights |
| `gui_validation.json` | Exact GUI/headless physical replay comparisons |
| `gui_nominal.png`, `gui_dive_immediate.png`, `gui_dive_delayed.png`, `gui_climb.png`, `gui_sideways.png` | Captures of actual PyBullet windows, including overlays |
| `REPORT.md`, `TECHNICAL_APPENDIX.md` | Short scientific account and implementation/reproduction detail |

## Exact commands from examples/staleguard

The following use the installed `scg` environment. GUI outputs use distinct default
single-flight directories. Repeated default GUI commands now allocate a new
numbered folder, preserving earlier results. Explicit `--out` paths and headless
outputs still require an empty directory; completed runs are never replaced.

```bash
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode nominal --gui --hold 10
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode dive --delay 0.0 --gui --hold 10
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode climb --start 0.5 --gui --hold 10
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode sideways --start 0.5 --gui --hold 10
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --mode dive --delay 0.4 --gui --hold 10
```

The sweep command below reproduces the same experiment without replacing the
archived original. It has no randomization, policies, predictions or other seeds.

```bash
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --sweep --out /tmp/underpass_pilot_reproduction
conda run --no-capture-output -n scg python -m unittest underpass_inspection.support.test_pilot -v
conda run --no-capture-output -n scg python underpass_inspection/support/validate_results.py underpass_inspection/results/development/pilot
```

`--playback-speed 0.5` is the default requested half-speed pacing. GUI rendering
adds overhead. The final scene can be held with `--hold`, and the camera can be
rotated or zoomed interactively. Optional `--capture` uses PyBullet's camera API,
which omits debug overlays; the archived overlay screenshots instead came from
the displayed client windows. New runs initially record `gui_inspected=false`;
the completed original was annotated only after actual window inspection.

Subsequent usability correction: default GUI playback now chooses numbered output
folders when replayed. Only CLI output selection and usage documentation changed;
the physical mission and archived outcomes did not change. The original recorded
`pilot.py` checksum is retained, so strict current-source verification flags that
CLI change rather than rewriting historical provenance.
