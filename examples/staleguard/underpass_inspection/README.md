# Underpass Inspection — North-East-Down, version 2

Underpass keeps its policy rules, prediction grid, storage/metric helpers and memory guard locally. It imports no code from Geofence-Turn or Inspection-Descent; only the shared `model/` dynamics and physics monitor remain shared. Existing result artifacts retain their historical provenance.

This contains the frozen seed-0 pilot, development calibration/deadline study,
and seven-policy development benchmark on the same 12 accepted scenarios.
Held-out calibration is complete: 865 accepted scenarios from 1,000 attempts.
The reduced held-out benchmark is complete: **9,100 flights on 100 scenarios**.
Results follow the same two-way layout as the other missions: `results/development/`
and `results/evaluation/`. Start with
[the completed evaluation report](results/evaluation/baseline_benchmark/REPORT.md). The aircraft approaches a finite bridge deck
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

The [development deadline report](results/development/deadline_sweep/REPORT.md)
answers how often command safety and recovery feasibility expire at different
times. Its [technical appendix](results/development/deadline_sweep/TECHNICAL_APPENDIX.md)
documents calibration, validation, artifacts and reproduction.

## Seven-policy development benchmark

The [benchmark report](results/development/baseline_benchmark/REPORT.md) compares
seven policies on the exact 12 frozen accepted scenarios, at all 51 dispatch
delays from 0 through 1.00 s. Its [technical appendix](results/development/baseline_benchmark/TECHNICAL_APPENDIX.md)
records policy semantics, measured prediction timing, physical validation and
reproduction. The full matrix contains **4,284 actual PyBullet flights**.
Each flight replays the original initial conditions and monitors the full horizon;
saved deadline-sweep outcomes are verification references, never policy inputs or
substitutes for new flights.

The primary fallback for every policy is the same independent **Eastward
sideways** maneuver. Climb remains diagnostic and is never chosen by these policies.

| Policy | Approval / pending behavior | Dispatch behavior |
|---|---|---|
| `no_recovery` | Continue nominal flight | Execute original relative dive |
| `always_recover` | Cancel and recover immediately at approval | Ignore canceled command |
| `fbs` | Observe clearance-feature changes through approval; freeze EMA-based TTL | Recover if ready and expired; otherwise dispatch |
| `use_time_recheck` | Continue nominal; no proactive check | One actual-state immediate-command forecast; act after measured computation |
| `staleguard` | One approval forecast; continue nominal while pending | Recover if ready and expired; unavailable forecast permits dispatch |
| `staleguard_early_recovery` | Same forecast; recover while pending once ready and expired | Same dispatch gate if still pending |
| `fixed_early_recovery` | Cancel/recover when elapsed time reaches 0.20 s | Dispatch before 0.20 s; recover at or after equality |

FBS keeps a local copy of the established alpha 0.1, epsilon 1e-12 m/sample, 210-step cap,
margin/EMA flooring, strict `age > TTL` expiry and readiness gate. Its signal is
required clearance minus the minimum current 3D body clearance. The EMA starts
at zero on the first observation and observes every control step through approval
inclusive. Zero volatility uses the numerical floor and cap. It does not simulate
the queued command and receives no future dispatch time.

StaleGuard reuses the shared `model.lightweight.LightweightPredictiveModel` and
a locally defined fixed 19-point prediction grid (20 ms spacing through 0.20 s, then
0.10 s spacing through 1.00 s). It first predicts nominal waiting and saves PID
checkpoints, then restores each checkpoint and anchors the unchanged relative
dive to its predicted state. A swept Euclidean segment/box/body checker handles
both fixed volumes, using the original body radius and clearance threshold.
An unsafe waiting prefix stays unsafe. Only the required waiting prefix is
evaluated; eventual unlimited nominal collision does not invalidate every command.
Expiration is the first predicted unsafe grid point. All-safe grids have
unobserved expiration. Safety and inspection completion are separate outcomes.

The two StaleGuard variants share one forecast and measured runtime per scenario.
Following Geofence, kernels are warmed once before approval forecasts; startup
cost is recorded separately. Computation is measured offline from the frozen
approval snapshot, and its duration is replayed on the physical control clock.
Nominal motion continues until readiness. EarlyRecovery needs both readiness and
expiry and never acts retroactively. If dispatch already permitted the command
while the forecast was unavailable, later readiness cannot cancel it.

Use-Time Recheck has no approval-time TTL sweep. It predicts one immediate command
from the actual dispatch state/PID and remaining horizon. The real UAV continues
nominal motion while its measured computation duration is replayed; application
or recovery occurs at `dispatch_step + floor(runtime*50) + 1`. Its check-state and
actual application-state can therefore differ. Earlier violations remain recorded.

The physical driver alone knows the configured dispatch schedule. The policy
object receives elapsed time and a dispatch event only when it occurs. At a shared
control step, approval is processed first, then readiness, dispatch, pending timers,
and the physical maneuver. Dispatch wins timer ties; the dispatch gate still uses
`>=` expiry. Recovery permanently cancels the inspection command; neither maneuver
can restart or override an already started maneuver. Realized events are journaled.

Run from `examples/staleguard` in the existing `scg` environment:

```bash
# 27 focused tests, including two actual out-of-grid waiting-violation stress flights
conda run --no-capture-output -n scg python -m unittest underpass_inspection.support.test_baselines underpass_inspection.support.test_pilot underpass_inspection.support.test_development -v

# Small paired physical benchmark: seeds 1 (negative gap), 6 (long positive), 14 (short positive)
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --smoke --out /tmp/underpass_policy_smoke_repro

# Full development matrix; the representative-seed option only chooses saved paths
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --representative-seeds 1 6 14
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --representative-seeds 1 6 14 --resume

# Derived results only: no model forecasts or physical flights
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --metrics-only

# Plot all seven retained actual paths; optionally watch one recorded decision in PyBullet
conda run --no-capture-output -n scg python underpass_inspection/support/visualize_policy_comparison.py --seed 6 --out /tmp/underpass_policy_comparison.png
conda run --no-capture-output -n scg python underpass_inspection/support/visualize_policy_comparison.py --seed 6 --gui --policy staleguard_early_recovery --hold 10
```

GUI playback repeats the recorded policy's maneuver and actual start time using
live PID/PyBullet, then checks its physical trajectory hash. It does not retime a
policy using a fresh computation or add a benchmark score. The saved comparison
uses actual 1,000 Hz positions without invented alternatives or smoothing. The
existing NED GUI camera, floor, scene, event labels and trajectory colors are kept.

The canonical layout follows the established missions: policy/prediction adapters
in `baselines/`, execution/metrics in `baseline_benchmark/`, frozen I/O in
`storage.py`, and diagnostics under `support/`. There is one native flight engine
and one shared lightweight dynamics implementation. No compatibility import
wrappers or duplicate root-level prediction engines are used.

The frozen calibration/deadline files remain unchanged. `storage.py` verifies
their historical hashes and explicit syntax-tree equivalence for the policy-free
physical engine, the mission definition except its seed-ID validation, and the
pilot functions outside its CLI. Generation now supports evaluation seeds beyond
20; the development draws and all frozen definitions remain unchanged. New benchmark
resumes require exact current source/package hashes, protocol, snapshot checksums
and committed journal bytes. The original calibration/deadline executables retain
their stricter historical resume checks; do not rewrite their old provenance.

`run.json` records the calibration hash, all actual scenario configurations, policy
parameters, model platform parameters, warmup, frozen forecasts/runtimes and source/
environment provenance. `executions.jsonl` records actual outcomes and events.
`metrics.json`, both CSVs and `REPORT.md` derive from that journal. One NPZ retains
only representative actual paths. Completed flights are never duplicated on
resume; only an unfinished uncommitted final line may be repaired. Use
`--max-flights N` for an invocation budget without changing the frozen protocol.
The existing one-worker, one-numerical-thread and memory guards remain active.

The driver accepts `--calibration` and `--ground-truth` paths for another frozen
cohort later. Policy logic does not branch on development seeds. Held-out
calibration is supported below; no held-out flights have been run here.

## Reduced held-out benchmark — 100 scenarios × 13 delays × 7 policies

The deadline protocol uses **9,100 physical flights**, about 2.3 hours at the
previous smoke throughput (budget 2.5–3 hours). It uniformly samples 100 scenarios
without replacement from the frozen 865 accepted IDs using NumPy
`default_rng(20261108)`, then executes the selected IDs in ascending order. The
[freeze](results/evaluation/preparation/freeze_reduced_100x13_v3.json) records the population hash,
all 100 selected IDs, original snapshot hashes, source/environment and parameters.
Selection reads no delayed outcomes or policy performance. No new calibration
or policy tuning is performed.

Physical delays are exactly:

```python
[0.00, 0.04, 0.08, 0.12, 0.16, 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.60, 2.00]
```

All seven policies and the 19-point predictor are unchanged. Physics, PID,
1,000 Hz safety monitoring, Eastward recovery, eight-second horizon and measured
runtime/readiness conventions are unchanged. The existing no-expiration rule
still permits dispatch beyond 1 s; there is no added horizon rejection.

Resolution is reduced: gaps are 0.04 s near approval and reach 0.40 s later.
Expiration labels are first tested unsafe delays, not exact deadlines. All ten
selected delays through 1 s are predictor-grid points, so this design does not
test between-grid admissions within 1 s. It has wider scenario uncertainty than
the 865-scenario protocol. Rates equally weight the selected delay points, not a
uniform continuous delay distribution.

| Analysis of one benchmark | Delays per scenario | Flights |
|---|---:|---:|
| Full, 0–2 s | 13 | 9,100 |
| Within prediction grid, 0–1 s | 10 | 7,000 |
| Beyond prediction grid, >1–2 s (1.2, 1.6, 2 s) | 3 | 2,100 |

Per-scenario paired comparisons and 95% intervals use the established whole-scenario
bootstrap (2,000 resamples, seed 2027). Correlated delay trials remain together.
The original full protocols and previous outputs remain preserved; this uses a
separate freeze and output folder. Matching original 2 s sources are archived in
`results/evaluation/preparation/source_2s_v2_freeze.zip`.

From `examples/staleguard`, launch:

```bash
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py \
  --reduced --calibration underpass_inspection/results/evaluation/calibration \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_reduced_100x13_v3.json \
  --out underpass_inspection/results/evaluation/baseline_benchmark
```

Resume:

```bash
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py \
  --reduced --calibration underpass_inspection/results/evaluation/calibration \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_reduced_100x13_v3.json \
  --out underpass_inspection/results/evaluation/baseline_benchmark --resume
```

Metrics only, without simulation:

```bash
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py \
  --reduced --calibration underpass_inspection/results/evaluation/calibration \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_reduced_100x13_v3.json \
  --out underpass_inspection/results/evaluation/baseline_benchmark --metrics-only
```

Validate completed results, including recomputing the fixed random sample:

```bash
conda run --no-capture-output -n scg python underpass_inspection/support/validate_baseline_benchmark.py \
  --calibration underpass_inspection/results/evaluation/calibration \
  --run underpass_inspection/results/evaluation/baseline_benchmark \
  --prior-hashes underpass_inspection/results/evaluation/preparation/historical_artifact_hashes_reduced_v3.json
```

Validation: 43 focused tests and 42 development-only smoke flights (seeds 1 and 3,
D=0, 1 and 2 s, all seven policies), source/hash preservation, completed-run resume
with zero added flights and byte-identical metrics regeneration. The reduced
held-out run has since completed all **9,100 flights**, and post-relocation
validation confirms the recorded metrics and physical traces. See the
[completed report](results/evaluation/baseline_benchmark/REPORT.md) and
[original preparation report](results/evaluation/preparation/PREPARATION_reduced_v3.md).

## Archived full 2-second held-out protocol

The existing evaluation calibration remains **865 accepted scenarios from 1,000
attempts**, unchanged. The new physical dispatch grid is 0–2.00 s every 0.02 s:
**865 × 101 × 7 = 611,555 actual flights** (the requested 611,455 total was an
arithmetic error). The predictor still evaluates exactly its original 19 candidate
delays through 1.00 s, using the same two-stage rollouts, full observation horizon,
readiness rules and computational workload. All seven policy source files are
byte-identical to the original 1 s freeze.

Beyond 1 s, the unchanged StaleGuard gate still rejects a ready finite TTL at or
after expiration. If no expiration was predicted, it continues permitting dispatch
beyond 1 s; this is not a prediction that such a dispatch is safe. An unavailable
forecast also permits dispatch. EarlyRecovery still requires readiness and a
finite expired TTL, and never cancels a command already started. No policy learns
the future actual dispatch delay from the experimental 2 s scheduling bound.

The eight-second observation deadline is unchanged. At D=2 s the finite dive
reference profile ends at approval +5.8 s, leaving 2.2 s of observation. An
inspection not observed by the deadline remains incomplete within that horizon;
it is not claimed impossible later. Abandonment and truncated reference profiles
are recorded separately.

One physical journal is analyzed over three windows:

| Analysis window | Delays per scenario | Physical flights |
|---|---:|---:|
| Full, 0–2.00 s | 101 | 611,555 |
| Within prediction grid, 0–1.00 s | 51 | 308,805 |
| Beyond prediction grid, 1.02–2.00 s | 50 | 302,750 |

Per-scenario paired comparisons and 95% percentile bootstrap intervals resample
whole scenarios (2,000 resamples, seed 2027), preserving all paired delay trials.
They never treat individual delay trials as independent scenarios. These are
analysis subsets of one benchmark. `per_delay.csv` retains one row per physical
flight; `per_scenario.csv`, metrics and reports label the analysis windows.

The original [1 s freeze](results/evaluation/preparation/freeze.json), reports and all historical
artifacts remain unchanged. Its original frozen sources are also retained in
`results/evaluation/preparation/source_1s_freeze.zip`; source hashes remain strict, so resuming an
old protocol requires its matching source version. The new
[freeze](results/evaluation/preparation/freeze_2s_v2.json) and
[preparation report](results/evaluation/preparation/PREPARATION_2s_v2.md) describe this version.

From `examples/staleguard`, launch:

```bash
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py \
  --calibration underpass_inspection/results/evaluation/calibration \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_2s_v2.json \
  --out underpass_inspection/results/evaluation/baseline_benchmark_2s_v2
```

Resume:

```bash
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py \
  --calibration underpass_inspection/results/evaluation/calibration \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_2s_v2.json \
  --out underpass_inspection/results/evaluation/baseline_benchmark_2s_v2 --resume
```

Regenerate metrics without simulation:

```bash
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py \
  --calibration underpass_inspection/results/evaluation/calibration \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_2s_v2.json \
  --out underpass_inspection/results/evaluation/baseline_benchmark_2s_v2 --metrics-only
```

Validate completed results without simulation:

```bash
conda run --no-capture-output -n scg python underpass_inspection/support/validate_baseline_benchmark.py \
  --calibration underpass_inspection/results/evaluation/calibration \
  --run underpass_inspection/results/evaluation/baseline_benchmark_2s_v2 \
  --prior-hashes underpass_inspection/results/evaluation/preparation/historical_artifact_hashes_2s_v2.json
```

Validation for this extension was development-only: 40 focused tests and 42 actual
flights (seeds 1 and 3, D=0, 1 and 2 s, all seven policies). Tests verified FBS
triggering at 2 s, finite-TTL and no-expiration behavior beyond 1 s, retention of
violations before intervention, controller continuity, full 1,000 Hz monitoring,
resumption and byte-identical metrics regeneration. No reference profile was
truncated in this smoke. No new held-out policy flights were run. The complete
2 s benchmark has not been launched.

## Creating a held-out calibration set

The evaluation split uses the same frozen sampling ranges and immediate-flight
acceptance tests as development. Seeds 0–20 remain reserved for the pilot and all
development attempts, including rejected ones. Evaluation requires seeds >=21
and cannot write inside `results/development/`. Its default output is
`results/evaluation/calibration/`. The manifest records the split, attempted seed
list, sampling/acceptance protocol, source/environment hashes, rejected attempts
and accepted approval snapshots. Existing development manifests still load.

From `examples/staleguard`, this command attempts **100 scenarios, seeds
1001–1100**, with no replacements. It does not promise 100 accepted scenarios:

```bash
conda run --no-capture-output -n scg python underpass_inspection/calibration.py \
  --split evaluation --start-seed 1001 --count 100 \
  --out underpass_inspection/results/evaluation/calibration
```

To resume, repeat the exact command with `--resume`. Do not change the seed list,
split, source or environment between invocations. `--count` counts attempted
seeds; `--max-scenarios` limits work in one invocation without changing that list.
An explicit `--seeds` list is also supported, but cannot be combined with
`--start-seed` or `--count`.

Calibration runs only nominal and immediate dive/sideways/climb flights, plus a
repeat immediate dive for each accepted scenario. Climb remains diagnostic.
There are no positive-delay sweeps, TTL predictions or policy comparisons during
selection. Freeze policy choices before inspecting held-out benchmark outcomes.
No evaluation calibration or policy flights were launched while adding this
support; tests use synthetic flight outcomes in temporary directories only.

## Development calibration and independent deadlines

Each seed produces exactly one draw. Calibration accepts it only if nominal
flight becomes unsafe, immediate dive completes inspection safely, and immediate
Eastward sideways recovery is safe. Immediate climb is measured separately and
cannot affect acceptance. Rejected draws remain in the manifest; there is no
replacement, approval-time search, or use of positive delays during calibration.

The provisional ranges were fixed before the development timing sweeps:

| Sampled parameter | Distribution / range | Units |
|---|---|---|
| Forward speed | Uniform 0.55–0.65 | m/s |
| Approval after flight start | Uniform 0.70–0.90, rounded to 0.02 | s |
| Deck front, world X / North | Uniform 1.35–1.65 | m |
| Pier center, absolute world X / North | Independent uniform 2.35–3.25 | m |

The NumPy `default_rng(seed)` draw order is speed, approval, deck front, pier X.
Speed, approval and front vary modestly around seed 0. The pier is sampled in a
common interior segment behind every possible inspection region; even its rear
edge fits the shortest deck. Its position is independent of the front, rather
than a fixed offset from it. The 2.5 m deck length stays fixed. The inspection
region translates with the deck front; its dimensions, altitude and dwell stay
fixed. Maneuver parameters, body radius, safety margin, control/physics rates and
the eight-second observation period after approval remain unchanged. These
ranges are design assumptions, not evidence of real-world scenario prevalence.

Run from `examples/staleguard` using the existing `scg` environment:

```bash
conda run --no-capture-output -n scg python underpass_inspection/calibration.py
conda run --no-capture-output -n scg python underpass_inspection/support/deadline_sweep.py
```

The default folders are `results/development/calibration/` and
`results/development/deadline_sweep/` beneath this mission. Existing outputs
require `--resume`; use `--out /tmp/fresh_name` for a separate reproduction and
pass that calibration folder with `--calibration` to the sweep. `--seeds 1 2 3`
can be used for a small calibration. `--max-scenarios N` and `--max-flights N`
limit work per invocation without changing the frozen protocol. Resume skips
committed work and rejects changed sources, settings or committed journal data.
It repairs only an unfinished final journal entry. A single flight interrupted
before it is committed may be repeated; completed flights are never duplicated.

For every accepted scenario, independent actual PyBullet flights test dive and
sideways starts from 0 through 1.00 s in 20 ms steps. The separate climb diagnostic
extends through 1.50 s on the same grid, keeping the original observation horizon.
`--climb-max 1` is available for a separately frozen shorter protocol. Every
flight monitors safety from reset through approval plus eight seconds, including
before initiation and after a violation. The side maneuver is the primary
safety-only recovery; climb never substitutes for it automatically.

The command threshold is the first tested unsafe dive delay. Side recovery is
reported with the last tested safe start and whether a subsequent unsafe start
was observed. If a grid never fails, expiration is unobserved, not equal to its
endpoint. Full safe/unsafe intervals expose any unsafe-to-safe return. Inspection
completion and whole-flight safety are reported separately. No positive gap
establishes prediction accuracy or policy performance.

Recompute derived tables and the report from the journal without new flights:

```bash
conda run --no-capture-output -n scg python underpass_inspection/support/deadline_sweep.py --metrics-only
```

The manifest freezes all attempted definitions, rejection reasons, immediate
flight records, source/environment provenance, and verified approval snapshots
for accepted scenarios. The sweep saves `run.json`, `executions.jsonl`,
`metrics.json`, `per_scenario.csv`, `per_delay.csv`, `REPORT.md`, and one compressed
file of representative actual trajectories. It does not save every flight path.

To view an accepted development scenario with the existing NED GUI:

```bash
conda run --no-capture-output -n scg python underpass_inspection/pilot.py --calibration underpass_inspection/results/development/calibration --seed 1 --mode dive --delay 0.2 --gui --overlay-all --hold 10
```

The accepted-seed loader refuses rejected seeds. For calibrated `--overlay-all`,
the other three paths come from actual fresh replays of that same frozen scenario
(nominal, dive at 0, climb/sideways at 0.5 s, replacing the selected mode's start
with the requested time). Those explanatory replays do not alter the scientific
journal or select a recovery. The checkerboard, camera and NED labels are unchanged.

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
predictive policies. A custom scenario or unrecorded seed-0 start cannot use the
archived overlay; accepted calibrated cases use actual fresh replays as above.
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
`--scenario` for an explicitly separate experiment. The development generator
is described above; no maneuver or approval-time search is implemented.
Moving a pier never changes a maneuver shape.

Maneuver joins have continuous position, velocity and acceleration. At initiation,
the required measured-position anchor differs slightly from the old nominal
reference. This small reference splice is recorded, and is not a physical state
jump or controller reset. Exact continuity to the old nominal reference and exact
measured-position anchoring cannot both hold when nominal tracking error is nonzero.

## Validation without new flights

```bash
conda run --no-capture-output -n scg python -m unittest underpass_inspection.support.test_pilot underpass_inspection.support.test_development -v
conda run --no-capture-output -n scg python underpass_inspection/support/deadline_sweep.py --metrics-only
```

The archived seed-0 reports and validation files remain historical evidence.
Development validation independently checks the four seed-0 immediate physical
trajectory hashes and outcomes; see `deadline_sweep/validation/` for those checks
and the final journal/snapshot audit. The older pilot validator writes into its
input folder and requires the exact historical source hashes, so it should not
be used to overwrite the preserved pilot validation after development edits.

One simulation worker and one numerical thread are used. The shared memory guard
retains a 2 GiB process limit and 2 GiB available-host-memory reserve. A mission lock
prevents overlapping pilot CLI runs. Existing mission code and results are untouched.

## Exploratory recovery-aware extension

Two additional policies share an optimized approval-time predictor. It evaluates
all original 19 delays through 1 s, predicting command safety, safe inspection
completion, and the independent Eastward sideways recovery. It reuses nominal
waiting/PID checkpoints, runs the unchanged numerical kernels in compiled loops,
and stops a continuation only after whole-flight safety has irreversibly failed.
It preserves every candidate outcome, including non-monotonic and censored profiles.
No online forecast reads physical counterfactual labels or the future dispatch time.

`staleguard_recovery_aware` triggers on the earlier of command expiration and the
last safe sampled recovery-prefix point. `staleguard_dual_horizon` also considers
safe-completion expiration. Both wait for the entire measured computation and
50 Hz scheduling. Dispatch wins timer ties; equality at an available command
expiration rejects. A late result never acts retroactively. Once the recovery
prefix is missed, neither policy forces recovery merely because recovery is
predicted unsafe; an expired command uses the existing fallback as best-effort.
Completion-only abandonment requires remaining predicted recovery feasibility.
All-safe sampled profiles have unobserved expiration; no 1 s rejection is invented.
Sampled prefixes do not establish safety between prediction-grid points or beyond
1 s. The unavailable-prediction rule remains fail-open.

The two policies independently compute/time the same forecast per scenario;
there is no cross-policy forecast cache. Full publication includes waiting, both
continuations, completion monitoring, extraction and cleanup. Compilation warmup
uses a discarded development snapshot and is recorded separately. Recovery is
computed first, but its intermediate result is diagnostic, never prematurely used.

The [development validation report](results/development/recovery_horizons/REPORT.md)
and [technical appendix](results/development/recovery_horizons/TECHNICAL_APPENDIX.md)
contain timing, physical prediction errors and limitations. The existing 9,100
flights remain unchanged. The extension executes **only 2,600 new flights**, on
exactly the same frozen 100 scenarios and 13 delays. A nine-policy merge is
**exploratory**, because these scenarios have already been examined.

From `examples/staleguard`, in the existing `scg` environment:

```bash
# Launch only the two new policies. Defaults use results/evaluation/calibration
# and the completed results/evaluation/baseline_benchmark as the paired source.
python underpass_inspection/baseline_benchmark/horizons.py \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_recovery_horizons_v1.json \
  --out underpass_inspection/results/evaluation/recovery_horizons_benchmark_v1

# Resume the identical run; uncommitted tail bytes are rolled back.
python underpass_inspection/baseline_benchmark/horizons.py \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_recovery_horizons_v1.json \
  --out underpass_inspection/results/evaluation/recovery_horizons_benchmark_v1 --resume

# Validate completion and deterministic metrics regeneration (no simulation).
python underpass_inspection/baseline_benchmark/horizons.py \
  --out underpass_inspection/results/evaluation/recovery_horizons_benchmark_v1 --validate

# Merge the unchanged 9,100 flights and completed 2,600 new flights; no simulation.
python underpass_inspection/baseline_benchmark/horizons.py --merge \
  --new-run underpass_inspection/results/evaluation/recovery_horizons_benchmark_v1 \
  --out underpass_inspection/results/evaluation/nine_policy_recovery_horizons_v1

# Regenerate the merged reports, metrics and CSVs without simulation.
python underpass_inspection/baseline_benchmark/horizons.py \
  --out underpass_inspection/results/evaluation/nine_policy_recovery_horizons_v1 --metrics-only

# Validate the merged journal and its two source journals.
python underpass_inspection/baseline_benchmark/horizons.py \
  --out underpass_inspection/results/evaluation/nine_policy_recovery_horizons_v1 --validate
```

Each new output retains `run.json`, `executions.jsonl`, `metrics.json`,
`per_scenario.csv`, `per_delay.csv`, `REPORT.md`, `TECHNICAL_APPENDIX.md` and
validation results. Only representative new physical trajectories are saved.
Metrics stream one scenario at a time. Paired differences and 95% bootstrap
intervals resample whole scenarios, with full / <=1 s / >1 s analyses. Prediction
quality is explicitly limited to matching physical labels: command/completion
labels use archived No Recovery flights; recovery labels use realized matching
starts, without inferring an independent recovery deadline from policy outcomes.

Focused tests and development-only smoke commands are in the technical appendix.
The prepared freeze verifies source/environment and the paired calibration/protocol;
changing scientific code requires a new versioned freeze/output, not editing an
existing result header. The two-policy held-out experiment is prepared, not run.
