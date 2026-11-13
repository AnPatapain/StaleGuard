# Geofence turn

The UAV patrols east, executes its approved fixed-radius left turn when
dispatched, then continues north through a fixed checkpoint. Waiting moves the
turn east; the radius, checkpoint and deadline stay fixed. The six supported
baselines share the same SCG/PyBullet flight loop and physical mission conditions.

| Code | Responsibility |
|---|---|
| `scenario.py` | Scenario generation, requested trajectories, geometry and completion conditions |
| `calibration.py` | Direct generate → nominal probe → descending D=0 search → verify → save loop |
| `storage.py` | Shared JSON/snapshot storage; `load_calibrated_scenarios()` loads the supplied development or evaluation dataset |
| `simulation.py` | Shared flight execution and 1000 Hz safety/completion monitoring |
| `baselines/` | StaleGuard, Use-Time Recheck, Fixed TTL, Always-Recover and FBS; No Recovery is the default simulation path |
| `baseline_benchmark/run.py` | Six-policy comparison on identical scenarios and dispatch delays |
| `baseline_benchmark/metrics.py` | Metric definitions and summaries from saved flight records |
| `ttl_pred_accuracy/` | Direct model and TTL-decision accuracy against saved No-Recovery flights |
| `support/` | Diagnostics, tests, reports and historical artifacts |

The lightweight dynamics model is reused from `inspection_descent/model/`.

## Workflow and artifacts

The research workflow is **calibration → prediction accuracy / baseline benchmark**. Development and
future evaluation use the same layout:

```text
geofence_turn/results/
├── development/
│   ├── calibration/
│   │   ├── calibrated_scenarios.json
│   │   └── snapshots/seed_<id>.pkl
│   ├── ttl_pred_accuracy/
│   │   ├── executions.jsonl
│   │   ├── metrics.json
│   │   ├── run.json
│   │   └── accuracy.log
│   └── baseline_benchmark/
│       ├── executions.jsonl
│       ├── metrics.json
│       └── run.json
└── evaluation/
    ├── calibration/                # Same convention
    ├── ttl_pred_accuracy/          # Same artifact convention; evaluation not run
    └── baseline_benchmark/         # Same convention
```

**Calibration has one readable JSON plus the necessary binary snapshots.** The
JSON contains accepted parameters and approval times, snapshot references and
checksums, configuration, source provenance, attempted/accepted/rejected counts,
and calibration status. The acceptance rate uses the **attempted-seed denominator**,
excluding seeds never reached before the target. Progress is committed after each
completed scenario, including rejections. On interruption, accepted scenarios are
retained; only the unfinished scenario is recalibrated when resuming.

A snapshot stores the UAV state, PID memory and remaining reference at approval.
These are needed for repeatable prediction and validation. There are no default
per-scenario configuration files, candidate journals or trajectory files.

**The baseline benchmark produces exactly three default files:**

| File | Meaning |
|---|---|
| `executions.jsonl` | Authoritative data: one unchanged-schema JSON record per completed flight, appended and flushed to disk |
| `metrics.json` | Derived outcomes, recovery/timing summaries and paired StaleGuard comparisons against all five other policies |
| `run.json` | Fixed protocol, source/configuration provenance, snapshot checksums, frozen approval predictions and their measured timing, status and resume information |

The flight record retains `seed`, `delay_s`, `safe`, `completed`,
`safe_completion`, clearance/completion/violation measurements, `event`,
`runtime_s`, and the other existing scientific fields. Prediction readiness,
computation duration and dispatch/application/recovery ticks stay inside `event`.

**Timestamp origins differ between the two execution journals.** All entries
below are in seconds; the existing field names and values are preserved.

| Field | Baseline benchmark | TTL accuracy |
|---|---|---|
| `first_violation_s` | Time since flight start | Time since approval |
| `pre_dispatch_violation_s` | Time since flight start | Not included in the comparison record |
| `completion_s` | Time since approval | Time since approval |

TTL accuracy also retains the original violation timestamp as
`first_violation_flight_s`. Illustrative example: if approval occurs at 2.0 s
and the UAV violates clearance at flight time 2.3 s, the benchmark records
`first_violation_s = 2.3`, while accuracy records `first_violation_s = 0.3`
and `first_violation_flight_s = 2.3`. Both describe the same event.
The benchmark's `horizon_s` is the final time since flight start; its deadline
duration is `scenario.deadline_s` since approval. Dispatch/application/recovery
steps in `event` are flight-start control ticks, at 50 Hz.

Benchmark metrics are written at completion or interruption. Progress remains visible in
the terminal and the growing journal. The run JSON retains the fixed protocol,
cached approval forecasts, provenance and complete/partial status; there is no
session manager or separate progress, failure or completion file. A shared ignored simulation lock and transient
atomic-write files are runtime housekeeping.

The benchmark CLI saves no per-flight trajectories and has no `--diagnostics`
option. For focused diagnostics, `simulation.execute(..., save_trace=True)`
returns a 50 Hz trace and includes FBS observation history in `event`; it does not
write a trace file. Safety/completion monitoring always runs at 1000 Hz.

Historical files remain under `support/logs/` and `support/results/`. The original
20-case dataset remains readable in place. A compact byte-for-byte snapshot copy
is now available at `results/development/calibration/`; no recalibration was done.
This copy retains **20 accepted / 22 attempted**, including the two rejections.

## Run the existing workflow

Run from `the repository root` using the existing SCG environment:

```sh
conda activate scg
```

The benchmark defaults to the compact 20-case development calibration. For a
small comparison using two of those cases:

```sh
python -m staleguard.geofence_turn.baseline_benchmark.run \
  --seeds 0 3 --delays 0 0.24 \
  --out staleguard/geofence_turn/results/development/baseline_benchmark_smoke
```

This runs all **six policies**: 2 scenarios × 2 delays × 6 policies = 24 flights.
The default policies are No Recovery, Always-Recover, Fixed TTL, FBS, Use-Time
Recheck and StaleGuard. The default dispatch grid contains **23 delays**:

```python
0., .02, .04, .06, .08, .10, .12, .14, .16, .18, .20,
.24, .30, .40, .44, .50, .60, .64, .70, .80, .84, .90, 1.
```

StaleGuard forecasts on the frozen **19-point grid**, which excludes the four
off-grid dispatch delays `.24, .44, .64, .84`. The actual dispatch delay remains
unknown when forecasting at approval. `--policies` selects policies and `--delays`
selects dispatch delays on the 0.02 s control grid. With all defaults, development
uses 20 × 23 × 6 = **2,760 flights**; evaluation uses 300 × 23 × 6 = **41,400 flights**.

Use `--calibration staleguard/geofence_turn/results/evaluation/calibration` to select the
evaluation dataset. Both datasets use the same `load_calibrated_scenarios()`
function; `--development` is a historical alias for `--calibration`.

`--policies always_recover fbs` selects just those two policies.
Always-Recover rejects the turn at scheduled dispatch,
including D=0, and activates the same PID-controlled safety-only recovery.

FBS uses actual 50 Hz observations from reset through approval. Its signed
violation feature is `f = d_min_m - clearance(x, fence_x)`, with safe-side margin
`m = -f`. Its self-contained module implements the same EMA and dispatch rule as Inspection-Descent:
alpha 0.1, initial volatility zero, epsilon 1e-12 m/sample, 210-step cap,
and expiry when `m <= 0` or `age_steps > ttl_steps`. The approval update and
TTL calculation are timed; the result is frozen at approval. At dispatch,
ready means `computation_s <= age_steps / 50`. An unavailable result executes
the original turn on schedule with no later cancellation. Available expired
results activate safety-only recovery immediately at dispatch. All diagnostic
fields remain inside the flight's `event` object; no new default artifacts exist.

The feature describes current clearance and does not certify the future turn.
The 210-step cap and numerical epsilon are retained from the existing adaptation;
no parameters were tuned to Geofence outcomes. The new baseline tests can be run
with `python -m unittest staleguard.geofence_turn.support.tests.test_new_baselines -v`.

Historical results and hashes remain unchanged, and experiment resumes require
matching provenance. TTL accuracy can reuse older No-Recovery flights after
checking that the No-Recovery execution path, scenario, snapshot, model, PID,
vehicle asset and numerical environment still match. Adding an unrelated policy
branch does not invalidate those flights; changing dispatch or safety monitoring
does. Unknown historical simulator versions are rejected.

Use `--out` to select another output directory. A new run requires absent/empty
output; rerunning the same command continues matching saved output automatically.
There is no `--resume` option. `--max-flights N` stops after N new flights;
rerun the same command to continue. Its default, 0, runs every remaining flight.
Completed flights are skipped and cached approval forecasts keep their measured
times. Duplicate records, changed committed data and incompatible source,
configuration or protocol are rejected. An unfinished final JSONL line can be
discarded on continuation. There are no `--max-trials` or `--max-seconds` options.

## Simple calibration

`calibration.py` has one job. For each seed it samples one fixed mission using
`scenario.generate_scenario()`, flies straight east to measure the first fence
violation, and rounds that time up to the 0.20 s approval grid. It tests approvals
from that measured bound backwards to 0.60 s, accepting the first actual D=0
No-Recovery flight with `safe_completion`. It repeats that flight to verify the
UAV/PID state, remaining reference and command before saving it. Speed, radius,
checkpoint and deadline duration remain frozen throughout the search.

`Config.approval_max_s = 3.20` is a legacy serialized field retained so frozen
datasets and snapshots remain compatible. It is **unused by current calibration**:
the measured nominal-flight bound determines the latest candidate, which can
exceed 3.20 s. `approval_min_s` and `approval_grid_s` still control the search.

The characterized speed–radius data is reused from
`support/logs/northbound_review/characterization/envelope.json`. Characterization,
standalone generation, export and replay modes were removed. The main scientific
loop is `calibrate_scenarios()`; `find_nominal_bound()`, `approval_candidates()` and
`verify_calibration()` are its three physical/search helpers. File operations and
existing experiment provenance checks live in the single `storage.py` module.
There are no callbacks, candidate caches, recursive source scans or automatic
batch orchestration. The independent evaluation dataset remains frozen at
**300 accepted / 306 attempted**, beginning at seed 22.

For a small future calibration, choose unused seeds and a fresh output directory:

```sh
python -m staleguard.geofence_turn.calibration \
  --start-seed 328 --attempts 10 --target 5 \
  --out /tmp/new_geofence_calibration
```

`--attempts` counts candidate scenarios; `--target` counts accepted scenarios.
A larger target uses the same loop and arguments, without `--evaluation` or
per-batch flags. Existing nonempty outputs are rejected. Add `--resume` to the
same command after an interruption: completed seeds are skipped in their original
order and the unfinished seed is restarted from its nominal probe. Input settings,
characterized-data checksum, configuration, source provenance and saved snapshots
are validated. Historical frozen datasets remain loadable; their recorded source
hashes are not rewritten to match this refactor.

Calibration remains serial, CPU-only and headless. The simulator's existing
2 GiB RSS and minimum-available-memory guards remain active. Errors stop the run
with completed-scenario checkpoints preserved; the old trial/time batching limits
and candidate-level resume machinery were removed. Each accepted scenario normally
uses one probe, its approval tests and one verification flight. No positive-delay
or policy outcome influences calibration.

Evaluation uses its own frozen calibration input and output under
`results/evaluation/`. TTL accuracy reads the supplied dataset's seeds and keeps
the same 19 dispatch delays for development and evaluation.

## TTL prediction accuracy

```sh
python -m staleguard.geofence_turn.ttl_pred_accuracy.run \
  --out staleguard/geofence_turn/results/development/ttl_pred_accuracy_new
python -m staleguard.geofence_turn.ttl_pred_accuracy.metrics \
  staleguard/geofence_turn/results/development/ttl_pred_accuracy_new/executions.jsonl

# Evaluation uses all 300 frozen scenarios in one run.
python -m staleguard.geofence_turn.ttl_pred_accuracy.run \
  --calibration staleguard/geofence_turn/results/evaluation/calibration
```

The runner loads frozen scenarios and approval snapshots, then obtains actual
No-Recovery outcomes at the same 19 delays. For development, it finds and verifies
all 380 saved flights through the calibration dataset's source reference. For
evaluation, it can reuse the 300 saved D=0 calibration verification flights.
It also looks for a sibling baseline benchmark journal; `--ground-truth PATH`
can supply another compatible flight journal, and may be repeated. If saved
automatically discovered ground truth is incompatible, the runner logs the reason
and executes fresh No-Recovery flights. Incompatible explicitly supplied ground
truth raises an error. Neither path uses incompatible saved outcomes.

Each scenario gets one approval forecast using the lightweight model on the
frozen **19-point prediction grid**, identical to its 19 dispatch delays. Only
missing actual outcomes are simulated, using the
existing No-Recovery flight and full-horizon safety monitor. The runner compares
the forecast with each actual outcome, appends one journal record, and calculates
metrics. The model kernels are warmed with one control step before measuring
call-to-result time. No recovery policy is executed.

It produces four files: `executions.jsonl` has one comparison per scenario/delay;
`metrics.json` separates raw model safety predictions from TTL dispatch decisions,
both evaluated at all 19 delays; `run.json` contains the input dataset, protocol,
prediction grid, measured forecast information and ground-truth reuse counts;
`accuracy.log` saves the terminal report on termination. Metrics and run metadata
are written at completion. There are no checkpoints, sessions or resume state.
Violation times in the comparison journal are relative to approval; the original
flight-start timestamps are retained as `first_violation_flight_s`.

The predictor function's default argument still uses the legacy **11-point,
0.10 s grid** for direct calls. Both current experiment runners explicitly pass
their frozen **19-point grid**. This distinction is intentional; inspect the grid
recorded in `run.json` when interpreting current or historical results.

Use `--out` to select a fresh directory. By default, output is
`ttl_pred_accuracy/` beside the supplied
calibration directory; `--development` remains an alias for `--calibration`.
The current TTL runner replaces output files at the selected path; choose a fresh
directory to keep an earlier run. There is no resume or trial/time batch management.
The journal is opened for writing and receives one record per comparison;
`metrics.json` and `run.json` are written after all comparisons finish.
CPU-only execution, memory guards and the shared simulation lock remain active. The
metrics-only command reads the journal and adjacent fixed protocol, without
importing or running PyBullet. Prediction readiness and mission completion are
not the prediction target; actual completion counts are retained for context.

## Regenerate and understand metrics

```sh
python -m staleguard.geofence_turn.baseline_benchmark.metrics \
  staleguard/geofence_turn/results/development/baseline_benchmark/executions.jsonl
```

This writes `metrics.json` beside the journal, without PyBullet. It uses the fixed
protocol from adjacent `run.json`. For historical journals without that file,
Geofence's fixed 50 Hz control frequency is used and policies/delays are read
from the records. `--out` can select a separate derived-metrics destination.

`Metric` in `metrics.py` lists the metrics with plain-English comments. Outcomes
are mutually exclusive: safe completion, safe non-completion, or unsafe flight.
Recovery attempts count actual fallback activation; a recovery succeeds only if
the whole flight stays safe, including before fallback. All rates are proportions
from 0 to 1; undefined rates and unavailable timing statistics are `null`.

Prediction timing uses only recorded `event.computation_s` values. StaleGuard's
cached approval forecast is counted once per seed in the overall timing summary;
Use-Time Recheck is counted per recorded flight. Per-delay summaries use the
corresponding records. FBS `prediction_computation_s` measures only its approval
feature/EMA update and TTL freeze. When diagnostic observation timers are present,
`fbs_cumulative_observation_computation_s` separately summarizes their sum through
approval; it is unavailable in default runs and excludes dispatch/recording overhead.
Dispatch-to-action delay uses actual application or
fallback ticks minus scheduled dispatch ticks, divided by 50. A decision tick
without an actual action is not treated as an action measurement. Timing units
are seconds; p95 uses nearest rank, and sample counts are included.

`paired_against_staleguard` matches StaleGuard with each selected alternative by
`(seed, delay_s)`. `paired_comparison` retains the StaleGuard/Use-Time Recheck summary.
It counts completion benefits/harms, unsafe executions prevented/introduced and
net count differences. Incomplete pairs are excluded and counted separately;
when the full protocol is available, planned but unstarted groups are also
identified. Per-policy metrics still include every recorded flight.

## Focused validation and reports

```sh
python -m unittest staleguard.geofence_turn.support.tests.test_mission \
  geofence_turn.support.tests.test_prediction \
  geofence_turn.support.tests.test_new_baselines \
  geofence_turn.support.tests.test_refactor_equivalence -v
```

The artifact/resume tests replay saved-result fixtures, including completed-scenario
interruption recovery, rather than running a new calibration or benchmark. Additional checks cover original trajectories, source equivalence,
real saved flight outcomes and unchanged predictions. See the
[artifact simplification report](support/reports/ARTIFACT_SIMPLIFICATION_REPORT.md),
[physical mission report](support/reports/PHASE1_REPORT.md), and
[delay-tolerance report](support/results/delay_tolerance_20/REPORT.md).

Historical reports under `support/reports/` describe their implementation at the
time of writing. The current calibration workflow is the short loop in
`calibration.py` described above.
