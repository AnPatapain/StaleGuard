# Geofence turn

The UAV patrols east, executes its approved fixed-radius left turn when
dispatched, then continues north through a fixed checkpoint. Waiting moves the
turn east; the radius, checkpoint and deadline stay fixed. The six supported
baselines share the same SCG/PyBullet flight loop and physical mission conditions.

| Code | Responsibility |
|---|---|
| `scenario.py` | Scenario generation, requested trajectories, geometry and completion conditions |
| `calibration.py` | Direct generate → nominal probe → descending D=0 search → verify → save loop |
| `storage.py` | Shared JSON/snapshot storage and validated loading of current and historical datasets |
| `simulation.py` | Shared flight execution and 1000 Hz safety/completion monitoring |
| `baselines/` | StaleGuard, Use-Time Recheck, Fixed TTL, Always-Recover and FBS; No Recovery is the default simulation path |
| `baseline_benchmark/run.py` | Existing four-policy comparison |
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
│   │   └── run.json
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
| `metrics.json` | Derived outcomes, rates, recovery statistics, timing summaries and paired StaleGuard/Use-Time Recheck comparisons |
| `run.json` | Fixed protocol, source/configuration provenance, snapshot checksums, frozen approval predictions and their measured timing, status and resume information |

The flight record retains `seed`, `delay_s`, `safe`, `completed`,
`safe_completion`, clearance/completion/violation measurements, `event`,
`runtime_s`, and the other existing scientific fields. Prediction readiness,
computation duration and dispatch/application/recovery ticks stay inside `event`.

Metrics are written at completion or interruption. Progress remains visible in
the terminal and the growing journal. Failures and sessions are recorded inside
the same run JSON; there are no separate progress, failure, completion or
per-scenario prediction files. A shared ignored simulation lock and transient
atomic-write files are runtime housekeeping.

**Benchmark traces require `--diagnostics`.** That option adds individual flight
traces under the benchmark's `diagnostics/` directory. Calibration has no optional
trajectory or candidate-log output. Recorded traces are at 50 Hz; safety and mission
completion continue to be checked at 1000 Hz regardless of trace saving.

Historical files remain under `support/logs/` and `support/results/`. The original
20-case dataset remains readable in place. A compact byte-for-byte snapshot copy
is now available at `results/development/calibration/`; no recalibration was done.
This copy retains **20 accepted / 22 attempted**, including the two rejections.

## Run the existing workflow

Run from `examples/staleguard` using the existing SCG environment:

```sh
conda activate scg
```

The benchmark defaults to the compact 20-case development calibration. For a
small comparison using two of those cases:

```sh
python geofence_turn/baseline_benchmark/run.py \
  --seeds 0 3 --delays 0 0.6 --max-trials 16 --max-seconds 120
```

This runs all **four** existing policies: 2 cases × 2 delays × 4 policies = 16
flights. The defaults remain all four policies and delays 0, 0.2, 0.6 and 1.0 s.
Use `--policies` to select implemented policies and `--delays` for dispatch
delays on the 0.02 s control grid.

`--policies always_recover fbs` selects the two additional baselines; the default
remains the original four. Always-Recover rejects the turn at scheduled dispatch,
including D=0, and activates the same PID-controlled safety-only recovery.

FBS uses actual 50 Hz observations from reset through approval. Its signed
violation feature is `f = d_min_m - clearance(x, fence_x)`, with safe-side margin
`m = -f`. It directly reuses Inspection-Descent's EMA and dispatch rule:
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
with `python -m unittest geofence_turn.support.tests.test_new_baselines -v`.

Historical results and hashes remain unchanged, and experiment resumes require
matching provenance. TTL accuracy can reuse older No-Recovery flights after
checking that the No-Recovery execution path, scenario, snapshot, model, PID,
vehicle asset and numerical environment still match. Adding an unrelated policy
branch does not invalidate those flights; changing dispatch or safety monitoring
does. Unknown historical simulator versions are rejected.

Use `--out` to select another output directory and `--development` to select a
calibration directory or scenario JSON. Fresh output must be absent or empty.
Use `--resume` with the same inputs, protocol and source to continue an
interrupted run. It skips committed flights and reuses their original approval
prediction times. Duplicate records, changed committed data and incompatible
source/configuration/protocol are rejected. An unfinished final JSONL line can
be removed on resume; complete records are preserved.

## Simple calibration

`calibration.py` has one job. For each seed it samples one fixed mission using
`scenario.generate_scenario()`, flies straight east to measure the first fence
violation, and rounds that time up to the 0.20 s approval grid. It tests approvals
from that measured bound backwards to 0.60 s, accepting the first actual D=0
No-Recovery flight with `safe_completion`. It repeats that flight to verify the
UAV/PID state, remaining reference and command before saving it. Speed, radius,
checkpoint and deadline duration remain frozen throughout the search.

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
python geofence_turn/calibration.py \
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
python geofence_turn/ttl_pred_accuracy/run.py
python geofence_turn/ttl_pred_accuracy/metrics.py \
  geofence_turn/results/development/ttl_pred_accuracy/executions.jsonl

# Evaluation uses all 300 frozen scenarios. This may need several resumable batches.
python geofence_turn/ttl_pred_accuracy/run.py \
  --calibration geofence_turn/results/evaluation/calibration \
  --max-trials 200 --max-seconds 1800
# Repeat the same command with --resume after a resource-budget interruption.
```

The runner loads frozen scenarios and approval snapshots, then obtains actual
No-Recovery outcomes at the same 19 delays. For development, it finds and verifies
all 380 saved flights through the calibration dataset's source reference. For
evaluation, it can reuse the 300 saved D=0 calibration verification flights.
It also looks for a sibling baseline benchmark journal; `--ground-truth PATH`
can supply another compatible flight journal, and may be repeated. Invalid or
conflicting saved data cause an error rather than being replaced silently.

Each scenario gets one approval forecast using the unchanged lightweight model
and prediction grid. Only missing actual outcomes are simulated, using the
existing No-Recovery flight and full-horizon safety monitor. The runner compares
the forecast with each actual outcome, appends one journal record, and calculates
metrics. The model kernels are warmed with one control step before measuring
call-to-result time. No recovery policy is executed.

It produces exactly three files: `executions.jsonl` has one comparison per
scenario/delay; `metrics.json` separates raw model accuracy at its 11 directly
evaluated delays from TTL dispatch-decision accuracy at all 19 delays; `run.json`
stores provenance, compatibility checks, cached forecasts, any newly simulated
No-Recovery flight records, resume state and the readable final report in
`report_markdown`. Off-grid raw predictions are `null`.
Violation times in the comparison journal are relative to approval; the original
flight-start timestamps are retained as `first_violation_flight_s`.

Use `--resume` to continue without recomputing cached forecasts, or `--out` for
a fresh directory. By default, output is `ttl_pred_accuracy/` beside the supplied
calibration directory; `--development` remains an alias for `--calibration`.
Existing output is never overwritten by a fresh run. Historical completed runs
remain readable but cannot resume across source changes. The default resource
budget is 40 forecasts/flights and 300 seconds per invocation; the existing
maximum is 200 forecasts/flights and 1800 seconds. Cached forecasts and newly
obtained ground truth survive interruption, including within a scenario. The
metrics-only command reads the journal and adjacent fixed protocol, without
importing or running PyBullet. Prediction readiness and mission completion are
not the prediction target; actual completion counts are retained for context.

## Regenerate and understand metrics

```sh
python geofence_turn/baseline_benchmark/metrics.py \
  geofence_turn/results/development/baseline_benchmark/executions.jsonl
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
corresponding records. Dispatch-to-action delay uses actual application or
fallback ticks minus scheduled dispatch ticks, divided by 50. A decision tick
without an actual action is not treated as an action measurement. Timing units
are seconds; p95 uses nearest rank, and sample counts are included.

The paired comparison matches StaleGuard and Use-Time Recheck by `(seed, delay_s)`.
It counts completion benefits/harms, unsafe executions prevented/introduced and
net count differences. Incomplete pairs are excluded and counted separately;
when the full protocol is available, planned but unstarted groups are also
identified. Per-policy metrics still include every recorded flight.

## Focused validation and reports

```sh
python -m unittest geofence_turn.support.tests.test_mission \
  geofence_turn.support.tests.test_prediction \
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
