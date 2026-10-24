# Geofence turn

The UAV patrols east, executes its approved fixed-radius left turn when
dispatched, then continues north through a fixed checkpoint. Waiting moves the
turn east; the radius, checkpoint and deadline stay fixed. The four existing
baselines share the same SCG/PyBullet flight loop and physical mission conditions.

| Code | Responsibility |
|---|---|
| `scenario.py` | Scenario generation, requested trajectories, geometry and completion conditions |
| `calibration.py` | Original D=0 approval search, snapshot loading and export |
| `simulation.py` | Shared flight execution and 1000 Hz safety/completion monitoring |
| `baselines/` | StaleGuard, Use-Time Recheck and Fixed TTL; No Recovery is the default simulation path |
| `baseline_benchmark/run.py` | Existing four-policy comparison |
| `baseline_benchmark/metrics.py` | Metric definitions and summaries from saved flight records |
| `support/` | Diagnostics, tests, reports and historical artifacts |

Always-Recover, FBS and mission-run TTL prediction accuracy remain unfinished.
The lightweight dynamics model is reused from `inspection_descent/model/`.

## Workflow and artifacts

The research workflow is **calibration → baseline benchmark**. Development and
future evaluation use the same layout:

```text
geofence_turn/results/
├── development/
│   ├── calibration/
│   │   ├── calibrated_scenarios.json
│   │   └── snapshots/seed_<id>.pkl
│   ├── ttl_pred_accuracy/          # Reserved; no implementation or placeholder files
│   └── baseline_benchmark/
│       ├── executions.jsonl
│       ├── metrics.json
│       └── run.json
└── evaluation/
    ├── calibration/                # Same convention
    ├── ttl_pred_accuracy/          # Reserved; no implementation or placeholder files
    └── baseline_benchmark/         # Same convention
```

**Calibration has one readable JSON plus the necessary binary snapshots.** The
JSON contains accepted parameters and approval times, snapshot references and
checksums, configuration, source provenance, attempted/accepted/rejected counts,
and calibration status. New calibration also records how many seeds were
generated; its acceptance rate uses the **attempted-seed denominator**, excluding
any generated seeds not yet attempted. An interrupted search keeps its current
candidate cache inside this JSON, then removes that cache after completion.

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

**Detailed diagnostics require `--diagnostics`.** Calibration then adds a
`diagnostics/` folder with combined candidate details and D=0 trajectory files.
The benchmark then adds individual flight traces under `diagnostics/`.
Default runs produce neither. Recorded traces are at 50 Hz; safety and mission
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
flights. This artifact refactor does not add policy selection or a new delay
grid; allowed dispatch delays remain 0, 0.2, 0.6 and 1.0 s.

Use `--out` to select another output directory and `--development` to select a
calibration directory or scenario JSON. Fresh output must be absent or empty.
Use `--resume` with the same inputs, protocol and source to continue an
interrupted run. It skips committed flights and reuses their original approval
prediction times. Duplicate records, changed committed data and incompatible
source/configuration/protocol are rejected. An unfinished final JSONL line can
be removed on resume; complete records are preserved.

Both calibration and benchmark retain serial CPU execution, numerical thread
limits, the 2 GiB memory guards, and existing trial/time caps. A resource cap
preserves partial results; it does not make a partial run complete.

To export another existing calibration into the compact layout without running
any simulations, choose an empty destination:

```sh
python geofence_turn/calibration.py --task export \
  --development geofence_turn/support/results/delay_tolerance_20/calibrated_scenarios.json \
  --out /tmp/geofence_calibration_copy
```

Fresh calibration uses `python geofence_turn/calibration.py` with `--start-seed`,
`--attempts`, `--target` and `--out` as needed. It retains the original
latest-feasible-approval rule and D=0 verification. The default output is
`results/development/calibration/`, which already contains the frozen 20 cases;
choose another empty location for a genuinely new calibration. An exported
frozen dataset is an input, not an interrupted search to resume.

Evaluation will use the same commands with `--out` under `results/evaluation/`
and its own frozen calibration input. No evaluation or TTL-accuracy experiment
was launched by this refactor.

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

The artifact/resume tests replay saved-result fixtures, not a new calibration or
benchmark. Additional checks cover original trajectories, source equivalence,
real saved flight outcomes and unchanged predictions. See the
[artifact simplification report](support/reports/ARTIFACT_SIMPLIFICATION_REPORT.md),
[physical mission report](support/reports/PHASE1_REPORT.md), and
[delay-tolerance report](support/results/delay_tolerance_20/REPORT.md).
