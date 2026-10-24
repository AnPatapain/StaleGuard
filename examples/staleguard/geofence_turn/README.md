# Geofence turn: small research scripts

The UAV patrols east, executes the same approved 90-degree left turn when
dispatched, then flies north through a fixed checkpoint. Waiting moves the turn
farther east; it never moves the checkpoint or creates a return path. The radius,
mission conditions, safety rules, four policy decisions and timing conventions
are unchanged by this reorganization.

[Refactor report](support/reports/REFACTOR_REPORT.md) explains the before/after checks.
[Phase 1 report](support/reports/PHASE1_REPORT.md) explains the physical experiment and its limits.

## Where to read the code

| File | Responsibility |
|---|---|
| `scenario.py` | Scenario and command definitions, generation, patrol/turn paths, geometry and completion tests |
| `calibration.py` | Turning screen, generation, D=0 approval search, replay, and `calibrated_scenarios.json` export |
| `simulation.py` | Shared SCG/PyBullet flight loop, physics monitoring and resource limits |
| `baselines/staleguard.py` | Existing lightweight-model safety forecast and expiry/readiness decision |
| `baselines/use_time_recheck.py` | Existing dispatch-time forecast and delayed application tick |
| `baselines/fixed_ttl.py` | Existing 0.4 s TTL rule; equality permits dispatch |
| `baseline_benchmark/run.py`, `metrics.py` | Existing paired four-policy experiment and its result counts |
| `support/` | Diagnostic tools, validation, tests, reports and archived data; see its [README](support/README.md) |

No Recovery is the default execution path in `simulation.py`: it executes the
approved command without a guard. It has no separate decision function. Neither
Always-Recover nor FBS has been added.

`mission.py` has been removed. The snapshot reader in `calibration.py` maps its
old class names to `scenario.py`, so existing snapshot bytes and checksums stay
unchanged. The existing lightweight model is imported in place from
`inspection_descent/model/`; its implementation and Inspection-Descent are untouched.

## Core scripts and supporting material

```text
geofence_turn/
├── scenario.py
├── calibration.py
├── simulation.py
├── baselines/
├── baseline_benchmark/
├── support/
│   ├── diagnostics.py
│   ├── readiness.py
│   ├── validation.py
│   ├── tests/
│   ├── logs/
│   ├── results/
│   └── reports/
└── README.md
```

The calibrated input data and saved results are research evidence, so they are
preserved under `support/` alongside the checks. They are not extra experiment
implementations. The only local `.gitignore` rules exclude the runtime lock and
allow the focused regression evidence through the parent repository's blanket
`results/` ignore rule; Python cache rules already come from the parent.

## Workflow and artifacts

The implemented research workflow is **calibration → baseline benchmark**.
These are separate commands: pass the calibration output directory or its
`calibrated_scenarios.json` to `baseline_benchmark/run.py` through `--development`.
Otherwise, the benchmark uses the existing five development cases.

Calibration already includes a D=0 verification flight for every accepted
scenario. Separate export, replay, readiness diagnostics and tests are optional.
The following inventory describes the files the current code writes; it does
not require rerunning calibration when using the existing cases.

**Calibration output root** — the directory selected by calibration's `--out`:

| Artifact | Purpose |
|---|---|
| `metadata.json` | Configuration, software environment and source hashes |
| `generated.json` | Generated scenario parameters, including geometry rejections |
| `attempts.jsonl` | Acceptance/rejection record for each attempted scenario |
| `summary.json` | Calibration progress, rejection counts and accepted scenarios |
| `calibrated_scenarios.json` | Accepted scenarios, snapshot paths and checksums |
| `finished.json` | Elapsed time, simulation count and final memory measurements |

**Per-scenario calibration output** — calibration creates `seed_<id>/` for each
scenario that reaches simulation:

| Artifact | Purpose |
|---|---|
| `nominal_bound.json` | Straight-east flight result used to determine the approval-search limit |
| `candidates.jsonl` | Results of the tested approval times |
| `approval.pkl` | Accepted approval snapshot: aircraft state, PID history and remaining reference |
| `scenario.json` | Accepted scenario and approval time |
| `configuration.json` | Configuration copied into this scenario directory |
| `d0_replay.json` | Accepted scenario's D=0 verification result |
| `d0_trace.npz` | Actual and requested trajectories from that verification flight |

The last five files are written only for accepted scenarios. A geometry
rejection creates no seed directory. `calibrated_scenarios.json` points to the
snapshot files; it does not replace them. The calibration summary and aggregate
scenario file are updated as calibration progresses, and `finished.json` is
written at successful command completion.

**Baseline benchmark output** — the directory selected by the benchmark's `--out`:

| Artifact | Quantity / when written | Purpose |
|---|---|---|
| `metadata.json` | One, at startup | Configuration, environment and source hashes |
| `protocol.json` | One, before flights | Selected scenarios and dispatch delays |
| `prediction_seed_<id>.json` | One per evaluated scenario, before its flights | StaleGuard prediction, expiry and computation time |
| `seed<id>_D<delay>_<policy>.npz` | One per completed flight | Actual and requested trajectory samples |
| `executions.jsonl` | One, appended after each completed flight | Detailed safety, completion, clearance, timing and fallback results |
| `summary.json` | One, updated after each completed flight | Running outcome and fallback counts |
| `paired_summary.json` | One, at successful comparison completion | Final counts using complete four-policy comparisons |
| `finished.json` | One, at successful command completion | Elapsed time, flights executed in this invocation and final memory measurements |

The NPZ traces contain actual state and requested trajectory samples at 50 Hz;
safety is checked at 1000 Hz, with clearance extrema and violation times saved
in the result records. `summary.json` counts all completed flights, while
`paired_summary.json` uses only scenario-delay groups containing all four
baselines and lists incomplete groups separately. No formatted research report
or plots are generated automatically.

**Conditional and temporary files:**

| Artifact | When produced |
|---|---|
| `failures.jsonl` | When a caught runtime error stops a command |
| `resume_sessions.jsonl` | When calibration, characterization or the benchmark resumes |
| `support/logs/.simulation.lock` | Shared file preventing simultaneous simulation jobs |
| `*.json.tmp` | Temporary files during JSON replacement; normally disappear immediately |
| Python/Numba caches | Runtime-generated caches |

Progress is printed to the terminal. A console `.log` file is created only if
the output is redirected or captured.

**Optional commands** add further files and are not automatically part of
calibration → benchmark:

| Command | Additional outputs, besides its metadata and completion record |
|---|---|
| Characterization | `sweep.jsonl`, `envelope.json`, nine trajectory files |
| Generation only | `generated.json` |
| Export | Another `calibrated_scenarios.json` pointing to existing snapshots |
| Separate replay | One result JSON per scenario |
| Readiness diagnostic | `timings.jsonl`, prediction JSONs and a timing `summary.json` |

The plot diagnostic separately writes `diagnostics/northbound_checkpoint.png`
under its selected logs root. Validation and tests print to the console;
integration-test trajectories use temporary directories that are cleaned up.
These supporting commands do not automatically write metadata or completion
records.

For scale, a successful fresh calibration accepting five scenarios with no
additional rejected seed directories produces **41 files**: six at the output
root and seven for each accepted scenario. A small benchmark with two scenarios,
two delays and four policies produces **24 more files**: six shared files, two
prediction files and 16 trajectory files. Together that is **65 persistent
output files**, before optional diagnostics, caches or error records.

Several files repeat configuration, scenario descriptions or summaries. That
duplication, together with one trajectory file per flight, accounts for much of
the current file count.

## Run directly with the existing five cases

Run from `examples/staleguard` in the existing `scg` environment. Every output
directory must be new. Scripts run headless, use one worker and one numerical
thread, and retain the 2 GiB memory guards and explicit trial/time caps.

```sh
SCG_PYTHON=/home/kean/miniconda3/envs/scg/bin/python
```

Export the existing five calibrated cases to a single JSON file, **without
recalibrating or copying/changing their approval snapshots**:

```sh
$SCG_PYTHON geofence_turn/calibration.py --task export \
  --out geofence_turn/support/results/calibrated_export_new
```

The output is `calibrated_scenarios.json`. Later scripts accept either that file
or the original development directory through `--development`. Loading checks the
schema, configuration, snapshot checksum, command, approval time and reference.

Replay the five frozen cases:

```sh
$SCG_PYTHON geofence_turn/calibration.py --task replay \
  --out geofence_turn/support/results/replay_new --max-trials 5
```

Run a **small paired comparison**, using two existing cases and delays already
present in the original experiment grid:

```sh
$SCG_PYTHON geofence_turn/baseline_benchmark/run.py \
  --out geofence_turn/support/results/paired_new \
  --seeds 0 3 --delays 0 0.6 --max-trials 16
```

This uses No Recovery, StaleGuard, Use-Time Recheck and Fixed TTL. Removing the
subset options retains the original five-case/four-delay workflow; the refactor
validation did not run that full benchmark. Use `--resume` only with unchanged
code, cases and delays; mismatches are rejected rather than mixing records.

## Prediction timing and unfinished accuracy work

The following runs the **existing readiness diagnostic**, using saved approval
states and the real predictor. It reports initialization separately from warmed
forecast time and does not execute mission flights:

```sh
$SCG_PYTHON geofence_turn/support/readiness.py \
  --out geofence_turn/support/results/readiness_new \
  --repeats 1 --max-trials 12 --max-seconds 120
```

The mission-run TTL prediction-accuracy experiment, its ground-truth comparison
and its accuracy metrics remain unfinished. The misleading `ttl_pred_accuracy/`
directory has been removed; the existing timing check lives in `support/`. There
are no empty accuracy, FBS or Always-Recover placeholder modules.

## Calibration and regression checks

Fresh calibration is available through the same direct script, with the existing
rules and parameters. This refactor reused the five cases instead of generating
another calibration dataset:

```sh
$SCG_PYTHON geofence_turn/calibration.py \
  --out geofence_turn/support/results/calibration_new \
  --attempts 10 --target 5 --max-trials 100
```

Use `--task characterize` for the existing nine-pair turning screen or `--task
generate` for generation only. `--envelope` selects its measured envelope.
Interrupted calibration/characterization can resume under the exact same source
hashes. Old interrupted runs cannot resume across this layout change; their
original files remain preserved. Reading their frozen calibrated cases is a
separate, checksum-checked operation.

```sh
$SCG_PYTHON -m unittest geofence_turn.support.tests.test_mission \
  geofence_turn.support.tests.test_prediction geofence_turn.support.tests.test_refactor_equivalence -v
$SCG_PYTHON -m unittest geofence_turn.support.tests.test_integration -v
$SCG_PYTHON -m geofence_turn.support.validation
```

The equivalence tests check the live functions against pre-refactor generation,
trajectory and prediction outputs, and compare the recorded 1 kHz before/after
flights and scheduling. Integration tests now write to temporary directories, so
rerunning them does not overwrite historical integration records. Existing
`support/logs/` holds the original calibration inputs and run records;
`support/results/` holds the refactor evidence. All 486 archived files retained
their original bytes when moved. Historical paths inside recorded metadata are
retained as provenance; the commands above use the current paths.
