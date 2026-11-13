# Inspection-Descent

The UAV flies forward at its usual altitude. An approved command requests a lower
altitude for a fixed duration, then returns to usual altitude. Dispatch delay shifts
that same command later; its duration, requested altitude and inspection target stay
fixed. Safety-only recovery cancels the descent, retains nominal forward flight,
and requests usual altitude. **It does not stop, hover, or retry the inspection.**

The workflow is **calibration → TTL accuracy → six-policy benchmark → recovery-policy
comparisons**. New scripts use the same compact artifact conventions as Geofence
Turn, while retaining Inspection-Descent's original scientific definitions.

## Scientific protocol

| Setting | Inspection-Descent definition |
|---|---|
| Controller / actual monitor / lightweight integration | 50 Hz / 1000 Hz / 100 Hz |
| Scenario generation | `scenario.generate_scenario(seed)` with original distributions and distributions |
| Calibration | Original finite candidate grid; maximize safe D=0 consecutive dwell; first candidate on ties |
| Completion | Position within strict bounds: `abs(x-inspection_x)<0.12`, `abs(y)<0.20`, `abs(z-low_height)<0.08`, continuously for 0.18 s |
| Inspection target | `box_x_min - 0.5 m`, at the approved low altitude |
| Unsafe region | Closed box: `box_x_min <= x <= box_x_max`, `abs(y) <= box_y_limit`, `z <= unsafe_below_z` |
| Operational deadline | 4.2 s after approval |
| TTL prediction / ground-truth horizon | 8.5 s from flight start, as in the original accuracy experiment |
| Prediction grid | Original `scenario.DELAY_GRID`, eleven points 0–1 s in 0.1 s increments |
| Default benchmark dispatch grid | The same eleven physical delays, not Geofence Turn's 23 delays |
| Recovery | Forward reference unchanged; usual-altitude request; safety-only |

`completed` reports inspection completion, even if unsafe. `safe_completion`
requires completion and no unsafe entry. Operational flights retain the original
behavior of stopping at the control endpoint containing the first unsafe event;
they do not continue an unsafe flight to obtain additional completion observations.
The monitor still observes every intervening physics tick. `monitored_until_s`
records where monitoring ended; `horizon_s` is the original planned deadline.

The lightweight predictor retains its original swept-segment safety checker between
50 Hz physical states. It is not a 1000 Hz monitor. The dynamics, PID state,
unsafe-volume boundaries and calibrated command are unchanged.

## Source organization

Each component has one canonical implementation. There are no legacy entry points,
import aliases, or re-export modules. Frozen pickles are read by `storage.load_pickle()`;
it maps their two historical data-class names to `scenario.py`, without changing
snapshot bytes or installing old modules.

| Location | Responsibility |
|---|---|
| `scenario.py` | Scenario sampling, descent references, geometry, safety and inspection predicates |
| `calibration.py` | Original ordered candidate search, maximum safe D=0 dwell, verification and saving |
| `simulation.py` | One PyBullet flight engine, approval capture, 1000 Hz monitoring and policy scheduling |
| `storage.py` | Frozen-data reading, snapshots, memory limits, provenance and historical schema conversion |
| `baselines/` | Canonical policy decisions and lightweight rollout logic |
| `baseline_benchmark/` | Cached approval forecasts, physical flights, resumable journal and derived metrics |
| `ttl_pred_accuracy/` | Approval predictions versus the original 8.5-second safety truth |
| `support/` | Focused scientific and workflow tests |
| `../model/`, `../physics_monitor.py` | Single shared UAV/PID model and physical monitor, used by both missions |

`../source_relocation.json` records reviewed old-source checksums and exact new-source
checksums. Historical hashes remain unchanged. Unknown source versions or later
scientific changes cannot pass a relocation check. New runs use the current source
hashes; **pre-cleanup runs cannot be resumed under changed source provenance**.
Use a fresh output, rather than rewriting their metadata.

## Policies

| Policy | Rule |
|---|---|
| `no_recovery` | Execute the descent at dispatch |
| `always_recover` | Preserve the historical rule: recover at approval while pending; immediate D=0 dispatch executes normally |
| `fixed_ttl` | Original **0.30 s** lifetime; dispatch at equality is permitted; recover strictly after it |
| `fbs` | Original signed feature, zero-initialized EMA α=0.1, epsilon=1e-12, frozen TTL and readiness rules |
| `use_time_recheck` | One current-state descent prediction at dispatch; nominal flight continues until the first tick strictly after measured computation finishes |
| `staleguard` | One approval prediction per seed; dispatch-time expiry check only if ready; otherwise fail-open with no later cancellation |
| `staleguard_early_recovery` | Same forecast; cancel while pending when the forecast is ready and its first-unsafe expiry is reached; original StaleGuard rule at dispatch |
| `fixed_early_recovery` | Retain the development-fixed 0.30 s timeout, now using StaleGuard's `>=` expiry convention |
| `ppr` | Periodically predict continued nominal flight and then delayed usual-altitude recovery; no queued-command safety prediction |

Fixed-EarlyRecovery's timeout is retained from the original development Fixed-TTL
protocol, not selected using independent seeds 20–319. Its equality convention is
intentionally different from the original Fixed TTL.

PPR monitors every **100 ms**, matching Inspection-Descent's original delay grid.
It budgets **10 ms** for the next prediction: strict 50 Hz scheduling permits one
additional 20 ms tick, so the rollout applies recovery **120 ms** after observation.
Twenty-five warmed development checks on seeds 0–4 measured 3.35 ms mean and
3.85 ms maximum before evaluation. This is an engineering allowance, not a proven
worst-case bound. Check records retain actual runtimes, availability/action ticks,
and overruns. Checks never overlap; missed opportunities are skipped. Dispatch
wins a same-tick PPR availability tie. Recovery permanently cancels the command.

**PPR's target is degenerate for this mission's nominal waiting behavior:** before
dispatch, the reference already requests usual altitude. The hypothetical fallback
therefore leaves the nominal reference unchanged. A safe prediction does not certify
the later descent; PPR can behave like No Recovery. Likewise, earlier cancellation
by StaleGuard-EarlyRecovery normally changes no physical reference before dispatch.
These policies are implemented faithfully, not presented as improvements.

Prediction latency is measured and replayed against simulated control time, as in
the original Use-Time Recheck experiment. This is CPU-only, serial research
simulation, not demonstrated concurrent onboard scheduling. The original guard
remains: maximum process RSS 2048 MiB and at least 2048 MiB available host memory.

## Datasets and historical migration

- **Development:** seeds 0–19, 19 feasible; seed 16 rejected. No new calibration was
  run during migration. The validated historical approval bundle supplies snapshots.
- **Original independent evaluation:** 300 attempted seeds 20–319, 276 feasible.
  Calibration manifests reference the original immutable approval pickle files.
- The original six-policy journal contains **18,216 records**: 12,980 native
  simulations and 5,236 verified trajectory reuses. These are not 18,216 newly run
  simulations. All six published unsafe/safe-completion counts were verified from
  the actual journal.

| Policy | Historical unsafe | Historical safe completion |
|---|---:|---:|
| No Recovery | 1841 | 930 |
| Always Recover | 0 | 276 |
| Fixed TTL | 139 | 819 |
| FBS | 1841 | 930 |
| Use-Time Recheck | 63 | 864 |
| StaleGuard | 22 | 917 |

`results/evaluation/baseline_benchmark/` is an explicitly normalized derivative of
that historical journal. Original files under `independent_evaluation_20_319/` remain byte-identical.
Historical reports formerly in `results/development/historical_reports/` now live in
`results/development/historical_reports/`; other root-level result folders moved
unchanged under `results/development/`. The root calibration JSON moved unchanged
to `results/development/calibration/generated_scenarios.json`.
Obsolete Python source copies and diagnostic scripts were removed; their previous
versions are recoverable from Git, not retained as a second implementation. `execution_kind`
retains native/reuse provenance; unrecorded historical completion timestamps and
physical runtimes are `null`. The derivative cannot receive new physical flights.
Use a fresh output name for a new six-policy evaluation.

The recovery-extension directories contain only initial smoke flights, with partial
status and full-cohort protocols. They are **not completed full evaluations**.
These extensions use previously examined scenarios and are exploratory, not fresh
held-out validation. Original clustered confidence intervals/sign tests remain in
historical reports; the new metrics contain descriptive paired counts and rates.

## Workflow and artifacts

```text
results/
├── development/
│   ├── calibration/             # calibrated_scenarios.json + snapshots/
│   ├── ttl_pred_accuracy/       # run.json, executions.jsonl, metrics.json
│   └── baseline_benchmark/      # same three files
└── evaluation/
    ├── calibration/             # manifest references frozen historical snapshots
    ├── baseline_benchmark/      # read-only historical schema conversion
    ├── early_recovery_benchmark/
    ├── fixed_early_benchmark/
    └── ppr_benchmark/           # each benchmark has the same three files
```

Evaluation `ttl_pred_accuracy/` is created only when that experiment runs. It cannot
reuse operational 4.2-second flights as 8.5-second truth. Development accuracy
reuses the validated corrected historical accuracy measurements; otherwise the
`ttl_pred_accuracy/run.py` classifies missing profiles using the shared
`simulation.run_ground_truth()` flight engine.

Every benchmark journal has one record per completed physical flight; historical
conversion is explicitly tagged. Records use `seed`, `delay_s`, `safe`, `completed`,
`safe_completion`, `first_violation_s`, `event`, `runtime_s`, physics sample counts,
and control/physics digests. Event fields retain scheduled dispatch, actual
application/recovery, availability and computation diagnostics. Steps and violation
seconds are from flight start; new completion seconds are relative to approval.
PPR check-level times and per-flight sums/maxima are separate from flight runtime.

`run.json` freezes scenarios, snapshot/dataset checksums, configuration, grids,
policies, source/environment hashes and approval forecasts. It records partial or
complete status and the journal checksum. Repeating the **same** command resumes,
rejects incompatible inputs/source/configuration, repairs an unfinished trailing
write and prevents duplicate `(seed, delay_s, policy)` records. Metrics are written
on completion or interruption and can be reproduced without running PyBullet.

## Clean-migration evidence and archived runs

The 24 representative six-policy flights (seeds 20 and 21, D=0 and 0.6 s)
matched their historical outcomes, control trajectories and 1000 Hz position
records exactly. All 320 candidate grids retained the original order. Seed 0's
full 30-candidate calibration was rerun and compared against the pre-cleanup
search. Development accuracy retains its original 209 comparisons and separate
raw-model versus TTL-decision metrics.

Folders ending in `_before_cleanup` preserve the preceding migration's artifacts
unchanged. They have old source provenance and cannot be appended to by the new
code. The original independent evaluation remains a read-only regression source.
Old documentation and archived run metadata may mention removed source paths;
these remain historical evidence, not executable entry points.

## Commands from the repository root

Activate the existing `scg` environment. The original frozen data are already
imported; these preparation commands are safe and do not recalibrate them:

```bash
python -m staleguard.inspection_descent.calibration --import-historical development
python -m staleguard.inspection_descent.calibration --import-historical evaluation
```

To calibrate **new** commands with the unchanged official search, use a fresh output:

```bash
python -m staleguard.inspection_descent.calibration --start-seed 0 --attempts 20 --out /tmp/inspection_new/calibration
# Repeat with --resume after interruption; no completed seed is lost.
```

Accuracy (development already executed; add `--calibration .../evaluation/calibration`
for the independent cohort, which will need new 8.5-second ground-truth profiles):

```bash
python -m staleguard.inspection_descent.ttl_pred_accuracy.run
python -m staleguard.inspection_descent.ttl_pred_accuracy.metrics staleguard/inspection_descent/results/development/ttl_pred_accuracy/executions.jsonl
```

Original six policies, **new physical evaluation**, isolated from the historical derivative:

```bash
python -m staleguard.inspection_descent.baseline_benchmark.run --calibration staleguard/inspection_descent/results/evaluation/calibration --out staleguard/inspection_descent/results/evaluation/baseline_benchmark_rerun
```

Recovery comparisons (each full run is 276 × 11 × 2 = 6072 physical flights):

```bash
python -m staleguard.inspection_descent.baseline_benchmark.run --calibration staleguard/inspection_descent/results/evaluation/calibration --policies staleguard staleguard_early_recovery --out staleguard/inspection_descent/results/evaluation/early_recovery_benchmark
python -m staleguard.inspection_descent.baseline_benchmark.run --calibration staleguard/inspection_descent/results/evaluation/calibration --policies staleguard_early_recovery fixed_early_recovery --out staleguard/inspection_descent/results/evaluation/fixed_early_benchmark
python -m staleguard.inspection_descent.baseline_benchmark.run --calibration staleguard/inspection_descent/results/evaluation/calibration --policies staleguard_early_recovery ppr --out staleguard/inspection_descent/results/evaluation/ppr_benchmark
```

These commands resume the few smoke flights already present; full runs are not
launched automatically. Add `--max-flights 4` for a bounded resumable batch, or
`--seeds 20 --delays 0 .6 --out /tmp/inspection_smoke` for a separate small protocol.

Historical schema conversion and metrics-only reproduction:

```bash
python -m staleguard.inspection_descent.baseline_benchmark.run --import-historical
python -m staleguard.inspection_descent.baseline_benchmark.metrics staleguard/inspection_descent/results/evaluation/baseline_benchmark/executions.jsonl
python -m unittest staleguard.inspection_descent.support.test_workflow
```

Legacy detailed reports remain under [docs/](docs/), [result/README.md](result/README.md)
and [independent_evaluation_20_319/README.md](independent_evaluation_20_319/README.md).
