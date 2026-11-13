# Experiment B after temporal correction — 21 October 2026

The official Experiment B completed **627 actual PyBullet policy flights**:
19 feasible calibrated scenarios × 11 delays × 3 policies. Seeds 0–19 were
considered; seed 16 remains infeasible and excluded. D=0.0–1.0 s, T=4.2 s after
approval, unknown dispatch delay, and safety-only fallback are unchanged.

**Only one operational outcome changed:** StaleGuard now safely cancels seed 9
at D=0.3 s. Seeds 7/8 at D=0.4 s remain unsafe model-error cases. Seed 10's new
false rejection causes no operational change on this grid. Prediction latency
causes no decision or outcome change in this measured run.

## Updated results

Each cell below is **safe mission completion / safe non-completion / unsafe
execution**, out of 19. Safe completion requires inspection completion and no
unsafe entry through the operational deadline. Prediction availability concerns
StaleGuard's stored reception prediction at the actual dispatch event.

| D (s) | No Recovery | Always-Recover | StaleGuard | Prediction ready | SG fallbacks |
|---:|---:|---:|---:|---:|---:|
| 0.0 | 19 / 0 / 0 | 19 / 0 / 0 | 19 / 0 / 0 | 0/19 | 0 |
| 0.1 | 16 / 2 / 1 | 0 / 19 / 0 | 16 / 3 / 0 | 19/19 | 1 |
| 0.2 | 12 / 5 / 2 | 0 / 19 / 0 | 12 / 7 / 0 | 19/19 | 2 |
| 0.3 | 5 / 8 / 6 | 0 / 19 / 0 | 5 / 14 / 0 | 19/19 | 6 |
| 0.4 | 3 / 4 / 12 | 0 / 19 / 0 | 3 / 14 / 2 | 19/19 | 10 |
| 0.5 | 3 / 2 / 14 | 0 / 19 / 0 | 3 / 16 / 0 | 19/19 | 15 |
| 0.6 | 1 / 1 / 17 | 0 / 19 / 0 | 1 / 18 / 0 | 19/19 | 17 |
| 0.7 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 | 19/19 | 19 |
| 0.8 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 | 19/19 | 19 |
| 0.9 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 | 19/19 | 19 |
| 1.0 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 | 19/19 | 19 |

Aggregate counts use 209 scenario-delay pairs per policy. These reuse nineteen
scenarios; they are not 209 independent scenarios.

| Policy | Safe completion before → after | Unsafe before → after | Safe non-completion before → after | Fallbacks before → after |
|---|---|---|---|---|
| StaleGuard | 59 → 59 (28.23%) | 3 (1.44%) → 2 (0.96%) | 147 → 148 | 126 → 127 |
| No Recovery | 59 → 59 (28.23%) | 128 → 128 (61.24%) | 22 → 22 | 0 → 0 |
| Always-Recover | 19 → 19 (9.09%) | 0 → 0 (0%) | 190 → 190 | 190 → 190 |

All 127 StaleGuard fallbacks and all 190 Always-Recover fallbacks avoided unsafe
entry. StaleGuard avoids 126 unsafe executions relative to No Recovery, while
preserving 40 safe completions that Always-Recover cancels. Safe-completion
benefit and harm against No Recovery both remain zero.

Inspection completion alone can include a subsequent unsafe entry. Its count
for StaleGuard decreases from 60 to 59 because the corrected seed 9 flight is
cancelled. Safe mission completion remains 59. No Recovery's inspection-only
count remains 74; Always-Recover's remains 19. The terminal log retains both
metrics.

## The diagnosed cases and seed 10

Times of unsafe entry below are absolute times from mission start.

| Seed | D | First predicted unsafe delay before → after | Measured prediction time | Executed result before → after |
|---:|---:|---|---:|---|
| 9 | 0.3 s | 0.4 → 0.3 s | 51.20 ms | Entry at 1.829 s → safe fallback, no inspection |
| 7 | 0.4 s | 0.5 → 0.5 s | 58.56 ms | Entry at 2.254 s → same entry |
| 8 | 0.4 s | 0.5 → 0.5 s | 71.76 ms | Entry at 2.091 s → same entry |

All three predictions were ready before dispatch. Seed 9 is now predicted
unsafe at D=0.3, so the original descent is cancelled at the existing dispatch
event. Seeds 7/8 are still predicted safe at D=0.4, so the original commands
execute. Their complete control/physics traces match the saved unsafe flights.
No new unsafe StaleGuard execution appeared.

Seed 10 changes from a finite cutoff of 0.1 s to `INVALID_AT_APPROVAL`, with a
first predicted unsafe delay of 0.0 s. Its measured prediction takes 27.33 ms.

- **D=0:** prediction is unavailable at dispatch. The original command executes
  on schedule and completes safely, exactly as before. Its prediction is a
  false rejection, but no operational rejection occurs.
- **D=0.1–1.0:** prediction is available and fallback cancels descent in all ten
  cases. Each flight is safe without inspection, exactly as before: the old
  0.1 s cutoff already rejected these delays. The original commands are unsafe
  at these delays in the measured ground truth and No Recovery flights.

Thus seed 10 loses no safe completion and gains no fallback in this run. Its
model error remains visible in TTL accuracy; this operational result does not
establish that the false rejection is harmless under other timing conditions.

## Prediction latency

One fresh prediction was measured per scenario after the existing JIT warmup,
before any actual delay case. The measured result and duration were reused
across all eleven delays, following official B scheduling semantics.

| Reception prediction time | Saved B | Corrected B |
|---|---:|---:|
| Minimum | 31.30 ms | 27.33 ms |
| Mean | 56.59 ms | 61.93 ms |
| Maximum | 76.16 ms | 93.54 ms |

The cross-run mean difference is +5.34 ms; it also includes host timing
variation. The earlier controlled paired overhead measurement is documented
in [the temporal correction report](13_temporal_safety_correction.md).

Both runs have 0/19 available predictions at D=0 and 19/19 at every positive
delay: 190/209 dispatches overall. Substituting the saved B computation times
into the corrected predictions changes zero dispatch decisions. There are no
latency-caused outcome changes on this grid. An unavailable result still means
execute on schedule, with no later cancellation; neither dispatch nor deadline
was postponed. These measurements do not guarantee readiness on other hosts.

## TTL observation horizon versus mission deadline

These are different quantities and use different time origins:

- **TTL prediction and ground truth:** observation ends at **8.5 s from mission
  start** (`experiment.MEASURE_SECONDS`). The predictor starts at approval, so
  its remaining horizon is 8.5 minus approval time: 7.92–8.50 s here. Ground
  truth monitors waiting and commanded flight after approval at 1,000 Hz.
- **Experiment B:** the deadline is **4.2 s after approval**, independent of D
  and computation time. Absolute deadlines range from **4.20 to 4.78 s** in
  these calibrated scenarios. Operational safety also uses 1,000 Hz monitoring
  and the identical unsafe-volume predicate. Execution stops early on unsafe
  entry; otherwise monitoring continues through the deadline, including after
  inspection completion.

Every one of the **209** new No Recovery safety labels matches the saved
corrected 8.5 s ground-truth label: 128 unsafe, 81 safe. The latest first unsafe
entry among these flights is **3.363 s** (seed 3, D=0.7; deadline 4.34 s), earlier
even than the minimum operational deadline. There is no unsafe-label difference
caused by the longer TTL horizon on this grid. The horizons remain different;
this empirical agreement does not justify treating them as interchangeable.

## Execution, preservation, and reproduction

No algorithm, scientific threshold, calibration, mission, recovery, scheduling,
or baseline source was changed for this task. The only Python addition is
[a diagnostic runner](../diagnostics/validate_corrected_experiment_b.py).
It uses the official `main()` and `run_actual()` with saved, verified approval
snapshots and platform parameters. Predictions are freshly computed by the
native reception helper in a separate process to keep the JIT compiler out of
simulator memory. No flight outcome is inferred from a baseline or substituted
for an unexecuted flight.

Each executed flight records outcomes, prediction availability, first unsafe
time, and a hash of every control-state/reference row and physics position.
The 418 baseline traces match pre-correction B exactly. Every StaleGuard trace
also matches its corresponding actually executed original-command or fallback
reference. The sole outcome difference among all 627 records is seed 9/D=0.3.
All source/input preservation hashes and the native metric/comparison
assertions passed. Four temporal geometry/monitor unit tests passed.

Execution used one simulator worker and one numerical-library thread. The
existing guard remains at 2,048 MiB maximum process RSS and 2,048 MiB minimum
host available memory. Early guard stops saved complete flights; resuming did
not repeat them or remeasure predictions. After host memory was freed, the
remaining batch completed normally. Garbage collection and returning unused
Linux allocator pages between flights kept steady worker RSS near 636 MiB;
the maximum recorded worker peak was 700.91 MiB.

Evidence:

- [Complete corrected official terminal output](../result/baseline_unknown_dispatch_swept.log)
- [Preserved pre-correction B output](../result/baseline_unknown_dispatch.log)
- [Machine-readable comparison](../results/corrected_experiment_b/summary.json)
- [627 actual flight records](../results/corrected_experiment_b/executions.jsonl)
- [19 measured reception predictions](../results/corrected_experiment_b/receptions.json)
- [Memory/execution metadata](../results/corrected_experiment_b/run_metadata.json)

Run from `examples/staleguard` in the existing `scg` environment:

```sh
python inspection_descent/diagnostics/validate_corrected_experiment_b.py --stage predict
python inspection_descent/diagnostics/validate_corrected_experiment_b.py --stage run
python inspection_descent/diagnostics/validate_corrected_experiment_b.py --stage report
```

The prediction stage refuses to overwrite recorded timings. The run stage
resumes its checkpoint; after completion it can regenerate the native report
from those executed records. A fresh independent run needs a new output
directory/log path. Old results are preserved. Experiment A was not rerun.
