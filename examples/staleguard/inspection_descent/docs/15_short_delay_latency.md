# Supplementary Experiment B: short dispatch delays — 21 October 2026

**342 actual PyBullet flights completed:** 19 feasible calibrated scenarios ×
six delays × three existing policies. Seeds 0–19 were considered; seed 16 is
infeasible and explicitly excluded. T=4.2 s after approval and safety-only
fallback are unchanged. This supplement preserves the official Experiment B
results and changes no scientific source, calibration, or policy.

The primary result is **zero unsafe executions attributable to unavailable
predictions in this measured run**. There were 49 unavailable-prediction
dispatches, all of which executed the original command safely. This does not
establish that fail-open dispatch is safe in general: the one scenario with
unsafe original commands on this short-delay grid had the fastest prediction.

## Prediction time and availability

The native `predict_at_reception()` was timed once per scenario after the
existing JIT warmup, before enumerating actual delays. Each scenario's measured
duration and prediction were reused across all six delays, following B's
existing timing semantics. Therefore the timing distribution in each row is
the **same 19 measured predictions**, not six independent timing samples.
No artificial delay was added, and timings were not retried or selected.

| D (s) | Mean (ms) | Median (ms) | p95 (ms) | Ready | Unavailable | Original commands because unavailable |
|---:|---:|---:|---:|---:|---:|---:|
| 0.00 | 42.52 | 41.48 | 61.18 | 0 | 19 | 19 |
| 0.02 | 42.52 | 41.48 | 61.18 | 1 | 18 | 18 |
| 0.04 | 42.52 | 41.48 | 61.18 | 9 | 10 | 10 |
| 0.06 | 42.52 | 41.48 | 61.18 | 17 | 2 | 2 |
| 0.08 | 42.52 | 41.48 | 61.18 | 19 | 0 | 0 |
| 0.10 | 42.52 | 41.48 | 61.18 | 19 | 0 | 0 |

Counts use N=19 at each delay. Across 114 StaleGuard dispatches, predictions
were ready at 65 and unavailable at 49. An unavailable prediction always led
to execution at the original dispatch step, without subsequent cancellation.

The measured range was **19.613–68.193 ms**. p95 uses NumPy's default linear
percentile interpolation over nineteen samples. The empirical distribution is:

| Computation time C | Scenarios |
|---|---:|
| 0 < C ≤ 20 ms | 1 |
| 20 < C ≤ 40 ms | 8 |
| 40 < C ≤ 60 ms | 8 |
| 60 < C ≤ 80 ms | 2 |
| 80 < C ≤ 100 ms | 0 |

Individual seed timings and their first usable control-step boundaries are
preserved in the [machine-readable results](../result/short_delay_latency.json).

## Executed policy outcomes

Each cell is **unsafe executions / safe mission completions**, out of 19.
These are separate metrics, so a cell need not sum to 19: a flight may be safe
without completing inspection. Safe completion requires inspection completion
and no unsafe entry through the operational deadline.

| D (s) | No Recovery | Always-Recover | StaleGuard |
|---:|---:|---:|---:|
| 0.00 | 0 / 19 | 0 / 19 | 0 / 19 |
| 0.02 | 0 / 18 | 0 / 0 | 0 / 17 |
| 0.04 | 1 / 17 | 0 / 0 | 0 / 17 |
| 0.06 | 1 / 17 | 0 / 0 | 0 / 17 |
| 0.08 | 1 / 17 | 0 / 0 | 0 / 17 |
| 0.10 | 1 / 16 | 0 / 0 | 0 / 16 |

Aggregated over 114 scenario-delay pairs per policy:

| Policy | Unsafe | Safely completed | Safe without completion | Fallbacks |
|---|---:|---:|---:|---:|
| StaleGuard | 0 | 103 (90.35%) | 11 | 5 |
| No Recovery | 4 (3.51%) | 104 (91.23%) | 6 | 0 |
| Always-Recover | 0 | 19 (16.67%) | 95 | 95 |

All five StaleGuard and all 95 Always-Recover fallbacks were safe, without
inspection completion. At D=0, Always-Recover retains its existing inability
to cancel before simultaneous dispatch. StaleGuard's prediction is also
unavailable there, so all three policies execute the same original command.

## Diagnostic counterfactual: late versus incorrect prediction

For every unavailable prediction, the diagnostic reads the stored reception
TTL after the flight and asks whether the existing
`ttl_expired_at_dispatch(prediction, waited_steps)` rule would have rejected
the command if that same result had been available. It does not run a new
policy, reschedule dispatch, or replace the prediction with ground truth.

| D (s) | Unavailable-prediction executions | Unsafe among those | Unsafe that stored TTL would reject | Unsafe that stored TTL would allow |
|---:|---:|---:|---:|---:|
| 0.00 | 19 | 0 | 0 | 0 |
| 0.02 | 18 | 0 | 0 | 0 |
| 0.04 | 10 | 0 | 0 | 0 |
| 0.06 | 2 | 0 | 0 | 0 |
| 0.08 | 0 | 0 | 0 | 0 |
| 0.10 | 0 | 0 | 0 | 0 |

An unsafe unavailable-result execution with a rejecting TTL would count as
latency-attributable exposure. An unsafe execution with an allowing TTL would
remain unsafe even if that TTL were ready; its false-safe TTL decision would
not be attributed to computation latency. Neither category occurred here.
There were also no unsafe executions with available predictions.

**Seed 10 explains the observed safety result and the completion harm.**
Its unchanged corrected prediction is `INVALID_AT_APPROVAL`, first unsafe
delay 0.0 s. It computed in **19.613 ms**, faster than every other scenario.

- D=0: the prediction is unavailable. The original command completes safely.
- D=0.02: it is available just **0.387 ms before dispatch**. StaleGuard cancels,
  but No Recovery completes safely through the deadline. This is one
  operational false rejection and one lost safe completion, caused by the
  TTL decision rather than a late prediction.
- D=0.04–0.10: it is available and cancels all four commands. No Recovery enters
  the unsafe region at 1.929, 1.929, 1.934, and 1.945 s from mission start,
  respectively. These four unsafe original commands are avoided.

All other original commands in this short-delay matrix are safe. Thus the
unavailable results belong to cases that do not become unsafe in this run.
StaleGuard loses one safe completion versus No Recovery, prevents four unsafe
executions, and preserves 84 safe completions versus Always-Recover.

## Comparison with the official 0.0–1.0 s Experiment B

The predictor itself still computes over the **unchanged 0.0–1.0 s TTL grid in
0.1 s increments**. Only actual dispatch uses the finer delays here; reception
does not receive actual D. All nineteen prediction outputs match the official
corrected run exactly. No extra model rollouts at 20–80 ms were introduced.

The shared D=0 and D=0.1 cases have **identical outcomes and complete physical
trace hashes in all 114 policy flights**, including 76 baseline flights.
The official coarse grid showed 0/19 ready at D=0 and 19/19 ready at every
positive delay. This supplement exposes the unobserved intermediate region:
18 unavailable predictions at 20 ms, 10 at 40 ms, and 2 at 60 ms.

| Prediction time distribution | Official corrected B | This supplement |
|---|---:|---:|
| Mean | 61.93 ms | 42.52 ms |
| Median | 57.12 ms | 41.48 ms |
| p95 | 90.18 ms | 61.18 ms |

These are distinct wall-clock timing passes of identical code and inputs.
The faster pass is not an algorithm improvement or a controlled speedup
measurement. In this pass all predictions are ready by the 80 ms dispatch
boundary; that was not evaluated by the coarse grid.

The official run's unsafe seeds 7/8 at D=0.4 s remain outside this supplement's
range and are not fixed by these results. Comparing 0/114 unsafe here with
2/209 in the wider experiment does not establish improved prediction accuracy.

## Interpretation and limits

Prediction is too slow to participate in a particular dispatch whenever its
measured computation time **C > D**. It becomes usable at the first control
boundary at or after C: `ceil(C × 50) / 50`. The observed maximum of 68.193 ms
therefore requires an **80 ms** dispatch delay for all nineteen predictions to
be available on this control grid. Neither the mean nor p95 guarantees that
every prediction is ready.

For unsafe commands in this particular matrix, the relevant scenario is seed
10. Its earliest observed unsafe original command is D=40 ms. Its measured
19.613 ms prediction is ready with 20.387 ms to spare. If it instead finished
after that dispatch, the existing fail-open rule would execute the measured
unsafe original command. This is the protocol's conditional implication, not
an additional experiment with an injected delay.

The run therefore demonstrates frequent loss of TTL availability at short
delays, but **does not observe an unsafe execution caused by that loss**.
Safety depends on which commands are unsafe and their individual computation
times, not just on the average readiness rate. The D=20 ms completion loss is
also sensitive to seed 10's timing being less than half a millisecond before
the decision boundary.

Limitations include one warm timing measurement per scenario, host timing
variation, a small deterministic scenario set, and reuse of those scenarios
across delays. The p95 is an empirical statistic from nineteen measurements.
Timing excludes JIT compilation and snapshot acquisition, as in official B;
the simulator reuses the measured wall time rather than running prediction
concurrently with flight. These results are not an onboard worst-case timing
bound. The stored TTL retains its original delay resolution and model errors.

Operational outcomes use T=4.2 s after approval (absolute deadlines
4.20–4.78 s); the predictor retains its horizon ending at 8.5 s from mission
start. No new full-horizon TTL ground-truth evaluation was performed for these
fine delays. The operational false rejection and counterfactual above concern
the actual safety and completion outcomes through the operational deadline.

## Validation and reproduction

The native `run_actual()` executed all 342 flights serially. Approval snapshots
and platform parameters were reused from the previously verified cache, and
fresh native reception predictions were measured in a separate process. The
existing 2,048 MiB maximum-RSS / 2,048 MiB minimum-host-available guard and
validation lock were retained. Steady simulator RSS was about 635 MiB; the
recorded peak was 700.36 MiB. No memory-guard stop occurred in this run.

Assertions checked all 114 StaleGuard readiness decisions using both the exact
elapsed-time comparison and the integer control-boundary calculation. The TTL
helper was called exactly once at dispatch for each ready prediction and never
for an unavailable result. Reference traces verified original-command altitude
was applied at the scheduled action in all 242 original-command executions.
Every StaleGuard flight matched the complete actually executed No Recovery
trace when it accepted, or the common fallback trace when it rejected. The 49
unavailable cases matched No Recovery through the entire flight, excluding
both postponement and cancellation after a late result arrived.

Source/calibration/input and previous-result hashes passed unchanged checks.
Four existing temporal geometry/monitor tests passed. A checkpoint-resume audit
reloaded all completed records without executing additional flights.

Run from `examples/staleguard` in the existing `scg` environment:

```sh
python inspection_descent/diagnostics/short_delay_latency.py --stage predict
python inspection_descent/diagnostics/short_delay_latency.py --stage run
python inspection_descent/diagnostics/short_delay_latency.py --stage report
```

The first stage refuses to overwrite recorded timings. The run stage can
resume saved flights without repeating prediction timing. A fresh independent
run requires a separate output directory and result path.

Evidence: [diagnostic source](../diagnostics/short_delay_latency.py),
[published results with individual latencies](../result/short_delay_latency.json),
[342 actual execution records](../results/short_delay_latency/executions.jsonl),
[flight progress log](../results/short_delay_latency/run_progress.log), and
[test output](../results/short_delay_latency/unit_tests.log).
