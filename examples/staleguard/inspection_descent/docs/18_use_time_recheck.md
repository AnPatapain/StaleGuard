> Source relocation: this report preserves its historical findings and commands. Current code: `staleguard_benefit.py` → [simulation.py](../simulation.py); `use_time_recheck.py` → [use_time_recheck.py](../baselines/use_time_recheck.py). Old entry points have been removed.

# Experiment B: Use-Time Recheck — 22 October 2026

Use-Time Recheck is an opt-in fourth Experiment B policy. It performs one
immediate-execution model rollout from the actual dispatch state, withholds the
command during measured checking time, and applies the stored result once at
the next control opportunity. It never rechecks from the later application state.

The full experiment executed **836 policy flights**: 19 feasible calibrated
scenarios × 11 delays × 4 policies. Seed 16 remains infeasible and excluded.
The three existing policies matched all **627** saved corrected-B traces.
Five additional physical timing replays matched their reference traces.

## Literature and scope

[Agrawal, Chen, and Panagou, *gatekeeper: Online Safety Verification and Control
for Nonlinear Systems in Dynamic Environments*, IEEE T-RO, 2024](https://arxiv.org/abs/2211.14361)
motivates verification before committing a candidate trajectory while control
continues on the previous committed trajectory. The
[author's paper](https://dev10110.github.io/pdfs/2024-gatekeeper-tro.pdf) describes
that verification/control separation and its assumptions.

This locked baseline borrows that separation. It does not implement Gatekeeper's
complete committed-trajectory construction, terminal backup-set conditions, or
safety proof. In particular, it intentionally permits check-to-use staleness.
Its performance is evidence about this baseline, not a test of Gatekeeper's
full theoretical guarantees.

## Execution and timing

At the scheduled dispatch step, the existing executor captures the latest
PyBullet observation and PID integrator/attitude history. It copies the remaining
nominal reference and retains the original descent height and duration.
The existing snapshot record is reused with its capture-time field rebased
to dispatch. No approval-state rollout or future measured state is used.

`use_time_recheck.predict_immediate()` constructs the same lightweight model and
calls StaleGuard's existing `rollout_from_checkpoint()` exactly once at relative
step zero. This reuses the closed unsafe predicate and swept-segment checker.
There is no nominal pre-rollout or hypothetical delay sweep. Prediction retains
the common horizon ending at 8.5 s from mission start; it is a safety check,
not a mission-completion optimizer.

Measured wall time includes dispatch snapshot capture, model construction, and
the single rollout. CPU kernels are warmed before measurement, as in the other
experiments. The action step is:

```text
decision_step = scheduled_dispatch_step + floor(computation_s * 50) + 1
```

This is the first control tick strictly after completion. While the result is
pending, the original descent is withheld and all ordinary PID/physics updates
continue on the nominal forward reference at usual altitude. SAFE applies the
original descent at the decision step. UNSAFE activates the existing safety-only
fallback at that step. The measured result is never recomputed.

The scheduled dispatch remains fixed. Only application/cancellation is delayed.
The original command duration is unchanged, and the mission deadline stays
4.2 s after approval (absolute 4.20–4.78 s for these scenarios). A check finishing
after the deadline would never apply its decision; none did so in this run.

This is a **serial measured-time simulation**: prediction runs synchronously
in wall time, then its measured duration is represented by subsequent nominal
simulated control steps before its result may be used. This preserves simulated
state evolution and controller cadence; it is not an actual concurrent onboard
implementation or a wall-clock real-time PID scheduling benchmark. No artificial
computation penalty, physical sleep, or instantaneous computation is used.

## Actual results

Each policy has 209 scenario-delay pairs. Counts for safe completion, safe
non-completion, and unsafe execution are mutually exclusive and sum to 209.
The scenarios are reused across delays, so these are not 209 independent scenes.

| Policy | Unsafe | Safe completion | Safe non-completion | Safety fallback |
|---|---:|---:|---:|---:|
| StaleGuard | 2 (0.96%) | 59 (28.23%) | 148 (70.81%) | 127 (60.77%) |
| No Recovery | 128 (61.24%) | 59 (28.23%) | 22 (10.53%) | 0 |
| Always-Recover | 0 | 19 (9.09%) | 190 (90.91%) | 190 (90.91%) |
| Use-Time Recheck | 1 (0.48%) | 55 (26.32%) | 153 (73.21%) | 130 (62.20%) |

All 130 Use-Time Recheck fallbacks avoided unsafe entry. The single unsafe
flight had a SAFE prediction and executed the delayed original command.

| Actual dispatch delay D | StaleGuard complete / non-complete / unsafe | Use-Time Recheck complete / non-complete / unsafe | Recheck fallbacks |
|---:|---:|---:|---:|
| 0.0 s | 19 / 0 / 0 | 17 / 2 / 0 | 1 |
| 0.1 s | 16 / 3 / 0 | 16 / 3 / 0 | 1 |
| 0.2 s | 12 / 7 / 0 | 10 / 9 / 0 | 2 |
| 0.3 s | 5 / 14 / 0 | 5 / 14 / 0 | 6 |
| 0.4 s | 3 / 14 / 2 | 3 / 16 / 0 | 12 |
| 0.5 s | 3 / 16 / 0 | 3 / 16 / 0 | 15 |
| 0.6 s | 1 / 18 / 0 | 1 / 17 / 1 | 17 |
| 0.7 s | 0 / 19 / 0 | 0 / 19 / 0 | 19 |
| 0.8 s | 0 / 19 / 0 | 0 / 19 / 0 | 19 |
| 0.9 s | 0 / 19 / 0 | 0 / 19 / 0 | 19 |
| 1.0 s | 0 / 19 / 0 | 0 / 19 / 0 | 19 |

All cells use N=19. Full four-policy per-delay metrics are in the terminal log
and machine-readable results.

## Measured computation and application delay

There were 209 measured dispatch checks: 79 SAFE and 130 UNSAFE.

| Quantity | Mean | Median | p95 | Minimum | Maximum |
|---|---:|---:|---:|---:|---:|
| Snapshot plus immediate prediction | 3.407 ms | 1.397 ms | 7.356 ms | 0.643 ms | 10.789 ms |
| Dispatch-to-decision delay | 20 ms | 20 ms | 20 ms | 20 ms | 20 ms |
| Actual descent application delay, 79 SAFE cases | 20 ms | 20 ms | 20 ms | 20 ms | 20 ms |

All measured checks finished within one 20 ms control interval. The remaining
time until the control tick contributes to the check-to-use gap. Position
displacement between scheduled dispatch and decision reached 22.334 mm.
The motion metric covers the full dispatch-to-decision interval, including
control quantization, rather than just CPU computation time.

## Verified check-to-use failure

**Seed 11, D=0.6 s** is the only observed SAFE-to-unsafe check-to-use case.

- Approval: 0.58 s; scheduled dispatch: **1.18 s**, step 59.
- Snapshot check: **SAFE**; measured computation: **6.657 ms**.
- Original descent applied: **1.20 s**, step 60; unchanged duration: **1.54 s**.
- UAV position advanced from x=0.450855 m to x=0.458924 m before application:
  **8.069 mm** of displacement during the 20 ms gap.
- The paired immediate command at 1.18 s stayed safe through the unchanged
  deadline of **4.78 s**, although it did not complete inspection.
- The delayed command first entered the unsafe region at **2.888 s**:
  x=1.221359 m, y=-0.004102 m, z=0.908071 m. The region starts at
  x=1.221049 m and includes z≤0.922189 m and |y|≤0.35 m.

The delayed execution turns a safe non-completion into an unsafe execution.
It does not remove an otherwise safe mission completion. An extra native
No Recovery replay applying the command at 1.20 s matched the complete recheck
trajectory; an immediate 1.18 s replay matched the safe original trajectory.
This isolates application timing as the cause, without a second model check.

There is also one **already false-safe prediction at checking time**:
seed 18/D=0.4 s. Its immediate No Recovery execution is unsafe, but its delayed
recheck execution is safe. It is retained as a model-error observation rather
than counted as a SAFE prediction made unsafe by latency. This distinction
prevents reporting zero model error merely because most recheck flights are safe.

Seeds 7/8 at D=0.4 s, unsafe under StaleGuard, are predicted UNSAFE from the
new dispatch snapshots and safely cancelled. The model itself was not tuned.

## Completion consequences

Use-Time Recheck loses four safe completions relative to StaleGuard and
No Recovery on this grid:

- Seed 10/D=0: an UNSAFE prediction cancels an actually safe original command.
- Seed 1/D=0 and seeds 12/19 at D=0.2: SAFE predictions lead to delayed descent;
  the flights remain safe but no longer meet the unchanged inspection objective.

At D=0 the new policy still performs a nonzero-time check; it is therefore not
instantaneous like an idealized on-time counterfactual. Existing policies retain
their earlier D=0 behavior. There are no completion gains compensating for the
four losses.

## Implementation and validation

`staleguard_benefit.run_actual()` contains the optional pending-check branch,
sharing its ordinary PID, physics monitor, command application, and fallback.
`use_time_recheck.py` contains only the immediate-rollout helper and control-step
rounding rule. `--use-time-recheck` adds the fourth policy to safety-only B;
omitting the flag preserves the existing three-policy commands and reports.

All 209 live dispatch snapshots matched the actual observation at dispatch.
They retained live PID state, rebased remaining reference, and original descent
duration. Trace assertions verified identical pre-dispatch state histories,
nominal forward references and usual altitude during the wait, continued physics
ticks, correct application step, and fixed deadlines. Rejected trajectories
matched the existing safety-only fallback traces exactly.

The complete run was CPU-only, headless, serial, and retained the existing
2 GiB maximum-RSS / 2 GiB minimum-host-available memory guard and validation lock.
Steady process RSS was approximately 685 MiB. Eight unit tests passed, including
single-rollout swept geometry, safe-horizon completion, initial unsafe rejection,
and strict control-boundary timing. Five physical timing replays matched full
control and 1,000 Hz physics traces. The 627 existing-policy traces matched the
saved results, and all other recorded scientific-source/calibration/prior-result
hashes remained unchanged.

Existing model inaccuracies, finite prediction horizon, 1 ms actual monitoring,
warm host timings, control quantization, and deterministic simulator assumptions
remain limitations. One observed unsafe recheck rules out a blanket safety
guarantee for this locked one-shot design. No tuning or corrective revalidation
was added after observing it.

## Running and evidence

From `examples/staleguard` in the existing `scg` environment:

```sh
python inspection_descent/staleguard_benefit.py --count 20 --experiment B --use-time-recheck
python inspection_descent/diagnostics/validate_use_time_recheck.py --stage experiment
python inspection_descent/diagnostics/validate_use_time_recheck.py --stage timing
python inspection_descent/diagnostics/validate_use_time_recheck.py --stage report
python -m unittest discover -s inspection_descent/diagnostics -p 'test*.py' -v
```

The validation runner reuses verified platform/approval data, while each policy
flight still runs from mission start. Recheck snapshots and prediction times
are captured live in the executor. Per-flight checkpoints and measured StaleGuard
reception predictions support resumption without retiming completed results.
Earlier result files were preserved.

- [Actual four-policy terminal output](../result/baseline_use_time_recheck.log)
- [Metrics, measured timings, and unsafe-case records](../result/use_time_recheck.json)
- [836 flight records](../results/use_time_recheck/executions.jsonl)
- [Five timing replay records](../results/use_time_recheck/timing_replays.json)
- [Immediate case-11 trace](../results/use_time_recheck/staleness_11_immediate.npz)
- [Delayed case-11 trace](../results/use_time_recheck/staleness_11_delayed.npz)
- [Validation source](../diagnostics/validate_use_time_recheck.py)
