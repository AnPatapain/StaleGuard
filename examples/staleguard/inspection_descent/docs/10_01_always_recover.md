# Stage 10.1: Always-Recover baseline

Task 3.1 adds only Always-Recover to the official benefit experiment.
Its purpose is to measure whether selective StaleGuard recovery improves safe
mission completion or reduces interventions compared with immediate recovery
of every pending command.

## Policy and shared execution

Always-Recover's decision is unconditional, with zero computation time. Its
recovery start is the command's receipt/approval step. It may cancel the command
only when that step is strictly before scheduled dispatch. At D=0, recovery is
therefore unavailable and the original command executes.

The baseline calls the same `run_actual()` function as StaleGuard. That function
and its recovery trajectory, dynamics, safety and inspection predicates, physics
monitoring, and GUI behavior are unchanged. Each policy receives the same
scenario, command, snapshot, dispatch time, and deadline.

The existing saved maximum-D=0-dwell calibrations are retained. The delay grid
is 0.0–1.0 s in 0.1 s steps; T=4.2 s remains relative to approval. Seed 16 is
still explicitly infeasible, leaving 19 tested scenarios from seeds 0–19.

## Reporting

Mission outcomes have three columns: No Recovery, Always-Recover, StaleGuard.
All counts come from their respective executions. Recovery frequency and
successful-recovery counts are shown in the same order. When no recovery
occurs, its conditional success rate is `N/A (0/0)`.

The StaleGuard versus No Recovery benefit, harm, and net-benefit definitions
remain unchanged. The additional paired comparison reports:

- StaleGuard's safe-completion percentage minus Always-Recover's.
- Cases where only StaleGuard safely completes the mission.
- Cases where only Always-Recover safely completes the mission.
- StaleGuard's recovery frequency minus Always-Recover's, in percentage points.

Both paired differences are checked against the corresponding per-flight
outcomes. Fewer interventions alone do not establish that every avoided
intervention was unnecessary; safe mission completion is reported separately.

## Small regression actually executed

The pre-baseline runner and the new runner were each executed on seed 0 across
all eleven delays: 22 old-policy flights and 33 three-policy flights. All 22
StaleGuard/No Recovery outcome flags and complete flight traces matched.

For this controlled regression only, StaleGuard decision time was set to zero.
Six cases then had both recovery policies start at the same time. Every pair
had identical control-state, reference, and physics-position traces. At D=0,
all three policies had identical traces and no recovery occurred.

The full experiment uses normal measured StaleGuard prediction time; the
zero-time setting is confined to this regression check.

## Reproduction and evidence

From `examples/staleguard`, using the SCG environment:

```sh
python inspection_descent/staleguard_benefit.py --count 20
python inspection_descent/diagnostics/validate_always_recover.py --stage small
python inspection_descent/diagnostics/validate_always_recover.py --stage experiment
```

Validation runs one simulation at a time and reuses the existing memory checks.
It needs the preserved pre-baseline runner and previous results under `results/`.
Small regression evidence is in [small_checks.json](../results/always_recover/small_checks.json).

Other baselines and mission-completion-time metrics are outside this task.

## Full experiment actually executed

All 19 feasible scenarios were executed under all three policies at all eleven
delays: **627 policy simulations**. Seed 16 was reported as infeasible and was
not counted as an unsafe flight. The full run used normal measured StaleGuard
prediction time, not the controlled zero-time clock used by the small test.

All **418 existing StaleGuard and No Recovery outcomes**, including inspection
completion, unsafe entry, recovery trigger, and safe completion, matched the
previous eleven-delay experiment. Their recovery decisions also matched. For
each of the nineteen scenarios, Always-Recover's complete flight trace was
identical across its ten positive-delay runs: its immediate cancellation makes
the unused original dispatch time irrelevant to that recovery flight.

All per-flight safety/completion identities and both paired-comparison
assertions passed. The three printed outcome columns, recovery counts,
conditional recovery-success counts, and paired comparisons were independently
checked against the recorded executions. Saved calibrations and all mission,
controller, predictor, and dynamics source files were unchanged. The largest
process RSS reported during the full run was about 1,033 MiB, with one worker.

Safe mission completions and recovery counts below are out of **19**:

| Delay (s) | No Recovery safe | Always-Recover safe | StaleGuard safe | Always-Recover recoveries | StaleGuard recoveries |
|---:|---:|---:|---:|---:|---:|
| 0.0 | 19/19 | 19/19 | 19/19 | 0/19 | 0/19 |
| 0.1 | 16/19 | 19/19 | 17/19 | 19/19 | 1/19 |
| 0.2 | 12/19 | 19/19 | 14/19 | 19/19 | 2/19 |
| 0.3 | 5/19 | 19/19 | 10/19 | 19/19 | 5/19 |
| 0.4 | 3/19 | 19/19 | 13/19 | 19/19 | 10/19 |
| 0.5 | 3/19 | 19/19 | 18/19 | 19/19 | 15/19 |
| 0.6 | 1/19 | 19/19 | 18/19 | 19/19 | 17/19 |
| 0.7 | 0/19 | 19/19 | 19/19 | 19/19 | 19/19 |
| 0.8 | 0/19 | 19/19 | 19/19 | 19/19 | 19/19 |
| 0.9 | 0/19 | 19/19 | 19/19 | 19/19 | 19/19 |
| 1.0 | 0/19 | 19/19 | 19/19 | 19/19 | 19/19 |

The paired comparison uses StaleGuard minus Always-Recover. Differences in
rates are percentage points; unique-success columns are execution counts.

| Delay (s) | Safe-completion difference | StaleGuard-only successes | Always-Recover-only successes | Recovery-frequency difference |
|---:|---:|---:|---:|---:|
| 0.0 | +0.0 | 0/19 | 0/19 | +0.0 |
| 0.1 | -10.5 | 0/19 | 2/19 | -94.7 |
| 0.2 | -26.3 | 0/19 | 5/19 | -89.5 |
| 0.3 | -47.4 | 0/19 | 9/19 | -73.7 |
| 0.4 | -31.6 | 0/19 | 6/19 | -47.4 |
| 0.5 | -5.3 | 0/19 | 1/19 | -21.1 |
| 0.6 | -5.3 | 0/19 | 1/19 | -10.5 |
| 0.7 | +0.0 | 0/19 | 0/19 | +0.0 |
| 0.8 | +0.0 | 0/19 | 0/19 | +0.0 |
| 0.9 | +0.0 | 0/19 | 0/19 | +0.0 |
| 1.0 | +0.0 | 0/19 | 0/19 | +0.0 |

In this run, selective StaleGuard recovery did **not** improve safe mission
completion over Always-Recover. It matched Always-Recover at D=0 and
D=0.7–1.0 s. At D=0.1–0.6 s it intervened less often, but had fewer safe mission
completions. There were no StaleGuard-only successes.

For example, at D=0.1 s StaleGuard recovered in 1/19 cases and safely completed
17/19 missions; Always-Recover recovered in 19/19 and safely completed 19/19.
StaleGuard used eighteen fewer interventions, with two fewer safe completions.
No Recovery itself safely completed 16/19 missions at that delay.

The results therefore show a trade-off between intervention frequency and
safe mission completion. A flight may remain safe while failing inspection,
so reporting unsafe entry alone would miss part of that trade-off. These
measurements concern these scenarios, commands, recovery policy, and deadline.
They do not establish results for unimplemented baselines.

Actual output and records:

- [Full three-column terminal output](../results/always_recover/official_experiment.log).
- [All 627 per-flight records](../results/always_recover/official_experiment.jsonl).
- [Measured results and regression checks](../results/always_recover/experiment_checks.json).
- [Printed-report checks](../results/always_recover/report_checks.json).
- [Controlled equal-time regression output](../results/always_recover/small_after.log).
