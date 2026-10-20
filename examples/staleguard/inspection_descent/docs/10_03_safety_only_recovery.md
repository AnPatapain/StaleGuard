# Safety-only recovery — primary experiment

The locked recovery objective is now safety: cancel the pending descent and
continue forward at usual height. Recovery does not stop the UAV, move back to
the inspection point, or attempt another inspection.

## Implementation

`staleguard_benefit.py` uses `safety-only` by default. Both StaleGuard and
Always-Recover call the same `run_actual()` implementation. On fallback, it
cancels the original descent, leaves the nominal horizontal references intact,
and requests usual height for the remaining flight. PyBullet and the PID
continue from the current physical state; there is no state or controller reset
when fallback starts. Safety and inspection monitoring continue through the
existing deadline, unless unsafe entry or simulator termination ends the run.

Always-Recover starts at receipt with zero decision time. StaleGuard starts
after its existing prediction computation, when its unchanged decision rule
requests recovery. Both still require recovery strictly before dispatch.
At D=0 neither can cancel the command, so all three policies execute it.

The existing stop-and-retry implementation remains under
`--recovery mission-retry`. Its original position/forward-speed gate, descent,
inspection attempt, and recovery-success definition remain intact. It shares
the same execution loop rather than duplicating the simulator. The original
runner was also saved under `results/safety_only_recovery/before/`.

No calibration, generated scenario, frozen JSON, prediction or decision logic,
original command, controller, dynamics, predicates, delay, or deadline changed.
Physics-rate monitoring, optional traces, and GUI branches are preserved.

## Outcome and fallback definitions

Every flight belongs to exactly one category:

- **Safe mission completion:** inspection completed and no unsafe entry.
- **Safe mission non-completion:** inspection did not complete and no unsafe entry.
- **Unsafe execution:** unsafe entry, whether or not inspection completed.

The program checks these definitions for each flight and verifies that the
three counts sum to the number of evaluated scenarios for each policy.

Fallback frequency is the number of started fallbacks divided by the number
of evaluated scenarios. Fallback success is the fraction of those started
fallbacks with no unsafe entry. It does not require inspection. When no
fallback starts, its success rate is `N/A (0/0)`.

Paired comparisons still use the same scenario and command. StaleGuard versus
No Recovery retains the existing safe-completion benefit, harm, and net
difference definitions and paired assertion. Their labels now say completion
benefit/harm/difference to distinguish them from safety-fallback success.
StaleGuard versus Always-Recover reports the safe-completion difference,
successes unique to each policy, and fallback-frequency difference.

The historical `mission-retry` report retains its earlier inspection,
unsafe-entry, safe-completion, recovery-frequency, and successful-inspection-
recovery metrics. Selecting a variant does not introduce another baseline.

## Reproduction and preserved results

From `examples/staleguard`, using the SCG environment:

```sh
# Primary safety-only experiment.
python inspection_descent/staleguard_benefit.py --count 20

# Historical stop-and-retry variant.
python inspection_descent/staleguard_benefit.py --count 20 --recovery mission-retry

# Serial validation, each command run after the previous one finishes.
python inspection_descent/diagnostics/validate_safety_only_recovery.py --stage small
python inspection_descent/diagnostics/validate_safety_only_recovery.py --stage experiment
python inspection_descent/diagnostics/validate_safety_only_recovery.py --stage historical
```

The validator writes new evidence to `results/safety_only_recovery/` and checks
that historical files and frozen scientific inputs have not changed. It uses
one simulation worker, one numerical-library thread, existing memory checks,
and the Always-Recover validator's launch lock.

The earlier calibration, mission-retry, and recovery-audit records remain in
their original locations. Historical diagnostics now explicitly select
`mission-retry`, so changing the primary default does not change their protocol.

## Executed validation

- Syntax, imports, and CLI options passed before simulation.
- The small controlled test executed 33 flights on seed 0. Setting StaleGuard
  computation time to zero only in this test produced six equal-time fallback
  pairs. Their complete control/reference/physics traces were identical.
- The official run executed **627 flights = 19 scenarios × 11 delays × 3
  policies**, with normal measured prediction time. Seed 16 remained explicitly
  infeasible. All policy percentages use N=19, not 20.
- All **311 flights without fallback**, including all 209 No Recovery flights,
  matched their historical outcomes and complete traces. StaleGuard's trigger
  decisions matched the previous saved run. At D=0, all three policies had
  identical traces and no fallback.
- Every execution passed the exclusive/exhaustive outcome checks. Printed
  outcome/fallback rows were independently checked against recorded flights;
  paired safe-completion identities passed at every delay.
- All 43 previously audited stop-and-retry recovery flights were executed
  through the retained variant and matched their original complete traces and
  outcomes. Historical files and frozen scientific inputs passed unchanged-file
  checks. The calibration code and JSON were preserved; no searches were rerun.
- One simulation worker was used. Full-run memory was about **1,050 MiB**.

The six equal-time comparisons belong to the controlled small test. Production
StaleGuard starts after computation; its timing rule was not changed.

## Actual primary results

Every cell below is **safe mission completion / safe mission non-completion /
unsafe execution**, out of 19. Each cell sums to 19. T=4.2 s throughout.

| Delay (s) | No Recovery | Always-Recover | StaleGuard |
|---:|---:|---:|---:|
| 0.0 | 19 / 0 / 0 | 19 / 0 / 0 | 19 / 0 / 0 |
| 0.1 | 16 / 2 / 1 | 0 / 19 / 0 | 16 / 3 / 0 |
| 0.2 | 12 / 5 / 2 | 0 / 19 / 0 | 12 / 7 / 0 |
| 0.3 | 5 / 8 / 6 | 0 / 19 / 0 | 5 / 13 / 1 |
| 0.4 | 3 / 4 / 12 | 0 / 19 / 0 | 3 / 14 / 2 |
| 0.5 | 3 / 2 / 14 | 0 / 19 / 0 | 3 / 16 / 0 |
| 0.6 | 1 / 1 / 17 | 0 / 19 / 0 | 1 / 18 / 0 |
| 0.7 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 |
| 0.8 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 |
| 0.9 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 |
| 1.0 | 0 / 0 / 19 | 0 / 19 / 0 | 0 / 19 / 0 |

No Recovery has zero fallbacks at every delay. At D=0, neither recovery policy
can intervene, and both fallback-success rates are N/A (0/0). At every positive
delay, Always-Recover triggers in 19/19 cases (100%) and avoids unsafe entry in
19/19 fallbacks (100%). These are safe non-completions, not inspection successes.

| Delay (s) | StaleGuard fallback frequency | StaleGuard fallback success: no unsafe entry | Safe-completion difference: StaleGuard minus Always-Recover |
|---:|---:|---:|---:|
| 0.0 | 0/19 (0.0%) | N/A (0/0) | +0.0 percentage points |
| 0.1 | 1/19 (5.3%) | 1/1 (100%) | +84.2 percentage points |
| 0.2 | 2/19 (10.5%) | 2/2 (100%) | +63.2 percentage points |
| 0.3 | 5/19 (26.3%) | 5/5 (100%) | +26.3 percentage points |
| 0.4 | 10/19 (52.6%) | 10/10 (100%) | +15.8 percentage points |
| 0.5 | 15/19 (78.9%) | 15/15 (100%) | +15.8 percentage points |
| 0.6 | 17/19 (89.5%) | 17/17 (100%) | +5.3 percentage points |
| 0.7 | 19/19 (100%) | 19/19 (100%) | +0.0 percentage points |
| 0.8 | 19/19 (100%) | 19/19 (100%) | +0.0 percentage points |
| 0.9 | 19/19 (100%) | 19/19 (100%) | +0.0 percentage points |
| 1.0 | 19/19 (100%) | 19/19 (100%) | +0.0 percentage points |

StaleGuard's safe-completion benefit, harm, and net difference against No
Recovery are zero at every delay. It preserves the same successful original
commands rather than recovering new inspection successes. Always-Recover has
no unique safe-completion successes over StaleGuard. At positive delays,
StaleGuard's unique successes over Always-Recover are 16, 12, 5, 3, 3, 1, 0,
0, 0, and 0 respectively.

Across eleven delays, each policy has 209 evaluated flights. StaleGuard has
59 safe completions, 147 safe non-completions, and three unsafe executions.
No Recovery has 59, 22, and 128 respectively. Always-Recover has 19, 190, and
zero. These totals reuse the same nineteen scenarios across delays; they are
not 209 independently generated scenarios.

StaleGuard therefore preserves 40 positive-delay safe mission completions that
Always-Recover cancels, and reduces unsafe executions compared with No
Recovery. It does not dominate Always-Recover on safety: the latter has zero
unsafe executions here, while StaleGuard has one at D=0.3 and two at D=0.4.
Those three flights did not start fallback. All 126 triggered StaleGuard
fallbacks and all 190 Always-Recover fallbacks avoided unsafe entry, as measured
by the existing monitor. This is not a guarantee for other states.

Pairing each cancellation with its No Recovery flight shows what was retained:
StaleGuard cancelled 125 commands that would enter the unsafe volume and one
that would be safe but miss inspection (seed 12, D=0.5 s). It cancelled no
command that would complete inspection safely. Always-Recover cancelled all
128 unsafe original executions and 62 safe ones, including 40 safe inspection
completions. These are measured comparisons, not assumptions that every
StaleGuard cancellation is necessary. Counts are saved in
[`paired_cancellation_checks.json`](../results/safety_only_recovery/paired_cancellation_checks.json).

The [complete actual terminal report](../result/baseline_safety_only.log)
contains all three-column counts, percentages, and paired comparisons.
Raw flights are in
[`official_experiment.jsonl`](../results/safety_only_recovery/official_experiment.jsonl),
with a [CSV export](../results/safety_only_recovery/per_scenario.csv).
Checks are saved in
[`official_experiment_checks.json`](../results/safety_only_recovery/official_experiment_checks.json),
[`small_checks.json`](../results/safety_only_recovery/small_checks.json), and
[`mission_retry_regression.json`](../results/safety_only_recovery/mission_retry_regression.json).

## Files changed

- `staleguard_benefit.py`: default safety fallback, preserved mission-retry
  selector, three outcome categories, and separate fallback safety success.
- `diagnostics/validate_safety_only_recovery.py`: new serial checks and records.
- `diagnostics/validate_always_recover.py`, `validate_official_calibration.py`,
  and `audit_recovery_realism.py`: explicitly select mission-retry when comparing
  earlier results. Their stored results were not overwritten.
- README, current-implementation report, recap, and handoff: identify the
  primary protocol and historical variant. The recovery audit retains its
  findings with a link to this later decision.

## Remaining limitations

These are deterministic simulator results with the existing exact state
feedback, actuator model, logical forbidden volume, and no wind or sensor
disturbances. Safe altitude is a requested reference, not a guarantee about
the actual flight; the unsafe-entry monitor determines the result.

The tested fallback starts before the original descent has been dispatched.
It therefore does not establish safe recovery from a descent already underway
or arbitrary initial states. StaleGuard still predicts unsafe execution rather
than inspection completion, and prediction errors remain possible. Its
measured computation time can vary across machines. No additional operating
limits, time costs, delays, or mission-completion requirements were introduced.
