> Source relocation: this report preserves its historical findings and commands. Current code: `staleguard_benefit.py` → [simulation.py](../simulation.py). Old entry points have been removed.

# Experiment B: unknown dispatch delay

Experiment A remains the known-scheduled-delay experiment. Experiment B uses
the same calibrated commands and three policies, but StaleGuard does not know
the actual dispatch delay when the command arrives.

## Execution flow

1. Capture the existing approval-time state and controller snapshot.
2. Call `predict_at_reception(snapshot, quad_params, pid_params)`. Its inputs
   contain no actual dispatch delay or scheduled dispatch timestamp. It calls
   the unchanged StaleGuard predictor over the fixed hypothetical delay grid.
3. Record actual prediction computation time. The simulator treats the result
   as unavailable until that much time has elapsed after receipt.
4. Let the scheduler reach the actual dispatch step. It computes waiting time
   from the current step minus the receipt step.
5. If prediction is ready and its TTL has expired, cancel the pending descent
   and start the shared safety-only fallback. Otherwise execute the original
   command at the scheduled step.

Prediction runs once per scenario before the scheduler evaluates the eleven
actual delay cases. The same approval-time prediction and measured computation
duration are reused across those cases. Simulation of each flight still runs
normally; the measured duration determines when the result becomes available,
not an extra pause or extension of the flight deadline.

The dispatch check uses only the stored approval-time TTL. It does not predict
again from the current state or introduce a Use-Time Recheck baseline.

## Timing and validity rules

- **Prediction not ready:** execute the original command on schedule. A result
  arriving later does not cancel or replace an already dispatched command.
  The terminal report counts unavailable predictions at every delay.
- **Prediction ready exactly at dispatch:** the scheduler may use it.
- **Finite TTL:** reject at the existing first predicted unsafe delay, including
  equality. Compare TTL and elapsed waiting time in control steps to avoid
  floating-point differences such as 0.6 versus 0.6000000000000001.
- **Non-monotonic prediction:** use the same first-unsafe cutoff. A later SAFE
  prediction does not reopen validity after the first unsafe point.
- **Invalid at approval or invalid nominal flight:** reject when a prediction
  is available at the dispatch check.
- **Always safe:** accept within the tested 0.0–1.0 s grid. This is not a claim
  about delays beyond the tested range.

Experiment A still permits recovery only strictly before its scheduled
dispatch step. B's new check is inside the dispatch event, before the original
altitude reference is applied. A rejected command therefore never reaches the
controller; fallback starts at that same step. No extra delay is introduced.
Always-Recover retains its existing immediate-recovery rule. At D=0 it cannot
cancel before dispatch; in the actual B run prediction also takes nonzero
time, so all three policies execute the original command.

The unavailable-prediction rule is an explicit protocol choice. Lack of a TTL
does not prove the command safe. The choice preserves scheduled dispatch and
allows late computation to expose risk rather than hiding it by postponement.

## Running the experiments

```sh
# Experiment A, unchanged default.
python inspection_descent/staleguard_benefit.py --count 20

# Experiment B, safety-only fallback.
python inspection_descent/staleguard_benefit.py --count 20 --experiment B
```

Both use maximum-D=0-dwell commands from the existing JSON, seeds 0–19,
D=0.0–1.0 s in 0.1 s steps, and T=4.2 s from receipt. Seed 16 is reported
infeasible; percentages use the nineteen evaluated scenarios. Actual dispatch
times, commands, snapshots, and deadlines match across the three policies.

The original command, StaleGuard prediction algorithm, calibration, dynamics,
safety/inspection predicates, and outcome definitions are unchanged. Both
recovery policies use the same safety-only execution branch. B explicitly
rejects selection of the historical mission-retry maneuver.

## Validation actually executed

- Syntax/import checks and the CLI passed before simulation.
- Fifty-five classification comparisons matched the existing Experiment A
  decision semantics, including invalid and non-monotonic cases.
- Five controlled timing cases were executed in PyBullet, each compared with
  the corresponding direct command or common-fallback execution. They covered
  prediction before dispatch, exactly at dispatch, after dispatch, D=0, and
  the floating-point boundary at 0.6 s. Expiry was checked inside the actual
  dispatch event. Late cases never accessed an unavailable result afterward.
- In the controlled late test, dispatch at 0.50 s with prediction unavailable
  until 0.52 s executed the command on time and entered the unsafe volume.
  This was a timing test, not an added production computation penalty.
- Thirty-three Experiment A flights matched the pre-change runner's complete
  traces and terminal output, using identical controlled computation timing.
  A historical mission-retry flight also matched its earlier complete trace.
- The complete Experiment B executed **627 flights: 19 feasible scenarios ×
  11 delays × 3 policies**. Both baselines matched their 418 Experiment A
  traces. Predictions were computed before any actual delay case, and each
  scenario used the same prediction and computation duration across all delays.
- Per-flight outcome identities, every printed outcome/fallback/availability
  column, and the paired-comparison assertions passed. Seed 16 remained
  explicitly infeasible. Existing experiment logs, calibration, prediction,
  and other scientific inputs passed unchanged-file checks.

All simulations ran serially with one worker and numerical-library threads
limited to one. Peak memory was about 1,050 MiB. Small checks ran before the
full experiment; no calibration searches were repeated.

## Actual results

Each cell is **safe mission completion / safe mission non-completion / unsafe
execution**, out of 19. Every cell sums to 19. T=4.2 s throughout.

| Delay (s) | No Recovery | Always-Recover | StaleGuard TTL |
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

Prediction computation was **31.3–76.2 ms**, with a mean of **56.6 ms**.
Predictions were unavailable for all 19 D=0 cases and available for all 190
positive-delay StaleGuard cases. The production grid therefore did not contain
a positive delay shorter than prediction computation; that case was covered
by the controlled timing test instead.

StaleGuard fallback counts at D=0.0, 0.1, …, 1.0 s were **0, 1, 2, 5, 10, 15,
17, 19, 19, 19, 19**. Every triggered StaleGuard fallback avoided unsafe entry
(126/126). Always-Recover triggered in 19/19 cases at every positive delay;
all 190 fallbacks avoided unsafe entry. No Recovery never triggered fallback.
These are fallback safety successes, not inspection completions.

StaleGuard's safe-completion difference against No Recovery was zero at every
delay, with zero completion benefit/harm. Against Always-Recover, its
safe-completion differences were +0.0, +84.2, +63.2, +26.3, +15.8, +15.8, +5.3,
+0.0, +0.0, +0.0, and +0.0 percentage points respectively. No case completed
safely only under Always-Recover.

All 627 outcomes and physical flight traces matched Experiment A's saved
results, although StaleGuard's fallback start moved to the dispatch event.
This is consistent with the model: every positive delay exceeded prediction
time, the TTL decisions agreed, and cancelling the pending descent leaves
the same nominal forward flight whether cancellation happens early or at the
dispatch check. It does not establish equivalence for other computation times,
states, or delay ranges. StaleGuard still had three unsafe executions without
fallback: one at D=0.3 and two at D=0.4 s.

The [complete actual terminal output](../result/baseline_unknown_dispatch.log)
is in the non-ignored `result/` folder beside the unchanged Experiment A logs.

## Files changed

- `staleguard_benefit.py`: separate B mode, reception-only prediction helper,
  dispatch-time TTL gate, and prediction-readiness reporting. A remains default.
- `diagnostics/validate_unknown_dispatch.py`: serial timing tests, A regression,
  and full B execution checks; its records stay separate from experiment logs.
- README, result index, recap, and handoff: explain the two experiment modes
  and link the new result. No historical diagnostic source was changed.

## Remaining limitations

Prediction time is measured on this host after the existing JIT warmup, not on
a specified onboard computer. It can differ across machines and runs. The
simulator models result availability using that measurement; it does not run a
second simulation worker concurrently with prediction.

The result applies to the existing deterministic scenarios, bounded delay
grid, exact state feedback, and simulator assumptions. Safety fallback is not
a guarantee against unsafe entry. A TTL can also be wrong, and an unsafe event
while waiting cannot be undone by a later dispatch-time check.
