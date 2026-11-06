> Source relocation: this report preserves its historical findings and commands. Current code: `experiment.py` → [scenario.py](../scenario.py); `staleguard_benefit.py` → [simulation.py](../simulation.py); `mission.py` → [simulation.py](../simulation.py); `max_dwell_calibration.py` → [calibration.py](../calibration.py). Old entry points have been removed.

# Current implementation and latest validation

This describes the current experiment after the automatic cache was replaced
by one JSON file and the mission deadline was fixed at 4.2 s. For the sequence
of research decisions, see the
[recap](../recap_visibility.md).

The primary experiment now uses safety-only recovery: cancel pending descent,
request usual height, and continue the nominal forward trajectory. Both
StaleGuard and Always-Recover use it. Fallback success means no unsafe entry,
not completed inspection. The historical stop-and-retry implementation remains
available with `--recovery mission-retry`, alongside its original results.
See the [safety-only protocol and validation](10_03_safety_only_recovery.md).

Task 3.1 introduced Always-Recover; its original mission-retry results are in
the [baseline report](10_01_always_recover.md). The current delay grid is
**0.0–1.0 s in 0.1 s steps**, shared by TTL prediction accuracy and StaleGuard
benefit through `experiment.py`. Earlier checks below
record the settings used at each preceding step.

The official experiment uses the command with the longest continuous
inspection time at zero delay, while still completing the mission safely.
The previous rule stopped at the first successful command.

## Current files

- `staleguard_benefit.py` reads the selected commands from
  `calibrated_scenarios.json`. It runs No Recovery, Always-Recover, and StaleGuard and reports
  their mission and recovery results.
- `calibrated_scenarios.json` contains the previously verified results for
  seeds 0–19, including each original scenario, selected command, and longest
  safe inspection time at zero delay. Seed 16 has no qualifying command.
- `max_dwell_calibration.py` contains the existing calibration algorithm.
  Run it manually to regenerate the JSON. It tests every allowed command at
  zero delay and keeps the safe, inspection-completing command with the longest
  continuous inspection time. Search order and ties are unchanged.

The automatic cache code was removed. Normal experiment runs do not search
for commands, compare code hashes, or recalibrate after code changes. You
choose when to regenerate the JSON.

The initial 0.4–1.0 second low duration is not a maximum. Longer durations are
allowed, but the low-altitude reference must end by estimated arrival at the
structure at zero delay, using the existing control-step rounding. Results of
later tests with a delay greater than zero do not affect command selection.

## What stayed the same

The original command keeps the UAV flying forward during inspection. The
primary safety fallback also preserves the nominal forward references. The
historical mission-retry variant holds the inspection position, as measured
in the [recovery realism audit](10_02_recovery_realism_audit.md).
StaleGuard, calibration, original-command execution, controller settings,
physics, inspection and safety checks, delays, deadlines, and safe-completion
definitions remain unchanged. Physics-step monitoring,
`trace`, `physics_trace`, and GUI functionality remain available.

Seed 16 is reported separately rather than counted as an unsafe flight.
Running all twenty seeds tests nineteen scenarios, so mission percentages use
nineteen as their total.

## Reproducibility

The JSON was copied from existing simulation results and checked against the
previously verified selections for all twenty seeds. Earlier calibration
validation executed all 2,241 candidate commands. Earlier flight comparisons
and the old cache files remain under `results/`.

Diagnostic scripts remain under `diagnostics/`. The updated validator checks
the saved JSON, can repeat calibration explicitly, and compares actual flights
with earlier execution code. Its memory checks and launch protections apply
to validation runs, not direct runs of `staleguard_benefit.py`.

## Checks after replacing the cache

- All twenty JSON entries matched the previously verified calibration records.
- Fresh searches for seeds 0 and 16 repeated 59 candidate simulations. Every
  candidate outcome and inspection dwell matched earlier measurements, and
  the collected results matched the new JSON.
- The twenty-seed experiment executed 228 policy simulations, each compared
  with an additional execution using the earlier flight code. All outcome
  flags and recorded flight positions matched exactly.
- All six printed metric blocks were identical to the earlier experiment.
  The paired benefit calculation and each flight's safe-completion calculation
  passed their assertions.

These checks used one simulation worker. The 2,241-command calibration search
and the 304-case delay check were not repeated in full for this file-format
change; their earlier verified results were preserved. New experiment output
is in `results/official_calibration_validation/saved_scenarios/official_experiment.log`.

The normal `--count 10` command also completed successfully: six metric blocks,
120 policy executions, 42.94 seconds on this machine.
Its full output is saved in the same directory as `cli_count10.log`.

## Fixed mission deadline

The official experiment now fixes T=4.2 s from command receipt/approval and
varies only the existing dispatch delays D=0.0, 0.5, and 1.0 s. The previous
T=5.0 s evaluations gave the same outcomes for these tested scenarios and
were removed. This observation does not prove deadline independence for
other scenarios or deadlines.

The calibration deadline remains 4.2 s. Saved commands, flight dynamics,
StaleGuard, recovery, and metric definitions are unchanged. With twenty seeds,
nineteen feasible scenarios now produce 114 policy executions across three
delay settings, instead of 228 across six delay/deadline combinations.

The fixed-deadline twenty-seed experiment was executed with one worker. All
114 policy executions matched the earlier flight code, including complete
recorded flight traces. Their individual outcomes and all three printed metric
blocks also matched the previous T=4.2 s experiment. The paired benefit and
per-flight safe-completion assertions passed. The saved calibration JSON is
unchanged.

| Delay (s) | No Recovery safe completions | StaleGuard safe completions | Net benefit (percentage points) |
|---:|---:|---:|---:|
| 0.0 | 19/19 | 19/19 | +0.0 |
| 0.5 | 3/19 | 18/19 | +78.9 |
| 1.0 | 0/19 | 19/19 | +100.0 |

See the [full terminal output](../results/official_calibration_validation/fixed_deadline/official_experiment.log)
and [validation summary](../results/official_calibration_validation/fixed_deadline/validation_summary.json).
The earlier two-deadline results are preserved in their original directories.

## Shared delay grid for both experiments

Both official experiments now use D=0.0–1.0 s, inclusive, in 0.1 s steps.
The eleven values are defined once in `experiment.py`. The separate three-value
benefit grid was removed. Prediction, ground-truth evaluation, and actual
benefit-policy dispatch all use the shared grid.

Before this change, TTL accuracy used 0.0–1.5 s in 0.1 s steps.
`mission.py` already passed that shared grid to `staleguard.predict()` and
`ground_truth.classify_scenario()`, so those three files needed no changes.
The benefit experiment had used the larger grid for prediction but only
0.0, 0.5, and 1.0 s for actual dispatch.

The benefit deadline remains 4.2 s from approval. TTL accuracy keeps its
existing generated commands and 8.5 s measurement horizon. It does not use
the benefit experiment's calibrated commands. TTL classifications apply only
to the tested range: `ALWAYS_SAFE` now means safe at every tested delay up to
1.0 s.

Calibration, mission execution, recovery, safety/inspection predicates, and
metric definitions were preserved. All 2,241 candidate commands in the
calibration grid were compared with the saved candidate lists and are
unchanged; no recalibration was needed and the saved JSON is unchanged.

The TTL accuracy experiment was executed on seeds 0–9. All ten scenarios
received the same eleven delays in prediction and ground truth: 110 delay
comparisons and 120 real ground-truth simulations, including ten nominal
flights without the descent command. Scenario classification matched in
10/10 cases. Delay-level false-safe predictions were 3/74 unsafe cases;
false rejections were 0/36 safe cases. The existing summary reported TTL
MAE 0.037 s across eight comparable cases, with maximum error 0.100 s.

See the [TTL terminal output](../results/official_calibration_validation/delay_grid_0_to_1/ttl_accuracy.log)
and [per-scenario validation records](../results/official_calibration_validation/delay_grid_0_to_1/ttl_validation.json).
The validation source is preserved with the generated results as
`verify_shared_grid.py`; it does not participate in either official experiment.

The benefit experiment was then executed on seeds 0–19: nineteen feasible
scenarios and seed 16 explicitly infeasible. Both policies executed all eleven
delays, giving 418 actual policy simulations. Each flight's safe-completion
identity and all paired benefit assertions passed. All 209 No Recovery outcomes
matched the earlier saved delay-sweep data; all 114 policy outcomes at the
previous delays 0.0, 0.5, and 1.0 s matched their earlier results. Both experiments
were checked serially, with one simulation worker.

| Delay (s) | No Recovery safe completions | StaleGuard safe completions | No Recovery unsafe | StaleGuard unsafe | Recoveries | Net benefit (percentage points) |
|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 19/19 | 19/19 | 0/19 | 0/19 | 0/19 | +0.0 |
| 0.1 | 16/19 | 17/19 | 1/19 | 0/19 | 1/19 | +5.3 |
| 0.2 | 12/19 | 14/19 | 2/19 | 0/19 | 2/19 | +10.5 |
| 0.3 | 5/19 | 10/19 | 6/19 | 1/19 | 5/19 | +26.3 |
| 0.4 | 3/19 | 13/19 | 12/19 | 2/19 | 10/19 | +52.6 |
| 0.5 | 3/19 | 18/19 | 14/19 | 0/19 | 15/19 | +78.9 |
| 0.6 | 1/19 | 18/19 | 17/19 | 0/19 | 17/19 | +89.5 |
| 0.7 | 0/19 | 19/19 | 19/19 | 0/19 | 19/19 | +100.0 |
| 0.8 | 0/19 | 19/19 | 19/19 | 0/19 | 19/19 | +100.0 |
| 0.9 | 0/19 | 19/19 | 19/19 | 0/19 | 19/19 | +100.0 |
| 1.0 | 0/19 | 19/19 | 19/19 | 0/19 | 19/19 | +100.0 |

Full inspection-completion, recovery-success, benefit, and harm counts are
in the [benefit terminal output](../results/official_calibration_validation/delay_grid_0_to_1/official_experiment.log).
See also the [benefit validation records](../results/official_calibration_validation/delay_grid_0_to_1/benefit_validation.json)
and [combined validation summary](../results/official_calibration_validation/delay_grid_0_to_1/validation_summary.json).

## Always-Recover — Task 3.1

The official report now compares No Recovery, Always-Recover, and StaleGuard.
Always-Recover starts the same recovery at receipt with zero computation time,
only before dispatch. At D=0 all three policies execute the original command.
The previous StaleGuard versus No Recovery metrics remain, with an additional
paired StaleGuard versus Always-Recover comparison.

The full nineteen-scenario, eleven-delay run executed 627 policy simulations.
All 418 prior StaleGuard/No Recovery outcomes matched. The small controlled
regression matched 22 old flight traces, six equal-time recovery pairs, and
all three policies at D=0. Printed metrics matched the actual records.
Always-Recover achieved 19/19 safe completions at every positive delay;
StaleGuard intervened less at D=0.1–0.6 s but had fewer safe completions there.

See [Task 3.1 methods, measured results, and evidence](10_01_always_recover.md).
No other baselines or mission-completion-time metrics were implemented.
