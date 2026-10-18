# Current implementation and latest validation

This describes the current experiment after the automatic cache was replaced
by one JSON file. For the sequence of research decisions, see the
[recap](../recap_visibility.md).

The official experiment uses the command with the longest continuous
inspection time at zero delay, while still completing the mission safely.
The previous rule stopped at the first successful command.

## Current files

- `staleguard_benefit.py` reads the selected commands from
  `calibrated_scenarios.json`. It runs No Recovery and StaleGuard and reports
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

The UAV keeps flying forward during inspection. StaleGuard, mission execution,
recovery, controller settings, physics, inspection and safety checks, delays,
deadlines, and metric definitions remain unchanged. Physics-step monitoring,
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
