# Inspection-descent experiment

Start with the [research recap](recap_visibility.md) to follow stages 1–10.
Detailed historical reports are grouped under `docs/`, with stage numbers in
their filenames. The [current implementation and validation report](docs/current_implementation_and_validation.md)
describes today's files and the latest executed checks. This README gives the
current run commands.

Run the official paired No Recovery / StaleGuard experiment from
`examples/staleguard`, using the existing `scg` environment:

```sh
python inspection_descent/staleguard_benefit.py
python inspection_descent/staleguard_benefit.py --count 20
```

The experiment reads `calibrated_scenarios.json`. It contains seeds 0–19;
`--count 10` uses seeds 0–9 and `--count 20` uses all twenty. Seed 16 has no
qualifying command and is reported separately. The twenty-seed experiment
therefore tests 19 scenarios and uses N=19 for mission and recovery rates.

The JSON stores each generated scenario, its selected command (`null` when
infeasible), and its longest safe D=0 inspection dwell in physics ticks.
At the existing 1000 Hz physics rate, one tick is one millisecond.
The file contains previously executed and verified calibration results.
It lives beside the code so it can be committed with the experiment.

## Manual calibration

To regenerate the JSON after changing calibration or mission settings, run:

```sh
python inspection_descent/max_dwell_calibration.py
```

This explicitly recalibrates seeds 0–19 in PyBullet, one scenario and candidate
at a time, and writes the JSON after all twenty searches finish. It limits
numerical-library threads to one. It does not use the old automatic cache.
The experiment never recalibrates automatically and does not check source
hashes, versions, or compatibility. You decide when to regenerate the file.

The shared maximum-dwell algorithm tests every candidate in the existing grid
at D=0. It selects the safe, inspection-completing command with the longest
consecutive inspection dwell. Ties keep the first command in the original
search order. Tests with a delay greater than zero and StaleGuard predictions
do not affect this selection. `plan_feasible_command()` remains available for
explicitly calibrating one scenario; the experiment does not call it.

The generator's initial 0.4–1.0 s low duration is not an operational maximum.
Calibration may increase it, while the low-altitude reference must end by
estimated nominal structure arrival at D=0, on the existing control grid.
Mission execution, safety and inspection checks, recovery, and metrics are
unchanged. Policy simulations still execute on every experiment run.

## Diagnostics and validation

Scripts under `diagnostics/` are separate from the official experiment.
`verify_feasibility.py` and `no_recovery_diagnostic.py` retain the historical
first-feasible rule. The sensitivity and comparison scripts retain their
original investigations. Previous results, including old cache files, remain
under `results/`; the official experiment does not read them.

```sh
python inspection_descent/diagnostics/validate_official_calibration.py --stage saved
python inspection_descent/diagnostics/validate_official_calibration.py --stage calibrate --seeds 0 16
python inspection_descent/diagnostics/validate_official_calibration.py --stage experiment
python inspection_descent/diagnostics/validate_official_calibration.py --stage delays
```

The `saved` check compares all twenty JSON records with prior measurements.
The `calibrate` check actually repeats the selected seeds' searches; without
`--seeds`, it repeats all twenty. The `experiment` check executes the official
experiment and compares every flight with the pre-cleanup execution code.
The `delays` check repeats the 304 previously recorded No Recovery cases.
These checks need the historical data under `results/`.

The validator runs one simulation at a time, limits numerical-library threads,
checks memory between simulations, and prevents overlapping validation runs.
Those protections apply to the validator. Results of the updated checks go to
`results/official_calibration_validation/saved_scenarios/`; older reports are
preserved. The existing git-ignore rule covers `results/`, but not the official
`calibrated_scenarios.json` file.
