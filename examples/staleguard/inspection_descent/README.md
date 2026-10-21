# Inspection-descent experiment

The research experiment logs are in
[result/README.md](result/README.md), outside the Git-ignored `results/` folder.

The current predictor checks straight segments between consecutive 20 ms
predicted positions. TTL ground truth and operational execution share 1,000 Hz
physics-step safety monitoring. The dynamics model and safety boundaries are
unchanged. See [the correction and measured results](docs/13_temporal_safety_correction.md).
The new TTL research log is `result/ttl_accuracy_swept_physics.log`. The complete
corrected Experiment B revalidation is in
[the operational report](docs/14_corrected_experiment_b.md) and
`result/baseline_unknown_dispatch_swept.log`. Older TTL and operational logs
remain preserved before-change results; Experiment A has not been rerun.

Start with the [research recap](recap_visibility.md) to follow stages 1–10.
Detailed historical reports are grouped under `docs/`, with stage numbers in
their filenames. The [current implementation and validation report](docs/current_implementation_and_validation.md)
describes today's files and the latest executed checks. This README gives the
current run commands.

Run the official No Recovery / Always-Recover / StaleGuard experiment from
`examples/staleguard`, using the existing `scg` environment:

```sh
python inspection_descent/staleguard_benefit.py
python inspection_descent/staleguard_benefit.py --count 20
```

These commands run **Experiment A**, where the scheduled delay is known.
For **Experiment B**, where StaleGuard predicts at receipt without knowing
the actual delay and the scheduler enforces TTL at dispatch:

```sh
python inspection_descent/staleguard_benefit.py --count 20 --experiment B
```

If B's prediction is not ready at dispatch, the command executes on schedule;
there is no later cancellation. B uses only safety-only fallback. See
[Experiment B](docs/11_unknown_dispatch_delay.md) for its timing rules and
[the corrected revalidation](docs/14_corrected_experiment_b.md) for current results.

The separate [short-delay latency supplement](docs/15_short_delay_latency.md)
tests actual dispatch at 0, 20, 40, 60, 80, and 100 ms with the unchanged
predictor and fail-open policy. Its measured results are in
[short_delay_latency.json](result/short_delay_latency.json); official B results
remain preserved.

These commands use **safety-only recovery**, the primary experiment. Recovery
cancels the pending descent, requests usual height, and continues the nominal
forward trajectory. It does not stop or retry inspection. StaleGuard and
Always-Recover use the same implementation; actual safety is monitored in
PyBullet rather than assumed.

The historical stop-and-retry variant remains available separately:

```sh
python inspection_descent/staleguard_benefit.py --count 20 --recovery mission-retry
```

The experiment reads `calibrated_scenarios.json`. It contains seeds 0–19;
`--count 10` uses seeds 0–9 and `--count 20` uses all twenty. Seed 16 has no
qualifying command and is reported separately. The twenty-seed experiment
therefore tests 19 scenarios and uses N=19 for mission and recovery rates.

Both official experiments use **D=0.0–1.0 s in 0.1 s steps**: eleven delays,
defined once as `DELAY_GRID` in `experiment.py`. The benefit experiment uses
this same grid for prediction and actual dispatch.

The benefit experiment's mission deadline is fixed at **T=4.2 s from command receipt/approval**.
Only dispatch delay varies. The previous 5.0 s
deadline gave the same reported outcomes for the tested scenarios, so that
duplicate evaluation was removed. This does not establish that deadlines are
irrelevant to every possible scenario. Calibration already used T=4.2 s.

The JSON stores each generated scenario, its selected command (`null` when
infeasible), and its longest safe D=0 inspection dwell in physics ticks.
At the existing 1000 Hz physics rate, one tick is one millisecond.
The file contains previously executed and verified calibration results.
It lives beside the code so it can be committed with the experiment.

## Always-Recover baseline — Task 3.1

Always-Recover cancels every pending command and starts the selected recovery
at command receipt/approval, with zero decision-computation time. Recovery must
start strictly before dispatch. At D=0 it therefore cannot cancel the command;
the original command executes, consistently with StaleGuard's timing rule.

All three policies receive the same scenario, approved command, approval
snapshot, dispatch time, and deadline. Always-Recover calls the same
`run_actual()` function as StaleGuard; there is no separate recovery trajectory.
The primary report shows three exclusive outcomes: safe mission completion,
safe mission non-completion, and unsafe execution. It also shows fallback
frequency and fallback success, defined as no unsafe entry. A safe cancellation
without inspection counts as non-completion, not mission completion.

The existing StaleGuard versus No Recovery benefit, harm, and net benefit are
unchanged. The new paired section reports **StaleGuard minus Always-Recover**:
the difference in safe mission completion, successes unique to each policy,
and the difference in recovery frequency. Differences use the number of tested
scenarios and are expressed in percentage points; negative recovery-frequency
differences mean StaleGuard intervened less often.

Only Always-Recover has been added. No other new baselines or mission-completion
time metrics are implemented. See the [Task 3.1 report](docs/10_01_always_recover.md).

The [recovery realism audit](docs/10_02_recovery_realism_audit.md) describes the
historical stop-and-retry maneuver. Its implementation, mission-completion-based
recovery metrics, and results remain available through `--recovery mission-retry`
and `results/always_recover/`. The safety-only protocol and its executed results
are documented in [the safety-only report](docs/10_03_safety_only_recovery.md).

## TTL prediction accuracy

```sh
python inspection_descent/mission.py --calibrated --count 20
```

`mission.py` passes the shared delay grid to `staleguard.predict()` and to
`ground_truth.classify_scenario()`. Prediction and ground truth therefore test
the same eleven delays. `--calibrated` loads the same saved commands as the
benefit experiment and reports seed 16 as infeasible. It keeps the existing
8.5 s measurement horizon from mission start, distinct from the benefit
experiment's 4.2 s deadline after approval. Without `--calibrated`, the earlier
generated-command experiment remains available.

TTL classifications describe the tested range. For example, `ALWAYS_SAFE`
means safe at every tested delay up to 1.0 s, not at arbitrary future delays.

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
Calibration and the original command's execution, safety, and inspection checks
are unchanged. Policy simulations still execute on every experiment run.

## Diagnostics and validation

Scripts under `diagnostics/` are separate from the official experiment.
`verify_feasibility.py` and `no_recovery_diagnostic.py` retain the historical
first-feasible rule. The sensitivity and comparison scripts retain their
original investigations. Previous results, including old cache files, remain
under `results/`; the official experiment does not read them.

```sh
python inspection_descent/diagnostics/validate_safety_only_recovery.py --stage small
python inspection_descent/diagnostics/validate_safety_only_recovery.py --stage experiment
python inspection_descent/diagnostics/validate_safety_only_recovery.py --stage historical
```

These serial checks write to `results/safety_only_recovery/`. They verify the
primary experiment, original-command flight traces, outcome categories, printed
metrics, and the preserved mission-retry trajectories. The small test uses zero
StaleGuard computation time to compare equal-time fallback executions; the full
experiment uses normal measured prediction time. Historical files are checked
for changes and are not overwritten by this validator.

Earlier calibration and mission-retry validation commands remain available:

```sh
python inspection_descent/diagnostics/validate_official_calibration.py --stage saved
python inspection_descent/diagnostics/validate_official_calibration.py --stage calibrate --seeds 0 16
python inspection_descent/diagnostics/validate_official_calibration.py --stage experiment
python inspection_descent/diagnostics/validate_official_calibration.py --stage delays
python inspection_descent/diagnostics/validate_always_recover.py --stage small
python inspection_descent/diagnostics/validate_always_recover.py --stage experiment
```

The `saved` check compares all twenty JSON records with prior measurements.
The `calibrate` check actually repeats the selected seeds' searches; without
`--seeds`, it repeats all twenty. The older `experiment` checks explicitly
select the historical mission-retry variant and compare its flights with the
earlier execution code.
The `delays` check repeats the 304 previously recorded No Recovery cases.
These checks need the historical data under `results/`.

The validator runs one simulation at a time, limits numerical-library threads,
checks memory between simulations, and prevents overlapping validation runs.
Those protections apply to the validator. Results of the updated checks go to
`results/official_calibration_validation/delay_grid_0_to_1/`; older reports are
preserved. The existing git-ignore rule covers `results/`, but not the official
`calibrated_scenarios.json` file.

Always-Recover checks are saved separately under `results/always_recover/`.
The small regression sets StaleGuard's decision time to zero only in that
controlled test, allowing exact equal-time flight-trace comparisons. The full
experiment uses normal measured StaleGuard prediction time. The checks use
the preserved pre-baseline runner and earlier execution records in `results/`.
