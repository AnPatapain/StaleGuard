# StaleGuard

**StaleGuard: When to Stop Waiting—Runtime Assurance for Queued UAV Commands.**

An approved UAV command can become stale while it waits for an unknown dispatch
time. StaleGuard predicts safe command execution, safe mission completion and
safe recovery, then monitors the pending command to decide when to recover.

Three missions are included: Inspection Descent, Geofence Turn and Underpass
Inspection. The eight published policies are No Recovery, FBS, Use-Time Recheck,
Fixed EarlyRecovery (0.20 s), StaleGuard (Dispatch), StaleGuard (Command-safety
aware), StaleGuard (Recovery-aware) and StaleGuard (Dual horizon). The full policy
retains later successful completion opportunities; command and recovery guards
use their original conservative first-loss rules. Future dispatch time is unknown.

## Install

Use Python 3.10 or later, from this repository root:

```bash
python -m pip install -e .
```

The exact tested dependency versions are in `requirements-reproduction.txt`
(`python -m pip install -r requirements-reproduction.txt` selects that environment).
The validation environment is recorded in `VALIDATION.md`; original environments
remain in immutable dataset metadata. The vendored `safe_control_gym/`, its
Crazyflie assets and PID implementation are retained. Its MIT license is in `LICENSE`.

## Reconstruct the six empirical artifacts

See [data/README.md](data/README.md). Each command verifies its primary input
hashes and recalculates the scientific values. It does not simulate flights or
read old metrics/table/plot CSVs as calculation inputs.

```bash
python -m staleguard.reproduce.fig2   --data-root data --out outputs/fig2
python -m staleguard.reproduce.fig3   --data-root data --out outputs/fig3
python -m staleguard.reproduce.table2 --data-root data --out outputs/table2
python -m staleguard.reproduce.table3 --data-root data --out outputs/table3
python -m staleguard.reproduce.fig4   --data-root data --out outputs/fig4
python -m staleguard.reproduce.fig5   --data-root data --out outputs/fig5
```

Figures export PDF, PNG and supporting data; tables export CSV and Markdown.
Every command writes `metadata.json` with input hashes and the output code revision.
Outputs never go in `data/`. Figure 1 and Table I are conceptual/descriptive.
Figure 2 uses recorded Underpass seed-0 flights, separate from the evaluation cohort.

## Fresh physical experiments

Use a new, empty output directory. The default uses the published frozen cohort
and the original experiment-specific grid. A bounded smoke example:

```bash
python -m staleguard.reproduce.run_experiments --data-root data \
  --mission underpass_inspection --experiment benchmark --seeds 1002 \
  --policies no_recovery staleguard_dual_horizon --delays 0 --max-records 2 \
  --workers 1 --out outputs/runs/underpass-example
```

Experiments: `benchmark`, `feasibility`, `prediction`, `calibration`. Missions:
`inspection_descent`, `geofence_turn`, `underpass_inspection`. `--seeds`, `--delays`
and `--policies` restrict the immutable preset; omit `--max-records` for the full
selected workload. Multiple physics workers run in separate processes.
Calibration repeats the original search/acceptance rules for selected evaluation
seeds and writes new scenarios/snapshots. See `--help` for all arguments.

New warmed runtime measurements are fresh evidence, not substitutes for the
paper's archived timings. Existing and canonical records are never resumed or
overwritten. Full physical evaluation is expensive and unnecessary for artifact reconstruction.

## PyBullet GUI

On a machine with a desktop display:

```bash
python -m staleguard.visualize --paper-figure --overlay-all
python -m staleguard.visualize --mission geofence_turn --seed 22 --delay 0.5
python -m staleguard.visualize --mission inspection_descent --seed 20 --delay 0
python -m staleguard.visualize --help
```

Underpass retains bridge/pier geometry, actual UAV flight, dive and recovery
playback, recorded trajectory overlays, camera interaction and optional capture.
The base and enlarged UAV mesh are display-only. Headless Figure 2 needs no display.
`--mode`/`--start` select Underpass maneuvers; `--policy` selects Inspection/Geofence policy playback.

## Scientific limits

These are simulation results, not formal safety guarantees or real-flight
validation. Boundaries are sampled delays, with censoring reported explicitly.
Computation was timed offline and replayed at 50 Hz. Rates weight tested delays
equally; confidence intervals resample whole scenarios. Full StaleGuard does not
dominate every baseline on both safety and completion.

See [VALIDATION.md](VALIDATION.md) for verified results, equivalence checks,
data-distribution limits and GUI verification status.
