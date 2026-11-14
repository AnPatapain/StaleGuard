# StaleGuard

**StaleGuard: When to Stop Waiting -- Runtime Assurance for Queued UAV Commands**

An approved UAV command can become stale while waiting for an unknown dispatch time. 
StaleGuard predicts three time-dependent opportunities: safe command execution, safe mission completion, and safe recovery, 
then monitors the pending command to determine when to intervene before dispatch.

## Benchmark Missions

StaleGuard is evaluated on three UAV missions using Safe-Control-Gym and PyBullet physics.

|                  Inspection Descent                   |                Geofence Turn                |                   Underpass Inspection                    |
|:-----------------------------------------------------:|:-------------------------------------------:|:---------------------------------------------------------:|
| ![Inspection Descent](figures/inspection_descent.png) | ![Geofence Turn](figures/geofence_turn.png) | ![Underpass Inspection](figures/underpass_inspection.png) |

## Installation

Requires Python 3.10+ and the dependencies specified in `requirements-reproduction.txt`.

```bash
python -m pip install -r requirements-reproduction.txt
python -m pip install -e .
```

The original simulator implementation and required assets are included in `safe_control_gym/`.

## Reproduce Paper Results

Each script independently reconstructs the corresponding paper artifact from recorded experimental data.

```bash
# Figure 2 — Underpass flight trajectories
python -m staleguard.reproduce.fig2 --data-root data --out outputs/fig2

# Figure 3 — Physical feasibility opportunities
python -m staleguard.reproduce.fig3 --data-root data --out outputs/fig3

# Table II — Prediction accuracy and computational readiness
python -m staleguard.reproduce.table2 --data-root data --out outputs/table2

# Table III — Safety and mission completion across eight policies
python -m staleguard.reproduce.table3 --data-root data --out outputs/table3

# Figure 4 — Underpass outcomes versus dispatch delay
python -m staleguard.reproduce.fig4 --data-root data --out outputs/fig4

# Figure 5 — Safety–completion trade-off
python -m staleguard.reproduce.fig5 --data-root data --out outputs/fig5
```

Figures are exported as PDF/PNG, tables as CSV/Markdown, and all artifacts include their underlying numerical data.

These commands reconstruct results without rerunning simulations.

**Dataset:** See [data/README.md](data/README.md) for the complete dataset requirements, integrity verification, and availability of large experimental records.

## Run New Experiments

Run a physical PyBullet evaluation using the existing mission configurations and policies.

For example, a small Underpass benchmark:

```bash
python -m staleguard.reproduce.run_experiments \
  --data-root data \
  --mission underpass_inspection \
  --experiment benchmark \
  --seeds 1002 \
  --policies no_recovery staleguard_dual_horizon \
  --delays 0 \
  --max-records 2 \
  --workers 1 \
  --out outputs/runs/underpass-example
```

Supported experiment types: `benchmark`, `feasibility`, `prediction`, and `calibration`.

Use `--help` for additional configurations. New experiments require an empty output directory and never overwrite the archived data.

## PyBullet Visualization

Launch interactive UAV simulations:

```bash
# Inspection Descent
python -m staleguard.visualize --mission inspection_descent --seed 20 --delay 0

# Geofence Turn
python -m staleguard.visualize --mission geofence_turn --seed 22 --delay 0.5

# Underpass Inspection
python -m staleguard.visualize --paper-figure --overlay-all
```

Requires a desktop display. Use `python -m staleguard.visualize --help` for more options.

## Additional Information
- [Dataset documentation](data/README.md) — Experimental records and integrity verification.
- [Citation](CITATION.cff) — Citation information.
- [License](LICENSE) — License information.