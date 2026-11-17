# Experimental data

This folder contains the simulation results used in the StaleGuard paper. The original records are kept unchanged so that others can check the reported numbers.

## What's included

| Mission | Scenarios | Dispatch delays | Prediction delays |
| --- | ---: | ---: | ---: |
| Inspection Descent | 276 | 11 | 11 |
| Geofence Turn | 300 | 23 | 19 |
| Underpass Inspection | 400 | 13 | 19 |

Each mission has `calibration/`, `benchmark/`, and `feasibility/` folders. Some missions also have `additional_policies/` records used to verify reused flight outcomes. These records may include older policies that are not among the eight reported in the paper.

The files contain:

- `executions.jsonl`: flight outcomes and policy decisions.
- `predictions.jsonl`: predicted feasibility and recorded prediction times.
- `run.json` and `progress.json`: experiment settings and run information.
- Calibration files and snapshots: the scenarios, approval states, and controller states used in the experiments.
- `.npz` files: recorded states and trajectories.

The Underpass `representative_flights/` folder contains the flights shown in Figure 2. The Underpass calibration archive contains 865 accepted candidates; the benchmark uses the selected 400 evaluation scenarios.

## Robustness study

The [Underpass robustness dataset](underpass_inspection/robustness_v1/README.md) contains results from 64 scenarios evaluated under nominal conditions, two mass-mismatch levels (±5%, ±10%), and two levels of position and velocity observation error.

Six policies were evaluated across 13 dispatch delays, producing 24,960 policy outcomes.

The dataset includes four result tables, experiment settings, and a compressed archive (`records.tar.gz`) containing the underlying simulation and prediction records.

To verify the archive and reproduce the tables without rerunning simulations:

```bash
python -m staleguard.robustness verify-archive --data-root data

python -m staleguard.robustness reproduce \
  --data-root data \
  --out outputs/robustness_reproduced
```

The reproduction checks the results against the archived reference tables. The original benchmark remains unchanged.

The [nominal reproduction audit](underpass_inspection/robustness_v1/nominal_audit/REPORT.md) explains the small differences caused by separately measured prediction runtimes.

## Downloading the data

The data are included in this Git repository. Two large files are stored with **Git LFS**:

| File | Contents |
| --- | --- |
| `data/underpass_inspection/benchmark/executions.jsonl` | Original Underpass benchmark flight records |
| `data/underpass_inspection/robustness_v1/records.tar.gz` | E01 robustness simulation and prediction records |

You do not need a separate download link.

Install Git LFS, then run:

```bash
git lfs install
git clone https://github.com/AnPatapain/StaleGuard.git
cd StaleGuard
git lfs pull
```

Verify that both files are tracked and downloaded:

```bash
git lfs ls-files

wc -c data/underpass_inspection/benchmark/executions.jsonl
sha256sum data/underpass_inspection/benchmark/executions.jsonl

wc -c data/underpass_inspection/robustness_v1/records.tar.gz
sha256sum data/underpass_inspection/robustness_v1/records.tar.gz
```

The original Underpass benchmark journal should be **176,132,260 bytes**, with SHA-256:

```text
8692aa2d9b1bb7cfb8b111743f92aff0c329862292862ee7e84639e5bac8f2e6
```

The robustness archive's expected size and checksum are recorded in its dataset manifest. Verify them using:

```bash
python -m staleguard.robustness verify-archive --data-root data
```

We verified that the original Underpass benchmark journal can be downloaded from a fresh checkout and that its size and checksum match.

## Reproducing the paper results

Follow the installation instructions in the main `README.md`. Use Python 3.10 and the versions in `requirements-reproduction.txt`.

From the repository root, for example:

```bash
python -m staleguard.reproduce.table2 --data-root data --out outputs/table2
python -m staleguard.reproduce.table3 --data-root data --out outputs/table3
python -m staleguard.reproduce.fig4 --data-root data --out outputs/fig4
python -m staleguard.reproduce.fig5 --data-root data --out outputs/fig5
```

The main `README.md` also lists the commands for Figures 2 and 3. These scripts recreate tables and figures **from the saved experimental records**, without rerunning PyBullet flights. Tables II and III have been checked against the paper using a fresh checkout.

## Checking the records

The top-level `manifest.json` lists the expected file size and SHA-256 checksum for each original benchmark data file. The robustness dataset has a separate manifest in `underpass_inspection/robustness_v1/`. The reproduction scripts check the files they read and report missing or changed inputs.

The saved run files also record software versions and source-code hashes. The code has since been reorganized, so those historical hashes should not be assumed to match today's source files. Recreating figures and tables from the records is different from rerunning the original simulations. The latter requires checking the historical source and environment separately.

The local `.stage2_archive/` folder is not part of the public dataset and is not needed to recreate the reported tables and figures.