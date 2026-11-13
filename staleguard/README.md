# StaleGuard reproducibility package

StaleGuard is a root-level Python package beside `safe_control_gym/`. Run the
following commands from the **repository root**, with the existing `scg`
environment active. The relocation changes imports and filesystem paths only.
All missions, policies, scientific implementations and archived data are retained.

## Entry points

| Mission | Calibration | Prediction accuracy | Benchmark |
|---|---|---|---|
| Inspection Descent | `python -m staleguard.inspection_descent.calibration --help` | `python -m staleguard.inspection_descent.ttl_pred_accuracy.run --help` | `python -m staleguard.inspection_descent.baseline_benchmark.run --help` |
| Geofence Turn | `python -m staleguard.geofence_turn.calibration --help` | `python -m staleguard.geofence_turn.ttl_pred_accuracy.run --help` | `python -m staleguard.geofence_turn.baseline_benchmark.run --help` |
| Underpass | `python -m staleguard.underpass_inspection.calibration --help` | `python -m staleguard.underpass_inspection.support.feasibility_sweep --help` | `python -m staleguard.underpass_inspection.baseline_benchmark.run --help` |

Three-profile prediction is available through
`staleguard.model.mission_evaluation.prediction` for Inspection/Geofence and
`staleguard.underpass_inspection.baselines.recovery_horizons.predict` for Underpass.
The existing APIs and parameters are unchanged.

GUI figure capture (requires a desktop display):

```bash
python -m staleguard.underpass_inspection.pilot --gui --paper-figure \
  --mode dive --delay 0 --paper-at 1.8 --capture /tmp/underpass_fig3_early.png
```

The recorded flight overlays, scene styling and visual-only enlarged UAV remain
unchanged. This command replays one flight, not an evaluation benchmark.

## Archived results and fresh outputs

`results/`, historical journals, CSVs, snapshots, source archives and
`source_relocation.json` retain their original bytes. `source_root_relocation.json`
records exact pre/post relocation source hashes against the initial commit.
These are distinct source revisions, not byte-identical source files.
Snapshot readers map historical Python module names while reading; no old import
modules or `sys.path` modifications are installed.

Use each experiment's existing `--out` argument to select a **fresh directory**
for a new run, for example `/tmp/geofence_relocation_run`. Do not resume archived
experiments under changed source provenance or regenerate their metrics in place.
The archived table coordinator keeps its existing output defaults and will reject
incompatible source revisions. Its help is available with:

```bash
python -m staleguard.tables.complete_experiments --help
```

The existing consolidated aggregator also writes mission-side derivative metrics
and figure data, so a fresh table output directory alone does not isolate its writes.
It was not executed during relocation. Read archived tables and metrics directly;
use the per-mission metric functions for analysis without changing archived files. See the mission READMEs for existing arguments;
all relative data paths now start with `staleguard/` from the repository root.

## Relocation verification

See [RELOCATION_VALIDATION.md](RELOCATION_VALIDATION.md) for the initial revision,
validation results, known pre-existing failures and a complete modified-file list.
No cleanup, dependency update, algorithm change or benchmark rerun is part of this stage.
