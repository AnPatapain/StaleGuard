# Independent Experiment B — completed evaluation archive

Exactly 300 seeds (20–319) were attempted; 276 were feasible. All six policies
were evaluated at eleven dispatch delays, producing 18,216 policy–delay results.
The experiment completed successfully on 2026-10-22 at 18:44:59 UTC.

```text
independent_evaluation_20_319/
├── main_result/           Final report, tables, figures and merged execution data
├── scenario_checkpoints/ Per-seed calibration and execution checkpoints
├── run_metadata/          Logs, validation evidence, status and launch records
└── README.md             This index
```

Start with [the detailed report](main_result/REPORT.md).
Other useful files are [aggregate results](main_result/aggregate.csv),
[per-delay results](main_result/per_delay.csv),
[the trade-off figure](main_result/safety_completion_tradeoff.png),
and [reconstructed execution times](main_result/execution_time.json).

## Scenario checkpoints

`scenario_checkpoints/SEED/` contains the original candidate journal and final
calibration choice; feasible scenarios also retain approval snapshots,
StaleGuard reception predictions and policy execution journals. These are the
same files formerly under `scenarios/`, renamed without changing their contents.
Keep them for reproducibility, detailed audit and possible checkpoint recovery.

## Run metadata

`run_metadata/` contains `run.log`, smoke/test logs, smoke-validation results,
`status.json`, `SUCCESS`, launch/inhibitor evidence, failure history and historical
PID/lock files. Recorded PIDs are historical identifiers, not proof of a process
currently running. [archive_manifest.json](run_metadata/archive_manifest.json)
records the organization, original/current file locations, hashes and checkpoint
integrity checks.

## Organization and runner compatibility

The original runner and launcher expect a flat output directory and a folder
named `scenarios`. Their scientific sources were left unchanged to preserve the
evaluated source hashes. This completed archive uses the clearer layout above;
do not run the original launcher directly against this reorganized directory.

If report regeneration or checkpoint recovery is needed later, first prepare a
**separate working copy** with the runner's original layout: place the 15 original
core outputs listed in `main_result/core_manifest.json` and the original files
listed in `run_metadata/archive_manifest.json` at its root, and copy
`scenario_checkpoints/` there as `scenarios/`. Do not change the canonical archive.
No rerun or resume is needed for this successful completed experiment.

Only byte-identical root copies already retained in `main_result/` were deleted.
Every other original root file was moved unchanged, and all checkpoint contents
were verified before and after the rename. No simulation, scientific source or
experimental result was changed by this organization.
