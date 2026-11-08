# Underpass evaluation results

- [baseline_benchmark/REPORT.md](baseline_benchmark/REPORT.md): completed 9,100-flight held-out benchmark.
- `baseline_benchmark/`: the eight original result files, plus validation after relocation.
- `baseline_benchmark.zip`: your original export, preserved byte-for-byte (its internal root retains the old versioned folder name).
- `calibration/`: the unchanged 865-scenario accepted evaluation calibration.
- `baseline_benchmark_smoke/`: original small evaluation smoke test.
- `preparation/`: freezes, preparation reports, audits and source backups, grouped away from the main results.

Flight records, numerical metrics, CSVs, reports and trajectories were moved without changing their bytes.
Only stored freeze paths in two run headers were updated. Original headers/source are in
`preparation/before_layout.zip`; reviewed path/source changes are documented in
`preparation/layout_migration.json` and `../../support/results_layout.json`.
Historical preparation reports retain their original text/paths. Current commands are in the mission README.
No simulations were rerun to reorganize these results.

Validate the completed benchmark from `examples/staleguard`:

```bash
conda run --no-capture-output -n scg python underpass_inspection/support/validate_baseline_benchmark.py \
  --calibration underpass_inspection/results/evaluation/calibration \
  --run underpass_inspection/results/evaluation/baseline_benchmark \
  --prior-hashes underpass_inspection/results/evaluation/preparation/historical_artifact_hashes_reduced_v3.json
```
