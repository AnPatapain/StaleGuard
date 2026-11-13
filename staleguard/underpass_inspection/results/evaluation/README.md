# Underpass evaluation results

| Folder | Scenarios | Policies | Flights | Status |
|---|---:|---:|---:|---|
| [baseline_benchmark](baseline_benchmark/REPORT.md) | 100 | 9 | 11,700 | Complete, independently validated; exploratory |
| baseline_benchmark_v2 | 400 new | 9 | 46,800 planned | Prepared; no physical flights launched |
| calibration | 865 accepted | — | — | Original frozen cohort, unchanged |
| baseline_benchmark_smoke | Original small operational smoke | — | — | Preserved unrelated experiment |

There is one canonical historical nine-policy benchmark. All 11,700 scientific
records match the previous combined journal byte-for-byte; numerical metrics and
paired confidence intervals reproduce exactly. Old seven/two-policy source
folders and redundant ZIP exports have been deleted after validation.

The canonical folder bundles approval snapshots, forecasts, representative actual
trajectories, original experiment provenance and code/preparation evidence.
Validation and metrics-only regeneration do not depend on any deleted folder.
Old freezes, audits and source backups are archived in
`baseline_benchmark/source_versions.zip`; they are historical evidence, not active
compatibility machinery.

The v2 cohort uses RNG seed **20261109** to sample 400 accepted IDs uniformly
without replacement from the remaining 765, excluding all historical 100.
The cohorts have zero overlap and cover 500 unique scenarios. No selection used
policy outcomes. Its immutable configuration is `baseline_benchmark_v2/run.json`;
status and completed counts are in `baseline_benchmark_v2/progress.json`.

Use the single runner from `examples/staleguard`:

```bash
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark --validate
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark --metrics-only
```

The [mission README](../../README.md) contains v2 launch/resume commands and the
current result layout. No historical or v2 evaluation flight was rerun during
consolidation; physical smoke flights were development-only.
