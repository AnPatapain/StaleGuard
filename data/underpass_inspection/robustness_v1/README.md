# Underpass robustness study (E01)

This is the locked robustness dataset used in the StaleGuard paper. It contains the completed evaluation of 64 scenarios selected from the original 400-scenario Underpass cohort with selection seed 2027. All six policies were evaluated at the original 13 dispatch delays under nominal conditions, physical mass errors of ±5% and ±10%, and two levels of supervisory observation bias. The study contains 24,960 policy outcomes and 320 full three-opportunity forecasts.

The results are preserved exactly as recorded. Prediction readiness is reconstructed from the original measured computation times, rather than measured again on the reader's machine.

## Reading the results

The CSV and Markdown files in this directory are the paper's reference tables:

| Table | What it reports |
| --- | --- |
| E01-A | Command, completion and recovery prediction accuracy, sampled boundary errors, runtime and readiness |
| E01-B | Unsafe flights, safe completion and recovery outcomes for each condition and policy, including separate mass-sign summaries |
| E01-C | Paired policy comparisons and 95% confidence intervals from 2,000 whole-scenario bootstrap resamples |
| E01-D | Scenario coverage, prediction computations, physical reuse, runtime and worker memory |

`REPORT.md` and `VALIDATION.json` are unchanged copies of the completed experiment's report and validation record. Missing boundary errors remain unobserved values; they have not been replaced with zero.

The nominal reference uses freshly measured full-predictor readiness shared across the three StaleGuard configurations. Its feasibility profiles match the original benchmark, but 50 actions and five safety outcomes differ because of readiness timing. The [nominal reproduction audit](nominal_audit/REPORT.md) explains those cases and the independent physical checks. Safe-completion counts are unchanged. The original benchmark and these robustness results have both been retained without adjustment.

## Underlying evidence

`records.tar.gz` preserves the original per-record files, including forecasts, physical continuations, policy decisions, outcomes, H2 comparisons, true approval snapshots, physical-execution measurements and invocation metadata. It also contains the frozen specification and selected seeds, the original provenance and checksums, and the nominal audit evidence. Files keep their original contents and paths within the bundle. Caches, terminal state, console logs and redundant JSONL exports have been left out.

The bundle contains **67,867 files** and is **40,350,457 bytes**. Its SHA-256 is:

```text
9c6ba5d699a092b2c5fa0ab8b93801ff1274e6c78d9cb589ad561f96c46b6ecc
```

The separate `manifest.json` records this checksum, the reference-file checksums, the study configuration and the original specification checksum. The pre-existing `data/manifest.json` has not been changed, so its historical checksum remains valid.

## Reconstructing the tables

With the repository installed and Git LFS data downloaded, run these commands from the repository root:

```bash
python -m staleguard.robustness verify-archive --data-root data
python -m staleguard.robustness reproduce \
  --data-root data \
  --out outputs/robustness_reproduced
```

The first command checks every reference file and every record inside the compressed archive. The second extracts the evidence into a fresh output directory and uses the existing analysis functions to recalculate the tables. It runs neither physical simulations nor new predictions. All four CSV and Markdown tables must match the locked reference files byte for byte; `ARCHIVE_REPRODUCTION.json` records the result.

Use an empty output directory for each reconstruction. The commands do not write into `data/` or overwrite the original experiment in `outputs/robustness_v1/`. The bundle is stored with Git LFS; if verification reports a short placeholder file, run `git lfs pull` before trying again.

The evidence covers only the stated mass errors and constant supervisory position/velocity biases. It does not establish robustness to arbitrary model errors, external disturbances, or real-flight conditions.
