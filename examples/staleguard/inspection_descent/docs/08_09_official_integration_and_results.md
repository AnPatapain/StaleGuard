# Stages 8–9: official integration, validation, and recovery benefits

This report covers the official integration and the subsequent StaleGuard
versus No Recovery results. For the later replacement of caching with a fixed
JSON file, see [current implementation](current_implementation_and_validation.md).

This report records the earlier recovery validation. The automatic cache
described below has since been replaced by `calibrated_scenarios.json`, by
explicit user decision. The selected commands are unchanged. See [README](../README.md)
for the current manual-calibration workflow. The earlier results remain available.

The official experiment now uses the previously tested maximum-D=0-dwell calibration through the shared implementation. It is ready for baseline integration. Scenarios with no qualifying command are reported separately, and mission rates use only the scenarios actually tested. No new baselines were implemented.

## Recovery findings

The official-planner wrapper, cache implementation, explicit infeasibility reporting, execution cleanup, moved diagnostics and pre-cleanup source snapshot survived the hard shutdown. Five complete cache JSON files survived; their JSON, filename/fingerprint hashes and result metadata were consistent. No malformed JSON or partial cache files were found. Sixteen abandoned temporary directories were empty and have been preserved, along with the interrupted logs. The old cache entries were retained; stale source fingerprints were not reused as current results. Final calibration/cache/experiment regression reports had not been completed before the interruption.

The previous attempt launched two overlapping eight-worker process pools, allowing sixteen simultaneous workers on a machine with about 15 GiB RAM and 1 GiB swap. One recovered simulation process initially used about 748 MiB RSS; sixteen such RSS values sum to roughly 12 GiB, before other applications. RSS includes shared pages, so this is not a measurement of unique physical memory. Concurrent controller/environment imports, PyTorch dependencies and numerical-library threads make memory/CPU pressure a plausible explanation. Previous-boot kernel logs were not accessible, so an OOM event cannot be confirmed. That parallel launch was too aggressive.

## Changes completed

- Preserved the existing promotion and reused `max_dwell_calibration.py`; no duplicate maximum-dwell search was introduced.
- Retained the locked grid, ordering, tie rule and D=0 nominal-structure-arrival bound. Longer low durations remain allowed. Every cold candidate is executed in PyBullet at D=0; positive-delay outcomes never select commands.
- Hardened cache reads against incomplete/inconsistent data. Damaged files are preserved as `.invalid` and recalibrated. Files are flushed/fsynced before atomic publication. Simulation exceptions propagate and are never stored as infeasibility.
- Fixed a diagnostic JSON export failure by converting its NumPy recovery-decision flag to a Python boolean. The completed comparison was rerun successfully; the failed export/logs were preserved. No calibration was repeated for this fix.
- Removed multiprocessing from the validator. It uses one simulation worker, limits numerical-library threads to one, refuses larger worker counts, and prevents overlapping launches with an exclusive lock.
- Added memory checks between simulations: default process RSS ceiling 2048 MiB and minimum host available memory 2048 MiB. A resource limit stops validation with an error; it does not become a mission outcome.
- Added per-seed resumable validation checkpoints tied to the current cache fingerprint. The initial seed 0/16 checks were reused in the complete run rather than repeated.
- Kept all prior diagnostics/results and the pre-cleanup source available. Historical first-feasible diagnostics are explicitly separated under `diagnostics/`.

`staleguard_benefit.py` retains physics-rate safety/inspection monitoring, `trace`, `physics_trace`, GUI branches and all mission/recovery metric definitions. Removed code was unused debug output/bookkeeping, an obsolete first-feasible implementation, commented statements, unnecessary temporary variables and unused imports. The intended change to the default command selection can change experiment metrics relative to historical first-feasible calibration. The cleanup itself was compared with identical commands and dispatch/recovery schedules.

## Validation actually executed after recovery

1. Syntax and official-module imports passed. Import-only peak RSS was about 94 MiB.
2. Initial serial cold checks: seed 0 (30 candidates, selected dwell 261 ms) and seed 16 (29 candidates, no feasible command; best safe dwell 141 ms). Process RSS was about 748 MiB.
3. Completed seeds 0–19 serially: all 2,241 D=0 candidate outcomes/dwells matched the previously saved measurements, and every selected command/infeasibility matched.
4. Verified all 20 official cache hits without invoking calibration, including cached infeasibility for seed 16. Ten scenario/settings/source/config/library changes caused cache misses. Two intentionally damaged test cache files were rejected and preserved. Simulation-error propagation and the all-infeasible N/A branch passed.
5. Ran the official 20-seed paired experiment: 19 calibrated scenarios, infeasible seed 16 explicitly reported, N=19 for every policy metric. All 228 policy executions were compared against 228 pre-cleanup executions with identical inputs; outcome flags and complete control/reference/physics trace hashes matched exactly. The paired net-benefit assertion passed.
6. Re-executed all 304 saved maximum-dwell No Recovery delay cases. Every individual outcome matched the previous result, including D=0 dwell replay checks.
7. CLI, validation-lock/resource-refusal and historical first-feasible diagnostic smoke checks passed. GUI branches were compared structurally with the pre-cleanup source and are unchanged; an interactive GUI window was not launched.

The maximum reported validation peak RSS was 1030 MiB. Every recovered simulation phase used one worker, and memory limits were not reached. The source files `experiment.py`, `mission.py`, `staleguard.py`, `model/scg.py` and `model/common.py` match the pre-cleanup snapshot; the shared maximum-dwell algorithm is unchanged.

| Seed | Commands checked | Selected approval (s) | Selected low duration (s) | Selected D=0 dwell (ms) |
|---:|---:|---:|---:|---:|
| 0 | 30 | 0.00 | 0.86 | 261 |
| 1 | 38 | 0.00 | 1.00 | 188 |
| 2 | 297 | 0.06 | 2.02 | 487 |
| 3 | 369 | 0.14 | 2.28 | 579 |
| 4 | 101 | 0.22 | 1.38 | 388 |
| 5 | 72 | 0.10 | 1.14 | 326 |
| 6 | 68 | 0.20 | 1.00 | 420 |
| 7 | 175 | 0.40 | 1.30 | 392 |
| 8 | 114 | 0.06 | 1.36 | 470 |
| 9 | 52 | 0.20 | 1.12 | 349 |
| 10 | 87 | 0.22 | 1.32 | 393 |
| 11 | 289 | 0.58 | 1.54 | 541 |
| 12 | 83 | 0.00 | 1.30 | 407 |
| 13 | 98 | 0.28 | 1.26 | 500 |
| 14 | 75 | 0.26 | 1.16 | 400 |
| 15 | 49 | 0.00 | 1.14 | 284 |
| 16 | 29 | infeasible | infeasible | best safe: 141 |
| 17 | 72 | 0.20 | 1.06 | 383 |
| 18 | 76 | 0.00 | 1.28 | 420 |
| 19 | 67 | 0.04 | 1.16 | 415 |

## Actual official experiment results

Calibration feasibility is **19/20**. Seed **16** is excluded explicitly. Every count/rate below uses **N=19**, not 20; the calibration failure is not counted as unsafe. All values are from the executed experiment.

| D (s) | T (s) | No Recovery safe | StaleGuard safe | No Recovery unsafe | StaleGuard unsafe | Recoveries | Benefit | Harm | Net benefit (pp) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 4.2 | 19/19 | 19/19 | 0/19 | 0/19 | 0/19 | 0/19 | 0/19 | +0.0 |
| 0.0 | 5.0 | 19/19 | 19/19 | 0/19 | 0/19 | 0/19 | 0/19 | 0/19 | +0.0 |
| 0.5 | 4.2 | 3/19 | 18/19 | 14/19 | 0/19 | 15/19 | 15/19 | 0/19 | +78.9 |
| 0.5 | 5.0 | 3/19 | 18/19 | 14/19 | 0/19 | 15/19 | 15/19 | 0/19 | +78.9 |
| 1.0 | 4.2 | 0/19 | 19/19 | 19/19 | 0/19 | 19/19 | 19/19 | 0/19 | +100.0 |
| 1.0 | 5.0 | 0/19 | 19/19 | 19/19 | 0/19 | 19/19 | 19/19 | 0/19 | +100.0 |

Full inspection-completion counts, recovery-success denominators and rates are in the actual [terminal output](../results/official_calibration_validation/recovery/official_experiment.log). The metric definitions are unchanged. Recovery benefit/harm divide the paired event counts by the evaluated N; net benefit is (StaleGuard safe completions − No Recovery safe completions)/N.

## Files and evidence

- [Official experiment](../staleguard_benefit.py), [shared calibration](../max_dwell_calibration.py), [saved scenarios](../calibrated_scenarios.json), [workflow documentation](../README.md).
- [Serial validation script](../diagnostics/validate_official_calibration.py).
- [Interruption audit](../results/official_calibration_validation/recovery/interruption_audit.json), [static checks](../results/official_calibration_validation/recovery/static_checks.json), [calibration checks](../results/official_calibration_validation/recovery/calibration.log), [cache checks](../results/official_calibration_validation/recovery/cache_checks.log).
- [Execution regression](../results/official_calibration_validation/recovery/experiment_checks.log), [delay regression](../results/official_calibration_validation/recovery/delays_checks.log), [CLI checks](../results/official_calibration_validation/recovery/cli_checks.json).
- [Machine-readable official metrics](../results/official_calibration_validation/recovery/official_metrics.json), [selected commands](../results/official_calibration_validation/recovery/calibrations.json), [per-execution trace hashes](../results/official_calibration_validation/recovery/execution_regression.jsonl).

Generated caches and validation data remain under the existing git-ignored `results/` directory. Interrupted logs and earlier scientific results were preserved. No commit was made as part of recovery.
