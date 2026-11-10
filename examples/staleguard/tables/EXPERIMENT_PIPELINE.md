# Complete Tables I and II: persistent evaluation workflow

Run these commands from the Git repository root (`safe-control-gym`). No command regenerates scenarios or reruns the completed Underpass benchmark.

## Launch or resume everything

```bash
conda run --no-capture-output -n scg python examples/staleguard/tables/complete_experiments.py --launch
```

This creates/verifies the detached tmux session `staleguard-tables-v1`. It runs focused tests, checks/reuses the successful physical smoke validation, standardizes Always Recover, computes missing forecasts, fills the independent physical feasibility grid, executes missing policy flights, validates each matrix, and automatically aggregates both tables. Repeating this command skips committed results. A single writer lock prevents duplicate orchestrators. The computer must remain powered and awake; tmux survives this terminal/Codex session, not a shutdown.

## Status

```bash
conda run --no-capture-output -n scg python examples/staleguard/tables/complete_experiments.py --status
```

## Live logs

```bash
tail -f examples/staleguard/tables/results/evaluation/complete_tables_v1/execution.log
```

## Resume

Use the same launch command. Do not create a second output directory or edit the frozen stage configurations. If an existing worker is running, the launcher reports that session rather than starting another.

## Tables only: no physical simulation or prediction recomputation

```bash
conda run --no-capture-output -n scg python examples/staleguard/tables/complete_experiments.py --metrics-only
```

## Artifact locations (relative to `examples/staleguard`)

| Artifact | Location | Origin |
|---|---|---|
| Inspection Always Recover corrections | `inspection_descent/results/evaluation/always_recover_corrected_v1/` | New corrected representation, equivalent physical approval recoveries reused |
| Geofence Always Recover corrections | `geofence_turn/results/evaluation/always_recover_corrected_v1/` | New corrected representation, original D=0 physical recoveries reused |
| Inspection physical feasibility and forecasts | `inspection_descent/results/evaluation/supplementary_feasibility_v1/` | 3036 command records and 276 approval-recovery records reused; 2760 additional recovery starts physically executed; two independently timed full forecasts per scenario |
| Geofence physical feasibility and forecasts | `geofence_turn/results/evaluation/supplementary_feasibility_v1/` | 5700 command and 5700 independent dispatch-time recovery records reused; two independently timed full forecasts per scenario |
| Inspection prediction accuracy/readiness | Above Inspection supplementary folder, `metrics.json` and `validation.json` | Newly compared frozen evaluation cohort; not the old 19 development scenarios |
| Geofence prediction accuracy/readiness | Above Geofence supplementary folder, `metrics.json` and `validation.json` | Newly compared frozen evaluation cohort; not the old 20 development scenarios |
| Inspection missing policy benchmark | `inspection_descent/results/evaluation/three_opportunity_benchmark_v1/` | 276 × 11 × 4 = 12144 policy cases; measured identical actions reused, only distinct missing trajectories physically executed |
| Geofence missing policy benchmark | `geofence_turn/results/evaluation/three_opportunity_benchmark_v1/` | 300 × 23 × 2 = 13800 policy cases; measured identical actions reused, only distinct missing trajectories physically executed |
| Underpass data | Existing `underpass_inspection/results/evaluation/baseline_benchmark_v2/` and `supplementary_feasibility_v1/` | Reused, never rewritten |
| Final tables | `tables/results/evaluation/complete_tables_v1/table_I.csv`, `table_I.md`, `table_II.csv`, `table_II.md` | Automatically aggregated |
| Paired comparisons | `tables/results/evaluation/complete_tables_v1/paired_comparisons.json` and `metrics.json` | Whole-scenario 95% bootstrap intervals; per-mission and within/beyond-horizon subsets |
| Scientific report/validation | `tables/results/evaluation/complete_tables_v1/REPORT.md`, `VALIDATION.json` | Automatically generated; complete only when all matrices and cohorts validate |
| Process, logs, errors and smoke validation | `tables/results/evaluation/complete_tables_v1/` (`process.json`, `status.json`, `execution.log`, optional `errors.json`, `smoke_validation*.json`) | Persistent execution evidence |

Each new mission stage uses the existing journal/progress storage (`run.json`, `executions.jsonl`, `predictions.jsonl`, `progress.json`). Flight/prediction appends are fsynced before atomic offset/hash commits. Restart repairs only uncommitted tails, verifies hashes and scientific inputs, rejects duplicates, and skips existing keys. A physical job retries at most twice after the initial failure. A failed stage is recorded; independent mission work continues, and incomplete cells are explicitly marked missing. Numerical/policy source or environment changes require a deliberate new version; a Git revision change alone does not invalidate byte-identical scientific sources.

## Scientific configuration and interpretation

- Always Recover now cancels at approval even at D=0 in all missions; approval wins the simultaneous-dispatch tie. It does not receive future delay information to decide. Existing positive-delay Inspection recoveries and zero-delay Geofence recoveries are physically equivalent approval-time trajectories, so no new Always Recover flights are necessary. Reuse lineage is recorded per row; represented dispatch cases are not relabeled as new physical executions.
- The full and RecoveryAware guards reuse **the existing Underpass `HorizonCommand` class directly**. Boundary extraction and policy plans preserve its first-loss, last-safe-prefix, equality, missed-prefix, unavailable-prediction/fail-open and finite-horizon rules. Action tokens are mapped to the original mission references; there is no adaptive recovery selector or new baseline.
- Prediction runs the same existing 100 Hz RK4 dynamics/PID kernels and preserves controller memory. Waiting is computed once; checkpoints are reused; command safety/completion share one rollout; unsafe continuations may terminate because neither safety nor safe completion can recover. Geofence completion requires position, northward velocity and dwell; Inspection uses its strict position/altitude region and dwell.
- The two new policies independently compute and time numerically identical full profiles. No cross-policy result cache advances readiness. Warmup/JIT is excluded and logged; all per-forecast adapter/reference/rollout/extraction/cleanup time is included. Predictions publish atomically. No physical labels or future dispatch schedule enter the online predictor.
- After independent action selection, a policy may reuse an already measured physical continuation with the same frozen scenario and command/recovery start. All these guards latch that single action, so subsequent reference, PID evolution, safety monitoring and horizon are identical. Remaining identical actions are grouped for one native flight; their outcomes are reused with explicit lineage. This shares physical evidence, never forecast computation or readiness. `reuse_smoke_validation.json` compares this reuse with four fresh native flights. New physical execution counts and represented policy cases are separate in the final CSV. The initial `run.json` budgets of 25944 policy flights and 2760 feasibility flights are conservative pre-reuse budgets; final metrics give actual execution counts.
- Inspection uses its original 11-point prediction and dispatch grids. Geofence prediction uses 19 candidates; its policy benchmark retains all 23 dispatch points, including four off-grid points. No untested continuation is skipped by assuming monotonicity.
- **Inspection horizon distinction:** its physical policy benchmark has always used approval + 4.2 s and first-unsafe termination; its historical command-only model forecast extends to 8.5 s from flight start. The new three-profile adapter targets the unchanged 4.2 s operational horizon, allowing safety/completion comparisons against existing physical benchmark outcomes. Neither old command-only policy behavior nor the physical deadline changes. This retained legacy model-horizon difference must be disclosed when comparing full versus command-only guards. The RecoveryAware/full ablation uses identical horizons, models and profiles. Geofence's full predictor uses its unchanged operational scenario horizon.
- The new Inspection Fixed EarlyRecovery is explicitly 0.20 s using the canonical guard. The previously unused Inspection helper with 0.30 s is left untouched; no old Fixed TTL outcome is modified.
- No development accuracy result enters the completed Table I. All initial-infeasibility, right-censoring and non-monotonic profiles are retained. Sample zero can be the first sampled loss; errors are null when either loss is unobserved. Readiness relative to a coarse physical boundary distinguishes last-safe sample, first-unsafe sample, unresolved bracket and censored cases.
- Table II retains the nine current policy types and the existing legacy Fixed TTL results where already evaluated (not a new policy for Underpass). Counts distinguish new physical executions, archived cases, verified reference reuse and corrected equivalent reuse. Bootstrap resampling uses whole scenario vectors (2000 resamples, seed 2027), never independent delay trials.

`tables/CURRENT_RESULTS.md` points to the current aggregated results. Existing table1.tex/table2.tex presentation fragments retain their earlier numbers; this workflow produces the requested final scientific CSV/Markdown tables without doing new table-layout work.
