# Paper tables: Overleaf integration and data sources

`table1.tex` contains prediction accuracy, sampled boundary errors, warmed runtime and readiness. `table2.tex` contains the original benchmark outcomes across three missions. Both are self-contained LaTeX table fragments with embedded measured values; compilation needs no CSV, JSON, simulation or Python process. Existing experiments and historical results are unchanged.

## Always Recover audit warning

The common Table II row has a confirmed cross-mission timing inconsistency. Its footnote documents it but does not make the policies equivalent. See [ALWAYS_RECOVER_AUDIT.md](ALWAYS_RECOVER_AUDIT.md) for the source and journal audit. No policy code or historical data was changed.

## Add to Overleaf

1. Create a `tables` folder at the manuscript root and upload `table1.tex` and `table2.tex`.
2. Add this package in the preamble (before `\begin{document}`):

```latex
\usepackage{booktabs}
```

3. Insert the tables in the Experiments section:

```latex
\input{tables/table1.tex}
\input{tables/table2.tex}
```

4. Reference them in prose with:

```latex
Table~\ref{tab:prediction_accuracy_readiness}
Table~\ref{tab:three_mission_policy_outcomes}
```

Compile twice for cross-references. `table*` spans both columns in IEEEtran and is placed at the top of an available page. Numbering is automatic: if these are the first two captioned tables, IEEEtran labels them I and II. Do not reset counters or put another `table` environment around the inputs.

## Table I: exact sources

| Part | Source | Fields / calculation |
|---|---|---|
| Inspection command accuracy | `staleguard/inspection_descent/results/development/ttl_pred_accuracy/metrics.json` | `raw_prediction`; `ttl.mean_absolute_error_s`; comparable cases and signed errors in `ttl.per_scenario`. Median absolute error is calculated from those 18 non-null errors. 19 development scenarios, 209 label pairs. |
| Geofence command accuracy | `staleguard/geofence_turn/results/development/ttl_pred_accuracy/metrics.json` | `raw_prediction`, `ttl.mean_absolute_error_s`, `ttl.median_absolute_error_s`, `ttl.absolute_error_samples`. 20 development scenarios, 380 label pairs. |
| Underpass command, completion and recovery accuracy | `staleguard/underpass_inspection/results/evaluation/supplementary_feasibility_v1/metrics.json` | `prediction_accuracy.staleguard_dual_horizon.{command,completion,recovery}`. 400 evaluation scenarios, 7600 paired labels/property. |
| Inspection evaluation timing/readiness | `staleguard/inspection_descent/results/evaluation/baseline_benchmark/metrics.json` | `by_policy.staleguard.prediction_computation_ms`; readiness `prediction_ready / flights`. 276 timed scenario forecasts. |
| Geofence evaluation timing/readiness | `staleguard/geofence_turn/results/evaluation/baseline_benchmark/metrics.json` | Same fields; 300 timed scenario forecasts. |
| Underpass evaluation timing/readiness | Supplementary Underpass `metrics.json` above | `readiness.{staleguard,staleguard_recovery_aware,staleguard_dual_horizon}.timing` (seconds converted to milliseconds). Dispatch availability comes from `original_policy_comparisons.by_policy.<policy>.approval_prediction_available_before_dispatch / flights`. Late readiness uses the stored physical recovery first-loss comparison, not a predicted deadline. |

Accuracy is `(TP + TN)/all paired labels`. False-feasible rate is `FP/(FP + TN)`; false-reject rate is `FN/(FN + TP)`. The table displays error counts and those conditional percentages. Predicted-safe precision has a different denominator and is not substituted.

### Important boundaries on interpretation

- Accuracy and timing are not the same cohort for the two legacy missions. This is explicitly footnoted; no development result is presented as held-out prediction accuracy.
- Command/completion accuracy and boundary errors coincide in Underpass. The recovery column uses a separate independent physical recovery sweep, not selected policy recoveries.
- Boundary MAE follows each original protocol. Inspection excludes its initially invalid forecast (seed 10); Geofence excludes an unobserved loss; Underpass excludes right-censored cases but retains one initially invalid command/completion sample as first loss at zero. Thus cross-mission MAEs retain a methodological difference; they are not a newly harmonized estimator. No archived metrics were modified.
- Readiness by dispatch counts scenario–delay cases. Runtime counts one forecast per scenario, not one per repeated delay.
- Late readiness relative to physical recovery loss has denominator 243 observed Underpass boundaries; 157 boundaries are unobserved through 1 s. All 400 full-forecast readiness starts are physically safe on directly matching samples. N/M for the other missions means this statistic was not measured/reported here.
- RecoveryAware and full StaleGuard have identical numerical profiles but independently measured runtime. Milliseconds and p95 values retain the original archived timing conventions and hardware/environment; this is not a controlled cross-mission hardware speed comparison.

## Table II: exact sources

All values are `100 * outcome_count / flights`, rounded to two decimals, from `metrics.json -> by_policy`:

- Inspection: `staleguard/inspection_descent/results/evaluation/baseline_benchmark/metrics.json`.
- Geofence original policies: `staleguard/geofence_turn/results/evaluation/baseline_benchmark/metrics.json`.
- Geofence command-only EarlyRecovery and Fixed EarlyRecovery: `staleguard/geofence_turn/results/evaluation/fixed_early_benchmark/metrics.json`.
- Underpass: `staleguard/underpass_inspection/results/evaluation/baseline_benchmark_v2/metrics.json`.

The table includes the nine policies in the current Underpass study; legacy Fixed TTL is not one of those nine. A dash is not evaluated, not a zero. The full policy is `staleguard_dual_horizon`; `staleguard_recovery_aware` is the completion-condition ablation.

Rows use each mission's native grid: 276 × 11 = 3036, 300 × 23 = 6900, and 400 × 13 = 5200 flights/policy. The Underpass range is 0–2 s; the other two ranges are 0–1 s. Do not pool rates or treat repeated delays as independent scenarios. Geofence's early/fixed policies come from a separate archived run on the same cohort; their computation timings are not forced to match the original dispatch-only run.

The Always Recover footnote is necessary: Inspection executes the command at D=0 and recovers at approval for positive delays; Geofence recovers at dispatch; Underpass recovers at approval. These legacy baseline definitions were not changed. In particular, Inspection's 9.09% safe-completion rate and Geofence's 7.25% unsafe rate must not be replaced with zeros.

The Underpass paired safety difference and 95% interval are copied from `paired_extensions` and `uncertainty.paired_extensions` for `staleguard_dual_horizon__vs__staleguard_early_recovery`. Confidence intervals resample whole scenarios. Differences use unrounded counts, so subtracting rounded cells can differ by 0.01 percentage points.

The table fragments require only `booktabs`. Data, column counts and LaTeX structure were checked; no LaTeX engine is installed in the execution environment, so a compiled rendering has not been verified.

See `figures/DATA_MAP.md` for the complete figure/table-to-data map.
