# Paper figures and tables: minimum data

All paths are relative to `examples/staleguard`. These are inputs for making the figures/tables; raw journals remain necessary for independent verification, extra analyses, or physical reproduction. The available data do not establish every planned measurement across every mission.

- **U:** `underpass_inspection/results/evaluation/baseline_benchmark_v2/` (400 new evaluation scenarios).
- **S:** `underpass_inspection/results/evaluation/supplementary_feasibility_v1/` (independent physical feasibility validation of the same 400 scenarios).
- **G:** `geofence_turn/results/evaluation/`.
- **I:** `inspection_descent/results/evaluation/`.

| Figure / plot / table | Minimum data to create it |
|---|---|
| **Fig. 2 — Physical feasibility profiles and deadline gaps** | **`S/profiles.csv` only.** Group by scenario and sort by `delay_s`. Plot `command_safe`, `safe_completion`, and `recovery_safe`. The conservative boundary is the first tested false value for each property; an all-true profile has an unobserved boundary. Compute `recovery first loss - command first loss` only when both are observed, and retain censored/non-monotonic categories separately. `S/per_scenario.csv` is an optional already-computed boundary export, not an additional required input. |
| **Fig. 3 — Representative 3D Underpass trajectories** | **`U/representative_trajectories.npz`, `U/run.json`, and `U/per_delay.csv`.** These supply actual positions/timestamps, geometry/approval time, and intervention/readiness/violation annotations. Retained v2 trajectories cover only seed 1002 at D=2 s, where command-only EarlyRecovery and full StaleGuard coincide. A different explanatory case needs an exact recorded-action replay/export. **Existing `figures/figure3_data/` is Geofence Turn seed 22 at D=0.5 s, not Underpass; do not relabel it.** |
| **Fig. 4 — Unsafe and safe-completion rates versus delay across three missions** | **Four JSON files:** `U/metrics.json`, `G/baseline_benchmark/metrics.json`, `G/fixed_early_benchmark/metrics.json`, and `I/baseline_benchmark/metrics.json`; use `by_delay`. The Geofence fixed-early file contains both command-only EarlyRecovery and Fixed EarlyRecovery. The six ready-to-plot CSVs are now in **`figures/figure4/`**: `inspection_unsafe_rates.csv`, `inspection_safe_completion_rates.csv`, `geofence_unsafe_rates.csv`, `geofence_safe_completion_rates.csv`, `underpass_unsafe_rates.csv`, `underpass_safe_completion_rates.csv`. CSV values are percentages, not fractions. |
| **Fig. 5 — Safety–completion scatter plot** | **`U/metrics.json` only**, `by_policy`: X = `100 * unsafe / flights`, Y = `100 * safe_completion / flights`. `staleguard_dual_horizon` is StaleGuard (full); RecoveryAware is its completion-condition ablation and coincides in this cohort. |
| **Table I — Prediction accuracy, boundary errors, runtime and readiness** | **Five JSON files:** `S/metrics.json`; `geofence_turn/results/development/ttl_pred_accuracy/metrics.json`; `inspection_descent/results/development/ttl_pred_accuracy/metrics.json`; `G/baseline_benchmark/metrics.json`; `I/baseline_benchmark/metrics.json`. Inspection/Geofence command accuracy uses 19/20 development scenarios, whereas evaluation timing/readiness uses 276/300 scenarios. Underpass accuracy and timing use 400 evaluation scenarios. Readiness before physical recovery loss and independently validated completion/recovery prediction accuracy are not reported for the other missions. Original boundary-error eligibility conventions are retained and documented in `tables/README.md`. |
| **Table II — Consolidated benchmark outcomes across three missions** | **The same four JSON files as Fig. 4**, using `by_policy`. Underpass `uncertainty` and paired-comparison entries supply its existing scenario-bootstrap comparisons. Do not pool the 11-, 23-, and 13-delay mission grids. Full StaleGuard and RecoveryAware were evaluated only for Underpass; mark other cells not evaluated. Original mission-specific Always Recover behavior differs and is footnoted in the table. |

## Current LaTeX locations

- Three-mission policy curves: `figures/figure4/plot.tex`.
- Improved Underpass-only curves: `figures/figure4/plot_underpass.tex`.
- Table I: `tables/table1.tex`.
- Table II: `tables/table2.tex`.
- Overleaf integration and table provenance: `tables/README.md`.

Figure/table numbers are assigned by the manuscript, not by folder names. Include the table files where Table I and Table II should occur; do not manually reset counters. No algorithms, benchmark journals, predictions or historical metrics are modified by these presentation files.

**Always Recover audit warning:** the archived Table II cells use different recovery timing across missions. See `tables/ALWAYS_RECOVER_AUDIT.md`; the common policy label is not evidence of equivalent semantics.
