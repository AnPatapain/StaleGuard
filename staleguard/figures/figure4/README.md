# Three-mission policy curves

Six CSV files and one ready-to-input LaTeX figure. The flat `figures/figure4/` layout matches the Overleaf project; LaTeX assigns the figure number according to its position in the manuscript.

```text
figures/figure4/
  plot.tex
  inspection_unsafe_rates.csv
  inspection_safe_completion_rates.csv
  geofence_unsafe_rates.csv
  geofence_safe_completion_rates.csv
  underpass_unsafe_rates.csv
  underpass_safe_completion_rates.csv
```

## Include in the paper

Put this in the manuscript preamble:

```latex
\usepackage{pgfplots}
\usepgfplotslibrary{groupplots}
\usetikzlibrary{calc}
\pgfplotsset{compat=1.18}
```

Keep `figures/figure4/` relative to the manuscript root and insert:

```latex
\input{figures/figure4/plot.tex}
```

Compile twice so the shared legend resolves. The snippet uses `figure*` for a two-column paper. No PDF is included. To locate the data elsewhere, define this before the input:

```latex
\newcommand{\StaleGuardFigureTwoData}{your/path/to/data}
```

The six panels show unsafe and safe-completion percentages for Inspection Descent (top), Geofence Turn (middle), and Underpass (bottom). Only Underpass extends to 2 seconds; its gray vertical lines indicate the unchanged 1-second prediction horizon. Policy colors/styles are shared across panels. Missing policies are omitted, never represented by zero-filled curves. Full StaleGuard is only shown for Underpass; it is not a relabeling of the older dispatch-only policy.

## Data sources

| CSV pair | Source | Scenario count | Tested delays |
|---|---|---:|---:|
| `inspection_*_rates.csv` | The two identically named files in `StaleGuard_Inspection_6policies_PGFPlots_CSV.zip`, extracted without modification | 276 | 11, 0–1 s |
| `geofence_*_rates.csv` | The two identically named files in `StaleGuard_Geofence_8policies_PGFPlots_CSV.zip`, extracted without modification | 300 | 23, 0–1 s |
| `underpass_*_rates.csv` | `underpass_inspection/results/evaluation/baseline_benchmark_v2/metrics.json`, `by_delay` | 400 | 13, 0–2 s |

All entries are **percentages on a 0–100 scale**. The Underpass export uses `100 * unsafe / flights` or `100 * safe_completion / flights` at each policy/delay, with 400 flights per cell. Its nine policy columns retain every evaluated policy, including policies not drawn. `staleguard_dispatch_recovery` maps to the benchmark's `staleguard`; `staleguard_dual_horizon` is plotted as StaleGuard (full). RecoveryAware and full StaleGuard have identical Underpass curves, so only the full curve is drawn.

The four ZIP-derived CSVs were also checked against their mission benchmark metrics. Geofence EarlyRecovery and Fixed EarlyRecovery values match `geofence_turn/results/evaluation/fixed_early_benchmark/metrics.json`; its other columns match the original baseline benchmark. All source paths here are relative to `examples/staleguard`.

Lines join tested delay points; they are not continuous-time feasibility guarantees. Do not pool the different missions' scenario counts or delay grids into one rate.

## Underpass-only comparison

`plot_underpass.tex` is the Underpass-only two-panel version. It uses thin solid policy lines, markers at every measured delay, a small margin below 0% and above 100%, and a separate 0–0.20 s unsafe-rate inset. All CSVs and measured values remain unchanged. Use:

```latex
\input{figures/figure4/plot_underpass.tex}
```

If the CSVs are in the Overleaf folder `figures/figure4`, define this before the input:

```latex
\newcommand{\StaleGuardUnderpassData}{figures/figure4}
```

Compile twice for the shared legend. The three-mission `plot.tex` remains available.
