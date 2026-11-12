# Technical appendix: read-only profile audit

Only Python standard-library parsing, checksums, CSV writing and arithmetic were used, with bytecode writing disabled. No project modules, predictor, simulator or policy runner were imported. No source file was created or modified. New outputs are confined to this versioned audit directory.

## Inputs and integrity

Each run manifest matches progress.config_sha256. Prediction and execution journal byte counts and SHA-256 values match committed progress, with no uncommitted tail. Every prediction key (policy, seed) is unique. Both RecoveryAware and DualHorizon have exactly the complete frozen cohort, with numerically identical delay/boolean profiles; their independent runtimes are not combined. The 400 Underpass command-only forecasts are checked for journal/key integrity but excluded from three-opportunity analysis because they do not contain all three properties. Full cohort IDs, grids, input paths and checksums are recorded in validation.json.

Inspection uses 11 candidates [0, 0.1, ..., 1.0]. Geofence and Underpass use 19 candidates [0, 0.02, ..., 0.20, 0.30, ..., 1.0]. Total: 976 scenarios, 2928 unique opportunity sequences and 49008 boolean labels. Independent policy copies are cross-checked rather than counted as extra scenarios.

## Classification

A return is any adjacent infeasible-to-feasible transition. This detects both feasible → infeasible → feasible sequences and initial infeasibility followed by feasibility. A second independent test checks whether any false sample has a later true sample. All independently derived non-monotonic flags and first-loss boundaries match the saved metadata. Total label changes count both directions; multiple return episodes means two or more infeasible-to-feasible transitions, not merely the two transitions needed for one return.

## Decision screening; no changed policy executed

Only cases with completion feasible after its first false sample enter completion_decision_audit.csv. The audit reads the existing command first-loss boundary and recovery last-safe-prefix trigger. It converts the recorded full-policy computation duration to the first 50 Hz tick satisfying duration <= elapsed time, including equality, then takes the later of readiness and the stored current intervention trigger. A feasible later dispatch point is a witness only if it has not passed at current intervention, is strictly before command-safety expiry, and is not beyond the existing recovery constraint. All nine affected completion cases have recovery feasible throughout the grid, so no missing recovery-rule case is silently approximated.

The next tested infeasible point after the final successful completion point is recorded for descriptive comparison. If the final grid point were feasible, exhaustion would be unobserved, not assigned to the horizon endpoint. This arithmetic identifies possible decision differences using the same stored full-policy runtime. It neither implements a later-aware policy nor predicts its runtime or physical outcome.

## CSV contents

- summary.csv: nine mission/opportunity rows, denominators, percentages, initial failures and reversal counts.
- affected_profiles.csv: ten unique affected seed/opportunity rows with complete delay/boolean arrays and exact JSON sequences.
- affected_samples.csv: 190 rows, one per affected seed/opportunity/delay; feasible is 1 or 0.
- completion_decision_audit.csv: nine returning-completion scenarios, all competing cutoffs, readiness, exclusion reasons and decision-change witnesses. Empty numeric cutoffs mean unobserved, as indicated by explicit flags; they do not mean zero.
- validation.json: checked cohorts, input digests, exact counts, unchanged-input/source checks and scope limitations.

The original scientific inputs remain in their original mission folders. This audit adds only derived CSV/JSON/Markdown outputs. No physical flight outcomes were inspected to infer improvement.
