# Technical appendix — unified nine-policy benchmark

## Code structure and changes

- `baseline_benchmark/run.py`: the only runner. Supports preparation, any policy subset, all nine policies, physical execution, resume, validation and metrics-only regeneration.
- `baselines/policies.py`: one registry and factory; the unchanged original lifecycle and the unchanged RecoveryAware/DualHorizon decision methods now live together.
- `baselines/staleguard.py`: unchanged original two-stage command predictor.
- `baselines/recovery_horizons.py`: unchanged optimized three-profile predictor. Kept separate because compiled continuation/safety/completion kernels are clearer outside the policy lifecycle.
- `baseline_benchmark/metrics.py`: one pipeline for all policy sets, predictor quality, failure decomposition, runtime/readiness, paired comparisons and full / <=1 s / >1 s intervals.
- `storage.py`: immutable configuration/input handling plus one journal/checkpoint mechanism for flights and predictions. No historical layout maps, compatibility allowlists or benchmark-folder path fallbacks.
- `support/validate_baseline_benchmark.py`: one self-contained validator called by the runner; no second CLI or original-folder dependencies.
- `support/visualize_policy_comparison.py`: reads the unified format and bundled inputs; no external calibration is needed for historical GUI replay.
- `support/test_benchmark.py`: current storage, disjoint sampling, nine-policy registry/subset preparation and regression tests.
- `support/test_baselines.py`, `support/test_recovery_horizons.py`: retained useful semantic/numerical tests, updated to the unified modules.
- Mission README and evaluation index: current layout, timing semantics and commands.

Removed runtime/source files:

- `baseline_benchmark/horizons.py` (385 lines).
- `baselines/recovery_policies.py` (73 lines; its class methods were moved unchanged).
- `support/results_layout.json` (64 lines of obsolete path/provenance mapping).
- `support/test_final_audit.py`, `support/test_2s_protocol.py`, `support/test_reduced_protocol.py` (265 lines of old freeze/runner-specific tests, replaced by current focused coverage).

The Python codebase decreased from **5,172 to 4,540 lines**: **632 fewer lines net**, including tests. The original FBS, Use-Time Recheck, Fixed EarlyRecovery, command predictor, optimized predictor and scenario source files are byte-identical. Simulator physical syntax and both policy classes' decision methods match the pre-refactor versions. Shared model/PID/physics files were not modified. A pre-existing unrelated `.gitignore` change was left alone.

## Migration and cleanup

1. Verified the original 9,100-row and 2,600-row journals and the previously combined 11,700-row journal.
2. Verified the combined record-key/hash map was exactly the union, without duplicates.
3. Built `.baseline_benchmark_staging`, copying the combined execution journal without reserialization.
4. Bundled 100 actual approval states, PID memories and remaining references in one NPZ; retained all 300 forecast computations with original measured runtimes and profiles.
5. Combined nine useful representative actual trajectories. Preserved original experiment configuration/provenance and source/preparation evidence.
6. Regenerated all numerical metrics and intervals; exact dictionary equality against the previous nine-policy metrics passed.
7. Independently validated all 11,700 decisions/physical records and nine retained actual trajectories.
8. Replaced the canonical directory by directory rename, holding the old source folders separately. Validation and regeneration passed with their original paths absent.
9. Deleted the held original seven-policy folder, the redundant two-policy and merged folders, and duplicate benchmark ZIPs. Old preparation evidence is now in `source_versions.zip`; the obsolete flat preparation folder was removed. No calibration or unrelated development dataset was deleted.

Canonical historical path: `results/evaluation/baseline_benchmark/`.

Removed redundant paths: `recovery_horizons_benchmark_v1/`, `nine_policy_recovery_horizons_v1/`, `recovery_horizons_benchmark_v1.zip`, and `baseline_benchmark.zip`. The old 9,100-record contents at the canonical path were replaced only after all records were verified in the 11,700-record canonical replacement.

## Minimal storage and provenance

`run.json` is the immutable experiment configuration, source/environment record, physical scenario definitions and input/archive hashes. `progress.json` is the small authoritative checkpoint with status, committed counts/offsets/hashes and configuration integrity. The two append-only files are `executions.jsonl` and `predictions.jsonl`. Each append is fsynced before the progress JSON is atomically replaced. Interrupted uncommitted tails are rolled back; changed committed bytes fail verification. Prediction cache entries are committed before any flight can use them, preserving measured readiness on resume.

Large forecast data are not rewritten after every physical flight. The original StaleGuard pair shares one command forecast exactly as before; new policy forecasts are independently computed/timed. A forecast is keyed only by policy/cache kind and approval snapshot, never by future actual dispatch time. Compilation warmup uses discarded development inputs and remains separate from online latency.

Validation and metrics-only use the same folder's configuration, inputs, forecast journal and physical journal. They never load deleted source folders or require the original calibration directory. Resume additionally requires current source/environment equality. Historical source versions and experiment provenance remain evidence rather than executable compatibility rules. New preparations also archive the actual executable source snapshot.

## Regression evidence

- `regression.json`: identical physical simulator syntax, unchanged policy methods/numerical module bytes, and exact before/after profiles/boundaries/plans on 12 × 19 candidate points.
- `cleanup.json`: deleted source modules, result copies and archived preparation evidence.
- `physical_smoke_regression.json`: 41 independent physical replay hashes/outcomes match; remaining starts are explicitly listed as outside archived coverage.
- `prediction_profile/optimization.json`: all 12 development scenarios, four computation strategies, 36 timings per strategy, full state/PID comparisons, broad-phase checks and physical prediction-label errors.
- `final_checks.json`: exact historical journal hash/metrics, new selection seed/count/disjointness, 500-ID union, 46,800 expected v2 flights and zero actual v2 flights/predictions.
- `../unified_smoke/validation.json`: all 54 development flights pass the same validator as the historical benchmark.
- `../../evaluation/baseline_benchmark/consolidation.json` and `validation.json`: 11,700 original records preserved, self-contained audit passed.

The archived 7-policy development metrics also reproduce exactly. Replaying policy decisions uses original recorded forecasts, runtimes and observations, not new predictions or physical flights. Use-Time Recheck's dispatch-state/check latency rule is checked separately; the retained physical trace verifies its recorded observed state. FBS feature history is independently recomputed from its retained actual control states. Earlier safety violations remain recorded.

The 54-flight smoke used development seeds 1 and 3 and delays 0, 1 and 2 s, with all nine policies. It stopped after five flights, then resumed after an intentional uncommitted tail. No historical or fresh-v2 evaluation flight was executed. Focused tests passed **51/51**, including every policy prepared individually, duplicate execution/prediction rejection, corrupted/config/source detection, journal resumption, grid/equality rules, censoring/non-monotonicity, prediction correctness and scenario-clustered uncertainty.

## v2 protocol

- Original frozen cohort: 865 accepted IDs, unchanged.
- Excluded: all 100 IDs in the canonical historical configuration.
- Eligible: 765 IDs.
- Draw: one NumPy `default_rng(20261109).choice(sorted eligible IDs, 400, replace=False)`, executed in sorted-ID order.
- Selected: exactly 400; zero overlap; historical + v2 union exactly 500.
- Delays: `[0, .04, .08, .12, .16, .20, .40, .60, .80, 1, 1.20, 1.60, 2]`.
- Policies: all nine, identical scenario/delay pairs; no outcome filtering.
- Expected executions: 46,800.
- Prediction: unchanged 19 candidates, unchanged model/physics/controller/reference/clearance and eight-second observation horizon.
- Readiness, pending-command cancellation and dispatch/timer equality semantics unchanged.
- Analysis: full, <=1 s and >1 s subsets; whole-scenario paired 95% bootstrap intervals (2,000 resamples, seed 2027), not independent-delay intervals.

The prepared `run.json` and source archive freeze the protocol without a separate external freeze file. The configuration checksum is recorded in `progress.json`. Status is `prepared`; both execution and prediction journals are empty. The command to prepare it was:

```bash
python underpass_inspection/baseline_benchmark/run.py --prepare-only \
  --calibration underpass_inspection/results/evaluation/calibration \
  --exclude-run underpass_inspection/results/evaluation/baseline_benchmark \
  --sample-count 400 --sample-seed 20261109 \
  --out underpass_inspection/results/evaluation/baseline_benchmark_v2
```

## Commands

From `examples/staleguard`, with `scg` active:

```bash
# Launch v2 (NOT executed during this task).
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark_v2

# Resume an interrupted v2 run.
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark_v2 --resume

# Validate completed results; before launch checks preparation/zero flights.
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark_v2 --validate

# Regenerate metrics/reports without simulation.
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark_v2 --metrics-only

# Canonical historical independent audit and metrics-only regeneration.
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark --validate
python underpass_inspection/baseline_benchmark/run.py \
  --out underpass_inspection/results/evaluation/baseline_benchmark --metrics-only
```

Focused tests and development smoke reproduction commands are in the [mission README](../../../README.md). Tests exercise current code; no historical policy flights are rerun. Do not launch v2 as a test: operational validation is already complete.
