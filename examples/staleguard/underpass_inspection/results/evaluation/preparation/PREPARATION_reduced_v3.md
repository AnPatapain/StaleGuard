# Underpass reduced held-out benchmark: 100 × 13 × 7

Prepared **9,100 actual physical flights**, approximately 2.3 hours at the earlier
measured throughput (budget 2.5–3 hours). Exactly 100 scenarios are uniformly
sampled without replacement from the frozen 865 accepted IDs, using RNG seed
**20261108**. The sample is frozen before any policy outcomes and executed in
ascending seed order. The UAV still approaches the bridge, dives underneath and
swerves East around the pier; recovery still abandons inspection and turns East.
All seven policies, the 19-point predictor, physics/PID, safety monitoring,
eight-second horizon and runtime semantics are unchanged. **43 focused tests and
42 development-only smoke flights passed**. All previous experiments are
preserved. The reduced study has coarser temporal resolution and wider scenario
uncertainty than the full study. **No reduced held-out benchmark was launched.**
Next: use the versioned launch command in the README.

## Frozen protocol

Physical dispatch delays after approval are exactly:

```python
[0.00, 0.04, 0.08, 0.12, 0.16, 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.60, 2.00]
```

The complete sample ID list, original manifest/population hash, frozen scenario
configurations and snapshot hashes are in
[freeze_reduced_100x13_v3.json](freeze_reduced_100x13_v3.json). The selector reads
only accepted IDs, not delayed outcomes, gaps, TTLs or policy performance. No
replacement or recalibration occurs. No policy receives the future actual delay.

| Analysis subset of one benchmark | Delays per scenario | Physical flights |
|---|---:|---:|
| Full, 0–2 s | 13 | 9,100 |
| Within prediction grid, 0–1 s | 10 | 7,000 |
| Beyond prediction grid, >1–2 s (1.2, 1.6, 2 s) | 3 | 2,100 |

Reports retain per-scenario paired comparisons and 95% percentile bootstrap
intervals, resampling entire scenario vectors (2,000 resamples, bootstrap seed
2027). Correlated delays are not independent experimental units. Rates equally
weight selected dispatch points; they do not represent a uniform continuous-delay
distribution. No independent deadline sweep is required: actual paired No
Recovery flights provide command-safety labels.

## Limits and unchanged behavior

The temporal gaps are 0.04 s through 0.20 s and reach 0.40 s later. An observed
expiration is the first tested unsafe delay, not an exact physical deadline.
All ten selected delays through 1 s are points in StaleGuard's prediction grid;
therefore this reduced study does not test between-grid admissions within 1 s.
Uncertainty intervals across scenarios do not remove this temporal sampling limit.

StaleGuard still uses 19 candidate delays through 1 s, first-unsafe TTL extraction,
its existing readiness gate and the same two-stage rollouts. With no predicted
expiration it still permits dispatch beyond 1 s; no new rejection rule was added.
Fixed EarlyRecovery remains 0.20 s, FBS remains unchanged, and all recovery remains
the same Eastward sideways maneuver. Safety is monitored over the whole flight
at 1,000 Hz. Inspection abandonment, incomplete observation by the fixed deadline
and reference-profile truncation remain separate records.

## Validation and preservation

The development smoke used seeds 1 and 3 at delays 0, 1 and 2 s, all seven
policies: 42 unique flights. Checks passed for common approval snapshots, actual
physics hashes, controller continuity, event ordering, cancellation, shared
StaleGuard forecasts/runtime and actual representative trajectories. Completed-run
resume added zero flights. Metrics-only regeneration was byte-identical. No
held-out delayed outcomes or comparative policy results were inspected.

The original freezes and previous artifacts are unchanged; original full 2 s
sources are also retained in source_2s_v2_freeze.zip. New results will go only to
results/heldout/baseline_benchmark_reduced_100x13_v3/. That directory has not been
created. Source/environment and journal integrity remain strict on resume.

Changed files: baseline_benchmark/run.py, baseline_benchmark/metrics.py (reduced
report explanation), storage.py (source inventory), support/validate_baseline_benchmark.py,
new support/test_reduced_protocol.py, and README.md. No policy, mission, physics
or predictive-model source changed.

See [launch/resume/metrics/validation commands](../../README.md#reduced-held-out-benchmark--100-scenarios--13-delays--7-policies),
[preparation validation](preparation_validation_reduced_v3.json), and
[development smoke validation](../development/baseline_smoke_reduced_v3/validation.json).

Full SHA-256 values:

- Protocol: `fcfd042ed692a01228a3c48ae9c7deccace99a5e7cd47a0f27db921f941967f5`
- Source/environment: `1c3428d26a2745dfc1d700c637b56cdcde385b1137607c199cbb2750b0f1fc32`
- Freeze file: `785ac2ed73046227b4b1ad41799dc342d97cff6a33d40c6bd000e0192b624799`
- Original calibration manifest: `4d440b4dca55fe5918bd9a4097f8502eea60cc4e7a987c16f81bc7d84075cb23`
