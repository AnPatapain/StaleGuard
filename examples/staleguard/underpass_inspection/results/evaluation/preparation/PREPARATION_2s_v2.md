# Underpass: frozen 2-second held-out protocol

Extended physical dispatch scheduling to **0–2.00 s in 0.02 s steps (101 points)**.
The full matrix is **865 × 101 × 7 = 611,555 flights**, correcting the requested
611,455 arithmetic total. All seven policy algorithms, the 19-point prediction
grid and prediction workload are unchanged. The aircraft still approaches the
bridge, dives underneath and swerves East around the pier; safety recovery still
abandons inspection and turns East beside the deck. **40 focused tests and 42
actual development-only smoke flights passed**, including D=0, 1 and 2 s. FBS
now naturally reaches its existing timer within the physical grid. No held-out
policy flights were run for this extension and the full run was not launched.
Next: launch the versioned benchmark using the README commands; preserve the
frozen source/environment for resumption.

| Analysis of the same benchmark | Dispatch points | Flights |
|---|---:|---:|
| Full, 0–2.00 s | 101 | 611,555 |
| Within prediction candidate grid, 0–1.00 s | 51 | 308,805 |
| Beyond candidate grid, 1.02–2.00 s | 50 | 302,750 |

## Unchanged policy behavior

StaleGuard still predicts nominal waiting followed by the same dispatch-relative
command at 19 candidate delays: 0–0.20 s every 0.02 s, then 0.30–1.00 s every
0.10 s. The first unsafe point becomes TTL. A ready finite TTL rejects at and
after expiry. If no expiration is predicted, both variants continue permitting
dispatch beyond 1 s; no new horizon-based rejection was introduced. An unavailable
forecast still permits dispatch. EarlyRecovery still acts only when ready and
expired while pending, and never cancels a command already started. The simulator
alone knows future dispatch; the 2 s experimental bound is not supplied to policies.
FBS, Use-Time Recheck and the fixed 0.20 s recovery retain their previous rules.

In development seed 3 the model predicts no expiration through 1 s. Both
StaleGuard variants therefore dispatch at D=2 s; those actual flights are unsafe.
This explicitly demonstrates that no predicted expiration within the grid is not
a safety certificate beyond it. In development seed 1, violations that occurred
before late intervention remain recorded. No rule was changed to avoid either
failure. FBS triggers at D=2 s in both development smoke scenarios, consistently
with its original TTLs; its algorithm was not retuned.

## Observation horizon and analysis

The physical deadline remains eight seconds after approval. The finite green
reference profile ends 3.8 s after initiation, hence approval +5.8 s at D=2,
leaving 2.2 s of monitoring. All 42 smoke profiles fit the fixed horizon. A
missing inspection is recorded as not observed by the fixed deadline, separately
from deliberate abandonment by recovery. Truncated reference profiles are tagged;
neither the deadline nor the completion predicate was changed. Safety still
covers every 1,000 Hz sample, including before intervention.

Full-interval metrics are at the root of metrics.json; its analysis_subsets hold
the within-grid and beyond-grid windows. All three preserve per-scenario paired
comparisons. Confidence intervals resample whole scenario vectors, retaining
paired policies and all correlated delay trials: 2,000 percentile bootstrap
resamples, seed 2027, 95% intervals. Complete scenario grids contribute; fewer
than two complete scenarios yields no interval. The smoke intervals are only
operational diagnostics on two development scenarios, not held-out conclusions.
per_delay.csv has one row per physical flight and a prediction-grid-region label;
per_scenario.csv labels all three analysis windows. Paired intervals and detailed
counts are retained in metrics.json and TECHNICAL_APPENDIX.md.

## Validation and preservation

The development smoke used seeds 1 and 3 at D=0, 1 and 2 s for every policy:
42 unique physical flights. Its validation checks event ordering, shared
forecast/runtime, actual approval snapshots, controller/physical continuity,
Eastward motion, earlier violations and seven representative actual trajectories.
A completed-run resume added zero flights; metrics-only regeneration reproduced
all saved output hashes. Journal integrity and one-scenario-at-a-time analysis
remain intact. Original development results, original 1 s freeze and all earlier
artifacts are unchanged. Original 1 s frozen sources are also archived in
source_1s_freeze.zip. Old and new protocols retain strict source checks.

Changed files: baseline_benchmark/run.py, baseline_benchmark/metrics.py,
storage.py (source inventory only), support/validate_baseline_benchmark.py,
new support/test_2s_protocol.py, and README.md. No baselines/, scenario.py,
simulation.py, controller or shared prediction model file changed.

See the [README launch/resume/metrics/validation commands](../../README.md#frozen-2-second-held-out-benchmark--ready-to-launch),
[new freeze](freeze_2s_v2.json), [validation summary](preparation_validation_2s_v2.json),
[development smoke validation](../development/baseline_smoke_2s_v2/validation.json), and
[semantic checks](../development/baseline_smoke_2s_v2/semantics_validation.json).

Full SHA-256 values:

- New protocol: `e0aa044db0c1cbd4fe550360c617c96d09edb2aaec1df8216cd616e98a944f9f`
- Source/environment: `b3f9c1e7ec20077b1192ae8cd30e2ed61d8d5a7c4464f24763082ad58cd9e70b`
- New freeze file: `4a0ff1e95146c897b6f8992299e17d87771bf01bf25f665a94bfad3938b81d6f`
- Unchanged calibration: `4d440b4dca55fe5918bd9a4097f8502eea60cc4e7a987c16f81bc7d84075cb23`
- Original 1 s freeze: `82d7fbd454084a42a55043150768bb8e55699ef9735ccbfbaf7b9c32d67230fd`
