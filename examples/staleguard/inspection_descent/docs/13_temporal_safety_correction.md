# Temporal safety correction — 21 October 2026

StaleGuard now checks the straight segment between consecutive predicted
positions. TTL ground truth now observes PyBullet every 1 ms, using the same
physics observation hook and the same `unsafe()` predicate as operational
execution. Model dynamics still integrate at 100 Hz with PID control at 50 Hz.
No dynamics parameters, calibration, scenario generation, safety boundaries,
recovery rules, A/B scheduling, or baseline definitions changed.

The reusable geometry function in `geometry.py` tests a segment against a
closed axis-aligned box. `experiment.unsafe_segment()` supplies this mission's
x bounds, y bounds, and unsafe height; the lower z bound is unbounded.
There is no tolerance or added safety margin. Both waiting and post-dispatch
prediction paths use the segment check. `physics_monitor.py` observes each
physics tick once, including the final control endpoint, without changing
actions or stepping the simulator.

**Three previously unsafe Experiment B cases were reproduced.** Their original
commands still produce exactly the saved control/physics traces. Predictions
were ready at dispatch in these actual before/after tests.

| Seed | D | Predicted TTL before → after | Prediction at D before → after | Operational result before → after |
|---:|---:|---:|---|---|
| 9 | 0.3 s | 0.4 → 0.3 s | SAFE → UNSAFE | Unsafe entry → safe cancellation without inspection |
| 7 | 0.4 s | 0.5 → 0.5 s | SAFE → SAFE | Unsafe entry → same unsafe entry |
| 8 | 0.4 s | 0.5 → 0.5 s | SAFE → SAFE | Unsafe entry → same unsafe entry |

Seed 9's fallback starts at the unchanged dispatch event. Its measured
prediction time in this test was 36.14 ms; seeds 7/8 took 44.75/50.48 ms.
Safety-only fallback and a historical mission-retry flight were compared
against the saved execution code with identical complete traces.

For the three original commands, old and corrected ground-truth executions
also had identical actions, control states, and pre-physics positions over the
entire 8.5 s horizon. The corrected first unsafe-entry observations were
1.829 s (seed 9), 2.254 s (seed 7), and 2.091 s (seed 8), from mission start.
Old 50 Hz ground truth missed seeds 9/7 entirely and detected seed 8 at 2.100 s.

**The full calibrated TTL evaluation covered 19 feasible scenarios and 209
scenario-delay pairs.** Seed 16 remains calibration-infeasible and excluded;
seed 10 remains included despite its newly invalid prediction at approval.
The 228 unique PyBullet ground-truth flights comprise 19 nominal flights and
209 delayed commands. D=0.0–1.0 s and the 8.5 s observation horizon are unchanged.

The first column below preserves the original published result. The middle
column reevaluates the old predictions against the corrected ground truth;
this is the relevant baseline for judging the prediction change.

| Metric | Old published evaluation | Old predictions, corrected truth | New predictions, corrected truth |
|---|---:|---:|---:|
| False-safe predictions / actual unsafe | 1/126 (0.79%) | 3/128 (2.34%) | 2/128 (1.56%) |
| False rejections / actual safe | 1/83 (1.20%) | 1/81 (1.23%) | 2/81 (2.47%) |
| Scenarios with a false-safe prediction | 1/19 | 3/19 | 2/19 |
| Scenario classification correct | 19/19 | 19/19 | 18/19 |
| TTL MAE | 0.011 s | 0.021 s | 0.017 s |
| TTL-comparable cases | 19 | 19 | 18 |
| Maximum absolute TTL error | 0.100 s | 0.100 s | 0.100 s |

Ground-truth labels changed only for seed 7/D=0.4 and seed 9/D=0.3. Prediction
labels changed only for seed 9/D=0.3 and **seed 10/D=0.0**. No new false-safe
prediction appeared. False-safe predictions remain at seeds 7/8, D=0.4.
The existing false rejection at seed 12/D=0.5 remains.

**The new false rejection is important:** seed 10's unchanged model has unsafe
1 ms samples at 1.922–1.927 s, between safe 20 ms samples. The segment check
detects this model-predicted crossing, while the actual PyBullet flight is safe.
It is therefore a newly exposed model error. No parameter was adjusted to hide
it. Its prediction becomes `INVALID_AT_APPROVAL`, which the existing TTL-error
formula excludes; this explains the change from 19 to 18 comparable TTL cases.
The aggregate MAEs therefore use different cohorts.

All **209 swept-segment classifications agreed with dense 1 ms model samples**.
Dense samples were obtained through diagnostic partial RK4 evaluations, with
each native 20 ms endpoint verified bit-for-bit unchanged. This includes the
predicted crossings for seed 9/D=0.3 and seed 10/D=0.0. No extension beyond
straight-segment checking was needed for this grid.

**Prediction overhead was measured using identical saved snapshots**, after
JIT warmup, with alternating old/new order and five repeats per scenario:
95 paired measurements. These measurements include model construction and
prediction, and exclude snapshot capture and PyBullet ground truth.

| Prediction time | Old | New |
|---|---:|---:|
| Mean | 51.48 ms | 54.84 ms |
| Median | 47.61 ms | 50.58 ms |
| 95th percentile | 115.55 ms | 122.33 ms |
| Maximum | 120.53 ms | 131.65 ms |

Mean additional time was **3.36 ms (+6.52%)**; median paired additional time
was 3.35 ms. Host timing varied substantially: the separate native TTL-runner
pass reported a 121.2 ms mean and 200.2 ms p95. The paired comparison controls
old/new inputs and order; neither set is an onboard timing guarantee. Some
measured durations exceed D=0.1 s, at which B's unchanged unavailable-result
rule can execute the original command. A complete updated operational matrix
was not run; only the three required operational cases were replayed.

Validation also passed four unit tests, including closed-boundary contact,
125 stationary-point comparisons against the original unsafe predicate,
crossings during nominal and commanded rollouts, and observation of physics
endpoints without changing steps. Source/AST checks preserve scenario generation,
command semantics, point predicates, scheduling, baseline helpers, and reporting.
All existing model files, calibration, historical logs, and diagnostic evidence
retain their recorded hashes.

The existing 2 GiB memory guard stopped the initial combined run as host memory
fell. Fourteen complete scenario results were retained from its log. The
remaining measured ground truth was completed in a simulator-only serial worker
(about 601 MiB RSS). Saved approval snapshots and measured truth were then reused
by the native `mission.main()` evaluation and paired prediction checks, without
loading the simulator. No guard threshold was weakened and no simulation pools
were launched. Per-flight remaining measurements were checkpointed on disk.

Remaining limitations: a straight segment approximates the curved model path;
agreement on this finite grid is not a proof for arbitrary trajectories. Dense
diagnostic samples and PyBullet ground truth are themselves sampled at 1 ms.
Model errors in seeds 7/8 and the seed 10 false rejection remain. The larger
model-error correction and uncertainty-margin problem is unchanged.

Evidence and reproduction:

- [Updated research log](../result/ttl_accuracy_swept_physics.log)
- [Before/after metrics and paired timing summary](../results/temporal_safety_correction/summary.json)
- [Three-case checks](../results/temporal_safety_correction/small_checks.json)
- [All dense comparisons](../results/temporal_safety_correction/dense_comparison.json)
- [Validation source](../diagnostics/validate_temporal_safety.py)
- [Geometry/monitor unit tests](../diagnostics/test_temporal_safety.py)

```sh
python -m unittest discover -s inspection_descent/diagnostics -p test_temporal_safety.py -v
python inspection_descent/mission.py --calibrated --count 20
```

The second command performs a new full evaluation; redirect it to a new log.
Old research logs remain unchanged. Detailed checkpoints, timing samples,
before-change source copies, and the simulator-only worker are preserved under
`results/temporal_safety_correction/`.
