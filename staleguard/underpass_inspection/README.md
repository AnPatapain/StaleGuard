# Underpass Inspection

The UAV approaches a bridge at deck height. Its approved command dives under the
deck, inspects a fixed region, then swerves East around a pier. Waiting shifts
that same relative maneuver toward fixed obstacles. Recovery abandons inspection
and uses the existing independent Eastward sideways maneuver. All policies use
the same SCG/PyBullet dynamics, PID, 50 Hz control, 1,000 Hz physical safety
monitoring and eight-second observation horizon after approval.

The historical benchmark contains **100 scenarios × 13 delays × 9 policies =
11,700 flights**. Its [report](results/evaluation/baseline_benchmark/REPORT.md)
is exploratory because those scenarios were already examined. The self-contained
canonical folder is `results/evaluation/baseline_benchmark/`; old source-result
folders are no longer required.

The prepared **v2 evaluation** contains **400 different scenarios × 13 delays ×
9 policies = 46,800 flights**. Its fixed RNG seed is **20261109**. Selection is
uniform without replacement from the remaining 765 accepted IDs, excluding all
100 historical IDs; it reads no policy outcomes. The two cohorts cover **500
unique scenarios**. No v2 physical flight has been launched during preparation.

## Physical convention

The mission uses North–East–Down (NED). PyBullet stays Z-up:

```python
x_north = x_world
y_east = -y_world
z_down = -z_world
```

Both lateral maneuvers move toward negative world Y, positive East. The bridge,
pier and inspection region remain fixed. No controller reset or state jump occurs
at dispatch/recovery. Continuing nominal flight eventually encounters the deck.
Safety is separate from inspection: successful inspection requires the existing
continuous dwell before the deadline and no safety violation anywhere in the
flight. Climb-over remains diagnostic and is never selected by the nine policies.
The existing camera, NED inset, transparent deck, checkerboard floor and
`pilot.py --overlay-all` remain unchanged.

## One implementation

```text
scenario.py                       fixed mission and geometry
simulation.py                     physical execution and monitoring
baselines/policies.py             nine-policy registry, lifecycle and decisions
baselines/staleguard.py            original command-only predictor
baselines/recovery_horizons.py     optimized command/completion/recovery predictor
baselines/fbs.py                   EMA-based FBS
baselines/use_time_recheck.py      dispatch-state immediate-command forecast
baselines/fixed_early_recovery.py  fixed 0.20 s trigger
baseline_benchmark/run.py          prepare, execute/resume, validate, metrics-only
baseline_benchmark/metrics.py      one metrics/reporting pipeline
support/validate_baseline_benchmark.py  self-contained result audit
```

| Policy | Behavior |
|---|---|
| `no_recovery` | Execute the approved command at dispatch. |
| `always_recover` | Cancel and recover at approval. |
| `fbs` | Freeze the existing clearance-feature EMA prediction at approval; decide at dispatch. |
| `use_time_recheck` | Observe at dispatch; continue nominal during checking; apply the decision afterward. |
| `staleguard` | Approval-time command forecast; reject at dispatch if ready and expired. |
| `staleguard_early_recovery` | Same forecast; recover once ready and expired while still pending. |
| `fixed_early_recovery` | Recover at elapsed 0.20 s if still pending. |
| `staleguard_recovery_aware` | Also consider predicted loss of sideways recovery feasibility. |
| `staleguard_dual_horizon` | Additionally consider loss of safe inspection completion. |

Policies never know future dispatch time. The physical driver announces dispatch
when it occurs. The original StaleGuard pair shares one forecast/runtime, as in
the validated experiment. Each newer policy independently computes/times its own
three-profile forecast. Neither receives the other policy's computation cache.
FBS retains alpha 0.1, epsilon 1e-12 and its existing 210-step cap.

The prediction grid is unchanged: 0–0.20 s every 0.02 s, then 0.30–1.00 s every
0.10 s: **19 candidates**. First-unsafe command expiration is conservative;
non-monotonic profiles are retained, and all-safe profiles have unobserved
expiration. No expiration found through 1 s does **not** establish safety beyond
1 s: the unchanged admission gate still permits later dispatch. An unavailable
forecast permits dispatch. Prediction readiness uses full measured warmed
computation, available on the first eligible 50 Hz step. Dispatch wins timer ties.

RecoveryAware uses the earlier of command expiration and the last safe sampled
recovery-prefix point. DualHorizon also considers completion expiration. A missed
recovery prefix does not invalidate a still-safe command. Completion-only
abandonment requires predicted recovery feasibility; an expired unsafe command
uses the same fallback as best-effort. Full online computation counts toward
readiness. The optimized predictor preserves numerical kernels and resolution,
reuses waiting/PID checkpoints and stops only irreversibly unsafe continuations.

The physical dispatch grid is **13 delays**, independent of prediction resolution:

```text
0.00, 0.04, 0.08, 0.12, 0.16, 0.20, 0.40,
0.60, 0.80, 1.00, 1.20, 1.60, 2.00 seconds
```

## Launch and resume v2

From `the repository root`, with the existing `scg` environment active:

```bash
# Launch the already-prepared 46,800-flight evaluation.
python -m staleguard.underpass_inspection.baseline_benchmark.run \
  --out staleguard/underpass_inspection/results/evaluation/baseline_benchmark_v2

# Resume after interruption, with identical inputs and policy configuration.
python -m staleguard.underpass_inspection.baseline_benchmark.run \
  --out staleguard/underpass_inspection/results/evaluation/baseline_benchmark_v2 --resume

# Validate completed results; no simulations. Before launch this validates preparation.
python -m staleguard.underpass_inspection.baseline_benchmark.run \
  --out staleguard/underpass_inspection/results/evaluation/baseline_benchmark_v2 --validate

# Regenerate reports, CSVs, paired metrics and confidence intervals; no simulations.
python -m staleguard.underpass_inspection.baseline_benchmark.run \
  --out staleguard/underpass_inspection/results/evaluation/baseline_benchmark_v2 --metrics-only
```

`--policies` supports any subset on a fresh development run. Prepared protocols
cannot silently change policies, delays or scenario IDs. Use `--smoke` only with
development data; do not use it for v2. Preparation can be reproduced into a fresh
folder without executing physical flights:

```bash
python -m staleguard.underpass_inspection.baseline_benchmark.run --prepare-only \
  --calibration staleguard/underpass_inspection/results/evaluation/calibration \
  --exclude-run staleguard/underpass_inspection/results/evaluation/baseline_benchmark \
  --sample-count 400 --sample-seed 20261109 \
  --out staleguard/underpass_inspection/results/evaluation/baseline_benchmark_v2_reproduction
```

## Self-contained results and integrity

- `run.json`: immutable experiment configuration, source/environment information,
  frozen scenario definitions and historical provenance where applicable.
- `progress.json`: status, counts, committed offsets and integrity hashes.
- `executions.jsonl`: actual physical flight records; no reconstructed outcomes.
- `predictions.jsonl`: independently timed forecasts, with the original pair's
  shared forecast stored once. It avoids rewriting the full cache after each flight.
- `approval_snapshots.npz`: actual approval state, PID memory and nominal references.
- `representative_trajectories.npz`: selected actual 1,000 Hz position traces.
- `metrics.json`, CSVs, `REPORT.md`, `TECHNICAL_APPENDIX.md`: reproducible analysis.
- `validation.json`: audit results. Historical source/preparation evidence is
  retained in `source_versions.zip`, without duplicate execution journals.

The validator needs only this folder and the code/environment; it does not open
old benchmark folders or the calibration dataset. It checks complete unique
matrices, input integrity, recorded policy decisions/readiness, physical monitoring,
representative trajectories and exact metric regeneration. Metrics-only analysis
also uses only this folder. Resume additionally requires unchanged executable
source/environment. Each flight or forecast is fsynced before its small progress
checkpoint is atomically replaced; resume discards only uncommitted tail bytes.

Paired comparisons and 95% percentile bootstrap intervals resample whole
scenarios (2,000 resamples, seed 2027), retaining all delay/policy pairs. Reports
include full 0–2 s, <=1 s and >1 s subsets. Delay trials are correlated, not
independent scenarios. Reduced temporal resolution limits boundary precision.

## Development and visualization

The [unification validation report](results/development/unification_validation/REPORT.md)
and its [technical appendix](results/development/unification_validation/TECHNICAL_APPENDIX.md)
record exact historical regression, development smoke tests, source cleanup and
predictor timing. Existing calibration, deadline-sweep, pilot and unrelated
development results remain preserved.

```bash
# Small development-only run; fresh output required. All nine policies by default.
python -m staleguard.underpass_inspection.baseline_benchmark.run --smoke --seeds 1 3 \
  --delays 0 1 2 --out staleguard/underpass_inspection/results/development/smoke_reproduction

# Focused tests, excluding optional physical stress tests.
python -m unittest staleguard.underpass_inspection.support.test_benchmark \
  underpass_inspection.support.test_baselines.PolicyTests \
  underpass_inspection.support.test_baselines.PredictionTests \
  underpass_inspection.support.test_recovery_horizons \
  underpass_inspection.support.test_pilot underpass_inspection.support.test_development -q

# Plot retained actual trajectories from the historical nine-policy comparison.
python -m staleguard.underpass_inspection.support.visualize_policy_comparison \
  --run staleguard/underpass_inspection/results/evaluation/baseline_benchmark \
  --seed 1008 --delay 1.0

# Original physical mission viewer: seed 0, corrected NED camera, half-speed playback.
python -m staleguard.underpass_inspection.pilot --mode dive --delay 0 --gui --hold 10
```

GUI policy playback adds `--gui --policy NAME` to the comparison command and
replays the recorded maneuver/start time, checking its physical hash. It does not
remeasure policy latency or produce a new benchmark score. The memory guard and
single-simulation-worker lock remain active.
