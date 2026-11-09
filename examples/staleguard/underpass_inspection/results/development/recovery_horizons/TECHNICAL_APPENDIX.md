# Recovery-aware extension — technical appendix

## Source changes

Created:

- `baselines/recovery_horizons.py`: three full feasibility profiles; original PID/RK4 kernels; compiled continuations and exact safety broad phase.
- `baselines/recovery_policies.py`: two policy subclasses reusing the existing readiness, event, cancellation and physical maneuver lifecycle.
- `baseline_benchmark/horizons.py`: separate two-policy execution, source/matrix checks, safe resume, streaming diagnostics, nine-policy metrics-only merge and validation.
- `support/profile_recovery_horizons.py`: development-only optimization and archived physical-label comparison.
- `support/test_recovery_horizons.py`: focused predictor, policy, equality, non-monotonicity, censoring, analysis and uncertainty tests.

Modified:

- `simulation.py`: optional `policy_factory` injection only. Physical execution syntax, controller and safety semantics are unchanged.
- `storage.py`: include extension source hashes; recognize the exact pre-injection simulation checksum only when its unchanged physical syntax matches the pinned physical hash.
- `baseline_benchmark/metrics.py`: recognize new approval predictors, stream two/nine-policy summaries, add the requested paired comparisons and whole-scenario bootstrap intervals. Existing seven-policy numeric development metrics reproduce exactly.
- `support/test_2s_protocol.py`: retain exact original baseline checksums; validate the reviewed simulator injection by pinned physical syntax.
- `README.md`: policy rules, limits and reproduction commands.

The existing five baseline/predictor modules, scenario/calibration definitions, shared lightweight model, PID and physical safety monitor are unchanged. No other mission is imported or modified. An unrelated pre-existing `.gitignore` modification was left alone.

## Prediction and policy rules

The fixed candidates are 0–0.20 s in 0.02 s increments, then 0.30–1.00 s in 0.10 s increments. For each candidate, retain `command_safe`, `command_complete` (safe AND inspected), and `recovery_safe`. Every candidate is evaluated independently; an unsafe flight can stop because whole-flight safety and safe completion cannot subsequently become true. Non-monotonic tails remain recorded; conservative scalar boundaries use the first invalid sample. Each boundary records the last tested safe point, the last safe prefix point, non-monotonicity and censoring.

The nominal waiting path is integrated once and checkpointed with actual approval state and PID memory. Recovery and command restore the same checkpoints; neither knows the scheduled dispatch delay. Compilation wraps the existing numerical kernels without fast-math, changed integration steps, reduced horizons or numerical approximations. A separating coordinate can certify safety; near boundaries/corners the original exact swept Euclidean body/box distance remains mandatory. Clearance thresholds are unchanged.

Safe completion is computed within the command rollout using continuous residence on piecewise-linear segments between native 100 Hz RK4 positions, with the original fixed inspection box, dwell and observation deadline. This is an explicit model-time approximation of physical 1,000 Hz monitoring, not a changed physical completion criterion. Broad-phase and early-exit strategies preserve all three profile labels. Full compiled/reference terminal state and PID arrays were equal on all 12 development scenarios.

RecoveryAware's pending trigger is the earlier of command first-invalid time and recovery's last safe sampled prefix point. DualHorizon also considers completion first-invalid time. A recovery profile unsafe at zero has no certified prefix; all-safe profiles have no invented finite expiration. An all-safe recovery profile permits using its sampled prefix through 1 s, but does not create a timer at 1 s. Non-monotonic returns to feasibility do not undo conservative first-invalid boundaries.

Readiness is atomic and measured for the entire forecast/plan construction. No policy uses the intermediate recovery-profile timestamp. Each new policy independently computes/times its own full forecast per frozen scenario; only that policy reuses it across paired delays. Discarded development-only numerical warmup is recorded separately, never reused as an evaluation forecast. The final profiling process's warmup/cache-loading call took 0.335 s; this is not a fresh-machine first-ever compilation measurement. Total benchmark wall time and warmed online computation are separate quantities.

Dispatch wins a timer tie. RecoveryAware's dispatch gate preserves existing safety admission: ready and at/after finite command expiry rejects; unavailable predictions or no observed expiry admit, including beyond 1 s. DualHorizon additionally rejects completion-only expiration only while inside the sampled conservative recovery prefix. Once that prefix is missed, a still-safe command is not invalidated merely because recovery is predicted unsafe. If command safety has expired, the unchanged fallback is best-effort and explicitly marked unverified. Pending recovery occurs only after readiness and the trigger. Command cancellation cannot be overridden by a later dispatch. No physical recovery is restarted. There is no recovery oracle or climb selection.

## Artifacts and validation

- `profiling_original.json`: original command-only timings and computational profile, collected before optimization.
- `optimization.json`: four strategies, 36 timings each, exact-state/PID checks, all three profiles for 12 development scenarios, archived physical-label errors and source/environment metadata.
- `beyond_horizon.json`: four actual development-only D=2 flights; seed 3's unfavorable outcome is retained.
- `integrity_checks.json`: duplicate-key, physical-protocol mismatch and changed-source rejection.
- `preservation_validation.json`: 1,058 pre-existing artifact checksums and 18 exact independent trajectory replays verified.
- `../recovery_horizons_smoke_v2/`: 18 actual new-policy flights plus validation, metrics, reports and one representative trajectory archive.
- `../recovery_horizons_merged_smoke_v1/`: 81 records (63 archived +18 new), no new original-policy simulations.

Five preliminary development-only flights were also used while wiring the driver, before the main interruption/resumption test. Their debug archive is retained at `/tmp/underpass_horizons_before/preliminary_5flight_smoke`; the main and supplemental validation contain 22 flights. Total new physical executions during this implementation: 27, all development-only. No 2,600-flight evaluation run was started.

The unfrozen smoke driver received reporting-only fixes after physical execution: consistent selected-delay truth filtering, extra regression tests, and replacing a compact whole-journal diagnostic list with streaming counters. Those reviewed updates are explicitly recorded with previous source provenance in the new smoke run headers. Predictor, policy, physical executions and cached readiness runtimes were unchanged. Metrics regeneration now reproduces exactly. The evaluation freeze was created only after these fixes and tests; source changes thereafter are rejected rather than silently resumed.

No Recovery labels for the extension come from the unchanged paired 9,100-flight source. No additional physical deadline sweep is required. Recovery accuracy in future benchmark reports is limited to unique actual recovery starts coinciding with predictor-grid points; those outcomes do not establish a complete independent recovery-expiration curve. The development diagnostic can compare all 228 sampled recovery labels because the archived independent sweep already exists.

The merger verifies old recorded freeze/provenance, shared numerical environment, calibration/snapshot hashes, matching physical protocol, exact scenario-delay matrix and unique keys. It streams one scenario at a time and pins both journals and run headers. It preserves the seven archived outcomes and uses only newly executed outcomes for the two additional policies. The resulting 11,700-record comparison is exploratory, with full / <=1 s / >1 s subsets, per-delay tables, model-label errors, policy admission errors, readiness, runtime and recovery failure decomposition. The seven requested new paired contrasts and policy rates have whole-scenario 95% percentile bootstrap intervals (2,000 resamples, seed 2027). Delays are never independent sampling units.

Each physical journal row is fsynced before its byte offset and cumulative SHA-256 are atomically committed to `run.json`. Resume verifies that prefix, rejects duplicates/incompatible provenance and truncates only uncommitted tail bytes. Metrics-only regeneration and validation perform no physical simulation. Only representative trajectories are retained, not one file per execution.

## Reproduction

From `examples/staleguard`, with the existing `scg` environment active:

```bash
# Focused tests: 58 tests, no original-policy physical flights.
python -m unittest \
  underpass_inspection.support.test_recovery_horizons \
  underpass_inspection.support.test_baselines.PolicyTests \
  underpass_inspection.support.test_baselines.PredictionTests \
  underpass_inspection.support.test_pilot \
  underpass_inspection.support.test_development \
  underpass_inspection.support.test_final_audit \
  underpass_inspection.support.test_2s_protocol \
  underpass_inspection.support.test_reduced_protocol -q

# Reprofile all 12 development scenarios; no physical flights.
python underpass_inspection/support/profile_recovery_horizons.py

# New development smoke, stop after five flights, then resume. Use a fresh output.
python underpass_inspection/baseline_benchmark/horizons.py --smoke --seeds 1 3 6 \
  --baseline underpass_inspection/results/development/baseline_benchmark \
  --calibration underpass_inspection/results/development/calibration \
  --out underpass_inspection/results/development/recovery_horizons_smoke_reproduction \
  --max-flights 5
python underpass_inspection/baseline_benchmark/horizons.py --smoke --seeds 1 3 6 \
  --baseline underpass_inspection/results/development/baseline_benchmark \
  --calibration underpass_inspection/results/development/calibration \
  --out underpass_inspection/results/development/recovery_horizons_smoke_reproduction --resume
python underpass_inspection/baseline_benchmark/horizons.py \
  --baseline underpass_inspection/results/development/baseline_benchmark \
  --calibration underpass_inspection/results/development/calibration \
  --out underpass_inspection/results/development/recovery_horizons_smoke_reproduction --validate

# Prepare/verify the 2,600-flight freeze, with zero physical executions.
python underpass_inspection/baseline_benchmark/horizons.py --prepare-only \
  --freeze underpass_inspection/results/evaluation/preparation/freeze_recovery_horizons_v1.json
```

Launch/resume, new-run validation, nine-policy merge, metrics-only regeneration and merged validation commands are in the [mission README](../../../README.md#exploratory-recovery-aware-extension). These use defaults pointing to the actual `results/evaluation/calibration` and completed `results/evaluation/baseline_benchmark` paths. Both output folders are separate from the historical benchmark. Stop before launching if the source/environment freeze no longer verifies; create a new reviewed version instead.

## Frozen preparation hashes

- `freeze_sha256`: `92366f9b7fa28b2d0bd3c95ab42d0b456062492318b6812662d0e4d710affa94`
- `protocol_sha256`: `d77da6b9a74bc33f9946731d40bcbb7b6beb1e729d1382515399785828c79cd4`
- `provenance_sha256`: `e6bd6143683bb2e69dca8acbefe59e26b3065e846ba4c55791fb20d6f421a76d`
- `calibration_sha256`: `4d440b4dca55fe5918bd9a4097f8502eea60cc4e7a987c16f81bc7d84075cb23`

Freeze: `results/evaluation/preparation/freeze_recovery_horizons_v1.json`. The reserved evaluation output directory does not yet exist; zero new evaluation flights were launched.
