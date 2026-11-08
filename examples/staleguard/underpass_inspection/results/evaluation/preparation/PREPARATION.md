# Underpass Inspection: held-out preparation

Prepared and froze the seven-policy benchmark for **865 accepted scenarios from
1,000 attempts**. The UAV approaches a bridge, dives underneath to inspect and
swerves East around a pier. Recovery abandons inspection and turns East beside
the deck. The full run will contain **308,805 actual flights**. All **36 focused
tests** and the **42-flight operational smoke** passed, including interruption,
resumption and metrics regeneration. Fixed a readiness timestamp rounding error;
policy decisions are unchanged. The driver now obtains command-safety labels
from No Recovery and processes the journal with bounded memory. All 1,001
historical artifacts are unchanged. The higher calibration acceptance rate has
no detected protocol/source inconsistency. No held-out comparative conclusions
were drawn, and the full benchmark was not launched. Next: use the frozen launch
command in the README; preserve source/environment for resumption.

| Audit | Finding and frozen behavior |
|---|---|
| StaleGuard grid | 19 points: 0–0.20 s every 0.02 s, then 0.30–1.00 s every 0.10 s. First unsafe point is TTL; equality rejects when ready. Off-grid false-safe admissions remain reported. |
| FBS | Correct EMA with alpha 0.1, initial volatility zero, margin/volatility TTL, epsilon 1e-12 and 210-step cap. Development TTLs were 57–97 steps (1.14–1.94 s), beyond the 50-step sweep. Zero recoveries are expected from these rules. |
| Readiness | Same forecast/runtime for both StaleGuard variants; measured warmed computation is replayed on the control clock. Late results never act retroactively. Only the logged first-ready step needed a floating-point boundary correction. |
| Use-Time Recheck | Observes actual dispatch state/PID, continues nominal flight while checking, then applies its decision to the later state. Earlier violations persist. |
| Information | Policies receive approval/current state, elapsed time and actual dispatch events; never future dispatch time or offline safety labels. |

## Calibration integrity

The completed manifest is `results/evaluation/calibration/calibrated_scenarios.json`:
**1,000 attempts, seeds 1001–2000; 865 accepted, 135 rejected**. Every deterministic
draw, acceptance decision, immediate-flight monitoring horizon and accepted
snapshot hash was checked. All 865 accepted D=0 replay hashes match their original
D=0 flights. One draw per seed, no replacement, no approval optimization and no
positive-delay selection were found. Geometry, controller and safety-source checks
pass. Development remains 12/20 accepted; all rejections in both cohorts are
D=0 pier-clearance violations. The sampled pier X averages are 2.668 m in the small
development cohort and 2.794 m in this cohort, consistent with more room before
the pier. This is a sampling observation, not grounds to alter or repair the set.

## Freeze and operational validation

All seven policies, fixed 0.20 s timer, FBS parameters, prediction grid, runtime
conventions, 51 dispatch delays, physical sources and metrics are pinned in
[freeze.json](freeze.json). The full protocol includes every accepted scenario;
full-run seed/policy/delay filtering is rejected. No separate deadline sweep is
needed. Paired No Recovery flights supply command labels and observed command
boundaries; complete all-safe grids remain censored and non-monotonic intervals
are retained. No independent recovery deadline is inferred.

The smoke used the first two accepted IDs, delays 0, 0.5 and 1 s, and all seven
policies. It stopped after 10 flights, repaired a deliberately uncommitted tail,
and resumed for 32 more: **42 unique records**. Calibration hashes match for both
immediate command flights and all six Always Recover flights. The seven saved
representative trajectories were independently checked at 1,000 Hz. Structural
checks confirm common initial snapshots, controller continuity, Eastward recovery,
shared forecasts and cancellation. These checks do not establish comparative
held-out policy performance.

Journal appends are fsynced before atomic metadata commits. Resumption verifies
the committed prefix and discards only uncommitted bytes. One scenario is retained
at a time for metrics; source/environment and frozen settings are checked on
launch/resume. Full launch, resume, metrics-only and completed-result validation
commands are in the [README](../../README.md#frozen-held-out-benchmark--ready-to-launch).
The full output directory has not been created.

Modified source files: `storage.py`, `baselines/staleguard.py`,
`baselines/policies.py`, `baseline_benchmark/run.py`, `baseline_benchmark/metrics.py`,
`support/validate_baseline_benchmark.py`, new `support/test_final_audit.py`, and
`README.md`. No physical mission, controller, sampling or baseline parameter changed.

Full checksums:

- Calibration manifest: `4d440b4dca55fe5918bd9a4097f8502eea60cc4e7a987c16f81bc7d84075cb23`
- Protocol: `da1a2d47f39dab1ad9cdba013315a13eec70e0b8659d488df1ac554eb353d833`
- Source/environment: `91e5a840409ea15a2c2889444f65bdc62f13f50f0e7d9a3cc272be28ac90cecf`
- Freeze file: `82d7fbd454084a42a55043150768bb8e55699ef9735ccbfbaf7b9c32d67230fd`

See [calibration audit](calibration_audit.json), [preparation validation](preparation_validation.json) and [smoke validation](baseline_smoke/validation.json).
