# E01 nominal reproduction investigation

## Finding

All 50 nominal action differences are explained by the recorded forecast computation times. Holding readiness to its original recorded value restores every archived StaleGuard decision. The audited evidence does not show a prediction-label or policy-rule mismatch, or physical nondeterminism in the independently checked flights.

E01 N0 uses freshly measured, shared full-predictor timing. It should be described as a fresh-timing nominal replication; exact archived-count reproduction requires replaying the archived policy-specific computation times. The completed E01 records have not been modified.

## Complete matched population

- 64 frozen scenarios × 13 original dispatch delays × six policies = 4,992 nominal outcomes.
- 64/64 true approval snapshots match archived physical state, reference, PID memory, scenario and remaining horizon exactly.
- All 64 full feasibility profiles match the archived Dual-horizon profiles and the archived Recovery-aware profiles exactly: labels, boundaries, policy plans, grids and rollout diagnostics. Command-only safety labels and command cutoffs also match on all 64 scenarios.
- Replaying the original policy classes reproduces all 4,992 archived decisions with their own recorded inputs, and all 4,992 E01 decisions with E01 inputs.
- Replacing only the runtime in the E01 full profile with the archived policy-specific runtime restores all 2,496 archived StaleGuard decisions. The converse runtime swap also reproduces all E01 StaleGuard decisions. No predictor computation or physical simulation is used for these decision-only checks.

## Why readiness differs

The archived benchmark measured a command-only predictor for Command-safety-aware StaleGuard, and separately measured full forecasts for Recovery-aware and Dual horizon. E01 follows its one-full-profile-per-scenario-condition requirement and shares that forecast and runtime across all three configurations. This changes the timed workload supplied to the command-safety ablation. Its intervention rule is unchanged.

Additionally, E01 freshly measured the full forecast runtime rather than reusing archived times. On the selected 64 scenarios, the archived full-predictor median/p95 were 147.205/167.894 ms, versus 165.244/196.822 ms in E01. Historical CPU scheduling, clock/thermal state and allocation profiles were not recorded in sufficient detail to identify the unique cause of this timing shift. Ordinary timing variation is plausible, but a specific hardware/OS explanation is unproven.

The unchanged monitor quantizes availability onto the 50 Hz control clock. Crossing a 20 ms readiness boundary can change recovery timing or cause fail-open command dispatch. No rounding bug was found: the policy loops in the native physical driver and E01 replay agree with recorded readiness.

## Nominal outcome accounting

All counts below use 832 scenario-delay trials per policy from the same 64 scenarios. They are not the full 400-scenario paper benchmark.

| Policy | Changed actions | Changed outcomes | Archived unsafe | E01 unsafe | Archived safe completion | E01 safe completion |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| No Recovery | 0 | 0 | 337 | 337 | 495 | 495 |
| Use-Time Recheck | 0 | 0 | 248 | 248 | 485 | 485 |
| Fixed EarlyRecovery | 0 | 0 | 15 | 15 | 305 | 305 |
| Command-safety-aware | 30 | 3 | 127 | 124 | 493 | 493 |
| Recovery-aware | 10 | 1 | 28 | 29 | 484 | 484 |
| Three-opportunity-aware | 10 | 1 | 28 | 29 | 484 | 484 |

The 50 action changes are five policy–scenario groups, each affecting the ten tested delays at or above 0.12 s. Only the 0.12 s trial changes its safety outcome in each group. Recovery-aware and Three-opportunity-aware share the affected seed 1061; Command-safety-aware changes at seeds 1374, 1777 and 1859. There are four distinct affected scenarios, five policy outcomes, and no changed safe-completion labels.

| Seed | Policy | Archived runtime | E01 runtime | Availability after approval | Safety at D = 0.12 s |
| --- | --- | ---: | ---: | --- | --- |
| 1061 | Three-opportunity-aware | 112.976 ms | 124.116 ms | 0.12 → 0.14 s | safe recovery → unsafe command |
| 1061 | Recovery-aware | 112.980 ms | 124.116 ms | 0.12 → 0.14 s | safe recovery → unsafe command |
| 1374 | Command-safety-aware | 144.428 ms | 110.669 ms | 0.16 → 0.12 s | unsafe command → safe recovery |
| 1777 | Command-safety-aware | 143.587 ms | 112.004 ms | 0.16 → 0.12 s | unsafe command → safe recovery |
| 1859 | Command-safety-aware | 140.049 ms | 119.793 ms | 0.16 → 0.12 s | unsafe command → safe recovery |

For seed 1061, the new full prediction misses the 0.12 s dispatch by one control step, so the unchanged fail-open rule executes the command. For the other three seeds, the shared full prediction is ready by dispatch while the archived command-only prediction was not, so recovery becomes available sooner.

## Independent physical checks

The original E01 nominal equivalence verification contains 128 independently simulated command/recovery-at-approval flights (two per selected scenario); all match archived trajectory hashes and outcomes. This audit additionally ran the native physical policy driver on the eight distinct old/new action continuations involved in the five changed outcomes. All eight match their expected action, trajectory hash, physical safety and safe-completion labels exactly. These diagnostic simulations are saved only in nominal_audit/ and are excluded from the E01 study totals.

Unchanged-action production records often reuse archived flights, so matching hashes in those cached records alone would not prove determinism. The independent replays above provide the additional physical evidence; they do not constitute an exhaustive rerun of every physical continuation.

## Scientific interpretation and recommendation

The nominal discrepancies are fully explained, and no change to the predictor or policy decision rules is justified by this audit. Preserve the completed E01 records and use E01 N0 as the nominal reference for E01 comparisons. Explicitly disclose that N0 uses freshly measured full-profile readiness and a common prediction workload across the three StaleGuard configurations. Do not claim exact reproduction of the archived nominal outcome counts.

Observed uncertainty-condition differences combine effects of uncertainty, any uncertainty-dependent predictor workload, and measured computation-time variability. The bootstrap represents scenario variation; it does not independently quantify repeated-run hardware timing variability. A latency-controlled decision replay would help isolate prediction/plant effects, but would be a separately documented diagnostic and must not replace or silently alter the locked completed experiment. No such additional experimental condition, predictor retuning, or result replacement was performed here.

If exact historical ablation timing is required in a future design, reconcile that requirement explicitly with the E01 requirement to compute only one full forecast per scenario-condition. This audit does not silently resolve that protocol tension by changing the experiment.

## Integrity and outputs

All 69,048 protected original/E01/source files were hash-verified before and after the audit and remain unchanged. See PROVENANCE.json for their hashes and the audit environment.

- SUMMARY.json: machine-readable findings.
- profile_equivalence.csv: all per-scenario forecast and timing comparisons.
- all_nominal_trials.csv: all 4,992 matched nominal comparisons.
- changed_decisions.csv: all 50 changed decisions with runtime and readiness steps.
- timing_decomposition.csv: old policy-specific timing → shared old-full timing → E01 shared timing. These stages can cancel changes; their counts must not be added to infer the net.
- policy_summary.csv: the six nominal policy outcome comparisons.
- independent_nominal_replays.csv: the 128 original independently simulated nominal checks.
- changed_outcome_physical_replays.csv and eight JSON/NPZ pairs: new bounded native-driver physical verification.
- investigate.py / audit.log: reproducible analysis and execution log.

Rerun from the repository root with the scg environment active:

```bash
python -B -m outputs.robustness_v1.nominal_audit.investigate --physical-replay
```

Completed diagnostic physical replays are reused on subsequent audit executions. Production and historical records are never overwritten.
