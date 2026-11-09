# Technical appendix

## Protocol and implementation

feasibility_sweep.py calls unchanged simulation.execute from reset; approval state and PID/reference arrays match the frozen benchmark inputs. Reuse: 4,000 matching No Recovery records. New flights: 3,600 missing command starts plus 7,600 independent sideways starts. Four isolated serial simulators use one numerical thread each; four smoke flights verify serial/parallel and archived D=0 physics hashes. All 400 immediate recovery hashes additionally match archived Always Recover flights. No prediction is run during ground-truth generation. All physical samples run the full original horizon, including before intervention. Existing journal/fsync checkpoints provide integrity, duplicate rejection and resume.

Grid: [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]. Boundaries use the first infeasible sample, not a mixed command-first-unsafe/recovery-last-safe definition. Last safe prefix and every sampled feasible/infeasible run remain available in metrics.json; per_scenario.csv lists both first loss and last safe prefix. Censoring is never replaced by a finite boundary or infinity. Non-monotonicity never skips a physical rollout.

## Complete confusion counts and boundary censoring

| Profile | TP | TN | FP | FN | False-feasible scenarios | False-reject scenarios | Predicted / physical right-censored |
|---|---:|---:|---:|---:|---:|---:|---|
| command | 5611 | 1935 | 25 | 29 | 25 | 29 | 79 / 82 |
| completion | 5611 | 1935 | 25 | 29 | 25 | 29 | 79 / 82 |
| recovery | 7032 | 538 | 12 | 18 | 12 | 18 | 156 / 157 |

command: predicted-versus-physical boundary categories: `{'observed__vs__observed': 317, 'right_censored__vs__right_censored': 79, 'left_censored__vs__observed': 1, 'observed__vs__right_censored': 3}`. Predicted/physical left-censored counts: 1/0.

completion: predicted-versus-physical boundary categories: `{'observed__vs__observed': 317, 'right_censored__vs__right_censored': 79, 'left_censored__vs__observed': 1, 'observed__vs__right_censored': 3}`. Predicted/physical left-censored counts: 1/0.

recovery: predicted-versus-physical boundary categories: `{'right_censored__vs__right_censored': 150, 'observed__vs__observed': 237, 'observed__vs__right_censored': 7, 'right_censored__vs__observed': 6}`. Predicted/physical left-censored counts: 0/0.

TP/TN/FP/FN count paired grid evaluations, not independently sampled scenarios. FP/(FP+TN) is the false-feasible rate; FN/(FN+TP) is the false-reject rate. FP/(TP+FP), separate error-scenario counts, signed boundary errors and last-safe-prefix errors are also retained. First-loss error is predicted minus physical sampled first loss; absolute error includes a first-sample failure as sample zero, explicitly flagged as left-censored. No finite error is assigned when a loss is unobserved. All 400 forecast records are retained, not only forecasts that a policy used.

## Readiness and failure definitions

Availability is first_ready_step(recorded warmed runtime)/50 seconds after approval. Equality with a first unsafe recovery sample is late; strictly between last safe and first unsafe is unresolved. A censored physical recovery profile does not supply a deadline. Every actual readiness step in this dataset has a directly matching safe physical recovery flight. No computation was retimed during the parallel physical sweep.

unsafe_recoveries.csv records actual action time, scheduled-command hindsight correctness, command correctness at intervention when measured, and overlapping error/latency tags. Off-grid intervention starts are untested by this sweep and never rounded to a nearby feasibility label. Repeated delays sharing a (scenario, recovery start) are counted separately as flights but not as independent model failures. Correct rejection of the scheduled command does not imply that the command was unsafe at an earlier cancellation time.

| Policy | All unsafe recoveries | After correct scheduled-command rejection | Scenarios for preceding column | Distinct starts for preceding column | Violated before recovery |
|---|---:|---:|---:|---:|---:|
| no_recovery | 0 | 0 | 0 | 0 | 0 |
| always_recover | 0 | 0 | 0 | 0 | 0 |
| fbs | 687 | 687 | 391 | 687 | 476 |
| use_time_recheck | 1462 | 1458 | 400 | 1458 | 496 |
| staleguard | 1230 | 1226 | 321 | 1226 | 407 |
| staleguard_early_recovery | 278 | 274 | 66 | 66 | 0 |
| fixed_early_recovery | 0 | 0 | 0 | 0 | 0 |
| staleguard_recovery_aware | 13 | 11 | 3 | 3 | 0 |
| staleguard_dual_horizon | 13 | 11 | 3 | 3 | 0 |

The complete policy’s 13 unsafe recoveries are seeds 1012, 1242 and 1748. All reproduce independently at the same recovery starts. They also start at/after physical first loss: model error and late physical intervention are overlapping descriptions, not additive causes. By contrast, unready command admission, ready unsafe command admission and unsafe recovery partition the complete policy’s 182 unsafe flights.

## Preserved paired comparisons

| Complete StaleGuard versus | Unsafe prevented | Unsafe introduced | Net unsafe reduction (pp), 95% CI | Completion change (pp) |
|---|---:|---:|---|---:|
| staleguard_early_recovery | 383 | 2 | 7.33 [6.12, 8.63] | -1.12 |
| fixed_early_recovery | 73 | 138 | -1.25 [-2.08, -0.40] | +19.56 |
| use_time_recheck | 1395 | 49 | 25.88 [24.63, 27.13] | +0.06 |

| Analysis subset | Policy | Flights | Unsafe | Safe inspections |
|---|---|---:|---:|---:|
| within_prediction_horizon | staleguard_early_recovery | 4000 | 156 | 2930 |
| within_prediction_horizon | staleguard_dual_horizon | 4000 | 54 | 2872 |
| within_prediction_horizon | fixed_early_recovery | 4000 | 117 | 1883 |
| beyond_prediction_horizon | staleguard_early_recovery | 1200 | 407 | 28 |
| beyond_prediction_horizon | staleguard_dual_horizon | 1200 | 128 | 28 |
| beyond_prediction_horizon | fixed_early_recovery | 1200 | 0 | 0 |

All existing nine-policy aggregate, paired, per-scenario and subset metrics including whole-scenario bootstrap confidence intervals were recomputed identically. The main analysis uses the 400 previously unused cases; no exploratory 100-scenario cohort was merged. policy_tradeoff_delay.png/pdf is a metrics-only rendering of the existing safety/completion and delay-dependent results.

## Historical evidence and integrity

Geofence and Inspection journals are read directly and checked against their recorded hashes/lengths. Geofence Always Recover starts at dispatch and can supply matched delayed recovery profiles (braking/position hold, not Underpass sideways). Inspection Always Recover cancels at approval for positive delays and executes the command at zero: repeated rows do not form a recovery-start sweep. Inspection records retain their historical conversion provenance. Original artifacts, including the stale preparation validation file, remain untouched; a complete read-only source audit is saved here as source_validation.json. Physical sweep completeness is validation.json. preserved_artifact_hashes.json protects all pre-existing result files; analysis asserts every checksum unchanged.

## Reproduction

Run from examples/staleguard with scg active. The sweep is already complete; resume performs no new flights when all keys exist.

```bash
python -m unittest underpass_inspection.support.test_supplementary -v
python underpass_inspection/support/feasibility_sweep.py --workers 4 --resume
python underpass_inspection/support/supplementary_analysis.py
```

A fresh physical reproduction uses feasibility_sweep.py --out <fresh_directory> --workers 4, then supplementary_analysis.py --out <same_directory>. Fig2 contains physical feasibility samples and first-loss gaps only; no representative Underpass flight trajectory figure was generated.
