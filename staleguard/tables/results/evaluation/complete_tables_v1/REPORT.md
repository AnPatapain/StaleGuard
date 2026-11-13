# Final experimental tables

The workflow fills the missing three-opportunity prediction and policy evaluations on frozen cohorts of 276, 300 and 400 scenarios. For example, while the UAV approaches a fence, it can cancel the pending turn and start its existing recovery before that escape becomes unsafe. Existing valid physical flights are reused; missing continuations are flown independently in PyBullet. Inspection Descent full StaleGuard: 8/3036 unsafe flights and 913 safe completions. Geofence Turn full StaleGuard: 44/6900 unsafe flights and 1656 safe completions. Underpass full StaleGuard: 182/5200 unsafe flights and 2900 safe completions. All required experiments and both tables validate. These are sampled simulation results with offline prediction timing, not hardware safety guarantees. Inspection’s legacy command-only predictor has a different forecast horizon, so that comparison does not isolate intervention logic alone. Next, transfer the verified values to the paper and retain these limitations.

| Mission | Scenarios × delays | Full unsafe / cases | Full safe completions | Predictor median / p95 (ms) |
|---|---:|---:|---:|---:|
| Inspection Descent | 276 × 11 | 8 / 3036 | 913 | 41.24 / 55.28 |
| Geofence Turn | 300 × 23 | 44 / 6900 | 1656 | 96.38 / 141.10 |
| Underpass | 400 × 13 | 182 / 5200 | 2900 | 149.00 / 167.44 |

The new policy matrices contain 25,944 cases: 735 new native physical records and 25209 cases linked to physically identical measured continuations. Inspection additionally required 2,760 delayed-recovery physical records; Geofence reused its complete existing independent recovery grid. Prediction timing remains independently measured for each policy. Smoke flights are separate validation evidence. Underpass was not rerun.

See [Table I](table_I.md), [Table II](table_II.md), machine-readable CSVs, metrics.json and paired_comparisons.json.

## Scientific limitations

Command-safety and recovery boundaries use first sampled losses. A pending command retains its completion opportunity through the final feasible sampled interval, with exhaustion at the next sample. At dispatch, the most recent sample supplies the completion label. Completion-based recovery requires a ready forecast and verified recovery. Unobserved boundaries remain censored. Inspection uses an operational horizon of approval plus 4.2 s for its three profiles. Its command-only forecast extends to 8.5 s from flight start, so horizon length also differs in that comparison. The full policy and RecoveryAware use the same predictor horizon and numerical profile. Geofence completion includes the original velocity and dwell conditions. Timing uses recorded offline computation replayed on the simulator clock.

Always Recover is now unconditional at approval, including a dispatch at D=0. Original historical outcomes remain immutable. Corrected legacy cases are physically equivalent reuses, not newly flown delay-specific trials. Their physical source references are recorded.

All 276 Inspection recovery profiles remain physically feasible throughout the sampled 0–1 s grid. Its 100% recovery classification agreement therefore does not estimate a recovery expiration error: no physical loss was observed, and the false-feasible conditional rate has no denominator. These fields remain inapplicable or censored rather than being reported as zero error.

## Completion condition: Geofence

Full StaleGuard and RecoveryAware differ in 3899 actual scenario–delay actions across 243 scenarios, and in safety/completion outcomes across 70 scenarios. Identical outcomes, if observed, are a negative ablation finding; no trigger is tuned to produce a difference.
With the same recorded forecast and runtime supplied to both existing decision rules, enabling the completion condition changes 3808 actions across 238 scenarios. This clock-only diagnostic separates the completion condition from timing differences; it does not supply additional physical outcomes. Details are in metrics.json.

The Geofence DualHorizon matrix contains 6,900 cases. Physical simulation covers 56 cases with revised actions, and the other 6,844 use their measured physical records. The reruns changed 0 safety labels and 0 safe-completion labels. The run manifest identifies each simulated seed and dispatch delay. Figure 4 rates use these same execution records.
