# Complete saved-prediction non-monotonicity audit

The audit reads all frozen evaluation forecasts: 276 Inspection Descent, 300 Geofence Turn, and 400 Underpass scenarios. Nine scenarios have feasibility that disappears and returns: eight Geofence completion profiles and one Underpass command/completion profile. For Geofence seed 65, executing the turn is predicted to complete the mission at 0.12 s, fail completion at 0.14 s, and succeed again at 0.16 s. Four Geofence scenarios retain a later predicted success after the current guard would cancel, with no earlier command-safety or recovery intervention. The other five returning-completion cases are blocked by safety or forecast readiness. These are recorded model predictions, not physical confirmations or improved-policy results. No code, policy, forecast, or physical result was changed. Review the four candidate cases before considering a separate algorithm experiment.

| Mission | Scenarios | Command safety affected | Safe completion affected | Recovery affected |
|---|---:|---:|---:|---:|
| Inspection Descent | 276 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| Geofence Turn | 300 | 0 (0.00%) | 8 (2.67%) | 0 (0.00%) |
| Underpass | 400 | 1 (0.25%) | 1 (0.25%) | 0 (0.00%) |

The Underpass command and completion rows refer to the same seed, 1611. No profile starts infeasible and later becomes feasible. Every affected sequence has three label changes: feasible → infeasible → feasible → infeasible. Each has one infeasible-to-feasible return; none has two or more returns. The CSVs retain all exact sampled values, including the final infeasible tail.

| Measured prediction example | Exact labels near reversal | Decision relevance |
|---|---|---|
| Geofence 65, completion | 0.12 true → 0.14 false → 0.16 true → 0.18 false | Forecast ready at 0.10 s; later 0.16 s point is still available after current cancellation at 0.14 s. |
| Geofence 47, completion | 0.08 true → 0.10 false → 0.12 true → 0.14 false | Forecast first available at 0.14 s; returned 0.12 s point has already passed. |
| Underpass 1611, command and completion | 0.12 true → 0.14 false → 0.16 true → 0.18 false | Unchanged command-safety cutoff is 0.14 s, so changing only completion handling cannot preserve the 0.16 s point. |

| Mission / seed | Current pending recovery (s) | Later feasible dispatch witness (s) | Command-safety first loss (s) |
|---|---:|---:|---:|
| Geofence 65 | 0.14 | 0.16 | Unobserved through 1 s |
| Geofence 191 | 0.16 | 0.18 | 0.70 |
| Geofence 318 | 0.18 | 0.20 | 0.70 |
| Geofence 326 | 0.16 | 0.18 | 0.80 |

These four are 1.33% of the Geofence cohort. Recovery remains predicted feasible at every sampled delay in all nine returning-completion scenarios. Underpass 1611 is blocked by command safety; Geofence 305 has already passed both its returned completion point and command cutoff when the forecast is ready. Geofence 47, 57 and 147 also receive their forecasts after their returned completion points. Thus eight Geofence cases are structurally unblocked at the first completion loss, but only four retain a usable later point once recorded computation time and 50 Hz scheduling are included.

A decision could differ only if the command is still pending when the current guard cancels, and later dispatch occurs at a retained feasible point. Earlier dispatch may make both policies execute before this issue arises. A redesigned rule at infeasible dispatch points is not specified or implemented here. The audit does not estimate changes in unsafe flights or successful inspections. Between-sample behavior and deadlines beyond the tested horizon remain unknown.

Data: [summary](summary.csv), [affected profiles](affected_profiles.csv), [all affected samples](affected_samples.csv), [completion cutoff/readiness audit](completion_decision_audit.csv). Integrity and full cohort IDs: [validation](validation.json). Method: [technical appendix](TECHNICAL_APPENDIX.md).
