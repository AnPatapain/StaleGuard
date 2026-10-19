# Inspection-descent research recap

Read this table from top to bottom to follow why each step was needed.
The report filenames in `docs/` use the same stage numbers. Some reports cover
two stages; stages without a separate report link to the relevant code or data
below.

| Stage | Original problem / Objective | What we did | Result / Status |
|---|---|---|---|
| **1. Initial experiment** | Randomly generated commands frequently failed inspection even at D=0. | Introduced offline `plan_feasible_command()`. | Established a method for finding D=0-feasible commands. |
| **2. Feasibility debugging** | Some scenarios had no feasible command under the original search. | Corrected trajectory integration, allowed approval at t=0, and fixed inspection dwell measurement at 1,000 Hz. | Successfully calibrated all original 10 scenarios. See [stages 2–3 report](docs/02_03_feasibility_and_initial_recovery_results.md). |
| **3. Recovery-benefit experiment** | Determine whether StaleGuard improves safety and mission completion under delayed dispatch. | Compared StaleGuard against No Recovery using first-feasible calibration. | Demonstrated preliminary benefits, but only against No Recovery. See [initial recovery results](docs/02_03_feasibility_and_initial_recovery_results.md#executed-benefit-matrix). |
| **4. Delay-grid diagnostic** | None of the ten first-feasible commands safely completed the mission at tested delays ≥0.1 s. | Executed No Recovery across delays D=0.0–1.5 s in 0.1 s increments, with T=4.2 s. | Confirmed failure under those tested delays and motivated investigation of calibration. Inspection completion alone does not mean safe mission completion. |
| **5. Calibration-sensitivity investigation** | Determine whether alternative D=0-feasible commands tolerate positive delays. | Tested earlier approvals and longer descent durations, checking D=0 feasibility before delayed tests. | Confirmed that multiple alternatives safely complete the mission under delay. First-feasible selection contributes substantially to the failures. See [stage 5 report](docs/05_calibration_sensitivity.md). |
| **6. Max-dwell calibration comparison** | Find a calibration rule independent of positive-delay outcomes. | Selected maximum D=0 inspection dwell from the existing candidate grid, keeping the safety and inspection-completion requirements. | Positive-delay safe mission completions appeared on both the original and fresh scenarios. See [stage 6 report](docs/06_max_dwell_calibration_comparison.md). |
| **7. Lock calibration rule — Task 1** | Resolve the `low_seconds` concern. | Allowed longer descents, bounded by estimated nominal structure arrival at D=0; selected maximum safe D=0 inspection dwell. Preserved the existing search space, candidate order, and tie rule. | **Completed.** The initial 0.4–1.0 s range is not an operational maximum. |
| **8. Official integration — Task 2** | Replace first-feasible calibration in the official experiment. | Integrated maximum-dwell calibration, persistent caching, infeasibility reporting, and code cleanup. Later replaced the automatic cache with one fixed `calibrated_scenarios.json` file, by explicit decision. Calibration is now regenerated manually. | **Completed and validated.** See [integration report](docs/08_09_official_integration_and_results.md) and [current implementation](docs/current_implementation_and_validation.md). |
| **9. Official StaleGuard vs No Recovery** | Reevaluate benefits using the locked calibration. | Considered seeds 0–19. Seed 16 had no feasible calibration, so executed the policy comparison on the other 19 scenarios. | **Preliminary results available.** Mission and recovery percentages use those 19 scenarios. See [official results](docs/08_09_official_integration_and_results.md#actual-official-experiment-results). |
| **10. Baseline comparison — Task 3** | Establish whether StaleGuard outperforms simpler alternatives. | Task 3.1 adds Always-Recover using the existing recovery path, with zero decision time and the same scenarios and commands. Scheduled rollout, use-time recheck, fixed TTL, and FBS adaptation remain planned. | **Task 3.1 completed and validated. Other baselines remain next.** See the [Task 3.1 report](docs/10_01_always_recover.md). No other new baselines or mission-completion-time metrics have been added. |

After stage 9, the official experiment fixed the mission deadline at T=4.2 s
and kept delays D=0.0, 0.5, and 1.0 s. The tested 5.0 s deadline had produced
the same outcomes. The original two-deadline results remain in the historical
reports; calibration and its saved commands are unchanged.

The later locked delay grid is **0.0–1.0 s in 0.1 s steps**, for both TTL
prediction accuracy and StaleGuard benefit. Both now read the same grid from
`experiment.py`. The older 0.0–1.5 s diagnostic results remain historical data.

## Where to find the details

- [README](README.md): how to run the current experiment and regenerate calibration manually.
- [Current implementation and validation](docs/current_implementation_and_validation.md): what the current files do and what was checked after replacing the cache.
- [Original handoff](../CODEX_HANDOFF.md): early research context and constraints. Its original handoff section describes the starting point, not today's implementation.
- Stage 4: [diagnostic script](diagnostics/no_recovery_diagnostic.py) and [per-scenario CSV](results/no_recovery_delay_sweep.csv). The CSV is generated data under the Git-ignored `results/` directory.
- Stage 7: [shared calibration code](max_dwell_calibration.py). It evaluates every candidate in PyBullet at D=0 and keeps the first candidate in the original order when maximum dwell is tied. No delayed outcome is used to choose the command.

Historical reports retain the measurements and conclusions from their own stage.
Their references to the default calibration or caching describe that point in
the work. The README and current-implementation report describe today's workflow.
Experiment code and raw results stay in their existing locations.
