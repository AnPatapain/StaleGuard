# Geofence research-code refactor

The code is now organized around the research tasks: define the flight, calibrate approval time, simulate it, and run the existing policy experiments. For example, a UAV still patrols east, makes its original fixed-radius left turn when dispatched, and flies north toward the same checkpoint. Moving the Python functions has not changed that flight or introduced a new recovery policy.

We recorded the corrected implementation before editing it and compared it with the refactored version. All five approval times, 60 requested-path arrays and 55 prediction results are identical. The 21 actual flights on each version also match exactly in the recorded arrays, covering 130,821 physics samples per version. All policy decisions used the same control steps, and 63 focused tests pass. The old datasets and logs remain unchanged. Mission-run TTL prediction accuracy and the two missing baselines are still unfinished; this refactor does not claim to complete them. The next work can use the direct scripts without changing the established experiment.

## What became simpler

`scenario.py` now contains the whole mission definition: generation, the eastbound patrol, the approved turn, body clearance and checkpoint completion. `calibration.py` contains the approval search and can export the existing five cases as `calibrated_scenarios.json`. `simulation.py` keeps the single SCG/PyBullet execution loop, so all policies still encounter the same aircraft, command and safety observations.

The existing policy functions are grouped in `baselines/`. No Recovery remains the default simulation path because it has no guard decision of its own. The paired experiment and its counts are under `baseline_benchmark/`; the existing saved-state readiness diagnostic is now `support/readiness.py` and clearly announces that it is not an accuracy experiment.

The large `cli.py` router and the `infrastructure.py`, `policies.py`, `prediction.py` and `reporting.py` wrappers were removed. Their useful functions were moved to the files above, rather than copied. The old budget class was replaced with a plain trial counter. The later support-folder cleanup removed the remaining `mission.py` alias file. The snapshot reader in `calibration.py` now maps its three old class names to the same definitions in `scenario.py`, without rewriting snapshots or weakening their checks.

## Evidence that the experiment stayed the same

| Check | Observed result |
|---|---|
| Scenario generation | Same ten seed records, including the same rejected geometry. |
| Approval selection | Same five times: 2.0, 1.8, 1.8, 1.4 and 1.4 s. The selection function was checked against all 33 existing candidate outcomes. |
| Requested flight paths | All 60 arrays exactly identical. |
| StaleGuard predictions | All 55 tested delay results and their expiry decisions exactly identical. |
| Actual PyBullet flights | All 21 before/after pairs match; maximum recorded array difference is **0**, against a tolerance of 1e-10. |
| Policy scheduling | All 16 representative policy cases have identical decisions and applied control steps. |
| Preservation and tests | All 326 historical data files have unchanged hashes; 33 unit/equivalence, four integration and 26 Inspection-Descent tests pass. |

The representative policy comparison uses cases 0 and 3, delays of 0 and 0.6 s, and the same four existing policies. Each row below contains four flights. These counts demonstrate preservation of successes **and failures**; they are not a new policy-performance conclusion.

| Policy | Before: safe completion / safe non-completion / unsafe | After: same categories |
|---|---|---|
| no_recovery | 2 / 0 / 2 | 2 / 0 / 2 |
| staleguard | 2 / 1 / 1 | 2 / 1 / 1 |
| use_time_recheck | 2 / 1 / 1 | 2 / 1 / 1 |
| fixed_ttl | 2 / 1 / 1 | 2 / 1 / 1 |

Actual computation times were allowed to vary. Use-Time Recheck took 3.12–4.91 ms before and 3.07–4.79 ms after; every decision still applied one 20 ms control step later. No timing value was forced to make the trajectories agree.

## What remains unfinished

Always-Recover and FBS were not added. The Geofence mission-run TTL-accuracy experiment was not implemented before this refactor and remains absent; the timing diagnostic is grouped with the other checks under `support/`, and there is no placeholder accuracy directory. No accuracy scores or full benchmark were invented.

The five calibrated cases were reused, not recalibrated. Their original snapshots are still checksum-checked, and the new export adds an index pointing to them. New scripts reject attempts to resume an interrupted run under different source hashes, including the old file layout. They can read the frozen calibrated inputs after validating their schema, configuration, command, approval time and reference.

The [README](../../README.md) gives the direct commands. [Machine-readable comparisons](../results/refactor_equivalence/comparison.json), [archived pre-refactor source and outputs](../results/refactor_equivalence/reference/), [post-refactor outputs](../results/refactor_equivalence/refactored/), and the [regression log](../results/refactor_equivalence/unit_tests.log) provide the technical evidence. The 42 before/after flights, four direct-entry-point checks and three integration flights total **49 mission flights**, all using the existing five cases. There was no 300-scenario calibration or full benchmark.
