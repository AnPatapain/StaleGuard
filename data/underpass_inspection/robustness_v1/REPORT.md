# E01 — StaleGuard robustness evaluation

**Status: COMPLETE**. 24960/24,960 policy outcomes; 320/320 production full forecasts. No incomplete subset is labeled as the full evaluation.

The immutable protocol is in specification.json and selection.json. All 64 scenarios remain selected; no recalibration, outcome filtering, retuning, wind or extra conditions are permitted.

Plant: 50 Hz PID / 1000 Hz PyBullet. Predictor: unchanged nominal 100 Hz model and original 19-point 0–1 s grid. H3 retains all 13 dispatch delays, including 1.2, 1.6 and 2 s.

A1/A2 change only actual base-link mass after the final reset; local inertia, PID mass compensation and predictor remain nominal. Actual mass/inertia are verified with getDynamicsInfo. True perturbed approval/PID snapshots are captured without resetting at approval.

B1/B2 add the same scenario-fixed world-frame position/velocity Gaussian bias at approval forecasts and Use-Time Recheck snapshots. Plant, PID observations/memory, geometry, references and physical maneuver anchors remain exact. Nominal H2 truth is reused only after snapshot and independent command/recovery replay equivalence checks.

Timing is measured offline. Approval forecasts and dispatch rechecks are timed serially; only physical continuations run in parallel, and all physical futures are joined before timed batches. Fresh wall-clock measurements still have ordinary hardware/OS variation. Resumption reuses recorded timings and decisions. JIT warmups are setup computations, reported separately from 320 production forecasts.

Workers retire between scenarios to bound native SCG/CasADi memory growth. Resource scheduling and analysis may change without invalidating physical-input cache keys; unchanged simulator/predictor/policy sources and numerical packages are checked. Original provenance is retained, with actual source hashes in each execution invocation.

All StaleGuard ablations share one full forecast per scenario/condition. The archived nominal command-aware benchmark used command-only forecast timing; N0 discrepancies are reported in nominal_archive_comparison.csv rather than hidden or tuned away.

Table E01-A: prediction accuracy, conditional false-feasible/reject rates, sampled boundary errors and readiness. Censored errors remain N/M. Table E01-B: absolute policy outcomes, signs separately for A1/A2, zero-delay failures and violation timing. Table E01-C: matched paired differences and shared-index, whole-scenario 2,000-resample bootstrap. Table E01-D: actual execution/caching statistics.

Partial tables explicitly include scenario IDs and tested delays. Comparative rows use a common completed rectangular scenario-delay subset across the available conditions for that comparator. This is completion-based accounting, never exclusion based on safety or success.

Only 64 scenarios are planned; statistical precision is limited. Report safety gains together with completion losses. Neither favorable point estimates nor a lower unsafe rate alone establish dominance. If an advantage reverses, retain and report it.

Scope: these results concern only ±5%/±10% mass mismatch and the two specified supervisory biases. They establish neither arbitrary-error/wind robustness, hardware safety, onboard timing nor continuous-time guarantees.

Nominal archive outcome differences observed so far: 5; action differences: 50.

Pilot runtime and approximate full-workload projection: `{"pilot_wall_s": 65.48353422699438, "pilot_scenarios": 2, "full_projected_wall_s": 2095.47309526382, "method": "Scenario-scaled pilot including full H2 grids; H3 pilot used 4/13 delays. Projection is approximate and may underestimate extra H3 continuations.", "workers": 2}`.

## Recorded outcomes

These are the actually completed records, not estimates for unexecuted trials. Partial pilot values verify the experimental pathway and cannot establish robustness of the full cohort.

| Condition | Policy | Trials | Unsafe | Safe completion | Recovery attempts | Unsafe recoveries | Status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| N0 | no_recovery | 832 | 337 | 495 | 0 | 0 | COMPLETE |
| N0 | use_time_recheck | 832 | 248 | 485 | 337 | 238 | COMPLETE |
| N0 | fixed_early_recovery | 832 | 15 | 305 | 512 | 0 | COMPLETE |
| N0 | staleguard_early_recovery | 832 | 124 | 493 | 297 | 82 | COMPLETE |
| N0 | staleguard_recovery_aware | 832 | 29 | 484 | 319 | 0 | COMPLETE |
| N0 | staleguard_dual_horizon | 832 | 29 | 484 | 319 | 0 | COMPLETE |
| A1 | no_recovery | 832 | 337 | 495 | 0 | 0 | COMPLETE |
| A1 | use_time_recheck | 832 | 252 | 484 | 336 | 240 | COMPLETE |
| A1 | fixed_early_recovery | 832 | 16 | 304 | 512 | 0 | COMPLETE |
| A1 | staleguard_early_recovery | 832 | 119 | 493 | 293 | 73 | COMPLETE |
| A1 | staleguard_recovery_aware | 832 | 30 | 483 | 319 | 0 | COMPLETE |
| A1 | staleguard_dual_horizon | 832 | 30 | 483 | 319 | 0 | COMPLETE |
| A2 | no_recovery | 832 | 337 | 495 | 0 | 0 | COMPLETE |
| A2 | use_time_recheck | 832 | 247 | 486 | 339 | 240 | COMPLETE |
| A2 | fixed_early_recovery | 832 | 15 | 305 | 512 | 0 | COMPLETE |
| A2 | staleguard_early_recovery | 832 | 122 | 491 | 293 | 74 | COMPLETE |
| A2 | staleguard_recovery_aware | 832 | 35 | 483 | 318 | 4 | COMPLETE |
| A2 | staleguard_dual_horizon | 832 | 35 | 483 | 318 | 4 | COMPLETE |
| B1 | no_recovery | 832 | 337 | 495 | 0 | 0 | COMPLETE |
| B1 | use_time_recheck | 832 | 252 | 485 | 331 | 236 | COMPLETE |
| B1 | fixed_early_recovery | 832 | 15 | 305 | 512 | 0 | COMPLETE |
| B1 | staleguard_early_recovery | 832 | 124 | 494 | 291 | 77 | COMPLETE |
| B1 | staleguard_recovery_aware | 832 | 34 | 485 | 313 | 0 | COMPLETE |
| B1 | staleguard_dual_horizon | 832 | 34 | 485 | 313 | 0 | COMPLETE |
| B2 | no_recovery | 832 | 337 | 495 | 0 | 0 | COMPLETE |
| B2 | use_time_recheck | 832 | 261 | 479 | 331 | 239 | COMPLETE |
| B2 | fixed_early_recovery | 832 | 15 | 305 | 512 | 0 | COMPLETE |
| B2 | staleguard_early_recovery | 832 | 126 | 495 | 290 | 79 | COMPLETE |
| B2 | staleguard_recovery_aware | 832 | 32 | 485 | 315 | 0 | COMPLETE |
| B2 | staleguard_dual_horizon | 832 | 32 | 485 | 315 | 0 | COMPLETE |

Executed verification: {"tests_run": 17, "failures": 0, "errors": 0, "passed": true, "skipped": 0, "log": "verification/tests.log"}. See verification/tests.log for the individual checks.

Completed-pilot resume verification: {
  "exit_code": 0,
  "elapsed_s": 7.63189579200116,
  "records_before": 966,
  "records_after": 966,
  "all_primary_bytes_unchanged": true,
  "extra_records": [],
  "changed_records": []
}

## Commands

```bash
python -m staleguard.robustness run --workers 2
# Native host unattended launcher (tmux preferred; POSIX session fallback):
python -m staleguard.robustness launch --workers 2
# Resume: the identical command reuses completed primary records.
python -m staleguard.robustness summarize
python -m staleguard.robustness status
```
