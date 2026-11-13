# Root relocation validation

The complete implementation moved from `examples/staleguard/` to `staleguard/`.
Package imports and source paths changed; flight dynamics, PID control, scenario
generation, prediction, policies and experiment parameters did not. For example,
the same UAV command and recovery replay produced exactly the same physical
trajectory hashes before and after the move in all six checked flights.
All archived scientific artifacts remain byte-identical. The next step is review
of this structural change; Stage 2 cleanup has not begun.

## Git state and preservation

- Branch: `refactor/staleguard-root-relocation`.
- Initial and current HEAD: `fb51b21f0fb31b2abeb9412d63fd0da699a12de1`.
- No commit, push, merge or history rewrite was performed.
- Initial `tree.txt` was untracked. It remains untracked at `staleguard/tree.txt`
  with identical bytes; it is not included in a relocation commit.
- All 4,527 original non-cache files are present. All 4,353 protected scientific
  and historical files match their pre-move SHA-256 hashes.
- `source_relocation.json` was not edited. The separate
  `source_root_relocation.json` pins exact before/after hashes for structural
  source changes. New source bytes are not represented as historical bytes.

## Observed validation results

| Check | Result |
|---|---|
| Python syntax and imports | All 93 active modules parsed and imported |
| CLI entry points | 20 module commands passed `--help` |
| Focused existing tests | 60 passed; no physical flights in this subset |
| Additional provenance tests | 2 passed, including 39 archived scientific function/class hashes and rejection of unknown/tampered source hashes |
| Frozen cohorts and snapshots | 276 Inspection, 300 Geofence and the 400 Underpass benchmark scenarios loaded; the full Underpass calibration contains 865 accepted cases |
| Predictions | Exact agreement for all three profiles on one frozen scenario per mission: 11/19/19 delays |
| Numerical continuation state | Saved terminal state and PID memory match exactly |
| Pending-policy clocks | Command-aware, fixed early, recovery-aware and full-policy decisions match at 0, 0.2 and 1 s, using the same 0.1 s readiness input |
| Physical replay | Six post-move flights (command and Dual horizon per mission) match the already captured six reference flights, including physical trajectory hashes and outcomes |
| Eight paper policies | All remain constructible; no policy removed |
| GUI compatibility | GUI CLI help and scene construction in PyBullet DIRECT pass; interactive rendering was not re-tested |
| Packaging | Setuptools discovers both root packages and mission namespaces; Poetry package declarations parse; no wheel installation was performed |
| Source compatibility | Pinned old hashes accepted, unknown hashes rejected; new and historical pickle module names readable |
| Historical artifact integrity | 4,353 protected files byte-identical; no result regeneration |

The first raw pre/post comparison reported 60 differences, all in incidental
`*_unix_s` timestamps. A second comparison of those same saved outputs, excluding
only these timestamps, passed exactly. Flights were not repeated for this check.
No benchmark or complete post-move test suite was run. Full calibration searches,
all-scenario prediction recomputation and onboard timing are not claimed verified.

## Pre-existing failures (not relocation regressions)

The saved **pre-move** logs show seven Underpass errors, rather than seven
Geofence test errors. Every one stops at the same development-provenance check:
the archived Underpass inputs expect Geofence simulator hash
`c80921f470e62bca9f24cc1177b9b78fd332c5043ff0f5d401dd7b4f32449758`,
but the actual simulator before relocation was
`c86f8e63a1c27bc75dd915bbfce5dca6bb7c901d5d7c5c736de2869d891b96e2`.
This mismatch was present with a clean tracked working tree before `git mv`.

| Underpass test | Exact stopping cause |
|---|---|
| `PhysicalWaitingTests.test_actual_waiting_violation_survives_dispatch_and_recheck` | Development calibration rejects the recorded Geofence simulator hash |
| `PredictionTests.test_a_prefix_violation_cannot_be_erased` | Same calibration provenance rejection |
| `PredictionTests.test_waiting_is_predicted_then_relative_command_uses_predicted_anchor` | Same calibration provenance rejection |
| `BenchmarkTests.test_archived_development_metrics_remain_identical` | Archived ground-truth provenance rejects that hash |
| `BenchmarkTests.test_duplicate_flight_rejected_by_validator` | Benchmark preparation reaches the same calibration rejection |
| `BenchmarkTests.test_each_policy_can_be_prepared_alone` | Benchmark preparation reaches the same calibration rejection |
| `HorizonPredictionTests.test_reference_compiled_and_early_exit_equivalence` | Same calibration provenance rejection |

The Geofence pre-move suite ran 66 tests with **one error and one failure**:
`TTLAccuracyTests.setUpClass` rejects an archived No-Recovery simulator signature;
`RefactorEquivalenceTests.test_moved_scientific_functions_and_physics_loop_are_unchanged`
compares to an older loop whose dispatch condition lacks the current
`and forced_action is None` guard. The relocation does not change that guard.
The pre-move Inspection suite had five errors rejecting old
`inspection_descent/ground_truth.py` source provenance. Those older compatibility
issues were not repaired or bypassed, and their historical hashes were not changed.

Existing table coordination and aggregation retain their established behavior.
Archived runs cannot be resumed under changed source provenance. Individual
experiment entry points retain `--out` for fresh runs. The consolidated aggregator
also writes source-side derivative metrics even with a fresh table destination;
it was not run, so archived tables and figure data were preserved.

## Technical appendix: commands and evidence

Commands ran from the repository root after relocation, using the existing `scg`
Python. During sandboxed checks, `NUMBA_CACHE_DIR` and `MPLCONFIGDIR` pointed into
`/tmp`; this avoids writing caches outside the sandbox and is not required for
normal execution. Normal `python -m staleguard.underpass_inspection.pilot --help`
was separately checked without a cache override.

```bash
git status --short
git diff --stat
git diff --check
python -m staleguard.underpass_inspection.pilot --help
python -m unittest \
  staleguard.geofence_turn.support.tests.test_mission \
  staleguard.geofence_turn.support.tests.test_prediction \
  staleguard.inspection_descent.support.test_temporal_safety \
  staleguard.inspection_descent.support.test_fbs \
  staleguard.inspection_descent.support.test_fixed_ttl \
  staleguard.inspection_descent.support.test_use_time_recheck \
  staleguard.underpass_inspection.support.test_pilot \
  staleguard.underpass_inspection.support.test_supplementary \
  staleguard.underpass_inspection.support.test_recovery_horizons.HorizonPolicyTests
```

The retained temporary evidence is:

- `/tmp/staleguard_relocation_smoke.py`: original pre/post comparison driver.
- `/tmp/staleguard-relocation-before.json`, `/tmp/staleguard-relocation-after.json`: captured numerical outputs.
- `/tmp/staleguard-relocation-equivalence.json`: corrected timestamp-independent comparison.
- `/tmp/staleguard-relocation-files-before.json`: original file-hash inventory.
- `/tmp/staleguard-relocation-preservation.json`: preservation check.
- `/tmp/staleguard-relocation-before-geofence-tests.log`.
- `/tmp/staleguard-relocation-before-underpass-tests.log`.
- `/tmp/staleguard-relocation-before-smoke.log`, `/tmp/staleguard-relocation-after-smoke.log`.
- `/tmp/staleguard-relocation-focused-tests.log`: 60 passing tests.
- `/tmp/staleguard-relocation-cli.log`: all 20 entry-point checks.
- `/tmp/staleguard-relocation-path-checks.log`: scene, policy, provenance and pickle checks.
- `/tmp/staleguard-relocation-provenance-tests.log`: two additional passing source-integrity tests.
- `/tmp/staleguard-relocation-preexisting-rechecks.log`: the same historical source rejections confirmed after moving, without flights.

These logs are temporary local evidence, not newly generated mission results.
The complete modified-file inventory is in [RELOCATION_FILES.md](RELOCATION_FILES.md).
