# Focused validation

The five tests in `tables/tables_test_complete.py` passed. They check the shared decision rules (including equality, fail-open operation, and cancellation), frozen cohort inputs, compiled versus original PID/model state and controller-memory agreement at every candidate delay for one scenario per new mission (absolute tolerance 2e-12), strict/inclusive predicate boundaries, interrupted-journal repair, corruption detection, duplicate rejection, and physical-result reuse with independently timed policy events. They run without new benchmark flights.

The corrected native Geofence Always Recover test also passed: recovery starts at approval at D=0, 0.02 and 0.10 s, no command executes afterward, and no forecast is requested.

The final physical smoke validation passed 52 flights on four existing evaluation scenarios. It checks the new policy action replay and equivalent original command/recovery trajectories, including their physical hashes or minimum clearance. An earlier 52-flight smoke run is retained in the archived smoke marker. These are validation flights, not added benchmark cases.

Two existing Underpass storage tests passed (torn-write resumption and configuration/duplicate-prediction detection). A third old test, which attempts to prepare a new Underpass run from its original development freeze, stopped at its strict shared-source guard because the authorized Geofence simulation edit changes that source hash. No Underpass flight was run and no old manifest was changed. The new workflow’s independent duplicate-flight validation test passes using current configuration, without relaxing that historical guard.

See `smoke_validation.json`, the execution log, each stage’s `validation.json`, and final `VALIDATION.json` for machine-readable evidence. The final scientific validation is complete only after every required matrix and both tables finish.

The four-flight trajectory-reuse smoke was repeated after extending its metadata checks; the latest validation also checks Geofence final positions and velocities against fresh continuations. Intentional worker interruptions while adding physical-action reuse were resumed from committed records. The earlier interrupted error was retained in `errors_previous_*.json`; the final run has no unresolved errors.

The complete launcher was executed again after completion. All 14 execution/prediction journal hashes remained identical, confirming that no completed physical flight or forecast was repeated. `resume_validation.json` records this check. Final aggregation independently recomputes 35,880 new/corrected policy actions, verifies all 576 frozen new-mission snapshots, and checks 35,201 reused outcome records against their physical sources.
