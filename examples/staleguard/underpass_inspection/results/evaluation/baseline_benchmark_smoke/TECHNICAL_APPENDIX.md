# Underpass frozen benchmark protocol

Each recorded outcome is an actual PID/PyBullet flight. Safety is monitored at 1,000 Hz over the whole fixed horizon; earlier violations persist. All recovery is the same Eastward sideways maneuver.

StaleGuard uses the fixed 19-point grid and first-unsafe cutoff; equality rejects only when ready. Both variants share one warmed forecast/runtime per scenario. The early variant can recover while pending. Unavailable forecasts permit dispatch. Fixed EarlyRecovery uses 0.20 s; FBS uses alpha 0.1, epsilon 1e-12 and a 210-step cap, with strict age > TTL expiration. Use-Time Recheck observes at dispatch, continues nominal motion while computing and acts at dispatch + floor(runtime*50) + 1.

No future dispatch time is supplied to policies. Counterfactual command labels come from paired No Recovery flights when no external truth is configured. Command expiration is the first tested unsafe delay; all-safe complete grids remain censored. No independent recovery deadline is inferred. Full intervals expose non-monotonic command safety. Scenario-level rates use complete scenario grids only.

Each journal append is fsynced before its offset/hash is atomically committed in run.json. Resume checks that prefix and rolls back only uncommitted tail bytes. Metrics are regenerated one scenario at a time, without simulation. Source/environment, frozen protocol and cached forecast runtimes must match on resume.

Freeze SHA-256: 82d7fbd454084a42a55043150768bb8e55699ef9735ccbfbaf7b9c32d67230fd

Protocol and source provenance are recorded verbatim in run.json.
