# Underpass nine-policy consolidation and fresh v2 preparation

The nine policies now use one benchmark runner. The UAV still flies North toward the bridge, dives to inspect and swerves East around the pier, or abandons inspection using the same sideways recovery. No physical maneuver, controller, safety rule or policy decision algorithm changed. The canonical historical folder contains all **11,700 original records**, byte-for-byte identical to the previously merged journal; every numerical metric and confidence interval reproduces exactly. Its validator no longer needs old result folders. We removed the duplicate folders, exports, separate runner and compatibility machinery, with about **630 fewer Python lines** overall. A **54-flight development smoke** and **51 focused tests** passed. The optimized predictor remains about **153 ms median / 185 ms p95**; its numerical profiles match before refactoring. We prepared **400 new scenarios**, excluding the historical 100, for **46,800 future flights**. No v2 flight or prediction has run. The next step is the user's launch of v2. Existing model errors and finite prediction-horizon limitations remain; this refactor establishes operational consistency, not policy superiority.

The historical experiment uses the same 13 dispatch delays for every policy, from 0 through 2 s. Dispatch is unknown to the policy until the event occurs. Successful inspection requires the existing dwell before the fixed deadline and no safety violation anywhere in the flight; recovery abandons inspection. The original command-only StaleGuard pair still shares a forecast/runtime. RecoveryAware and DualHorizon retain independently timed three-profile forecasts on the original 19-point grid through 1 s.

| Verification | Observed result |
|---|---:|
| Historical scientific records preserved | 11,700 / 11,700 |
| Historical journal bytes changed | 0 |
| Historical numerical metrics / confidence intervals changed | 0 |
| Historical policy decisions replayed and checked | 11,700 |
| Historical representative actual trajectories independently checked | 9 |
| Before/after prediction profiles, boundaries and plans | Identical on 12 development scenarios × 19 delays |
| Compiled/reference terminal state and PID maximum difference | 0 / 0 |
| Focused tests passed | 51 |
| Actual development smoke flights | 54: 2 scenarios × 3 delays × 9 policies |
| Smoke flights matching available independent physical replay hashes/outcomes | 41 / 41 available references |
| New v2 actual flights / predictions | 0 / 0 |

The remaining 13 smoke executions started outside the archived independent sweep's coverage; they were checked for physical continuity, safety monitoring, readiness and decisions, without fabricating matching references. The smoke test intentionally stopped after five flights, received an unfinished uncommitted journal tail, then resumed to 54 unique records. Prediction-journal interruption, configuration/source changes and duplicate records are also covered by focused tests. No historical evaluation flight was repeated.

| Predictor timing diagnostic | Median ms | p95 ms |
|---|---:|---:|
| Earlier optimized three-profile reference | 142.0 | 169.8 |
| After consolidation, unchanged optimized numerical code | 152.5 | 184.5 |

The new timing row uses 36 warmed measurements over all 12 development scenarios. This is comparable computation cost, with normal host/timing variation; it is not a measured speedup. Numerical predictor files are byte-for-byte unchanged. The command-only predictor is also unchanged. The large forecast cache is now an append-only journal; each physical flight updates a small progress checkpoint rather than rewriting megabytes of forecast metadata. This storage change does not advance prediction readiness or change intervention rules.

| Evaluation cohort | Scenarios | Delays | Policies | Flights | Status |
|---|---:|---:|---:|---:|---|
| Canonical historical | 100 | 13 | 9 | 11,700 | Complete, self-contained and validated |
| Fresh v2 | 400 | 13 | 9 | 46,800 planned | Prepared, not launched |

The 400 are sampled once without replacement from the remaining 765 accepted IDs using **RNG seed 20261109**. Selection reads only calibration definitions and historical configuration IDs, never policy outcomes. Independent regeneration of the draw matched exactly. The cohorts have **zero overlap** and cover **500 distinct scenarios**. Existing calibration and unrelated development datasets remain unchanged.

The original source/preparation evidence is archived inside the canonical historical folder. Both required redundant source folders and their ZIP copies are deleted. Self-contained metrics regeneration and validation passed with those folders absent. Source hashes, environment, configuration, approval/PID inputs, forecast runtimes and committed result integrity remain available; no new provenance framework was introduced.

Prediction is still imperfect, and no expiration found through 1 s does not guarantee later safety. Inspection residence in the prediction model remains evaluated on native 100 Hz integration segments; physical safety is still monitored at 1,000 Hz. These limitations and all unfavorable historical outcomes remain visible. The historical comparison remains exploratory; v2 is the separate fresh cohort. Commands, source inventory and integrity evidence are in the [technical appendix](TECHNICAL_APPENDIX.md).
