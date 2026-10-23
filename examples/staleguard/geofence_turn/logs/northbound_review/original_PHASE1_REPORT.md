# Geofence emergency turn — Phase 1 report

Priority 0 is implemented and validated. The bounded development run accepted **5/9 attempted seeds**, reproduced all five at D=0, and completed **80 real paired policy flights**. There was no 300-scenario calibration, final evaluation, policy tuning, network access, package installation, or git commit. This small, feasibility-filtered development sample does not support a general policy ranking.

## Preservation and changed files

Initial `git status --short` (also [saved verbatim](logs/initial_git_status.txt)):

```text
AM Codex_Geofence_Phase1.md
```

That pre-existing task-document modification was preserved. All work is new under `geofence_turn/`. `git diff --stat -- inspection_descent` is empty; Mission A retains no dependency on Mission B. No existing calibration, result, or scientific-source file was edited. The ordinary tracked diff still shows only the user's task-document change; new mission files are untracked, not staged.

Exact new Python files:

- `__init__.py`
- `cli.py`
- `diagnostics.py`
- `infrastructure.py`
- `mission.py`
- `policies.py`
- `prediction.py`
- `reporting.py`
- `scenario.py`
- `simulation.py`
- `tests/__init__.py`
- `tests/test_integration.py`
- `tests/test_mission.py`
- `tests/test_prediction.py`
- `validation.py`

Diff summary: **15 new Python files, 1,342 Python lines**, plus three source/documentation files (`.gitignore`, `README.md`, this report); zero changed Mission A source files. Logs and plot artifacts are additional outputs.

Additional new deliverables: `.gitignore`, `README.md`, this report, and `logs/` artifacts. [The artifact inventory](logs/artifact_inventory.txt) enumerates output paths; [the source patch](logs/new_mission_source.patch) makes the untracked implementation reviewable. [Shared-source provenance](logs/provenance.json) records hashes of the unchanged reused files.

## Frozen execution contract and completion

Coordinates are SCG's `[x,vx,y,vy,z,vz,roll,pitch,yaw,p,q,r]`, with +X east/forward, +Y north/left, constant Z, and zero yaw reference. The nominal velocity ramps from 0 to the selected speed in the existing **0.4 s**. Position analytically integrates that ramp and then continues +X. Approval candidates start at **0.6 s**, after the ramp.

The immutable command contains one radius, speed, left-turn direction, world-frame waypoint, north-leg duration, and terminal duration. It is sampled **before** any candidate search and never changed for approval time, prediction, delay, or policy.

At dispatch, the anchor is the existing nominal **reference row** at that control step, mirroring the reference-editing contract of `apply_descent_command`. It is not an optimized state/route. The arc follows `theta=v*t/R`, `x=x0+R*sin(theta)`, `y=y0+R*(1-cos(theta))` until pi/2. It then flies north for 0.5 s. A fixed-duration, 3 s quintic Hermite connector joins that endpoint and northward velocity to the original waypoint with zero terminal velocity/acceleration, then holds there. All phase positions and velocities are continuous; acceleration at arc entry/exit need not be continuous. The measured PID tests screen this finite-curvature reference.

**Start-relative arc / fixed-waypoint conflict:** changing dispatch moves the arc endpoint in X. The deterministic quintic absorbs that X displacement, sometimes flying west while continuing north, and ends at the same original waypoint. Its duration is fixed and its coefficients are direct endpoint interpolation; it does not inspect the fence, search routes, select a new goal, or tune speed/radius. Unit tests explicitly exercise different candidate/dispatch anchors and the resulting non-collinear endpoint. Every policy uses this exact command/reference function.

Timing uses the existing nearest-control-step convention: the sampled deadline draw is serialized as `deadline_s`, and the effective mission deadline is `T=round(deadline_s*50)/50`. The saved snapshot also freezes that exact horizon in integer steps. No policy gets a different quantization. The table below reports this effective T.

Completion is actual 3D Euclidean position error <= **0.10 m**, actual speed norm <= **0.12 m/s**, continuously for **0.20 s**, by the same per-scenario deadline **T from approval**. These are named, configurable, **provisional physical requirements**, fixed before all runs. Safe Mission Completion requires completion AND no violation through the full horizon. Monitoring starts at the initial state and ends at approval+T, continuing after completion and after violations. It does not infer completion from a reference or from fallback activation.

## Physical geofence definition

SCG's [`cf2x.urdf`](../../../safe_control_gym/envs/gym_pybullet_drones/assets/cf2x.urdf) represents its collision body as a cylinder of radius 0.06 m and full length 0.025 m. The orientation-independent enclosing sphere has radius `sqrt(0.06^2 + 0.0125^2)` = **0.061288253 m**. This is a conservative approximation of the simulator's collision body, not an experimentally validated rotor/sensor envelope for hardware. Tests read the URDF and check tilted-cylinder support against this sphere.

Signed **body** clearance is `fence_x - actual_x - body_radius`. The requirement is clearance >= `d_min_m=0.05 m`, a **provisional authorized stand-off**. There is no additional tracking-error padding or other safety margin, and the body radius/d_min are each applied once. Failure of d_min is a violation even if the body has not yet intersected the plane.

The reused physics-step monitor reads actual PyBullet positions at **1000 Hz**, including tick zero and the final endpoint once each. Actual PyBullet velocity supplies dwell checks. Predispatch violations remain violations regardless of subsequent turns/fallback. Sample-count assertions cover every flight. The separate predictor checks the sphere's half-space clearance at segment endpoints over its **50 Hz** output intervals, using unchanged **100 Hz** internal Numba integration. For a straight segment and a fixed sphere/plane, minimum clearance is attained at an endpoint; this does not eliminate numerical-model error or inter-sample curvature approximation.

## Parameter provenance and empirical turning screen

All mission-specific constants are explicit in `mission.Config`, generator code, and policy constants; there are no unreported fitted gains.

| Quantity | Value / construction | Basis |
|---|---|---|
| PID / physics / model | 50 / 1000 / 100 Hz | Existing infrastructure settings; unchanged SCG PID/dynamics |
| Nominal acceleration ramp | 0.4 s | Existing Inspection-Descent convention |
| Body radius | URDF cylinder enclosing sphere | Geometry-derived, above |
| d_min / completion tolerances / dwell | 0.05 m / 0.10 m / 0.12 m/s / 0.20 s | Provisional physical requirements, frozen before characterization |
| Characterization speeds | 0.4, 0.6, 0.8 m/s | Existing operating-envelope starting points; feasibility measured here |
| Characterization radii | 0.2, 0.4, 0.6 m | Provisional engineering sweep, not a claimed prior envelope |
| Characterization height / approval | 1.25 m / 1.2 s | Provisional representative screen point, after startup |
| Screen acceptance | completion + safety + max 3D tracking error <= 0.15 m | Provisional empirical tracking bound; not added to fence margin |
| Characterization deadline | maneuver duration + 1.5 s | Provisional settling allowance |
| Generated speed / radius | Speed uniform over measured passing speed values, then one passing measured radius conditional on speed | Empirical grid only; no interpolation/extrapolation claim |
| Height | uniform 1.15–1.40 m | Existing suggested envelope; every selected scenario separately tested |
| Initial X / Y | uniform [-0.15,0.15] / [-0.10,0.10] m; initial velocities zero | Provisional variation of nominal conditions |
| Fence X | initial_x + uniform[1.5,2.5] m | Provisional patrol-space geometry, independent of waypoint anchor draw |
| Fixed waypoint | X=initial_x+v*(a-0.2)+R; Y=initial_y+R+v*(0.5+3/2); Z=height; a uniform[1.2,2.4] s | Provisional preselected patrol anchor; no calibration/delay dependence |
| North leg / connector | 0.5 / 3.0 s | Provisional deterministic endpoint-following rule; C1 boundaries tested |
| Deadline T | round((maneuver duration + uniform[1,2])*50)/50 s | Provisional settling/dwell draw and existing nearest-control-step convention; fixed before calibration |
| Approval grid | >=0.6 s, spacing 0.2 s | Provisional finite calibration resolution; not a continuous optimum claim |
| Approval upper bound | ceil(actual first nominal violation / 0.2)*0.2 | Real nominal-prefix measurement; any later approval is already unsafe |
| `approval_max_s=3.2` | Initial pilot / optional helper default only | Superseded by measured upper bound in production CLI; never caps final calibration |
| Nominal probe horizon | geometric nominal fence arrival + frozen maneuver duration | Finite probe construction; actual crossing must be observed or calibration stops |
| Prediction / smoke grids | 0:0.1:1 s / 0,0.2,0.6,1 s | Preregistered before smoke; unknown future D not supplied at approval |
| Fixed TTL | 0.4 s; equality permits dispatch | Provisional smoke control, existing strict-expiry convention; not tuned |
| Resource bounds | 1 worker, 1 numerical thread; RSS <=2048 MiB, available >=2048 MiB | Existing serial memory guardrails; allocator collection/trim reused in spirit |
| CLI hard caps | <=20 attempts, <=200 flights per command, <=1800 s; defaults 10 attempts, 5 acceptances, 160 flights, 900 s | Explicit development resource limits, not scientific thresholds |

These are joint constrained samples: radius depends on the measured speed-specific screen, waypoint depends on the sampled nominal motion, and T depends on maneuver duration. Fence and waypoint draws can be incompatible; those attempts are logged and rejected rather than repaired. The screen is measured only at 1.25 m and one startup time; full per-scenario PyBullet calibration is still required.

Real SCG/PID characterization (all nine reached/stabilized at their target; one fails the independent tracking screen):

| Speed m/s | Radius m | v²/R m/s² (screening relation only) | Max actual tracking error m | Screen |
|---|---|---|---|---|
| 0.4 | 0.2 | 0.80 | 0.0599 | pass |
| 0.4 | 0.4 | 0.40 | 0.0350 | pass |
| 0.4 | 0.6 | 0.27 | 0.0304 | pass |
| 0.6 | 0.2 | 1.80 | 0.1079 | pass |
| 0.6 | 0.4 | 0.90 | 0.0717 | pass |
| 0.6 | 0.6 | 0.60 | 0.0440 | pass |
| 0.8 | 0.2 | 3.20 | 0.1578 | reject |
| 0.8 | 0.4 | 1.60 | 0.1067 | pass |
| 0.8 | 0.6 | 1.07 | 0.0723 | pass |

[Raw sweep](logs/characterization/sweep.jsonl), [envelope/config](logs/characterization/envelope.json), [plot](logs/diagnostics/turning_envelope.png). No policy result participated in the envelope choice.

## D=0 calibration and reproducibility

Seeds **0–9 were generated once**; attempts stopped after seed 8 because five acceptances had been obtained. Thus **9 attempted, 5 accepted (55.6%)**, with seed 9 generated but unattempted. Selected seeds: **1,2,4,6,8**. Rejected seeds: **0,3,5,7**. All four rejected fixed waypoints have negative nominal body clearance at the designated endpoint (-0.0243, -0.0830, -0.2754, -0.4997 m respectively); full SCG candidate simulations confirmed that no approval on the searched grid met safety. None was repaired by moving the target or resampling radius.

The final search evaluated **67 D=0 candidate flights**, rejecting 62 candidates for body clearance, plus **9 actual nominal bound probes** and **5 selected D=0 replays**: 81 flights in the final calibration command. All candidates reached the completion predicate; clearance was the limiting criterion. For each seed, approval steps are searched in descending order from the measured nominal bound. Every candidate later than the selected one on that grid was evaluated and failed. Later than the nominal bound is impossible because the unchanged prefix has already violated clearance. The objective is latest feasible approval on this grid, not maximum dwell or policy performance.

An initial pilot used the 3.2 s cap (83 candidates + 5 replays = 88 flights). Inspection exposed that cap as insufficient: seed 6 later passed at 3.4 s. The pilot remains under `logs/development/`, clearly **superseded** by `logs/development_measured_bound/`. Both runs kept the same radius, waypoint, requirements, and seeds. Final policy tests use only the measured-bound run.

Final selected examples; completion time and T are relative to approval:

| Seed | Speed m/s | R m | Approval s | Minimum actual body clearance m | Completion s | T s |
|---|---|---|---|---|---|---|
| 1 | 0.6 | 0.4 | 2.2 | 0.096996 | 4.121 | 6.380 |
| 2 | 0.8 | 0.4 | 2.0 | 0.098567 | 3.740 | 5.480 |
| 4 | 0.8 | 0.6 | 1.8 | 0.092617 | 4.087 | 6.480 |
| 6 | 0.6 | 0.4 | 3.4 | 0.051299 | 4.361 | 6.220 |
| 8 | 0.8 | 0.4 | 2.4 | 0.059321 | 4.016 | 5.720 |

Every selected scenario then passed the separate five-case [D=0 replay command](logs/d0_replay_console.log). Captured approval state, PID integrals/last attitude, remaining reference, and command matched the saved snapshot exactly. Ground truth always flies from the original initial state; it does not teleport to the snapshot. Each seed directory stores JSON parameters/configuration, a full state/PID/reference `approval.pkl`, SHA256, candidate journal, nominal probe, D=0 result, and compressed control trajectory. Pickle follows the existing checkpoint convention; load only locally produced snapshots.

## Adapted policies and smoke observations

Completed **5 scenarios × 4 delays × 4 policies = 80 paired full-physics flights**, with identical seeds, command, goal, initial conditions, horizon and completion requirements. The [journal](logs/policy_smoke/executions.jsonl) and [paired summary](logs/policy_smoke/paired_summary.json) contain all results. No rows were synthesized from counterfactual outcomes.

- **No Recovery:** dispatch the frozen command on schedule.
- **StaleGuard:** approval-only shared lightweight model, predicted waiting prefixes/checkpoints, then identical turn semantics for each hypothetical delay. The nominal patrol eventually crossing the fence must *not* invalidate every turn as in the legacy mission's `INVALID_NOMINAL` shortcut; only each actual hypothetical waiting prefix plus dispatched path is checked. First unsafe grid point defines expiry, conservatively retaining any later nonmonotonic tail as expired. Actual future D is absent from the approval prediction API. At actual dispatch, use the prediction only if measured computation finished within elapsed queue time; otherwise execute on schedule, with no later cancellation.
- **Use-Time Recheck:** capture real dispatch-time state and PID history; compute one immediate-turn forecast with the same model. Continue nominal execution during measured compute time; apply the stored decision at `dispatch + floor(compute_s*50)+1`, strictly after completion. No new check or route is generated at that later tick.
- **Fixed TTL:** 0.4 s lifetime; dispatch at equality; otherwise the same safety-only fallback.
- **FBS:** not adapted or evaluated. Existing feature geometry and lifetime cap belong to Inspection-Descent and must be specified for this mission before use.
- **Periodic:** not adapted or evaluated; no reusable periodic runner was found in the inspected current mission code.

The common **safety-only fallback** cancels the pending turn and asks the existing position PID to brake/hover at the actual position at the fallback tick. It performs no waypoint retry or new route selection. This is the minimal mission-specific reference adaptation: continuing +X after cancellation would drive directly into the fence. It adds no dynamics/controller and offers no stopping guarantee. Its actual stopping clearance is measured. A hover that misses the designated waypoint is safe non-completion, never mission success.

| Policy | Flights | Safe completion | Safe non-completion | Unsafe | Fallback |
|---|---|---|---|---|---|
| no_recovery | 20 | 5 | 0 | 15 | 0 |
| staleguard | 20 | 5 | 7 | 8 | 15 |
| use_time_recheck | 20 | 3 | 7 | 10 | 15 |
| fixed_ttl | 20 | 5 | 2 | 13 | 10 |

Per delay: at D=0 No Recovery/StaleGuard/Fixed TTL each complete safely 5/5; Use-Time Recheck safely completes 3/5. At D=0.2 StaleGuard and Recheck yield 5/5 safe non-completions, while No Recovery and Fixed TTL violate 5/5. At D=0.6 all guards yield two safe non-completions and three unsafe executions; No Recovery violates 5/5. At D=1 all methods violate 5/5, and all five already violate **before dispatch**. The stored initial nominal prefixes match exactly across all policy traces.

Warmed StaleGuard approval computation took **118.7–196.3 ms**; all five forecasts first become unsafe at the 0.1 s grid point. All were unavailable at D=0 and available at D>=0.2 in this run. Use-Time Recheck compute took **8.22–16.94 ms**, imposing one full 20 ms scheduling tick in every recorded case. For seed 6 its immediate forecast clearance was 0.051076 m (actual immediate: 0.051299 m), but actual application after the tick reached only **0.037450 m**. Seed 8 similarly fell from 0.059321 m immediate to **0.039309 m**. Both still reached the waypoint but were unsafe. Latency was retained as observed; it was not rounded down or removed to improve scores. These observations demonstrate protocol behavior, not a calibrated comparative performance claim.

The smoke command stopped three times at the unchanged available-memory guard: original stop at process/available **747/1930 MiB**, then **736/2012 MiB**, then **667/2014 MiB**. Completed flights were retained (65, then 66, then 67). The resume implementation verifies configuration and physics/policy source hashes, retains original per-seed approval forecasts and timing, skips journaled flights, and appends missing flights only. Existing-style `gc.collect()`/`malloc_trim(0)` before guard checks reduced retained allocator memory; no limit was weakened. After host memory recovered, the last 13 flights completed. [Failure/resume logs](logs/policy_smoke/), plus each session console log, retain this history. `finished.json` reports trials in the final session, not total flights; total=80 is verified from the unique journal keys.

## Validation and diagnostics

- **17 unit tests PASS:** seeds, immutable command, fixed waypoint across candidates/delays, leftward course, C1 phase boundaries, timing/no hidden replanning, measured-bound search, explicit infeasibility, cylinder/body clearance, completion requirements, safety-only hold, model waiting-prefix semantics, snapshot non-mutation, scheduler readiness and strict latency/TTL boundary conventions. [Log](logs/unit_tests.log).
- **4 integration tests PASS:** three fresh SCG/PyBullet flights (safe D=0 checkpoint replay; D=0.6 with original goal; deliberate predispatch violation) plus inspection of real characterization traces at 0.4/0.6/0.8 m/s and R=0.4 m. Actual +Y travel/velocity changes while yaw remains near zero. [Log](logs/integration_tests.log), [fresh flight artifacts](logs/integration/).
- **26 existing Inspection-Descent regression tests PASS**, without editing that mission. These are the focused `diagnostics/test_*.py` tests, not the repository's entire test suite. [Log](logs/inspection_descent_regression.log).
- **Artifact audit PASS:** all 67 final candidate commands retain the generated waypoint/radius; latest-on-grid selection, snapshot checksums, 80 classifications/sample counts, exact nominal state prefixes, measured scheduling/readiness, original terminal goal on dispatched traces, and 20 complete paired blocks. [Result](logs/artifact_validation.json).
- **9 characterization + 5 independent D=0 replays** executed as described above; all evidence is actual physics or explicitly identified model forecasts/unit fixtures.

![Actual turn and body clearance](logs/diagnostics/actual_turn_and_clearance.png)

This diagnostic plots saved **50 Hz** actual state samples; reported extrema and pass/fail use the **1000 Hz** monitor. The sweep plot and this figure were rendered and visually inspected. [Raw development files](logs/development_measured_bound/) retain individual attempts and outcomes.

## Reproduction commands and Phase 2 handoff

Run from `examples/staleguard`, serially. Do not overlap another simulation command. New output directories must not exist; replace `repro_*` names on a second run. The command-level locks prevent overlapping geofence runs. No dependency installation is needed.

```sh
SCG_PYTHON=/home/kean/miniconda3/envs/scg/bin/python
$SCG_PYTHON -m geofence_turn.cli characterize --out geofence_turn/logs/repro_characterization --max-trials 9
$SCG_PYTHON -m geofence_turn.cli generate --envelope geofence_turn/logs/repro_characterization/envelope.json --out geofence_turn/logs/repro_generated --attempts 10
$SCG_PYTHON -m geofence_turn.cli calibrate --generated geofence_turn/logs/repro_generated/generated.json --out geofence_turn/logs/repro_development --attempts 10 --target 5 --max-trials 200
$SCG_PYTHON -m geofence_turn.cli replay --development geofence_turn/logs/repro_development --out geofence_turn/logs/repro_d0 --max-trials 5
$SCG_PYTHON -m geofence_turn.cli smoke --development geofence_turn/logs/repro_development --out geofence_turn/logs/repro_smoke --max-trials 80
# Only if interrupted; retains existing forecasts and completed flights:
$SCG_PYTHON -m geofence_turn.cli smoke --resume --development geofence_turn/logs/repro_development --out geofence_turn/logs/repro_smoke --max-trials 80
$SCG_PYTHON -m geofence_turn.reporting geofence_turn/logs/repro_smoke
$SCG_PYTHON -m unittest geofence_turn.tests.test_mission geofence_turn.tests.test_prediction -v
$SCG_PYTHON -m unittest geofence_turn.tests.test_integration -v
$SCG_PYTHON -m unittest discover -s inspection_descent/diagnostics -p 'test_*.py' -v
$SCG_PYTHON -m geofence_turn.validation
$SCG_PYTHON -m geofence_turn.diagnostics
```

The integration tests and artifact audit default to the Phase 1 saved directories. For fresh integration runs set `GEOFENCE_DEVELOPMENT` to the new development directory; the characterization inspection still uses the saved Phase 1 sweep. `validation`/`diagnostics` target the Phase 1 directory layout via `--logs`.

Phase 2 should retain this development sample separately, decide physical tolerance/body-envelope validation, characterize additional initial/altitude/connector conditions before broadening the empirical grid, and define geofence-specific FBS/Periodic adaptations before comparing them. Preserve the observed late-fallback and Use-Time Recheck latency failures; do not retune to erase them. Any new calibration protocol or parameter change needs a separately versioned run, not overwriting these artifacts. There is no 300-scenario dataset or final evaluation from this session.
