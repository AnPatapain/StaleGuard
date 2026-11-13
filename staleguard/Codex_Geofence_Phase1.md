# StaleGuard — Geofence Emergency Turn, Phase 1 (unattended implementation)

You are working in my existing StaleGuard repository. Implement Phase 1 of a second safety-critical mission using Safe-Control-Gym (SCG) 3D Quadrotor, the EXISTING 50 Hz PID trajectory-reference mechanism, PyBullet ground-truth execution, and where feasible the existing Numba lightweight prediction/baseline harness. I will be unavailable to approve or answer questions. Investigate repository code and make reasonable, explicitly documented engineering choices; complete as much as possible without asking me. Do not fake results.

## Scope and priorities

Priority 0: Implement and validate mission reference, geofence safety checks, scenario generator, and D=0 calibration.
Priority 1: Run a small reproducible DEVELOPMENT set and verify calibration; smoke-test the existing runner/policies after proper adaptation (No Recovery, StaleGuard, Use-Time Recheck, then any already available Fixed TTL/Periodic/FBS). Produce diagnostics and logs. Do not claim valid comparisons if an adapter is incomplete.
OUT OF SCOPE tonight: 300-scenario calibration; full final evaluation; parameter tuning to make StaleGuard win; new UAV dynamics/controller; PX4/Gazebo; new recovery policy; major refactors.

## First: inspect code and protect existing work

Read the existing `examples/staleguard/inspection_descent` implementation and tests, especially `make_forward_trajectory`, `apply_descent_command`, scenario generation/calibration, approval snapshot creation, PID/reference shapes and indices, predictive rollouts, safety-only recovery, baseline runners and result persistence. Use actual APIs and names from the repo; don't invent dependencies.
Before edits, capture `git status --short` in your report; preserve any user modifications. Prefer new `geofence_turn/` mission files with small shared abstractions, rather than changing Inspection-Descent. Never overwrite/delete existing calibrated 300 scenarios, existing results, or scientific-source files; do not make a git commit. Avoid network or package installs. Use the existing `scg` Python environment and run CPU-only, headless.

## Mission definition (locked)

A UAV follows a nominal +X horizontal patrol at constant usual altitude after an existing-style acceleration ramp. A supervisor approves a `TURN_LEFT_90(radius)` command, while its future queue delay D is unknown. Without dispatch, nominal +X flight continues. When applied, the command induces a real **horizontal course change** (not yaw-only rotation): a dynamically trackable quarter-circle from +X to +Y, followed by a northbound leg and stabilization at a predesignated waypoint. A fixed x-positive geofence boundary must not be crossed. The approved radius, turn direction, waypoint and other command parameters are frozen at approval. Execution delay changes *when* the maneuver starts, but never triggers an optimized new route or new goal selection. An execution-state-relative reference may be generated with the same deterministic rules at dispatch, mirroring how `apply_descent_command(reference, start_step, ...)` works, but make this contract explicit and identical across StaleGuard and baselines.

Coordinate convention: X positive = forward/east; +Y = left/north; Z constant. If the actual repository uses different indices, follow those indices. Example reference fields: X,VX,Y,VY,Z,VZ. The arc can use theta=speed*t/radius; x=x0+R*sin(theta); y=y0+R*(1-cos(theta)); vx=speed*cos(theta); vy=speed*sin(theta); theta<=pi/2. Include a feasible, smooth end segment that reaches the predesignated fixed waypoint and stabilizes: continuity of x/y and vx/vy at arc entry, arc exit, and terminal hold is mandatory. The target waypoint must not be recomputed as approval time or D varies. Explicitly test the potential conflict between a start-relative turn and fixed waypoint; resolve with a deterministic, non-optimizing endpoint-following rule valid for all policies rather than silently moving the target.

Safety definition: the physical UAV body (not only its center/reference) stays within the authorized geofence with signed minimum clearance >= d_min over the entire monitored interval. Set a body footprint approximation consistent with SCG quadrotor geometry; document its source, validate geometry for x-positive fence, and avoid double-counting clearance. Observe actual PyBullet trajectory, not reference-only safety. Treat any pre-dispatch violation as an actual violation. No hard-coded arbitrary safety margins.

Mission completion: reach the *same fixed designated patrol waypoint* before deadline T (measured from approval), within explicit position and velocity tolerances and a short stabilization dwell. All such thresholds must be named/configurable, documented as physical/empirical/provisional, and fixed prior to baseline evaluation. Safe Mission Completion = completion AND no geofence violation through the defined horizon. Safety-only recovery that never reaches the waypoint = safe non-completion, never success.

## Scenario generation and calibration (locked)

1. Seeded scenarios independently vary nominal forward speed, altitude, geofence geometry, initial/nominal conditions, feasible turn radius, fixed waypoint, deadline, with clearly documented joint constraints. For initial speed/height candidates, the existing Inspection-Descent range 0.4–0.8 m/s and 1.15–1.40 m is an acceptable starting operating envelope, NOT proof of turning feasibility.
2. Derive an empirical speed-vs-radius turning envelope via small real SCG/PID tests; use a_lateral ~= v^2/R only as a screening relation. Define feasible turning radii using measured tracking, available geometry, and deadline, not guesses. Record sweep results. Document every constant—code default vs physically derived vs provisional vs experimentally measured. Do not manufacture ranges based on which policy wins.
3. Per scenario sample ONE feasible turn radius and a fixed world-frame target waypoint BEFORE calibration. Search approval steps AFTER the nominal acceleration ramp, simulate immediate command application at D=0 using full SCG/PyBullet + PID, and select the LATEST approval time / maximum useful pre-turn patrol progress for which all are true: geofence minimum clearance satisfied, fixed waypoint reached and stabilized, mission before T.
4. If no feasible approval time exists, reject and LOG why. Log generator attempts, feasibility rate, search grid/resolution, selected approval time and radius, minimum actual fence clearance, waypoint completion time, and parameters. Do not implicitly adjust the target waypoint per candidate approval time.
5. Freeze and serialize calibrated scenario, approval snapshot/state/PID history/command parameters, seed, and configuration for deterministic replay; if a pre-existing serializer is available, reuse it. Keep calibration independent of delayed-dispatch and baseline performance.

## Tests and small development run

- Unit tests: deterministic seed generation; correct 90-degree direction; x/y/vx/vy continuity at phase boundaries; frozen command and fixed waypoint across candidates; correct fence/body signed clearance; no waypoint drift with delayed dispatch; D=0 pass criteria; explicit rejected scenarios; phase timing and no hidden replanning.
- Integration tests: PyBullet/PID actually follows a curved horizontal turn at several feasible speeds/radii; observed body clearance evaluated on actual states; a safe D=0 trajectory; a delayed rollout uses identical frozen command semantics and original waypoint.
- Development run: start SMALL: a speed/radius characterization sweep, then attempt up to 10–20 distinct seeds to obtain about 5 successfully calibrated DEVELOPMENT scenarios (do not silently continue thousands of attempts). Print acceptance rate and explain infeasibility. Run D=0 replay on all successful cases.
- If adaptations are correct and cheap, run paired smoke tests on these development scenarios at D=0 and a few preregistered positive delays using No Recovery, StaleGuard, Use-Time Recheck, and any available policy (Fixed TTL, Periodic, FBS). Use common seeds/conditions and safety-only recovery, and keep actual future D unknown at approval. Explicitly report which policies are NOT yet adapted. StaleGuard prediction readiness and Use-Time Recheck actual computation/tick latency must be respected. Do not report synthetic scores or assume a method wins.
- Avoid an overnight 300-scenario run or large memory/time consumption. Preserve SCG CPU-only/headless and existing memory guardrails; cap trial work explicitly.

## Deliverables

Provide new mission-specific code, CLI commands for the characterization, scenario generation, D=0 calibration and development smoke test, focused unit/integration tests, and development logs in separate clearly named directories. Write `geofence_turn/PHASE1_REPORT.md` documenting: exact changed files; frozen command semantics; parameter ranges and derivations (mark provisional values); calibration acceptance count/reasons; reproduced D=0 examples; tests and results; baseline status and any failures; how to resume in Phase 2. Give paths and git diff summary. Preserve existing Inspection-Descent behavior and results and never claim completion of unrun tests.

When a design ambiguity arises, choose the minimal physically reasonable option that respects the locked requirements and document it; do not ask for approval. If an essential simulation dependency cannot run, complete code/tests that can run, accurately explain the blocker, and stop instead of faking an evaluation.

ARCHITECTURE REQUIREMENT

Treat inspection_descent/ and geofence_turn/ as independent
mission implementations sharing the same underlying
StaleGuard prediction infrastructure.

Keep all new mission-specific code inside geofence_turn/.

Reuse existing models and utilities where practical, but do
not copy the entire inspection_descent implementation.

Do not perform a large refactor of inspection_descent/.
If a shared abstraction is genuinely necessary, extract only
the minimum reusable component and verify that existing
Inspection-Descent tests still pass.

Mission A must remain executable without any dependency
on Mission B.

Prefer simple, explicit code over complex abstractions.