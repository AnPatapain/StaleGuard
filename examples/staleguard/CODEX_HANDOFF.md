# Current status — 18 Oct 2026

The official inspection-descent experiment now uses maximum consecutive D=0
inspection dwell through the shared `max_dwell_calibration.py` implementation.
The experiment reads the fixed seeds 0–19 from
`inspection_descent/calibrated_scenarios.json`. The automatic cache was removed
by explicit user decision. No source/version checks or automatic recalibration
remain. Run `python inspection_descent/max_dwell_calibration.py` manually to
regenerate the file after changing calibration or mission settings.
`plan_feasible_command()` remains an explicit single-scenario calibration helper;
the official experiment does not call it.
Both official experiments now share DELAY_GRID=0.0–1.0 s in 0.1 s steps,
defined in experiment.py. mission.py passes that grid to prediction and ground
truth. staleguard_benefit.py uses it for prediction and actual dispatch.
The benefit experiment fixes the mission deadline at T=4.2 s relative to
receipt/approval. TTL accuracy retains its existing 8.5 s measurement horizon.
The prior T=5.0 s results were identical for the tested scenarios. Calibration
already used 4.2 s, so the saved calibrated commands are unchanged.
The 0.4–1.0 s low-duration range is only the generator's initial range; the
existing calibration grid may increase it, while returning the reference to
usual height by estimated nominal structure arrival at D=0 on the control grid.

Recovery validation executed all 2,241 candidate commands for seeds 0–19 and
reproduced every prior maximum-dwell selection. Seed 16 remains infeasible
(best safe dwell 141 ms versus required 180 ms). The official 20-seed evaluation
therefore reports calibration feasibility 19/20 and uses N=19 for policy rates.
All 228 official executions matched the pre-cleanup runner's complete traces;
all 304 prior maximum-dwell No Recovery delay outcomes were reproduced.

Use **one simulation worker**. A prior attempt launched overlapping eight-worker
pools and the computer froze. The recovered validator is serial, limits library
threads, checks memory, and prevents overlapping validation launches. Updated
reports go under
`results/official_calibration_validation/delay_grid_0_to_1/`.
Its maximum reported recovery-validation peak RSS was 1030 MiB. Do not restore
multiprocessing or start overlapping simulation runs. Preserve existing caches
and results; diagnostics are under `inspection_descent/diagnostics/`.

Read [the research recap](inspection_descent/recap_visibility.md) for the sequence
of stages, [README](inspection_descent/README.md) for the current workflow, and
[current implementation and validation](inspection_descent/docs/current_implementation_and_validation.md)
for the latest checks. Detailed historical reports are grouped by stage under
`inspection_descent/docs/`. The official workflow is ready for baseline integration; no
additional baselines have been implemented. The older research context below
is historical and must not be treated as the current implementation state.

# StaleGuard — original handoff (15 Oct 2026)

## Your immediate task
Debug and make the **original continuous-forward-flight inspection-descent command** physically feasible at **zero dispatch delay (D=0)** in Safe-Control-Gym/PyBullet, without changing the underlying mission or inventing constants to force results. Inspect the actual repository before editing and **run the experiment to verify changes**. Start with Scenario 1 (seed 1), then test all 10 seeds. Do not claim a patch works until its output has been observed.

## Research purpose
StaleGuard estimates at approval/receipt time how long a supervisory UAV command remains safe if it sits pending in a dispatch queue. The eventual paper needs a fair comparison of safe mission completion with a guard/recovery versus execution of the same originally approved command without recovery. The scientific object is **command staleness from pending delay**, not mission-planner optimization.

**Original mission (preserve):** A quadrotor follows a nominal trajectory moving forward in +x, at `usual_height`. A supervisory command approved at `approval_time` is scheduled for dispatch after delay D; at dispatch, only the altitude reference changes to `low_height` for `low_seconds`, then returns to `usual_height`. The PID controller and quadrotor dynamics take time to physically follow the reference. Crucially, the drone **keeps moving forward**: do not quietly replace this with stop-and-hover, waypoint dwell, or a different recovery route. An unsafe event is entry into a structure volume while z is below a threshold.

**Inspection success definition (leave unchanged initially):** UAV actual state stays simultaneously within `abs(x - scan_x)<0.12m`, `abs(y)<0.20m`, and `abs(z - low_height)<0.08m` for at least `SCAN_DWELL_S=0.18s` consecutively. These tolerances/dwell are existing **unvalidated experiment assumptions**, not empirically proven or literature-derived. `scan_x=box_x_min-INSPECTION_STANDOFF` and `INSPECTION_STANDOFF=0.5m` (also a currently unvalidated design assumption). Current `run_actual()` treats inspection as completed when dwell is satisfied, but continues monitoring safety until deadline; do **not** add `break` without separately defining and agreeing on mission termination.

## Code to inspect
Run from the repository root (`safe-control-gym`), likely using `conda activate scg`:

```bash
python examples/staleguard/inspection_descent/staleguard_benefit.py
```

- `examples/staleguard/experiment.py` (or nearby; find actual file): `Scenario`, `generate_scenario()`, `make_forward_trajectory()`, `apply_descent_command()`, `unsafe()`, `inspection_x()`. **Use actual current paths.**
- `examples/staleguard/inspection_descent/mission.py`: `capture_approval()` creates the approval-time state/PID snapshot by flying the nominal trajectory.
- `examples/staleguard/inspection_descent/staleguard_benefit.py`: `run_actual()`, `in_inspection_region()`, `plan_feasible_command()` (experimental offline search), `main()`; program prints per-candidate diagnostics and aggregates 7 benefit metrics.
- `examples/staleguard/inspection_descent/staleguard.py` (or nearby): `predict()`; lightweight predictor exists in `model/`.

Check that the same nominal trajectory is used consistently by `capture_approval`, actual execution and StaleGuard prediction. Do not change the StaleGuard algorithm to solve nominal trajectory feasibility.

## Key parameters
- PID control 50 Hz, PyBullet physics 1000 Hz, model 100 Hz.
- `EPISODE_SECONDS=9`, `MEASURE_SECONDS=8.5`, `ACCELERATION_SECONDS=0.4`.
- Scenario generator: speed 0.4–0.8 m/s, usual height 1.15–1.40 m, structure `box_x_min` 0.8–1.5 m, structure length 0.3–0.6 m, `unsafe_below_z` 0.90–1.05 m, `low_height` 0.60–(`unsafe_below_z`−0.10) m, initial `low_seconds` 0.4–1.0 s.
- Initial nominal reference speed ramps from zero over 0.4 s, then continues forward at scenario speed; target x integrates reference velocity; z remains `usual_height` except during the command.
- Previously `approval_time=max(0.1, scan_time-low_seconds/2)` where `scan_time=scan_x/speed+ACCELERATION_SECONDS/2`. This centers **reference command interval** around a nominal x-crossing, but fails to account for delayed physical altitude response. A test explicitly tried approval time 0.10 s; it still failed. A zero-time command has not yet been tested in the logs provided.
- Pending delays `D=[0.0,0.5,1.0]` seconds; mission deadlines `T=[4.2,5.0]` seconds relative to command approval/receipt. The absolute simulation cutoff is `(approval_time + T)*50` steps, not `T*50`.

## Empirical evidence — not estimates
The original random commands at D=0 failed inspection 10/10 scenarios (and 1/10 had an unsafe-volume entry). Five never got within ±0.08m target altitude; five reached altitude late relative to scan_x.

An offline `plan_feasible_command()` was added, searching progressively earlier `approval_time` and longer `low_seconds`, simulating actual PyBullet each candidate with D=0. It found Scenario 0 feasible at `approval_time=0.12s`, `low_seconds=0.76s`, giving best valid inspection dwell 0.20s and no collision. It found **no feasible candidate for Scenario 1** in the tested grid including `approval_time=0.10s`.

**Scenario 1 crucial logs (D=0, approval=0.10s, low_seconds=0.90s):**
```
scan_time=0.88s actual_z=0.802m reference_z=0.704m target_z=0.704m vx=0.859m/s
first_target_height_time=0.98s x_at_target=0.473m scan_x=0.401m x_offset=+0.072m
min_z=0.773m target_z=0.704m low_seconds=0.90s
collision=False completed=False best_inspection_dwell=0.08s best_z_error_at_scan=0.069m
```
Required scan x window is 0.281–0.521m; at the moment altitude enters tolerance, only 0.048m horizontal margin remains. It stays in the intersection of altitude and x bounds 0.08s, below required 0.18s. Departure time from region was **not directly measured**.

Lengthening the descent from 0.90s to 1.30s while keeping approval=0.10s did **not** improve best inspection dwell above 0.08s; at 1.40–1.50s an unsafe event occurred. At approval=0.12s, best dwell 0.02s. This strongly indicates the main issue is timing/available approach distance and horizontal movement, not merely duration of altitude command.

An observed altitude tolerance response for Scenario 1 is `0.98−0.10=0.88s`, including physical quadrotor response, NOT PID compute time or a universal SCG constant. It depends on altitude step, UAV state, controller, and motion; cannot apply as a magic global offset.

## Scientific and implementation constraints
1. Maintain the **continuous-forward inspection-descent mission** and original command semantics. Do **not** adopt a stop-and-hover redesign just to get success.
2. Do not relax existing inspection/safety bounds merely to produce positive results. If bounds appear inconsistent/physically implausible, identify that explicitly rather than silently altering them.
3. Every new numeric constant/range must be derived from existing settings, a simple transparent physical constraint, real test measurements, or explicitly labeled an unvalidated assumption. No arbitrary tuning for successful results.
4. Prefer minimal code modifications. No complex infrastructure, new frameworks, or learned model changes. Change one variable/factor at a time, and isolate the original nominal D=0 feasibility before running the benefit study.
5. Do not cherry-pick passing scenarios or present feasibility-filtered results as unbiased. Preserve scenario seeds and report feasibility/filter rates.
6. Keep StaleGuard prediction unchanged, and do not use predicted TTL to calibrate the original mission. Ensure all methods see exactly the same approved command and nominal reference, and deadlines keep the same definition.
7. If physically feasible continuous-flight inspection cannot be guaranteed for some scenario parameter combinations, diagnose and **report** the constraint, then propose a principled scenario-generator feasibility domain rather than forcing every random scenario to pass.

## Work requested from Codex
1. Open/read the current actual code and identify possible mistakes in generator, reference time indexing, `capture_approval`, planner search, and physical execution; don't assume the previous assistant patch is correct.
2. Instrument and reproduce Scenario 1 at D=0. Explain precisely what the time/position/altitude logs show, with measured (not assumed) entry/exit times and reference tracking where needed.
3. Decide if a minimally changed **approach distance / initial flight position / physically justified timing** can permit continuous forward inspection while preserving the scenario's structure/safety geometry. Account for initial trajectory alignment and snapshot semantics. Compare against any alternative you consider.
4. Implement smallest justified patch and run it; show actual D=0 success/collision/dwell for seed 1 and seeds 0–9. If infeasible, state that with measured evidence and explain the admissible parameter bounds.
5. Only after D=0 is feasible, run the original D,T benefit matrix. Preserve seven metrics: safe completion, unsafe rate, recovery trigger, recovery success, recovery benefit (`R&S1&!S0`), recovery harm (`R&!S1&S0`), net benefit (`(S1−S0)/N`). `S0` is no-recovery counterfactual, `S1` uses recovery, and `R` means recovery actually started.
6. Show a concise `git diff` and results; do not add a stop-and-hover or change `break` on inspection completion without explicit approval.

If you cannot execute the code, say exactly why and do not pretend a patch was validated.
