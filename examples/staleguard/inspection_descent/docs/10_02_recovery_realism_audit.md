> Source relocation: this report preserves its historical findings and commands. Current code: `staleguard_benefit.py` → [simulation.py](../simulation.py). Old entry points have been removed.

# Recovery realism audit — 20 October 2026

Later decision: safety-only recovery is now the primary protocol. The maneuver
audited below remains the historical `--recovery mission-retry` variant. See
the [subsequent implementation and results](10_03_safety_only_recovery.md).

The existing recovery is physically simulated, but it replaces the forward
inspection pass with a second inspection while holding position. Whether that
is an allowed recovery maneuver needs a mission decision before changing it.
The 100% success rate alone does not establish a physics error.

**Status: audit executed; no recovery protocol change made.** The original
results remain available. No new operating limits, penalties, planning delays,
success conditions, or baselines have been introduced.

## What the code actually does

Both StaleGuard and Always-Recover call the same recovery branch of
[`run_actual()`](../staleguard_benefit.py). It:

1. Cancels the pending descent command before dispatch.
2. Sets the horizontal position target to the inspection point, requests zero
   forward velocity, and requests usual height. The physical UAV still moves
   through PyBullet and the PID; its position and velocity are not reset.
3. Starts the low-altitude command on the first control step with horizontal
   position error below 0.08 m and absolute forward speed below 0.15 m/s.
4. Uses the original command's calibrated low-altitude duration, then requests
   usual height again. It continues holding horizontal position through the
   deadline. It never resumes the forward reference.

There is no general trajectory planner in this branch. It copies and edits
reference arrays and uses feedback for the descent transition. A large planning
delay cannot be justified by assuming an expensive planner that is not present.
Always-Recover's zero decision time is the existing baseline definition;
StaleGuard uses its measured prediction time. Neither models controller or
mode-switch execution time on a specific onboard computer.

## Constraints already enforced, and limits of the model

| Part | Already enforced | Missing or not established |
|---|---|---|
| Physical movement | Vehicle mass, inertia, gravity, attitude response, PID feedback, bounded motor commands and actuator conversion in SCG/PyBullet. No teleporting or instantaneous physical stop. | No explicit recovery speed, acceleration, jerk, tilt, or body-rate operating limits. Motor response has no spool-up model. |
| Descent transition | Actual horizontal position and forward-speed checks. | No attitude, angular-rate, lateral-velocity, or vertical-settling checks, and no sustained stabilization interval. |
| Time | Repositioning, waiting, descent, inspection dwell, and ascent all consume simulation time. The deadline stays 4.2 s from original receipt; it is not restarted. | There is no separately defined end of the recovery maneuver or required time to resume the route. |
| Completion and safety | At least 0.18 s of consecutive inspection dwell; inspection and unsafe-volume checks at 1,000 Hz. Safety monitoring continues after inspection until the deadline. Safe completion remains inspection completed AND no unsafe entry. | Completion does not require finishing ascent, resuming forward flight, or exiting the area. Adding these would change the mission definition. |
| Availability | Official policy callers permit recovery only at/after receipt and strictly before dispatch. D=0 executes the original command. A recovery attempt can still fail through unsafe entry or the deadline. | No separate test proves a maneuver feasible before attempting it. `run_actual()` assumes callers supply a valid recovery schedule. |
| Environment and sensing | The same deterministic simulator, controller, and scenario geometry for both policies. | Exact state feedback, no wind, no sensor/actuator disturbances, no explicit aerodynamic drag in the selected physics mode. The unsafe structure is a logical volume, not an added collision mesh. No battery/energy or camera attitude/blur requirements. |

The physical limits are implemented in SCG's `controllers/pid/pid.py` and
`envs/gym_pybullet_drones/{quadrotor,base_aviary}.py`. In particular, the PID
clips its PWM/motor commands, the environment clips physical actions, and
PyBullet integrates the applied rotor forces and torques. These are physical
limits, not a complete set of limits for an inspection operation.

The nominal-arrival bound in calibration applies to the original forward
command. It is not currently a recovery cutoff. Applying it to a vehicle that
has stopped before the structure would require a new operational rule.

## What was executed

The audit first executed 27 distinct flights for seeds 0 and 11. It then
executed **252 distinct flights covering all 627 saved policy cases**: the
19 feasible scenarios, eleven delays, and three policies. Seed 16 remains
infeasible and is excluded from the policy counts.

Repeated identical flights run once: when a command is cancelled, its unused
dispatch time cannot affect the recovery. Without recovery, the unused recovery
time cannot affect the flight. This gives 43 distinct recovered flights and
209 distinct original-command flights. They represent 316 recovered cases and
311 cases without recovery in the earlier three-policy experiment.

All 627 outcome tuples matched the saved experiment. All 209 available saved
trace hashes matched, including repeats across delays. The previous full run
did not retain StaleGuard trace hashes; its recovered flights were checked
against the saved outcomes, and their raw traces are now saved. The audit
replayed recorded StaleGuard decision times rather than rerunning prediction
and introducing new host-timing variation. It therefore verifies flight
execution, not a new measurement of prediction latency.

Each execution checked the individual safe-completion identity. The paired
benefit-minus-harm identity passed at every delay. All mission/calibration/model
source hashes and the original experiment file remained unchanged during the
audit. One simulation worker was used, with numerical threads limited to one
and the existing memory checks. Full-run peak RSS was **944 MiB**.

## Measurements from the 43 recovered flights

| Measurement | Observed range |
|---|---:|
| Recovery start after simulation begins | 0.00–0.64 s |
| Forward speed when recovery starts | 0.000–0.628 m/s |
| Recovery start to descent command | 0.78–1.84 s |
| Recovery start to first completed inspection | 1.511–2.802 s |
| Time left at first completed inspection | 1.398–2.689 s |
| Longest consecutive inspection dwell | 0.221–1.679 s |
| Closest horizontal distance to structure's leading face | 0.263–0.480 m |
| Total backward travel after recovery starts | 0.009–0.422 m |
| Recorded peak horizontal speed | 0.708–2.615 m/s |
| Recorded peak absolute vertical speed | 0.670–1.447 m/s |
| Recorded peak tilt from vertical | 13.17–56.07 degrees |
| Tilt when descent is commanded | 7.08–20.67 degrees |
| Angular speed when descent is commanded | 0.0003–2.622 rad/s |

Positions and inspection dwell were measured at 1,000 Hz. Velocity and attitude
measurements use the existing 50 Hz control trace; these recorded peaks are
not guarantees about unobserved peaks between control samples. Backward travel
is the sum of negative horizontal displacement, including small oscillations.
The structure distance is horizontal distance, not a complete 3D body clearance.

For example, seed 4's Always-Recover flight starts at 0.120 m/s, reaches
2.615 m/s and 56.07 degrees of tilt, and travels backward by a total of 0.422 m.
Its nominal forward-speed setting is 0.777 m/s. The largest speed-to-nominal
ratio across recovery traces is 4.77. This is evidence of an aggressive
position-target response; the nominal reference speed is not currently defined
as a physical or operational speed limit, so exceeding it is not by itself
proof of an impossible flight.

Seed 3 begins descent with about 20 degrees of tilt and 2.622 rad/s angular
speed. Passing the existing position/forward-speed gate therefore does not mean
the vehicle is fully stabilized. At the deadline, one recovery trace is still
0.223 m below usual height. Existing success rules allow this. The time to
inspection in the table is **not** the time to complete an ascent or resume a
route; those completion requirements have not been defined.

## Why Always-Recover succeeds in these cases

Its nineteen immediate-recovery flights are identical across all ten positive
delays. These are not 190 different tests of starting recovery later in flight.
Five scenarios start recovery at t=0; all nineteen start by t=0.58 s. The
vehicle can move to a point 0.5 m before the structure and inspect while holding
there, rather than fitting descent into a short moving inspection window.
The measured time margins leave room for this maneuver under the existing
deadline. This evidence does not establish recovery feasibility from arbitrary
later or disturbed states.

StaleGuard predicts unsafe entry, not inspection completion. A retained command
can be safe and still miss inspection. Across the 209 StaleGuard cases, 21
were safe but missed inspection and three entered the unsafe volume. All 126
triggered StaleGuard recoveries succeeded. Thus the comparison also reflects
the difference between safety-based intervention and automatically retrying
inspection, not just the realism of braking dynamics.

## Reproduced policy results

Each entry is **A / B / C**, with A = safe inspection completion,
B = safe but inspection incomplete, and C = unsafe regardless of inspection.
Each entry sums to 19. These reproduce the original protocol, not a revised one.

| Delay (s) | No Recovery | Always-Recover | StaleGuard |
|---:|---:|---:|---:|
| 0.0 | 19 / 0 / 0 | 19 / 0 / 0 | 19 / 0 / 0 |
| 0.1 | 16 / 2 / 1 | 19 / 0 / 0 | 17 / 2 / 0 |
| 0.2 | 12 / 5 / 2 | 19 / 0 / 0 | 14 / 5 / 0 |
| 0.3 | 5 / 8 / 6 | 19 / 0 / 0 | 10 / 8 / 1 |
| 0.4 | 3 / 4 / 12 | 19 / 0 / 0 | 13 / 4 / 2 |
| 0.5 | 3 / 2 / 14 | 19 / 0 / 0 | 18 / 1 / 0 |
| 0.6 | 1 / 1 / 17 | 19 / 0 / 0 | 18 / 1 / 0 |
| 0.7 | 0 / 0 / 19 | 19 / 0 / 0 | 19 / 0 / 0 |
| 0.8 | 0 / 0 / 19 | 19 / 0 / 0 | 19 / 0 / 0 |
| 0.9 | 0 / 0 / 19 | 19 / 0 / 0 | 19 / 0 / 0 |
| 1.0 | 0 / 0 / 19 | 19 / 0 / 0 | 19 / 0 / 0 |

## Evidence and minimum changes to consider

Braking and hovering are valid multicopter capabilities. ArduPilot documents
a braking mode that requires a position estimate and respects controller
acceleration and angle limits. This supports the maneuver in principle, but
does not validate our particular gains or operating envelope.
[ArduPilot: Brake Mode](https://ardupilot.org/copter/docs/brake-mode.html).

PX4's autonomous trajectory generation uses smooth motion with configurable
acceleration and jerk limits. This supports adding an explicit operating
envelope if required; its default values are not evidence for limits on our
vehicle. The abrupt reference change here is a modeling simplification,
while the resulting physical motion remains subject to the simulated dynamics.
[PX4: Jerk-limited trajectories](https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory).

The minimum next step is to settle what recovery is permitted to do:

1. **Stopping and retrying inspection is allowed.** Keep that maneuver. To
   claim operational realism, establish vehicle-specific speed/tilt/acceleration
   limits and what counts as stable enough for inspection. Add only the missing
   constraints justified by those requirements, identically for both policies.
2. **Recovery must preserve forward flight.** Holding position and returning
   toward the inspection point are not allowed. A common recovery would then
   need to respect the forward pass; maintaining or regaining usual height is
   a possible safety response, but inspection may legitimately be lost.
   This is a different recovery objective and needs explicit agreement.

Whether finishing ascent or resuming the route belongs in mission completion
is a separate decision. It cannot be added while claiming the current success
definition is unchanged. A fixed stabilization delay or a new planning delay
also needs evidence; none was added merely to lower recovery success.

**Decision pending:** may recovery stop the UAV and retry inspection while
holding position, or must it preserve forward flight? No revised-protocol
results are claimed before that decision.

## Files and reproduction

The added diagnostic is
[`diagnostics/audit_recovery_realism.py`](../diagnostics/audit_recovery_realism.py).
It reuses the existing simulator, recovery, and mission predicates. It does
not run calibration or define another policy. Documentation now distinguishes
the original forward command from the recovery's position hold.

```sh
python inspection_descent/diagnostics/audit_recovery_realism.py --seeds 0 11 --output inspection_descent/results/recovery_realism_audit/small
python inspection_descent/diagnostics/audit_recovery_realism.py
```

The final diagnostic's shared-predicate measurements were also checked against
all 43 retained recovery traces, and seed 4 was replayed after that reporting
cleanup. The full replay used the same predicates and numerical thresholds;
the cleanup removes their duplicate spelling from the diagnostic only.

Evidence is saved under `results/recovery_realism_audit/`:
[`full_run.log`](../results/recovery_realism_audit/full_run.log),
[`summary.json`](../results/recovery_realism_audit/summary.json),
[`recovery_measurements.csv`](../results/recovery_realism_audit/recovery_measurements.csv),
per-flight replay records, original-file hashes, and 43 compressed raw traces.
The preceding experiment remains under `results/always_recover/`. Results are
Git-ignored as before; the diagnostic and this report can be committed.
