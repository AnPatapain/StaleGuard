> Source relocation: this report preserves its historical findings and commands. Current code: `staleguard_benefit.py` → [simulation.py](../simulation.py). Old entry points have been removed.

# Stages 2–3: feasibility fixes and initial recovery results

Historical report, executed 15 October 2026. The command selections and benefit
results below use the earlier first-feasible calibration. For today’s workflow,
see the [README](../README.md).

Scenario 1 and all retained seeds 0–9 pass at D=0 after offline command
calibration, with 0.187–0.246 s of elapsed inspection dwell and no unsafe-volume
entry. These are measured Safe-Control-Gym/PyBullet results, not TTL predictions.

## Diagnosis and measured Scenario 1 failure

The generator centers the **reference** altitude interval on the nominal scan
crossing. It does not account for the physical altitude response. Some randomly
generated intervals end before the vehicle can spend enough time at the requested
height. The existing offline planner additionally excluded approvals before
0.10 s, although the initial state can receive a command before its first action.

With the original position reference, physics-rate measurements give:

| Approval (s) | Low duration (s) | First altitude entry (s) | Last inspection state (s) | Elapsed inspection dwell (s) | Unsafe entry |
|---:|---:|---:|---:|---:|:---:|
| 0.414651 (generated) | 0.896622 | 1.284 | no overlap | 0 | no |
| 0.10 | 0.90 | 0.963 | 1.047 | 0.084 | no |
| 0.10 | 1.30 | 0.963 | 1.046 | 0.083 | no |
| 0.00 | 0.90 | 0.860 | 0.961 | 0.101 | no |
| 0.00 | 1.00 | 0.860 | 1.037 | 0.177 | no |

For the handoff command (0.10 s, 0.90 s), the first outside-x state is at
1.048 s. At the 0.900 s scan crossing, actual z=0.801703 m versus a reference
of 0.704466 m, and actual vx=0.858963 m/s versus 0.604729 m/s commanded.
Lengthening the dip alone cannot recover time already lost beyond the scan window.
An initial-position translation alone would leave the original command's
altitude-tolerance interval shorter than the required dwell, so it cannot solve
the duration problem either.
The original 50 Hz reproduction gave the handoff's 0.08 s sample-count dwell.
Its old `scan_time=0.88` log describes the state at 0.90 s; that log was one step early.

Two smaller implementation errors matter at the feasibility boundary:

* `cumsum(speeds)*dt` integrates through the current sample's full interval. For
  the linear ramp this leads the intended position by half a control step after
  acceleration: speed/(2*50), or 0.006047 m for seed 1. Trapezoidal integration
  gives the exact integral of the stated ramp; the analytic-integral check has
  maximum error 1.55e-15 m.
* Nine qualifying 50 Hz samples were reported as 0.18 s, although their first-to-last
  elapsed time is only 0.16 s. The apparent zero-time/1.00 s pass above has only
  0.177 s between qualifying physics states. It was a false positive under the
  old sample counter.

## Changes and physical justification

The existing uncommitted offline planner is preserved. The mission changes are
the natural earliest approval boundary (zero) and correct integration of its
existing velocity ramp. Inspection and unsafe entry are now observed before each
existing 1 kHz physics step and at control-step endpoints; inspection requires
180 elapsed physics ticks, derived from the existing 0.18 s requirement.
This observation hook does not modify forces, controller actions, or physics steps.

The two small mission edits are:

```diff
- trajectory[:, STATE_X] = np.cumsum(speeds) * dt
+ trajectory[:, STATE_X] = (np.cumsum(speeds) - speeds / 2) * dt
- earliest_approval_step = round(0.1 * CTRL_FREQ_HZ)
+ earliest_approval_step = 0
```

No initial-position offset is needed. Position and velocity references remain
aligned from initial x=0. The reference keeps its acceleration ramp and then
continues forward at the generated speed. The altitude command still lasts
`low_seconds` and returns to `usual_height`. Geometry, inspection tolerances,
standoff, and StaleGuard/predictive-model algorithms are unchanged. Safety remains
monitored until the original approval-relative deadline after inspection completes.
The existing recovery policy is preserved.

The planner's duration values are selected by actual D=0 rollouts using its
existing search resolution and structure-arrival bound. They are command-specific
calibration results, not a new global response-time constant.

## Corrected D=0 results

| Seed | Approval (s) | Low duration (s) | Elapsed dwell (s) | Safe completion | Unsafe entry |
|---:|---:|---:|---:|:---:|:---:|
| 0 | 0.12 | 0.76 | 0.213 | yes | no |
| 1 | 0.00 | 1.00 | 0.188 | yes | no |
| 2 | 1.26 | 0.82 | 0.240 | yes | no |
| 3 | 1.44 | 0.88 | 0.225 | yes | no |
| 4 | 0.62 | 0.88 | 0.187 | yes | no |
| 5 | 0.20 | 1.04 | 0.214 | yes | no |
| 6 | 0.40 | 0.80 | 0.238 | yes | no |
| 7 | 0.70 | 0.90 | 0.211 | yes | no |
| 8 | 0.26 | 0.96 | 0.191 | yes | no |
| 9 | 0.50 | 0.92 | 0.246 | yes | no |

For corrected seed 1, qualifying physics states run from 0.861 to 1.049 s;
the first outside state is at 1.050 s. Its elapsed dwell is 0.188 s, an 8 ms
margin. No unsafe entry occurs through the 4.200 s deadline. At the control-sampled
scan crossing (0.900 s), x=0.406421 m, z=0.775144 m, and vx=0.871135 m/s.

Feasibility: 10/10, retained seeds: 10/10, filtered: 0. Exact approval-state and
forward-reference agreement checks pass for every original and calibrated run.
Across calibrated runs, the minimum control-sampled vx after the ramp is
0.210644 m/s; the original forward mission continues through inspection.
The original pre-patch random commands reproduced 0/10 completion and 1/10
unsafe entry. These calibrated results establish feasibility for these ten
instances; the random generator still does not guarantee feasibility in general.
The original tolerances, dwell, and standoff remain unvalidated assumptions.
Boundary times have 1 ms physics sampling resolution.

## Reproduction and artifacts

From `safe-control-gym`, with the `scg` environment:

```sh
python examples/staleguard/inspection_descent/diagnostics/verify_feasibility.py --scenario 1
python examples/staleguard/inspection_descent/diagnostics/verify_feasibility.py
python examples/staleguard/inspection_descent/staleguard_benefit.py
```

This execution used `/home/kean/miniconda3/envs/scg/bin/python`. Syntax compilation
and `git diff --check` passed. Raw artifacts are in `inspection_descent/results/`
(the repository ignores directories named `results`):

* `baseline_before_patch.jsonl`: original reproduction before edits.
* `scenario1_before_fix_physics.jsonl`: original-reference ablations at physics rate.
* `all_scenarios_d0.jsonl` and `scenario1_d0.jsonl`: corrected measurements.
* `benefit_matrix.log`: corrected original D,T study output.
* `session_changes.patch`: delta against the files present at session start.

Files with `sample_count_only` in their names and `physics_dwell.jsonl` document
the preliminary experiment that exposed the dwell-counter false positive. They
are excluded from the corrected results reported here.

## Executed benefit matrix

N=10 retained seeds per row. Safe completion is S1; recovery success is
conditional on R. Benefit is R&S1&!S0, harm is R&!S1&S0, and net is (S1-S0)/N.

| D (s) | T (s) | Safe completion | Unsafe | Recovery trigger | Recovery success | Benefit | Harm | Net (percentage points) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 4.2 | 100.0% | 0.0% | 0.0% | N/A | 0.0% | 0.0% | 0.0 |
| 0.0 | 5.0 | 100.0% | 0.0% | 0.0% | N/A | 0.0% | 0.0% | 0.0 |
| 0.5 | 4.2 | 50.0% | 0.0% | 60.0% | 83.3% | 50.0% | 0.0% | 50.0 |
| 0.5 | 5.0 | 50.0% | 0.0% | 60.0% | 83.3% | 50.0% | 0.0% | 50.0 |
| 1.0 | 4.2 | 90.0% | 0.0% | 100.0% | 90.0% | 90.0% | 0.0% | 90.0 |
| 1.0 | 5.0 | 90.0% | 0.0% | 100.0% | 90.0% | 90.0% | 0.0% | 90.0 |

At D=0.5, recovery succeeds in 5/6 triggered runs; at D=1.0, it succeeds in
9/10. S0 has 10/10 safe completions at D=0 and 0/10 for both delayed cases.
The runner's paired net-benefit assertion passed in every matrix cell. Both
deadlines produce the same seven metrics in this execution.

The final full runner exited with status 0. Its independently recalibrated
commands match the D=0 verification for all ten seeds. `benefit_matrix.json`
contains the seven metrics with counts and denominators. Preliminary matrix
output based on the old sample counter is not used.
