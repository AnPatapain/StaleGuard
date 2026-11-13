# Stage 5: sensitivity of first-feasible calibration

Historical diagnostic, executed 17 October 2026. “Current commands” below
means the first-feasible commands used at that stage. This investigation did
not change the official calibration. See the [recap](../recap_visibility.md)
for the later steps.

The hypothesis is supported as a major contributor, with an important boundary
case. D=0-feasible alternatives in the fixed local inventory succeed at 100 ms
delay for nine scenarios and at 200 ms for eight. The existing first-feasible
commands all fail at those delays. Scenario 1 remains unsuccessful even when its
complete original planner grid is examined.

The previous sweep also did not test every positive delay: its first positive
point was 100 ms. With the current commands, nine scenarios succeed at 20 ms.

## Method and scope

All runs use the existing `run_actual()`, no recovery, T=4.2 s relative to approval,
50 Hz PID, 1000 Hz physics, and the unchanged 180 ms inspection requirement.
Geometry, speed, heights, trajectory, predicates, and simulation termination are
unchanged. No StaleGuard prediction is used. No diagnostic candidate replaces a
production command or alters `plan_feasible_command()`.

The current ten commands were read from `results/no_recovery_delay_sweep.csv`,
then independently re-executed at D=0. The prior search audit also confirms that
each selected command ended its scenario's search at the first passing result;
the 337 preceding/current search trials are historical records, not new trials
counted below.

Before examining delayed outcomes, the local design was fixed as:

* Approval: selected time, one search step earlier, two steps earlier; clipped at zero.
* Low duration: selected duration, one step longer, two steps longer.
* Step: the existing planner's 0.1 s grid increment. The original structure-arrival
  bound is retained. Duplicate commands caused by clipping are removed.

This gives 84 unique candidates across all ten scenarios. All 84 are first run at
D=0; eligibility is frozen before delayed testing. The D=0 inventory contains
78 A, five B, and one C. The 78 eligible commands comprise the ten current
selections and 68 alternatives. Every eligible command is tested at D=0.1 and
0.2 s, including failures and unsafe outcomes; there is no selection on delayed
success. The selected commands are additionally tested at 20/40/60/80 ms.

This is a local sensitivity inventory, not an exhaustive optimization or a new
calibration rule. Counts of alternatives are not success rates for a deployed
ten-scenario policy; several candidates belong to each scenario.

## Why the selected commands fail at 100 ms

The current D=0 inspection dwell is only 187–246 ms, giving 7–66 ms of dwell
margin above the required 180 ms. The planner checks feasibility, without any
requirement for delay tolerance, and returns immediately on the first pass.

At 100 ms, all ten selected commands remain safe but their actual inspection
overlap is only 0–162 ms. For each flight, the horizontal window alone lasts
245–522 ms and the altitude-tolerance window alone lasts 183–475 ms. Each window
can therefore be long enough by itself, while their intersection is too short.
Nine best-overlap intervals end on the x boundary; Scenario 8 has no overlap at
all. For Scenario 8, altitude tolerance is first reached 0.1279 m beyond scan_x,
already outside the unchanged 0.12 m x tolerance.

The failures occur well before the deadline, with approximately three seconds
remaining. They are loss of inspection-window overlap, rather than unsafe entry
or expiration of the 4.2 s allowance. Dwell margin is not itself a delay budget:
the vehicle's horizontal and vertical tracking also changes with dispatch time.

| Scenario | Current D=0 dwell (ms) | Current D=100 ms dwell (ms) | D=0-feasible local alternatives | Alternatives A at 100 ms | Alternatives A at 200 ms |
|---:|---:|---:|---:|---:|---:|
| 0 | 213 | 120 | 8 | 6 | 0 |
| 1 | 188 | 91 | 2 | 0 | 0 |
| 2 | 240 | 130 | 8 | 6 | 3 |
| 3 | 225 | 162 | 8 | 6 | 3 |
| 4 | 187 | 145 | 6 | 4 | 2 |
| 5 | 214 | 48 | 7 | 5 | 2 |
| 6 | 238 | 153 | 8 | 6 | 3 |
| 7 | 211 | 121 | 7 | 5 | 2 |
| 8 | 191 | 0 | 7 | 5 | 2 |
| 9 | 246 | 155 | 7 | 5 | 2 |
| Total | | | 68 | 48 | 19 |

## Controlled factors distinguish timing from duration

| Diagnostic factor | D=0 tested | D=0 feasible | At 100 ms: A/B/C | At 200 ms: A/B/C |
|---|---:|---:|---:|---:|
| Earlier approval only | 18 | 13 | 13/0/0 | 4/9/0 |
| Longer duration only | 20 | 19 | 0/19/0 | 0/13/6 |
| Earlier approval and longer duration | 36 | 36 | 35/0/1 | 15/19/2 |

Only D=0-feasible commands enter the delayed columns. In particular, making the
dip longer at the existing approval does not produce a 100 ms success among any
of the 19 eligible duration-only alternatives. Moving approval earlier provides
the main benefit in this inventory. Some alternatives become unsafe when delayed;
D=0 feasibility is not a robustness guarantee.

A concrete counterexample to inherent positive-delay infeasibility is Scenario 2:

| Diagnostic command | Approval (s) | Low duration (s) | D=0 | D=100 ms | D=200 ms |
|---|---:|---:|---|---|---|
| Current selection | 1.26 | 0.82 | A, 240 ms | B, 130 ms | B |
| One step earlier, same duration | 1.16 | 0.82 | A, 265 ms | A, 240 ms | B, 130 ms |
| Two steps earlier, one step longer | 1.06 | 0.92 | A, 365 ms | A, 337 ms | A, 238 ms |

These are reported examples from the full inventory, not adopted calibrations.

## Finer positive delays for the existing commands

| Delay (ms) | A | B | C |
|---:|---:|---:|---:|
| 0 | 10 | 0 | 0 |
| 20 | 9 | 1 | 0 |
| 40 | 6 | 4 | 0 |
| 60 | 5 | 5 | 0 |
| 80 | 4 | 6 | 0 |
| 100 | 0 | 10 | 0 |

Completion is not strictly monotone at this resolution. Selected Scenario 4 has
dwell 180 ms at D=20 ms, 179 ms at D=40 ms, 169 ms at D=60 ms, 197 ms at D=80 ms,
and 145 ms at D=100 ms. Six fresh-snapshot replays (including D=0) reproduce all
recorded measurements exactly. A single monotone completion TTL should not be
inferred from these data.

## Scenario 1 boundary follow-up

Scenario 1's selected approval is already zero, so earlier approval is unavailable
within this mission. Its complete original planner grid contains 38 commands;
all were executed at D=0 before any delayed follow-up. Five pass D=0: approval
0.00 s and low durations 1.00, 1.10, 1.20, 1.30, and 1.40 s.

Every one of those five is tested at each follow-up delay:

| Delay | A | B | C | Best inspection dwell across the five |
|---:|---:|---:|---:|---:|
| 20 ms | 0 | 5 | 0 | 163 ms |
| 100 ms | 0 | 4 | 1 | 91 ms |
| 200 ms | 0 | 3 | 2 | 0 ms |

Thus returning a different feasible command from the existing grid would not
restore success at those three delays for Scenario 1. This is evidence for the
tested original grid, not a proof over every possible continuous timing/duration.
The local inventories for the other scenarios likewise do not establish their
maximum achievable delay tolerance.

## Execution, validation, and artifacts

339 new PyBullet rollouts were executed: 280 in the local/fine-delay inventory,
53 in the Scenario 1 full-grid follow-up, and six exact-reproduction checks.
Every rollout's safe-completion identity and A/B/C classification passed checks.
Current commands and shared delayed outcomes match the previous sweep. The
full local dataset covers all 84 planned candidates, all 78 eligible commands
at both positive delays, and all ten current commands at the fine delays.

The experiment and validation exited successfully. Existing production files
remain unchanged. Added diagnostic source: `diagnose_calibration_sensitivity.py`.

Raw artifacts are under `results/calibration_sensitivity/` (Git-ignored):

* `design.json`, `d0_eligible.json`: design and eligibility frozen before delayed testing.
* `executions.csv`, `executions.jsonl`: all 280 primary rollouts and measurements.
* `summary.json`, `validation.log`: checked aggregates and validation evidence.
* `seed1_full_grid.py`, `seed1_full_grid/`: follow-up source, manifest, eligibility, CSV, and JSONL.
* `replay_seed4.py`, `seed4_replay.json`, `seed4_replay.log`: fresh-snapshot checks.
* `validate_and_report.py`: validation/aggregation script.

From the repository root with the SCG Python environment, reproduce the primary
inventory using the existing cohort CSV:

```sh
python examples/staleguard/inspection_descent/diagnostics/diagnose_calibration_sensitivity.py
```

The result demonstrates that the no-recovery failure curve depends materially
on the chosen calibration and the tested delay grid. It does not establish that
all D=0-feasible commands lose mission success at any positive delay, and it does
not propose a replacement calibration or change StaleGuard.
