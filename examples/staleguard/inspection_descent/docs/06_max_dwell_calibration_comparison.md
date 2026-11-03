> Source relocation: this report preserves its historical findings and commands. Current code: `max_dwell_calibration.py` → [calibration.py](../calibration.py). Old entry points have been removed.

# Stage 6: first-feasible versus maximum-dwell calibration

Historical comparison, before maximum-dwell calibration became official.
The statement below that the alternative had not replaced the default describes
that stage. Official integration followed at stages 8–9; see the
[recap](../recap_visibility.md).

The alternative rule produces positive-delay safe inspection completions on both the original scenarios and fresh scenarios, without using delayed outcomes to select commands. It also produces more unsafe entries at several delays. It is a defensible separate rule for maximizing the D=0 inspection objective, but these results do not establish a generally safer calibration policy.

Only the No Recovery policy was executed. Every command follows the existing continuous-forward trajectory and existing descent/return behavior. T=4.2 s remains relative to the selected approval time. StaleGuard, recovery, scenario generation, PID, physics, inspection/safety predicates, and the original first-feasible planner are unchanged. The alternative has not replaced the default planner.

The rule and evaluation procedure were fixed before delayed testing:

1. Generate the original ten scenarios (seeds 0–9) and the next ten fresh scenarios (seeds 10–19). A seed identifies a physical scenario; the many candidate commands below are different approval times and low-altitude durations for that same scenario.
2. Use the complete existing planner grid, in exactly its original order. Approval times and low-altitude durations are the only candidate fields that vary. The existing structure-arrival bound and grid increment are retained. A structural check against the original planner confirmed the exact candidate list and order for all 20 seeds.
3. Execute every candidate at D=0 with recovery disabled. Measure its longest consecutive inspection dwell as elapsed integer physics ticks, using the existing inspection predicate. Unsafe candidates are ineligible even if they completed inspection earlier.
4. First-feasible selects the first safe mission completion in that inventory. Maximum dwell selects the greatest dwell among all D=0-safe candidates; a tie retains the first candidate in the original search order. It returns no calibrated command if the best safe dwell is below the unchanged 180 ms inspection requirement. This feasibility requirement is identical for both policies.
5. Complete all 20 inventories and freeze both policies’ selections before any positive-delay evaluation. No positive-delay result, predictor output, TTL, or recovery outcome enters calibration. Code/protocol hashes and a frozen-selection hash are saved with every trial.
6. Execute each frozen command at all 16 existing delays: 0.0 through 1.5 s, in 0.1 s increments. Report infeasible scenarios separately; they have no issued calibrated command and are not relabeled as unsafe executions.

There were 2,241 D=0 candidate executions and 608 subsequent command executions across the delay grid. Calibration was partitioned by seed across independent processes; each trial used the unchanged deterministic simulation settings. The default driver can reproduce the same procedure sequentially.

Calibration feasibility:

| Scenario group | Scenarios with a feasible calibration, both rules | D=0 command trials | D=0 A/B/C among candidate commands | Mean selected dwell, first → maximum |
|---|---:|---:|---|---|
| original seeds 0–9 | 10/10 | 1,316 | 490/522/304 | 215.3 → 386.0 ms |
| fresh seeds 10–19 | 9/10 | 925 | 327/345/253 | 224.6 → 415.9 ms |

Fresh seed 16 is infeasible: all 29 existing-grid candidates were executed, and the greatest dwell among its 10 safe candidates was 141 ms, below 180 ms. Therefore the fresh delay results contain nine actual executions per policy at each delay, plus one calibration failure out of the ten attempted scenarios. No scenario was replaced. For example, seven fresh A outcomes mean 7/9 of calibrated scenarios and 7/10 of all attempted scenarios.

A = no unsafe entry and inspection completed. B = no unsafe entry but inspection failed. C = unsafe entry, regardless of inspection completion. Each cell below is A/B/C.

| D (s) | Original: first | Original: maximum dwell | Fresh: first | Fresh: maximum dwell |
|---:|---|---|---|---|
| 0.0 | 10/0/0 | 10/0/0 | 9/0/0 | 9/0/0 |
| 0.1 | 0/10/0 | 9/1/0 | 0/9/0 | 7/1/1 |
| 0.2 | 0/10/0 | 6/3/1 | 0/9/0 | 6/2/1 |
| 0.3 | 0/9/1 | 3/5/2 | 0/7/2 | 2/3/4 |
| 0.4 | 0/6/4 | 2/2/6 | 0/5/4 | 1/2/6 |
| 0.5 | 0/4/6 | 2/0/8 | 0/4/5 | 1/2/6 |
| 0.6 | 0/3/7 | 1/0/9 | 0/2/7 | 0/1/8 |
| 0.7 | 0/1/9 | 0/0/10 | 0/1/8 | 0/0/9 |
| 0.8 | 0/0/10 | 0/0/10 | 0/1/8 | 0/0/9 |
| 0.9 | 0/0/10 | 0/0/10 | 0/0/9 | 0/0/9 |
| 1.0 | 0/0/10 | 0/0/10 | 0/0/9 | 0/0/9 |
| 1.1 | 0/0/10 | 0/0/10 | 0/0/9 | 0/0/9 |
| 1.2 | 0/0/10 | 0/0/10 | 0/0/9 | 0/0/9 |
| 1.3 | 0/0/10 | 0/0/10 | 0/0/9 | 0/0/9 |
| 1.4 | 0/0/10 | 0/0/10 | 0/1/8 | 0/0/9 |
| 1.5 | 0/1/9 | 0/0/10 | 0/1/8 | 0/0/9 |

For every delay and policy, A+B+C=10 in the original group. In the fresh group, A+B+C=9 and the separately reported calibration failure count is 1, accounting for all ten attempted scenarios.

At D=0.1 s, safe completions increase from 0/10 to 9/10 on the original scenarios, and from 0/9 to 7/9 on feasible fresh scenarios. At D=0.2 s the corresponding changes are 0/10 to 6/10 and 0/9 to 6/9. These outcomes were obtained with the commands selected using D=0 alone and without recovery. Across the positive-delay grid, maximum dwell produces 23 A executions on the original group and 17 on the fresh group; first-feasible produces zero on both. The largest tested successful delays are 0.6 s and 0.5 s respectively. These are observed grid outcomes, not certified delay tolerances or evidence of monotonic success between grid points.

The safety tradeoff is material. At D=0.2 s, original-group unsafe entries rise from 0/10 to 1/10. At D=0.1 s, fresh-group unsafe entries rise from 0/9 to 1/9. On fresh seed 10, first-feasible chooses approval 0.42 s and low duration 0.92 s; maximum dwell chooses 0.22 s and 1.32 s. Thus its nominal return-to-usual-height reference starts at 1.54 s rather than 1.34 s. Both are safe at D=0, but at D=0.1 s the first command is B and the maximum-dwell command is C. Maximizing inspection dwell does not constrain safety at delayed dispatch.

The evidence supports the original diagnosis: stopping at the first passing command often leaves little inspection-time margin. Searching the same existing grid for a longer D=0 dwell creates delayed successes without selecting for delayed success, and this result repeats on new seeds. It does not repair an infeasible scenario domain, guarantee delayed safety, or show that longer D=0 dwell is equivalent to maximum delay tolerance. Original seed 1 keeps exactly the same command and 188 ms dwell under both rules; fresh seed 16 remains infeasible. Ten fresh seeds are a limited validation sample.

All outcome labels retain the existing finite, approval-relative deadline. They make no claim about safety after that deadline. The nominal forward reference is unchanged. Across the 38 selected-command D=0 replays, measured forward velocity at the 50 Hz control states stayed positive after the existing acceleration ramp (minimum 0.067371 m/s), and stayed positive inside the inspection region (minimum 0.354098 m/s). No stop or hover command was introduced.

Selected commands (approval time / low duration / consecutive D=0 dwell):

| Seed | Candidates tried | Safe inspection-completing candidates | First: seconds / seconds / ms | Maximum dwell: seconds / seconds / ms |
|---:|---:|---:|---|---|
| 0 | 30 | 14 | 0.12 / 0.76 / 213 | 0.00 / 0.86 / 261 |
| 1 | 38 | 5 | 0.00 / 1.00 / 188 | 0.00 / 1.00 / 188 |
| 2 | 297 | 120 | 1.26 / 0.82 / 240 | 0.06 / 2.02 / 487 |
| 3 | 369 | 170 | 1.44 / 0.88 / 225 | 0.14 / 2.28 / 579 |
| 4 | 101 | 34 | 0.62 / 0.88 / 187 | 0.22 / 1.38 / 388 |
| 5 | 72 | 14 | 0.20 / 1.04 / 214 | 0.10 / 1.14 / 326 |
| 6 | 68 | 27 | 0.40 / 0.80 / 238 | 0.20 / 1.00 / 420 |
| 7 | 175 | 53 | 0.70 / 0.90 / 211 | 0.40 / 1.30 / 392 |
| 8 | 114 | 29 | 0.26 / 0.96 / 191 | 0.06 / 1.36 / 470 |
| 9 | 52 | 24 | 0.50 / 0.92 / 246 | 0.20 / 1.12 / 349 |
| 10 | 87 | 22 | 0.42 / 0.92 / 238 | 0.22 / 1.32 / 393 |
| 11 | 289 | 135 | 1.08 / 0.84 / 227 | 0.58 / 1.54 / 541 |
| 12 | 83 | 23 | 0.16 / 1.00 / 185 | 0.00 / 1.30 / 407 |
| 13 | 98 | 38 | 0.48 / 0.96 / 270 | 0.28 / 1.26 / 500 |
| 14 | 75 | 28 | 0.46 / 0.86 / 241 | 0.26 / 1.16 / 400 |
| 15 | 49 | 7 | 0.08 / 1.04 / 201 | 0.00 / 1.14 / 284 |
| 16 | 29 | 0 | infeasible | infeasible |
| 17 | 72 | 19 | 0.30 / 0.86 / 231 | 0.20 / 1.06 / 383 |
| 18 | 76 | 27 | 0.30 / 0.88 / 212 | 0.00 / 1.28 / 420 |
| 19 | 67 | 28 | 0.24 / 0.96 / 216 | 0.04 / 1.16 / 415 |

Implementation and artifacts:

- [max_dwell_calibration.py](../max_dwell_calibration.py) implements the separate D=0-only rule and measures elapsed dwell with the unchanged inspection predicate.
- [compare_calibration_policies.py](../diagnostics/compare_calibration_policies.py) constructs the two fixed scenario groups, records the full D=0 inventories, freezes selections, executes the full delay grid, exports per-execution flags/command parameters, and checks the per-execution outcome identities and per-delay counts.
- [outcomes.csv](../results/max_dwell_policy/outcomes.csv): all 608 policy/delay executions, including scenario parameters, flags, dispatch/deadline steps, selected candidate ranks, hashes, and D=0 replay measurements.
- [calibration_choices.csv](../results/max_dwell_policy/calibration_choices.csv): every selected command and an explicit infeasibility row for seed 16 under each policy.
- [evaluation.log](../results/max_dwell_policy/evaluation.log): actual terminal output from the full delay evaluation.
- [verification.log](../results/max_dwell_policy/verification.log) and [verify_results.py](../results/max_dwell_policy/verify_results.py): independent audit, including exact equality with all 160 prior original first-feasible executions.
- [protocol.json](../results/max_dwell_policy/protocol.json), [frozen_selections.json](../results/max_dwell_policy/frozen_selections.json), and per-seed candidate JSONL files record the code, fixed design, all candidate outcomes, eligibility, and exact selection/tie decisions.

The audit checked each candidate’s dwell/completion identity and safety, each selected command’s exhaustive maximum and tie order, every delay execution’s safe_completion = inspection_completed AND NOT unsafe_entry, no recovery triggers, complete grid coverage, unchanged non-calibration scenario fields, both policies’ respective commands, and every A/B/C denominator. All 20 D=0 inventories were complete before the selections were frozen and positive-delay execution began. The 38 D=0 replay dwells exactly reproduce their calibration measurements. The production mission and StaleGuard files have no modifications.

Run from examples/staleguard using the existing scg environment:

```sh
MPLCONFIGDIR=/tmp/staleguard-mpl /home/kean/miniconda3/envs/scg/bin/python inspection_descent/diagnostics/compare_calibration_policies.py --output-dir inspection_descent/results/max_dwell_policy_rerun
```

The prepare/calibrate/evaluate stages and --resume option support the same calculation with per-seed checkpoints. The generated results remain under the already git-ignored results/ directory; source files and this report are separate from those generated artifacts.

Protocol SHA-256: `4fa0b2a684cac58ebd86410236cb0729f4a90d76d76c002e52103e5c68941133`. Frozen selections SHA-256: `9dcd85a514562be16c70c4e17553a050f4495070a2119ff20ef6c5e29dddaf54`.
