# Independent six-policy Experiment B: seeds 20–319

This is the locked independent evaluation, not a further parameter search.
Exactly 300 scenarios are attempted. Seeds 0–19 and their calibrated commands
remain development data. All new outputs are isolated in
`inspection_descent/independent_evaluation_20_319/`.

## Frozen scientific configuration

- Seeds 20–319 inclusive, with the existing scenario-generation distributions.
- The unchanged official D=0-only maximum-dwell calibration, candidate grid,
  order, first-on-tie rule and 180 ms feasibility requirement. Infeasible seeds
  remain in the calibration denominator and are never replaced.
- No Recovery, Always-Recover, StaleGuard, Use-Time Recheck, FBS and Fixed TTL
  **0.3 s**. Existing fail-open rules and safety-only fallback are retained.
- Dispatch delays 0.0–1.0 s in 0.1 s steps; original command duration; original
  deadline of approval + 4.2 s; unchanged PID, dynamics, inspection predicate
  and closed unsafe region; actual PyBullet monitoring at 1,000 Hz.
- StaleGuard predicts once per approval before the delay loop. Use-Time
  Recheck predicts once from the live dispatch snapshot, using its actual
  computation time to delay application while simulated nominal PID/physics
  continue. FBS observes at 50 Hz through approval, then freezes its TTL:
  alpha=0.1, v[0]=0, epsilon=1e-12 m/sample and maximum TTL=210 steps.

The generated `protocol.json` and `preserved_files.json` freeze the settings,
scientific source files and existing development artifacts. A mismatch stops
the run. No method or parameter is changed using independent outcomes.

## Implementation and incremental execution

`independent_evaluation.py` is a serial driver around the existing code. It
calls `max_dwell_calibration.plan_max_dwell_command()` directly. A streaming
physics sink computes the same elapsed-tick inspection dwell, avoiding a full
trajectory list. Candidate outcomes are durably journaled. On restart, the
official planner consumes the recorded prefix without repeating simulations,
then continues its unchanged selection loop.

The full run completes all 300 D=0 calibrations before its full operational
delay grid. The required preliminary smoke executions use seeds 20 and 21;
their calibrated commands, measured predictions and completed flights remain
in the final cohort. Smoke outcomes are not used to tune anything.

Each feasible scenario has an approval snapshot, one measured StaleGuard
prediction and a per-flight journal. Each journal entry is flushed and
fsynced. An interrupted partial final entry is discarded on restart; complete
entries are retained. Resources are released between simulations. No full
physics or control trajectories accumulate in RAM; streaming hashes and a few
approval/dispatch observations support validation.

No Recovery, StaleGuard, Use-Time Recheck and FBS execute their native policies
at every delay. Identical nonpredictive trajectories may be reused:

- Always-Recover at D=0 is the immediate original execution. At positive D,
  recovery begins at approval and its nominal forward/altitude trajectory is
  independent of D. One native positive-delay execution supplies that path.
- Fixed TTL accepts the matching original trajectory or cancels at dispatch
  onto the same nominal fallback. Native checks at D=0.3 and 0.4 for **every
  feasible scenario** verify the inclusive boundary and dispatch-time fallback,
  including full control/physics hashes against the references.

Reused records identify their source. If a flight terminates before dispatch,
the reuse adapter preserves the corresponding absence of a dispatch action.
Prediction timings are never fabricated or substituted from development data.

## Statistics fixed before evaluation

The eleven delays of a scenario are a cluster. A 10,000-replicate percentile
bootstrap (fixed analysis seed 1729) resamples feasible scenarios, carrying
all delays, all six policies and all outcome metrics together. Aggregate and
per-delay rates and paired StaleGuard-minus-baseline differences receive 95%
intervals. Outcome rates condition on calibration feasibility; calibration
feasibility itself has a Wilson 95% interval over all 300 attempts.

Exact paired sign tests use scenario-level outcome differences, with ties
excluded and all-tie p=1. Aggregate tests use each scenario's count across all
eleven delays. Holm correction covers the 180 reported comparisons jointly:
five baselines × three metrics (unsafe, safe completion, fallback frequency)
× twelve strata (aggregate and eleven delays). These tests concern the sign
of scenario-level differences; the bootstrap intervals concern mean rate
differences. Positive StaleGuard-minus-baseline differences mean more of the
named metric, not automatically better performance.

Percentile intervals can be degenerate when no events are observed. A separate
scenario-any-unsafe Wilson interval is also reported; zero observed unsafe
entry does not establish zero risk. Scenario-clustered mean-computation-time
intervals accompany measured mean/median/p95 timing summaries.

StaleGuard/FBS readiness is measured at scheduled dispatch. Use-Time Recheck
withholds the command and applies its result at the first post-computation
control step; its application readiness and decision delay are labeled
separately. Serial measured-time simulation is not a concurrent onboard
implementation or a real-time controller scheduling benchmark.

## Validation before the detached launch

The 26-test suite passed. Tests cover the existing policies plus streaming
dwell equivalence, incomplete-journal recovery, candidate-level calibration
resume, first-on-tie selection, infeasibility, paired cluster resampling,
exact sign tests, Holm adjustment and the full six-policy report schema.

The new-scene smoke test calibrated seeds 20 and 21 and executed all six
policies at D=0, 0.3 and 0.4 s. It deliberately interrupted and resumed after
a calibration candidate and a flight checkpoint, then resumed the completed
smoke set again and checked that its files were unchanged. Both scenarios
were feasible, giving 36 native smoke flights. Development preservation checks
passed. The smoke data are retained, not rerun or excluded from the final set.

## Overnight lifecycle and resources

`launch_independent_evaluation.sh` launches a host process using
`nohup setsid systemd-inhibit --what=sleep --mode=block`. The inhibitor encloses
the supervisor, Python calibration/evaluation, statistical analysis, figure
generation and final status write. It remains active until the job exits.
No sudo or system-wide power-setting change is used.

There is exactly one simulator worker, with CPU-only/headless execution and
one numerical-library thread. The existing guards require at least 2 GiB of
available host memory and at most 2 GiB process RSS. They run between
simulations and periodically in streamed physics monitoring. A triggered
guard stops safely with FAILURE and durable checkpoints; it is not disabled
or automatically forced past the limit.

The command sandbox initially reported insufficient memory and stopped the
preflight before simulator loading. The successful smoke and overnight job
run on the host, where process IDs, memory and sleep inhibition can be
verified independently of Codex's short-lived command sandbox.

The output directory contains:

- `run.log`: complete background-process progress and failures.
- `status.json`: authoritative RUNNING, SUCCESS or FAILURE status and phase.
- `worker.pid`, `supervisor.pid`: process IDs.
- `scenarios/SEED/calibration_candidates.jsonl`, `calibration.json`: D=0
  candidate checkpoints and selection/infeasibility.
- `scenarios/SEED/approval.pkl`, `reception.json`, `flights.jsonl`: immutable
  approval/prediction checkpoints and incremental execution records.
- `smoke_validation.json`, `unit_tests.log`: pre-launch validation.

After successful completion, the program automatically writes:

- `six_policy_report.log`: required six-column per-delay terminal report and
  all five paired StaleGuard comparison sections.
- `summary.json`, `aggregate.csv`, `per_delay.csv`: outcomes and 95% intervals.
- `paired_comparisons.json`, `paired_comparisons.csv`: paired differences,
  cluster intervals, scenario-level tests and Holm-adjusted p-values.
- `calibration_feasibility.json`, `calibrated_scenarios.json`.
- `computation_times.json`: computation distributions, intervals, readiness
  and Use-Time application delay.
- `executions.jsonl`: merged machine-readable policy executions and reuse
  provenance.
- `safety_completion_tradeoff.png` and `.pdf`.
- `interpretation.txt`: at most 15 lines of descriptive interpretation.
- `SUCCESS`, or `FAILURE` on an unsuccessful run. `status.json` is authoritative
  during restarts; prior failures are retained in `failure_history.jsonl`.

From `examples/staleguard`:

```sh
# Check current status without supervising continuously.
cat inspection_descent/independent_evaluation_20_319/status.json

# Resume after an interruption; retains all durable work and prediction times.
bash inspection_descent/launch_independent_evaluation.sh
```

The launcher refuses a duplicate live worker or a missing smoke-validation
record. It starts only this locked evaluation, with no imperfect-observation
or second-mission experiments.
