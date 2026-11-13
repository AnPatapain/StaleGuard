# Underpass seven-policy benchmark — technical appendix

The [main report](REPORT.md) describes actual physical outcomes. All scores come
from new SCG/PyBullet policy executions. The earlier 2,136-flight deadline study
is an immutable verification/analysis input, never a source of constructed policy
outcomes. No held-out generation, recalibration, PPR or recovery selector is run.

## Architecture and exact code inventory

The canonical organization follows Geofence-Turn and Inspection-Descent:

```text
underpass_inspection/
├── scenario.py                 # original mission; unrestricted nonnegative seed IDs
├── simulation.py               # one physical engine, optional policy scheduling
├── calibration.py              # unchanged
├── pilot.py                    # existing GUI; frozen loader updated
├── storage.py                  # frozen inputs, reviewed source equivalence, I/O
├── baselines/
│   ├── policies.py             # single queued-command scheduler; seven rules
│   ├── staleguard.py           # mission adapter using shared lightweight dynamics
│   ├── use_time_recheck.py     # one immediate-command rollout
│   └── fbs.py                  # mission-specific current-clearance feature
├── baseline_benchmark/
│   ├── run.py
│   └── metrics.py
├── support/
│   ├── test_baselines.py
│   ├── validate_baseline_benchmark.py
│   ├── visualize_policy_comparison.py
│   └── ...                    # original development/GUI utilities unchanged
├── README.md
└── results/development/
    ├── calibration/            # preserved
    ├── deadline_sweep/         # preserved
    ├── pilot_ned_east_v2/      # preserved
    ├── baseline_smoke/
    └── baseline_benchmark/
```

Created: `storage.py`; `baselines/policies.py`, `baselines/staleguard.py`,
`baselines/use_time_recheck.py`, `baselines/fbs.py`; `baseline_benchmark/run.py`,
`baseline_benchmark/metrics.py`; `support/test_baselines.py`,
`support/validate_baseline_benchmark.py`, `support/visualize_policy_comparison.py`.

Modified: `simulation.py` (policy-guarded scheduling/recording), `scenario.py`
(seed-ID validation only), `pilot.py` (reviewed frozen-data loader), `README.md`.
No files in Geofence-Turn, Inspection-Descent, the shared model, PID or safety
monitor were modified. The original calibration and deadline-sweep code and all
their artifacts remain unchanged.

The shared `model/lightweight.py` remains the sole predictive dynamics/PID engine:
50 Hz control, 100 Hz RK4 model integration. Actual physics remains 1,000 Hz.
Geofence supplies the canonical 19-point TTL grid, StaleGuard readiness/expiry
gate, FBS EMA/floor/cap/dispatch rules, fixed 0.20 s timer, Use-Time Recheck
control-tick latency, atomic JSON/journal helpers, journal reading, timing and
paired metrics. Mission-specific adapters change only the Underpass reference,
3D geometry feature and safety test. No compatibility import wrappers or second
root-level prediction implementation were introduced.

## Frozen cohort and physical invariants

All twelve accepted IDs are retained: 1, 2, 3, 6, 8, 9, 11, 12, 14, 16, 17, 20.
Seeds 1 and 8 have negative measured sideways gaps; seed 3 has unobserved command
expiration. None is filtered by prediction or policy outcome. Original calibration
attempted 20 draws and rejected eight; no draw is regenerated or adjusted here.

NED remains North=world X, East=−world Y, Down=−world Z. Actual/model states,
references and obstacle volumes remain Z-up. The original `apply_maneuver`
dispatch-relative dive or independent sideways recipe is used at the actual
physical initiation state. Every recovery chooses `sideways`; climb is not an
alternative in policy code. Body radius, clearance, geometry, profiles, dwell and
eight-second observation period after approval are unchanged. Obstacles remain
visual-only: the existing signed 3D body-clearance predicate detects violations,
not contact forces. Monitoring continues after unsafe entry and inspection dwell.

Every native execution starts from the original reset state and flies through
approval without teleportation or PID reset. The frozen approval state, all three
PID-memory arrays, remaining nominal reference, approval time, horizon and
scenario are verified at approval. Snapshots are read-only; no state restoration
is used to shortcut actual physical flights. State/PID equality is also checked
when the actual maneuver reference changes.

## Seven policy definitions

| Policy | Information observed | Decision/action | Predictor / fixed parameters |
|---|---|---|---|
| No Recovery | Actual dispatch event | Start original dive at dispatch; never recover | None |
| Always Recover | Approval event | Cancel and start sideways immediately at approval, including D=0 | None |
| FBS | Actual scalar clearance feature at each control observation through approval; elapsed age at dispatch | Freeze TTL at approval; recover at dispatch only if ready and expired | EMA alpha 0.1; epsilon 1e-12 m/sample; cap 210 steps; floor margin/EMA |
| Use-Time Recheck | Actual state/PID and previous safety history at dispatch | One immediate-command safety check; continue nominal while computing; then dive or sideways | Same model/checker; one candidate delay 0 |
| StaleGuard | Frozen actual approval state/PID/reference and known geometry; elapsed age; dispatch event | Recover at dispatch if forecast is ready and age reaches predicted first-unsafe delay; otherwise dispatch | Fixed Geofence TTL grid; shared model/checker |
| StaleGuard EarlyRecovery | Exactly the same approval forecast and runtime | While pending, recover on the first control step that is both ready and expired; otherwise same dispatch gate | Same grid/model/forecast as StaleGuard |
| Fixed EarlyRecovery | Approval timer and actual dispatch event | Recover at elapsed 0.20 s if still pending; dispatch only before the trigger | No predictor; 0.20 s; equality recovers |

All policy logic lives in the baseline package. Trivial actions share the queued-
command scheduler rather than creating one-line wrapper modules. The policy object
has no actual scheduled-dispatch-time input. It receives dispatch only when the
driver announces it. At a control step: approval first, then prediction readiness,
actual dispatch, pending timers, maneuver initiation, and physical integration.
Dispatch wins a timer tie, but the dispatch gate uses its own expiry equality.
Always Recover's approval action therefore cancels even a D=0 dispatch. Once a
command or recovery starts, later prediction/timer/dispatch events cannot restart
or override it. `finish()` attaches scheduling metadata only after the flight.

FBS's Underpass feature is required clearance minus minimum current body clearance
to either volume. Positive means unsafe. Its EMA starts at zero on the first
observation; each subsequent update is 0.1 times absolute feature change plus 0.9
times the previous EMA. Approval freezes margin/volatility in control steps,
floored and capped. A constant/no-history feature uses the numerical epsilon and
cap. Margin ≤0 is expired; a positive-margin TTL expires strictly after its last
step. If computation is unavailable at dispatch, FBS permits the command. It does
not forecast the command's dive or pier encounter.

Always Recover deliberately follows this task's approval-time definition; the
current Geofence implementation instead recovers at dispatch. Fixed EarlyRecovery
uses Geofence's helper behavior `>=0.20 s`; an older comment in its physical engine
describes equality differently. The helper and focused equality test are used.
Inspection-Descent's 0.30 s fixed trigger is not mixed into this protocol.

## Prediction, safety and readiness

The approval predictor never receives actual dispatch delay, future PyBullet
states, ground-truth deadlines or policy outcomes. It predicts nominal waiting
once and saves model-state/PID checkpoints at each hypothetical dispatch delay.
For each candidate, it restores that checkpoint and anchors the unchanged dive to
the **predicted** position at that time. It then rolls out the remaining horizon.
An unsafe waiting prefix remains unsafe; unbounded nominal continuation is not
used to reject every Underpass command. Only safety is predicted, not inspection
completion or recovery expiration.

The 19 candidate delays are 0–0.20 s in 0.02 s steps, then 0.30–1.00 s in 0.10 s
steps, exactly Geofence's benchmark grid. Actual dispatch uses all 51 delays.
The first predicted unsafe point is the expiry; all-safe grids have no observed
expiry. Full predicted labels and any unsafe-to-safe return are saved. A coarse
first-invalid cutoff can admit unsafe starts between sampled points; the metrics
separate sampled-model errors from full-grid TTL-admission errors.

The Underpass safety adapter computes minimum signed Euclidean distance between
each straight segment joining predicted control states and each box, minus the
same body sphere radius. Slab-crossing intervals give the quadratic outside
distance; affine face-distance intersections give deepest inside penetration.
This preserves the rounded body-clearance envelope rather than replacing it with
an inflated rectangular volume. The actual 1,000 Hz predicate is unchanged.
Predicted segments approximate model motion between 50 Hz outputs; they are not
an actual 1,000 Hz physics monitor or a real-world certificate.

The historical approval pickle has no stored prefix-minimum field. Approval
prediction starts with current observed clearance; accepted immediate-flight
calibration establishes that its prior prefix was safe. Every native replay still
retains the complete physical safety history. Dispatch-state recheck snapshots
include the actual accumulated minimum, so an earlier physical violation makes
the check unsafe even if subsequent positions clear the obstacle.

One warmed approval forecast per scenario is measured before enumeration of
actual dispatch delays, then shared by both StaleGuard variants. Its computation
duration is replayed on the physical simulation clock starting at approval, as in
Geofence. Wall-clock start/end and duration are stored separately from simulated
start, completion and first usable control step. Startup platform load and shared
kernel warmup are recorded separately; these forecasts assume a warmed system.
First-forecast timing is distinguished from subsequent scenario forecasts.

Readiness allows equality. EarlyRecovery acts at the first actual control step
after both readiness and expiry while the command is still pending. If readiness
comes after expiry, action occurs then, never at an earlier fictitious time.
Dispatch before readiness permits the command and later readiness cannot undo it.
Plain StaleGuard never acts solely on a pending TTL timer.

Use-Time Recheck runs one command forecast at dispatch, then the actual UAV
continues nominal flight until `dispatch + floor(runtime*50) + 1`. The command or
recovery is anchored to that later actual state. The check itself predicts an
immediate command at its observation state, matching the established baseline;
it is not silently upgraded to an end-to-end latency-aware planner. Check-result
accuracy and unsafe outcomes after movement during computation are distinguished.

## Validation and scientific limitations

Twenty-seven focused tests pass, including exact policy equality/readiness cases,
FBS history/EMA, cancellation, unknown-delay interfaces, predicted waiting-state
anchoring, reference immutability, 3D corner/interior clearance, NED direction and
historical physical-source equivalence. One test executes two actual out-of-grid
stress flights at D=2.00 s: No Recovery and Use-Time Recheck. Both retain the
clearance violation encountered while waiting; recheck rejection and later
sideways avoidance cannot erase it. These two diagnostic flights are not scores
in the development matrix and use no synthetic predictor runtime.

The final smoke benchmark has 189 native flights: seeds 1 (negative gap), 6
(long positive) and 14 (short positive), nine shared delays and seven policies.
Its audit confirms all 27 No Recovery and all 27 Always Recover physical hashes
match archived truth, identical actual approval snapshots, full 1,000 Hz horizons,
55 actual early recoveries, cancellation, and no state/PID resets. Of 189 flights,
186 have an applicable exact archived maneuver/start reference and match it;
the remaining recheck starts fall outside the original 0–1 s reference grid.

Smoke resumption stops after seven committed flights, appends an unfinished JSON
tail only to the **new smoke journal**, repairs it on resume, and completes 189
unique records. The original seven records remain byte-for-byte unchanged. No
archived calibration or deadline-sweep journal is corrupted for this test.

The independent unit is the scenario, not each policy/delay cell. Only twelve
accepted development scenarios, one idealized simulator/controller family, no
wind/noise randomization, and visual-only obstacles are tested. Negative physical
gaps, unobserved expirations, model mismatch, coarse-grid admissions, late
readiness, and inspection loss must all remain visible. These results do not
establish real-world safety or guarantee EarlyRecovery dominance.

## Metrics and paired interpretation

Every flight records minimum body clearance, first violation/time/component,
inspection dwell, raw completion, whole-flight safe completion, absolute
approval/dispatch/action steps, forecast/check readiness, actual event order and
physical trajectory hash. All safety violations are irreversible in a record.

`false_safe_decisions` counts ready checks/forecasts that permit a command whose
counterfactual original scheduled-delay No Recovery flight is unsafe.
`false_reject_decisions` counts rejected/canceled commands whose corresponding No
Recovery flight is safe. These use offline truth **only after execution**.
Unavailable unsafe dispatches are separate. `ready_accepts_with_unsafe_actual_outcome`
additionally includes actual unsafe accepted flights caused by application delay,
even when the immediate scheduled-delay command would have been safe. Always and
Fixed recovery are not labelled predictors.

Predictor metrics retain the ground-truth label at every tested delay, TTL-based
admission, and direct model label only where sampled. Scalar TTL error is reported
only when both predicted and physical first-unsafe boundaries are observed;
unobserved cases remain in all outcome denominators. Approval timing samples are
one per scenario, not multiplied by 51 delays or by two StaleGuard policies.
Use-Time Recheck timing is one actual measurement per flight. FBS final-update and
cumulative-observation computation are separated.

Paired metrics compare identical scenario/delay cases: unsafe prevented and
introduced, safe inspections preserved and lost, plus explicit EarlyRecovery
comparisons against StaleGuard, Use-Time Recheck and Fixed EarlyRecovery. All
per-scenario and per-delay results are retained; no unfavorable case is omitted.

## Provenance, artifacts and resumption

Historical manifests and all original snapshots stay unchanged. The loader checks
their hashes plus explicit reviewed syntax-tree equivalence: original simulation
execution with policy-guarded additions removed, mission definition with only its
seed-ID validation removed, and pilot functions outside its CLI. Unknown historical
versions or changes to physical code fail. This follows Geofence's physical-source
equivalence convention; it does not install legacy modules or rewrite provenance.
All new benchmark resumes require exact current source/package/environment hashes.

`run.json` freezes the calibration hash, snapshot hashes, all scenario definitions,
policy parameters, dispatch/prediction grids, primary recovery, numerical platform,
source/environment, warmup and cached forecasts/runtimes. `executions.jsonl` stores
new physical outcomes. `metrics.json`, both CSVs and `REPORT.md` derive from it.
One compressed NPZ retains actual trajectories for three representative seeds at
D=0.92 s, not thousands of per-flight files. The additional validation JSON and
plots are diagnostics, not substituted scores.

Rows are synced and a committed byte-count/hash is saved after each flight.
Resume verifies protocol, sources, snapshots, prior bytes and forecast timing,
repairs only an unfinished trailing record and skips completed keys. A valid
completed row written just before interruption is also retained. Uncommitted
interrupted work may rerun but never duplicates a journal key. Partial reports
identify pending cells. Invocation budgets do not change the frozen matrix.
One simulation worker, one numerical thread and the existing 2 GiB process/host
memory guards remain active. Fresh measured-timing runs can change action timing
by a control tick under different CPU load; resume reuses the recorded durations.

## Reproduction from examples/staleguard

Use the existing `scg` environment; all commands below are directly runnable.

```bash
conda run --no-capture-output -n scg python -m unittest underpass_inspection.support.test_baselines underpass_inspection.support.test_pilot underpass_inspection.support.test_development -v

conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --smoke --out /tmp/underpass_policy_smoke_repro

conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --representative-seeds 1 6 14
conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --representative-seeds 1 6 14 --resume

conda run --no-capture-output -n scg python underpass_inspection/baseline_benchmark/run.py --metrics-only

conda run --no-capture-output -n scg python underpass_inspection/support/validate_baseline_benchmark.py

conda run --no-capture-output -n scg python underpass_inspection/support/visualize_policy_comparison.py --seed 6 --out /tmp/underpass_policy_comparison.png
conda run --no-capture-output -n scg python underpass_inspection/support/visualize_policy_comparison.py --seed 6 --gui --policy staleguard_early_recovery --hold 10
```

For a separate full reproduction use `--out /tmp/underpass_policy_full_repro`.
GUI playback uses live physical dynamics with the recorded policy action/start,
then checks the same trajectory hash; it does not remeasure and retime the policy.
The native NED camera, translucent structures, checkerboard floor and colors are
unchanged. Another frozen cohort can be passed through `--calibration` and
`--ground-truth`; policies do not contain development-seed branches.

Final numeric diagnostics and held-out readiness assessment follow the completed
matrix and its independent artifact audit; no partial result is a readiness claim.

## Final measured diagnostics

| Policy | Raw unsafe inspections | Unavailable unsafe dispatches | Ready accepts with unsafe actual flight | False rejects |
|---|---:|---:|---:|---:|
| No Recovery | 294 | None | None | None |
| Always Recover | 0 | None | None | None |
| FBS | 294 | 0 | 294 | 0 |
| Use-Time Recheck | 11 | 0 | 11 | 2 |
| StaleGuard Dispatch | 26 | 9 | 17 | 0 |
| StaleGuard EarlyRecovery | 26 | 9 | 17 | 0 |
| Fixed EarlyRecovery | 12 | None | None | None |

| Seed | Predicted first unsafe (s) | Physical first unsafe (s) | Error (s) | Runtime (ms) | Ready by predicted expiry |
|---|---:|---:|---:|---:|---|
| 1 | 0.9 | 0.82 | 0.08000000000000007 | 141.6 | True |
| 2 | 0.14 | 0.14 | 0.0 | 158.6 | False |
| 3 | None | None | None | 162.9 | None |
| 6 | 0.4 | 0.36 | 0.040000000000000036 | 141.2 | True |
| 8 | 1.0 | 0.98 | 0.020000000000000018 | 148.6 | True |
| 9 | 0.8 | 0.72 | 0.08000000000000007 | 145.1 | True |
| 11 | 0.08 | 0.1 | -0.020000000000000004 | 157.2 | False |
| 12 | 0.14 | 0.14 | 0.0 | 221.9 | False |
| 14 | 0.9 | 0.86 | 0.040000000000000036 | 149.2 | True |
| 16 | 0.3 | 0.28 | 0.019999999999999962 | 152.4 | True |
| 17 | 0.2 | 0.18 | 0.020000000000000018 | 142.9 | True |
| 20 | 0.8 | 0.76 | 0.040000000000000036 | 158.3 | True |

The main report is editorially clarified from the completed numerical artifacts. `--metrics-only` regenerates the base report and identical numerical tables; this detailed appendix remains preserved.

## Final paired comparisons against No Recovery

| Policy | Unsafe prevented | Unsafe introduced | Safe inspections preserved | Safe inspections lost |
|---|---:|---:|---:|---:|
| always_recover | 294 | 0 | 0 | 318 |
| fbs | 0 | 0 | 318 | 0 |
| use_time_recheck | 216 | 9 | 307 | 11 |
| staleguard | 204 | 0 | 318 | 0 |
| staleguard_early_recovery | 261 | 0 | 318 | 0 |
| fixed_early_recovery | 282 | 0 | 108 | 210 |

The complete native audit passes: 4,284 unique cells, 612 No Recovery and 612 Always Recover physical hash matches, 4,272 applicable archived references matched, 612 paired StaleGuard admission/forecast checks, and 21 retained physical arrays independently verified. The other 12 recheck initiation times lie beyond the archived reference grid. All 110 pre-existing results remain unchanged. Metric regeneration leaves both the journal and NPZ unchanged and regenerates metrics.json identically. No actual pre-dispatch violations occurred inside the 0–1 s matrix; the two actual D=2 s stress tests establish retention of earlier violations.

Native GUI playback of seed 6 at scheduled D=0.92 s was inspected: [GUI screenshot](gui_seed6_early.png), [seven-policy actual-flight plot](comparison_seed6.png). The recorded EarlyRecovery starts sideways at approval +0.40 s, remains safe and repeats the physical hash exactly. North projects upper-right, East lower-right and Down downward; the camera/floor are unchanged.
