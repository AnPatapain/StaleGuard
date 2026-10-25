# Geofence Turn: end-to-end scenario calibration

Calibration finds the latest approval time at which the UAV can patrol east, execute its fixed-radius left turn, and safely pass through the northbound checkpoint with immediate dispatch, **D = 0**. Speed and radius are sampled once per scenario. Only approval time is searched.

Characterization and calibration use **different missions**. A successful turn in the earlier roomy test does not establish success with a newly sampled fence and checkpoint. In saved development scenario 0, the new search first succeeded at **2.00 s**, after testing six approval times. Characterization of that same speed–radius pair had succeeded at **1.20 s**. The new search does not start from that earlier success.

This report explains current code using saved actual PyBullet results. It runs no new experiments and does not claim the requested 300-scenario run has finished. Acceptance establishes D=0 success; delayed-dispatch performance remains to be evaluated separately.

## 1. Load the previously measured turning combinations

Characterization tested nine speed–radius pairs in a fixed setup: approval at 1.2 s, altitude 1.25 m, fence at X = 10 m, and a checkpoint along the intended route. Eight pairs passed **safe completion plus a 0.15 m tracking-error limit**.

Actual code: [calibration.py:256](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:256).

```python
deadline = round((cmd.arc_s+config.checkpoint_north_travel_s+1.5)*50)/50
s = Scenario(42, speed, 1.25, 0., 0., 10., deadline, cmd, approval)
result, _, trace = execute(s, save_trace=True, config=config, work=work)
feasible = result['safe_completion'] and result['max_tracking_error_m'] <= config.tracking_limit_m
```

In `Scenario(...)`, `10.` is the fence X coordinate. This is an empirical test under those conditions, not a guarantee for every new mission. Normal calibration loads its saved results rather than rerunning characterization.

## 2. Generate one new, fixed mission

For each seed, choose a speed from the passing results and then a passing radius for that speed:

Actual code: [scenario.py:179](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:179).

```python
speeds = sorted(set(r['speed'] for r in feasible))
speed = float(rng.choice(speeds))
radii = sorted(set(r['radius'] for r in feasible if r['speed'] == speed))
radius = float(rng.choice(radii))
```

Sample the initial position, altitude, fence, checkpoint and deadline once. Reject impossible checkpoint geometry before simulation. **Keep all these quantities fixed while searching approval time.** Deadline duration T is fixed relative to approval; the absolute monitoring endpoint is approval + T. The new mission does not inherit characterization's fence, checkpoint or approval time. Appendix A gives the sampling code.

## 3. Find a late starting point for the new search

Fly the new scenario east **without turning**. Record its first actual clearance violation, then round upwards to the 0.20 s approval grid:

Actual code: [calibration.py:395](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:395).

```python
nominal,_,_ = execute(replace(s,deadline_s=probe_s-s.approval_s),None,config=config,work=work)
# ... intervening code omitted ...
upper = math.ceil(nominal['first_violation_s']/config.approval_grid_s)*config.approval_grid_s
selected,snapshot,rows = calibrate(s,evaluate,config,audit,latest_approval_s=upper)
```

`None` means no turn is dispatched. The probe temporarily uses a longer monitoring duration; turn candidates retain their sampled T. The script stops with an error if the probe observes no violation. This probe supplies a late bound, not a successful turn to modify.

## 4. Move approval earlier until this mission safely completes

Each test starts a fresh flight from the initial state. The UAV patrols east until approval, turns immediately at D=0, then flies north. Approval times descend in 0.20 s steps to the earliest allowed time, 0.60 s:

Actual code: [calibration.py:30](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:30).

```python
def approval_candidates(s, config=Config(), latest_approval_s=None):
    lo = round(config.approval_min_s*HZ)
    hi = round((config.approval_max_s if latest_approval_s is None else latest_approval_s)*HZ)
    stride = round(config.approval_grid_s*HZ)
    if lo < round(RAMP_S*HZ) or stride < 1:
        raise ValueError('Invalid post-ramp calibration grid')
    # Exhaustively descending finite grid; first feasible is its LATEST member.
    return [replace(s, approval_s=step/HZ) for step in range(hi, lo-1, -stride)]
```

Success requires at least **0.05 m body clearance throughout the flight** and **0.20 s continuously inside the checkpoint**, with velocity within **0.12 m/s** of the intended northbound velocity, before T expires. Reject either failure; stop at the first test satisfying both:

Actual code: [calibration.py:52](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:52).

```python
for candidate in approval_candidates(s, config, latest_approval_s):
    result, snapshot, trace = evaluate(candidate)
    reasons = rejection_reasons(result)
# ... intervening code omitted ...
    if not reasons:
        return candidate, snapshot, rows
return None, None, rows
```

Every later grid time has then failed, so this is the **latest feasible approval on the grid**. Earlier times cannot improve patrol distance. This does not assume earlier turns always work: they can miss the checkpoint too. If no time succeeds, reject the scenario without retuning its radius or moving its checkpoint. Appendix B gives the actual predicates.

## 5. Read the measured example correctly

These are saved PyBullet observations for speed **0.8 m/s** and radius **0.6 m**:

| Quantity | Characterization | New development scenario 0 |
|---|---:|---:|
| Initial X | 0.0000 m | −0.1377 m |
| Fence X | 10.0000 m | 2.1756 m |
| Checkpoint X interval | 1.1000–1.7000 m | 1.4048–2.0048 m |
| Successful approval | 1.20 s, fixed test input | 2.00 s, selected by search |

Scenario 0's no-turn probe violated clearance at **2.941 s**, so its new search began at **3.00 s**:

| Approval (s) | Entire flight safe? | Checkpoint completed? | Search action |
|---:|---|---|---|
| 3.00 | No | No | Try earlier |
| 2.80 | No | No | Try earlier |
| 2.60 | No | No | Try earlier |
| 2.40 | No | No | Try earlier |
| 2.20 | No | No | Try earlier |
| 2.00 | Yes | Yes | Accept and stop |

At 2.00 s, minimum body clearance was **0.1528 m**, above the required 0.05 m. The search moved backwards **from 3.00 s, not from characterization's 1.20 s**. Steps 3–4 show the code implementing this search.

Evidence: [characterization results](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/support/logs/northbound_review/characterization/envelope.json), [scenario 0](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/support/logs/northbound_review/development/seed_0/scenario.json), [probe](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/support/logs/northbound_review/development/seed_0/nominal_bound.json), and [approval trials](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/support/logs/northbound_review/development/seed_0/candidates.jsonl).

## 6. Verify, save, and process the next scenario

Repeat the selected flight. Require the approval-time UAV state and PID memory to reproduce exactly, and safe completion to pass again. Then save the accepted scenario and its approval snapshot reference:

Actual code: [calibration.py:408](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:408).

```python
replay,replayed,trace = execute(selected,save_trace=detailed,config=config,work=work)
np.testing.assert_array_equal(snapshot.state,replayed.state)
for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
    np.testing.assert_array_equal(getattr(snapshot.pid_state,name),getattr(replayed.pid_state,name))
assert replay['safe_completion'],replay
data['scenarios'].append(dict(scenario=scenario_dict(selected),
    approval_snapshot=str(snapshot_path.relative_to(out)),snapshot_sha256=file_hash(snapshot_path),
    d0_verification=replay))
```

Scenario 0 used **eight flights**: one probe, six approval tests and one verification. Continue through seeds in order until 300 scenarios are accepted or the 500-candidate pool is exhausted:

Actual code: [calibration.py:341](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:341).

```python
for generated in data['progress']['generated']:
    if len(data['scenarios']) >= args.target:
        break
    if generated['seed'] in completed:
        continue
# ... intervening code omitted ...
data['calibration']['status'] = 'complete' if len(data['scenarios']) >= args.target else 'exhausted'
data.pop('progress')
checkpoint()
```

The output is `calibrated_scenarios.json` and necessary snapshots. No trajectory files or baseline experiments are produced by default. Appendix C explains saved data and counts; D explains automatic bounded batches and errors; E explains logs and elapsed time. The accepted dataset becomes a frozen input for the separately specified delayed-dispatch evaluation.

---

# Technical appendix

The main workflow above explains the mission and selection rule. This appendix provides the remaining implementation details and commands. All Python excerpts are copied from the current source. Where separate portions are combined, a comment explicitly marks omitted intervening code; the excerpts are not standalone programs.

## A. Fixed mission generation and the physical path

The generator reads the passing characterization rows and checks that their configuration matches the current configuration:

Actual code: [scenario.py:173](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:173).

```python
feasible = [r for r in envelope['results'] if r['feasible']]
if not feasible:
    raise ValueError('No empirically feasible turning pair')
if envelope['config'] != asdict(config):
    raise ValueError('Envelope/config mismatch; characterize again')
rng = np.random.default_rng(seed)
```

The fence lies 1.5–2.5 m east of the sampled initial position. The following code samples the initial conditions and rejects a checkpoint that cannot fit. The geometric test is a preliminary screen, not a guarantee that a real flight can reach the checkpoint.

Actual code: [scenario.py:183](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:183).

```python
height = float(rng.uniform(1.15, 1.40))
initial_x = float(rng.uniform(-.15, .15))
initial_y = float(rng.uniform(-.10, .10))
# Same provisional patrol-space envelope as the original development run.
fence_x = initial_x + float(rng.uniform(1.5, 2.5))
# A fixed-width checkpoint must fit wholly inside the body-safe half-space,
# and overlap lanes reachable by a turn after the acceleration ramp. This is
# a geometric sampling constraint, not an approval-time or policy adjustment.
width = config.checkpoint_width_m
earliest_lane = initial_x + speed*(config.approval_min_s-RAMP_S/2) + radius
latest_center = fence_x - BODY_RADIUS_M - config.d_min_m - width/2
earliest_center = earliest_lane + width/2
if earliest_center > latest_center:
    raise ValueError(f'checkpoint_geometry: seed={seed} speed={speed} radius={radius} '
                     f'fence_x={fence_x} earliest_center={earliest_center} '
                     f'latest_center={latest_center}; no fixed-width region fits')
```

Checkpoint Y is the initial Y plus one turn radius plus two seconds of northbound travel. Checkpoint width is 0.60 m in X and 0.20 m in both Y and Z. The deadline duration is arc duration + 2 s + a sampled 1–2 s allowance, rounded to the 50 Hz controller grid. These quantities are constructed before approval calibration:

Actual code: [scenario.py:199](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:199).

```python
center_x = float(rng.uniform(earliest_center, latest_center))
center_y = initial_y + radius + speed*config.checkpoint_north_travel_s
tolerance = config.position_tolerance_m
command = Command(radius, speed,
                  (center_x-width/2, center_y-tolerance, height-tolerance),
                  (center_x+width/2, center_y+tolerance, height+tolerance))
# Freeze the deadline on the existing 50 Hz grid before any calibration.
deadline = round((command.arc_s + config.checkpoint_north_travel_s
                  + float(rng.uniform(1., 2.)))*HZ)/HZ
return Scenario(int(seed), speed, height, initial_x, initial_y, fence_x, deadline, command)
```

The initial eastbound reference ramps speed over 0.40 s. Once dispatched, the requested path is a quarter-circle followed by straight northbound travel. The checkpoint does not steer the trajectory. The scalar path function expresses that geometry directly:

Actual code: [scenario.py:101](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:101).

```python
x0, y0, z0 = np.asarray(anchor)[POS]
if t <= cmd.arc_s:
    theta = max(t, 0) * cmd.speed / cmd.radius
    return (np.array([x0 + cmd.radius*np.sin(theta),
                      y0 + cmd.radius*(1-np.cos(theta)), z0]),
            np.array([cmd.speed*np.cos(theta), cmd.speed*np.sin(theta), 0]))
end = np.array([x0 + cmd.radius, y0 + cmd.radius, z0])
velocity = np.array([0., cmd.speed, 0.])
return end + velocity * (t - cmd.arc_s), velocity
```

`apply_turn_command()` implements the same geometry using array operations in `scenario.py:112`. Its anchor is the nominal reference at dispatch, not the measured UAV position. The PID controller tracks that reference in PyBullet. Calibration tests the actual resulting positions and velocities.

## B. What the candidate flight must satisfy

The thresholds below come from these actual configuration defaults:

Actual code: [scenario.py:17](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:17).

```python
class Config:
    # Provisional physical requirements, frozen before the characterization/run.
    d_min_m: float = .05
    position_tolerance_m: float = .10
    velocity_tolerance_mps: float = .12
    dwell_s: float = .20
    # Provisional characterization acceptance bound, not a safety margin.
    tracking_limit_m: float = .15
    checkpoint_width_m: float = .60  # Provisional fixed east-west extent.
    checkpoint_north_travel_s: float = 2.0  # Preselect north distance = speed * 2 s.
    approval_min_s: float = .60
    approval_max_s: float = 3.20
    approval_grid_s: float = .20
    max_rss_mb: int = 2048
    min_available_mb: int = 2048
```

The call in the approval evaluator uses the simulator's existing defaults: zero dispatch delay and No Recovery. Thus a failed flight cannot be rescued by a policy during calibration.

Actual code: [calibration.py:380](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:380).

```python
result,snapshot,trace = execute(candidate,config=config,work=work)
if result['safe_completion']:
```

Actual code: [simulation.py:71](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/simulation.py:71).

```python
def execute(s, delay=0., config=Config(), policy='no_recovery', received=None,
            platform=None, save_trace=False, work=None):
# ... intervening code omitted ...
approval = round(s.approval_s * HZ)
total = approval + math.ceil(s.deadline_s * HZ - 1e-10)
dispatch = None if delay is None else approval + round(delay * HZ)
```

Body clearance is distance from the fence plane to the simulated UAV's enclosing sphere, with radius approximately 0.0613 m. The required additional clearance is 0.05 m:

Actual code: [scenario.py:136](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:136).

```python
def clearance(x, fence_x):
    """Signed BODY-to-plane distance. Apply d_min separately, exactly once."""
    return fence_x - np.asarray(x) - BODY_RADIUS_M
```

The monitor records a violation whenever actual body clearance drops below the required value. It checks from flight start through the full monitoring horizon, including before approval and after checkpoint completion. The shared physics monitor observes at 1000 Hz; the controller runs at 50 Hz.

Actual code: [simulation.py:119](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/simulation.py:119).

```python
current = float(clearance(position[0], s.fence_x))
min_clear = min(min_clear, current)
if current < config.d_min_m:
    if first_violation is None:
        first_violation = tick / PHYSICS_HZ
    if dispatch is None or tick <= dispatch * (PHYSICS_HZ//HZ):
        if first_wait_violation is None:
            first_wait_violation = tick / PHYSICS_HZ
```

To complete, actual position must enter the fixed checkpoint and actual velocity must be within 0.12 m/s of the intended northbound velocity:

Actual code: [scenario.py:147](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/scenario.py:147).

```python
position = np.asarray(position)
return (np.all(position >= command.checkpoint_lower)
        and np.all(position <= command.checkpoint_upper)
        and np.linalg.norm(np.asarray(velocity)-[0., command.speed, 0.])
        <= config.velocity_tolerance_mps)
```

These conditions must hold continuously for at least 0.20 s before the approval-relative deadline:

Actual code: [simulation.py:128](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/simulation.py:128).

```python
elapsed = tick / PHYSICS_HZ - approval/HZ
if (0 <= elapsed <= s.deadline_s
        and in_checkpoint(position, velocity, s.command, config)):
    if entered is None:
        entered = tick
    if completion is None and tick-entered >= round(config.dwell_s*PHYSICS_HZ):
        completion = elapsed
else:
    entered = None
```

Safety and completion are combined into the recorded outcome:

Actual code: [simulation.py:193](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/simulation.py:193).

```python
safe = first_violation is None
result = dict(seed=s.seed, delay_s=delay, min_clearance_m=min_clear, safe=safe,
              completed=completion is not None, completion_s=completion,
              safe_completion=safe and completion is not None,
```

The approval search rejects either a safety failure or a completion failure:

Actual code: [calibration.py:40](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:40).

```python
def rejection_reasons(result):
    reasons = []
    if not result['safe']:
        reasons.append('body_clearance')
    if not result['completed']:
        reasons.append('checkpoint_deadline_velocity_or_dwell')
    return reasons
```

The separate 0.15 m tracking-error threshold is used for characterization. It is not an additional rejection condition in this per-scenario approval search. The accepted scenario's `d0_verification` records its actual measured outcome.

## C. Saving, resumption and counts

The approval snapshot contains actual UAV state, PID memory, remaining reference, mission command and monitoring information. It is captured inside the running flight, without resetting the UAV at approval:

Actual code: [simulation.py:140](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/simulation.py:140).

```python
for step in range(total):
    if step == approval:
        snapshot = Snapshot(obs.copy(), capture_pid_state(ctrl), ref[step:].copy(),
                            total-step, s.approval_s, s.command, s.fence_x,
                            config, min_clear)
```

While searching, the code saves the straight-flight probe and completed approval tests so an interrupted batch can continue without recomputing those completed tests. A successful approval snapshot is written as a pickle file. On resume, input settings, source/configuration provenance, snapshot checksums and references are validated before work continues.

Actual code: [calibration.py:303](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:303).

```python
check_provenance(data['provenance'],config)
if 'settings' not in data['calibration']:
    raise ValueError('Exported calibration is frozen input, not an interrupted calibration run')
if data['calibration']['settings'] != settings:
    raise ValueError('Resume calibration inputs/settings differ')
load_development(path,config)
```

The JSON contains accepted scenarios, configuration and provenance, acceptance/rejection counts, and calibration status. During an unfinished search it also contains the generated pool and current candidate cache. That temporary progress section is removed at completion or pool exhaustion. JSON updates use a temporary file and atomic replacement so readers see a complete saved version.

| Term | Meaning in this script |
|---|---|
| Scenario | One sampled mission with fixed speed, radius, fence, checkpoint and T |
| Approval candidate | That same mission tested at one approval time |
| Trial | One actual PyBullet flight: probe, approval test, or verification |
| Batch | A bounded stretch of work before saving and automatically continuing |

The log's `candidates` value counts approval tests for that scenario. It excludes the nominal probe and final verification. A geometric rejection uses zero flights. An accepted scenario requiring K approval tests normally needs K + 2 flights. A scenario rejected after searching all approvals has no final successful verification flight.

Statistics use processed seeds as their denominator:

Actual code: [calibration.py:427](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:427).

```python
statistics.update(attempted=len(data['calibration']['attempts']),accepted=len(data['scenarios']),
                  rejected=len(data['calibration']['attempts'])-len(data['scenarios']),
                  candidate_rejections=dict(counts))
statistics['acceptance_rate'] = statistics['accepted']/statistics['attempted']
```

## D. The 300-scenario command and its stopping rules

Run from `examples/staleguard` in the SCG environment. Use an empty output directory for a fresh run:

```bash
conda activate scg

nohup python geofence_turn/calibration.py \
  --evaluation \
  --start-seed 22 --attempts 500 --target 300 \
  --out geofence_turn/results/evaluation/calibration \
  --max-trials 200 --max-seconds 1800 \
  >> geofence_turn/results/evaluation/calibration.log 2>&1 &
```

| Argument | Actual effect |
|---|---|
| `--evaluation` | Allows the larger scenario target and automatically continues at batch-budget boundaries |
| `--start-seed 22` | Starts a new pool after development seeds 0–21 |
| `--attempts 500` | Generates candidate seeds 22–521; not a limit of 500 flights |
| `--target 300` | Stops after 300 accepted scenarios, if enough are feasible |
| `--max-trials 200` | Limits each batch to 200 simulator trials |
| `--max-seconds 1800` | Checks a 30-minute batch budget before starting another flight; a running flight can finish after that boundary |

The batch counter and clock are checked before a new trial. They are not limits on the full calibration job:

Actual code: [simulation.py:51](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/simulation.py:51).

```python
def check_work(work, config, release=False):
    if work['trials'] >= work['max_trials'] or time.monotonic()-work['start'] >= work['max_seconds']:
        raise RuntimeError('Explicit development trial/time budget reached; partial logs preserved')
```

Only that explicit budget exception automatically starts another batch. Memory failures, source/configuration mismatches, unexpected simulator termination and other errors stop the job:

Actual code: [calibration.py:493](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:493).

```python
except RuntimeError as error:
    # Only the explicit per-batch budget permits continuation.
    # Memory, source compatibility and scientific errors stop.
    if not args.evaluation or str(error) != 'Explicit development trial/time budget reached; partial logs preserved':
        raise
    print('Batch budget reached; resuming saved calibration in the next bounded batch.',flush=True)
    args.resume = True
    work = start_work(args.max_trials,args.max_seconds)
    continue
```

Execution remains CPU-only, headless, with one simulation worker and one numerical thread. Existing guards require RSS at most 2048 MiB and host available memory at least 2048 MiB. A completed batch does not alter any physical parameter, checkpoint, search grid or acceptance rule.

| Saved status | Meaning |
|---|---|
| `running` | Work is in progress; includes intermediate saved checkpoints |
| `interrupted` | A budget boundary or error was recorded; evaluation automatically continues only for the budget case |
| `complete` | The requested accepted-scenario target was reached |
| `exhausted` | All candidate seeds were processed before the target was reached; the command reports an error |

The configured `approval_max_s = 3.20` is only a fallback in `approval_candidates()`. Normal scenario calibration passes the probe-derived `upper`, so its search can extend beyond 3.20 s.

If the process stops, the saved status is its last checkpoint; a sudden OS kill may leave it labelled `running`. Confirm the process separately when deciding whether to relaunch. For an interrupted run, use the same command and inputs with `--resume`. Do not change scientific source code during a resumable job: compatibility checks intentionally reject such changes.

## E. Outputs and elapsed time

```text
geofence_turn/results/evaluation/
├── calibration.log
└── calibration/
    ├── calibrated_scenarios.json
    └── snapshots/
        └── seed_<accepted_seed>.pkl
```

On successful completion, the JSON has 300 accepted entries and there is one approval snapshot for each. During an interrupted search there can also be a snapshot for the current passing candidate before final verification has committed that scenario. There are no default candidate-log or trajectory files. Characterization's historical files remain separate in `support/logs/northbound_review/characterization/`.

Watch the requested shell log with:

```bash
tail -f geofence_turn/results/evaluation/calibration.log
```

Each batch stores its start time, process ID, trial count and elapsed seconds:

Actual code: [calibration.py:326](/home/kean/Documents/work/stale-guard/safe-control-gym/examples/staleguard/geofence_turn/calibration.py:326).

```python
session = dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),pid=os.getpid(),trials=0)
data['calibration']['sessions'].append(session)
# ... intervening code omitted ...
def checkpoint():
    session.update(trials=work['trials'],elapsed_s=time.monotonic()-work['start'])
    write_json(path,data)
```

Summing `calibration.sessions[*].elapsed_s` gives cumulative recorded batch time, including setup and bookkeeping charged to those batches. It does not include downtime between separate launches, and is not an exact process-start-to-exit timer. A time estimate is not a completed-run measurement.

## F. What acceptance proves, and what remains to evaluate

A saved accepted scenario has reproduced safe mission completion at D = 0 using the current simulation, controller and mission predicates. It has the latest passing approval time among the tested grid candidates. The prior characterization is an empirical screen under one fixed test setup, not a general guarantee for all missions or real aircraft.

Calibration does not establish a positive-delay safety margin, successful completion under queued dispatch, predictor accuracy or any policy's recovery performance. Those questions require the separate frozen evaluation protocol. No StaleGuard or baseline result is used to choose the calibrated approval time.
