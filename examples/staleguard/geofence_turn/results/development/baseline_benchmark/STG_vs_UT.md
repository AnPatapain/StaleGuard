# StaleGuard vs Use-Time Recheck — 20 development scenarios

Completed **520 unique flights and 260 complete pairs**, using the locked protocol and unchanged scenarios, algorithms and timing semantics. StaleGuard achieved **101 safe completions versus 82**, but also produced **27 unsafe flights versus 18**. In measured seed 1 at D=0.10 s, StaleGuard turns at 1.90 s and completes; Use-Time Recheck turns at 1.92 s and misses the checkpoint. Use-Time Recheck's current-state checks also rejected unsafe turns that StaleGuard permitted. When StaleGuard's result was unavailable, all 64 flights still executed the approved turn: 51 completed safely, 10 remained safe without completing, and three were unsafe. Recovery requests braking and hovering at the current position; it does not guarantee an instantaneous stop. These results show a completion–safety tradeoff, not an overall winner. They describe 20 development scenarios with repeated delays, not 260 independent scenarios. Further work should examine TTL-grid resolution and check-to-use latency separately without altering this recorded experiment.

## Overall results

| Policy | Flights | Safe completion | Safe non-completion | Unsafe | Recovery attempts |
|---|---:|---:|---:|---:|---:|
| StaleGuard | 260 | 101 | 132 | 27 | 59 |
| Use-Time Recheck | 260 | 82 | 160 | 18 | 75 |

Recovery successes/failures were **47/12 for StaleGuard** and **63/12 for Use-Time Recheck**. A recovery succeeds only when it activates and the entire flight remains safe, including the period before recovery.

## Results by delay

D is the time between approval and scheduled dispatch, in seconds. Each delay was tested on all 20 scenarios. The three mutually exclusive flight outcomes are safe completion, safe non-completion and unsafe execution; completing the checkpoint does not erase a safety violation.

| D (s) | SG safe completion | UT safe completion | SG unsafe | UT unsafe |
|---:|---:|---:|---:|---:|
| 0.00 | 20/20 | 16/20 | 0/20 | 0/20 |
| 0.02 | 16/20 | 13/20 | 0/20 | 2/20 |
| 0.04 | 13/20 | 12/20 | 2/20 | 1/20 |
| 0.06 | 12/20 | 11/20 | 3/20 | 0/20 |
| 0.08 | 11/20 | 9/20 | 3/20 | 1/20 |
| 0.10 | 9/20 | 8/20 | 0/20 | 0/20 |
| 0.12 | 8/20 | 6/20 | 0/20 | 1/20 |
| 0.14 | 6/20 | 4/20 | 2/20 | 0/20 |
| 0.16 | 4/20 | 2/20 | 2/20 | 1/20 |
| 0.18 | 2/20 | 0/20 | 3/20 | 0/20 |
| 0.20 | 0/20 | 1/20 | 0/20 | 0/20 |
| 0.50 | 0/20 | 0/20 | 2/20 | 2/20 |
| 1.00 | 0/20 | 0/20 | 10/20 | 10/20 |

## Paired outcomes

Each delay contains 20 pairs with identical scenario and dispatch delay. Completion benefits mean StaleGuard completes safely and Use-Time Recheck does not; completion harms mean the reverse. Unsafe prevented means Use-Time Recheck is unsafe while StaleGuard is safe; unsafe introduced means the reverse. The table reports counts, with no independence assumption across delays of the same scenario.

| D (s) | Completion benefits | Completion harms | Unsafe prevented | Unsafe introduced |
|---:|---:|---:|---:|---:|
| 0.00 | 4 | 0 | 0 | 0 |
| 0.02 | 3 | 0 | 2 | 0 |
| 0.04 | 1 | 0 | 1 | 2 |
| 0.06 | 1 | 0 | 0 | 3 |
| 0.08 | 2 | 0 | 1 | 3 |
| 0.10 | 1 | 0 | 0 | 0 |
| 0.12 | 2 | 0 | 1 | 0 |
| 0.14 | 2 | 0 | 0 | 2 |
| 0.16 | 2 | 0 | 1 | 2 |
| 0.18 | 2 | 0 | 0 | 3 |
| 0.20 | 0 | 1 | 0 | 0 |
| 0.50 | 0 | 0 | 0 | 0 |
| 1.00 | 0 | 0 | 0 | 0 |
| **Overall** | **20** | **1** | **6** | **15** |

The required identities hold:

- Net completion benefit: **20 − 1 = 19 = 101 − 82**.
- Net unsafe reduction: **6 − 15 = −9 = 18 − 27**.

Thus StaleGuard has 19 more safe completions and nine more unsafe flights overall.

## Timing

All timing values below are milliseconds. StaleGuard computation counts each cached approval forecast once; Use-Time Recheck computation is measured per flight. These are the measured warmed predictor calls used by the benchmark, not total mission runtimes. Dispatch-to-action time is the interval from scheduled dispatch to actual turn or recovery application.

| Policy | Computation samples | Computation mean / median / p95 | Action-delay samples | Dispatch-to-action mean / median / p95 |
|---|---:|---:|---:|---:|
| StaleGuard | 20 | 53.25 / 52.31 / 69.98 | 260 | 0.00 / 0.00 / 0.00 |
| Use-Time Recheck | 260 | 5.11 / 4.95 / 6.87 | 260 | 20.00 / 20.00 / 20.00 |

StaleGuard was **ready in 196 flights and unavailable in 64**. All 64 unavailable cases executed unverified through fail-open. Their cached TTLs were still unexpired, so making those predictions available earlier would not have changed the selected action.

## The 64 flights with unavailable StaleGuard predictions

These flights were included in the 520-flight experiment. Actual safety and checkpoint completion were monitored normally at 1000 Hz, even when the prediction was unavailable. All 64 executed the original approved turn, and none activated recovery.

| D (s) | Unavailable flights | Safe mission completion | Safe mission non-completion | Unsafe |
|---:|---:|---:|---:|---:|
| 0.00 | 20/20 | 20 | 0 | 0 |
| 0.02 | 20/20 | 16 | 4 | 0 |
| 0.04 | 20/20 | 13 | 5 | 2 |
| 0.06 | 4/20 | 2 | 1 | 1 |
| **All delays** | **64/260** | **51** | **10** | **3** |

The outcome columns count only the unavailable-prediction subset: each row's three outcome counts sum to its unavailable-flight count. At D=0.00, 0.02 and 0.04 s, predictions were unavailable for all 20 scenarios. At D=0.06 s, the four unavailable cases were **seeds 11, 14, 16 and 21**. No predictions were unavailable at any tested delay of **0.08 s or more**.

The three unsafe unavailable-prediction flights were:

| Seed | D (s) | Minimum body clearance (mm) | Checkpoint completed? |
|---:|---:|---:|---|
| 3 | 0.04 | 46.738 | Yes |
| 11 | 0.04 | 49.986 | Yes |
| 11 | 0.06 | 37.995 | No |

The required body clearance is **50 mm**. Two of these flights completed the checkpoint objective, but their safety violations still place them in the unsafe category. Seed 11 at D=0.04 s is particularly close to the boundary: its clearance is approximately 0.014 mm below the requirement.

“Unavailable” describes elapsed computation time, not a missing prediction record. The benchmark caches each forecast and its measured duration; at dispatch, the simulator checks whether enough queue time has passed for that forecast to be ready. If not, it executes the original turn without waiting or cancelling it later.

## Current recovery behavior

Recovery requests **braking followed by hovering at the UAV's measured position when recovery is applied**. From that moment onward, the requested position remains fixed and the requested velocity is zero. The existing PID controller and PyBullet physics determine how the UAV actually slows and settles.

For StaleGuard, this happens at scheduled dispatch when a ready prediction indicates expiry. For Use-Time Recheck, it happens at the delayed action tick following the check, using the position measured at that later tick. The recovery does not retry the turn or generate a route back to the checkpoint. Requesting zero velocity does not stop the physical UAV instantly: it can continue east while braking and violate clearance before reaching a hover.

## Why outcomes differed

All **38 disagreeing pairs** were investigated.

- **Check-to-use latency:** Immediate-application counterfactuals support 19 completion benefits and all six safety benefits for StaleGuard; these counts overlap. The evidence holds Use-Time's recorded SAFE decision fixed and compares its delayed application against the saved No-Recovery flight applying the same turn immediately. The claim is not based merely on Use-Time finishing later.
- **Different recovery decisions:** In 15 pairs, Use-Time recovered safely while StaleGuard executed an unsafe turn. Diagnostic predictions at those exact delays identified 14 coarse-TTL-grid gaps and one false-safe model prediction. The primary prediction grid and policies were not changed.
- **False-unsafe prediction:** In seed 21 at D=0.12 s, Use-Time rejected an actually safe immediate turn. Applying its recovery decision immediately also missed completion. This accounts for the remaining StaleGuard completion benefit; it is not attributed to latency.
- **A beneficial delay:** The single completion harm occurred in seed 14 at D=0.20 s. Immediate execution missed completion, while the postponed turn completed. A focused No-Recovery replay at effective delay 0.22 s reproduced this non-monotonic tracking result.
- **Insufficient stopping margin:** Both policies had 12 failed recoveries. Five cases were already unsafe at dispatch. In the other seven, immediate-recovery replays crossed the required clearance boundary while the UAV was still moving east.

Nine focused diagnostic flights supplemented the benchmark; they were not added to its 520-record journal. The case-by-case counterfactual evidence is retained in `run.json`, under `disagreement_analysis`.

## Validation and artifacts

All required invariants passed, metrics reproduced from the journal, and **41 regression tests passed**. Calibration, scientific implementations and previous results remained unchanged. Results are descriptive for the 20 development scenarios; repeated delays are not independent scenarios.

The three experiment artifacts are:

- [executions.jsonl](executions.jsonl): authoritative flight records, including all 64 unavailable-prediction cases.
- [metrics.json](metrics.json): derived overall, per-delay, paired and timing results.
- [run.json](run.json): fixed protocol, provenance, cached predictions, resume status, validation and disagreement evidence.

This Markdown report was added separately at the user's request.

## Recovery implementation — technical appendix

The recovery reference is defined in [scenario.py, line 154](../../../scenario.py#L154):

```python
def safety_only_reference(reference, step, state):
    changed = reference.copy()
    changed[step:, POS] = state[POS]
    changed[step:, VEL] = 0
    return changed
```

`POS` selects the three position coordinates; `VEL` selects the three velocity components. The code replaces every remaining requested position with the current measured position and every remaining requested velocity with zero.

The recovery branch is activated in [simulation.py, line 167](../../../simulation.py#L167):

```python
if step == action_step:
    if recover:
        ref = safety_only_reference(ref, step, obs)
        event['fallback_step'] = step
```

The resulting reference is assigned to `ctrl.reference` at line 176. Here `obs` is the actual state at the recovery application tick, so Use-Time Recheck's hover position is taken after its computation-related postponement.

The fail-open decision is defined in [baselines/staleguard.py, line 67](../../../baselines/staleguard.py#L67):

```python
def staleguard_status(received, waited_steps):
    ready = received['computation_s'] <= waited_steps/HZ
    expired = received['expiry_steps'] is not None and waited_steps >= received['expiry_steps']
    return ready, ready and expired
```

When `ready` is false, `recover` is false. The simulator therefore takes the original-turn branch at scheduled dispatch. This is the behavior observed in all 64 unavailable-prediction flights.
