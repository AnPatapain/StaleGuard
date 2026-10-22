# Experiment B: FBS baseline — 22 October 2026

FBS is an opt-in fifth policy, using the locked model-free signed-clearance
feature and margin/EMA-volatility TTL rule. The implementation executed all
**209 FBS flights**: 19 feasible calibrated scenarios × 11 dispatch delays.
Seed 16 remains excluded. The comparison reuses the **836 saved executions**
of the existing four policies; those policies were not retimed or rerun in
full. Twelve representative existing-policy flights were physically replayed
with frozen recorded predictions and computation times and matched exactly.
All execution was CPU-only, headless and serial.

## Algorithm and timing

[Shraga et al., Section III-D](https://arxiv.org/html/2608.26306#S3.SS4)
supplies the margin/volatility ratio, EMA coefficient 0.1, positive-margin
gate and inclusive comparison against the floored TTL. The UAV adaptation is:

```text
f = max(x_min - x, x - x_max, abs(y) - y_limit, z - unsafe_below_z)
v[0] = 0
v[t] = 0.1 * abs(f[t] - f[t-1]) + 0.9 * v[t-1]
delta = min(210, max(f, 0) / max(v, epsilon))
ttl_steps = floor(delta)
```

The state is observed at reset and every 20 ms through approval, including
the approval observation. No post-approval observation updates the TTL.
The denominator is metres per sample, so the ratio is in control steps;
210 steps are 4.2 seconds. At dispatch, the approval is expired when
`margin <= 0` or `waited_steps > ttl_steps`. Equality with TTL remains valid.
The pending upstream approval is taken as given; no oracle supplies it.

Measured approval computation includes the final clearance/EMA update and
TTL calculation. Earlier monitoring updates are measured separately and
occur during their existing control steps. Logging and offline validation
are outside these timers. Readiness is `computation_s <= waited_steps / 50`.
The integer step difference is the paper's age conversion for observations
and dispatches already aligned to the 50 Hz grid.

At dispatch, an available expired result cancels descent and invokes the
existing safety-only fallback. An unavailable result **fails open**: the
original descent starts on schedule, with no later cancellation. A valid
result also executes the original command on schedule. The original command
duration, PID, 1,000 Hz physics monitoring and deadline of approval + 4.2 s
remain unchanged. The 8.5 s TTL ground-truth observation horizon is not used
to decide FBS actions or to extend the operational deadline.

This is measured computation represented in simulated time, following
Experiment B's existing availability semantics. It is not a concurrent
onboard implementation or a real-time controller scheduling measurement.

## Epsilon chosen before outcomes

The frozen [protocol](../results/fbs/epsilon_protocol.json) selects
**epsilon = 1e-12 metres per control sample**. Inspection of the environment
configuration found no observation disturbances or sensor quantization;
actual recorded observations are float64. Therefore no physical sensor-noise
floor can be justified from this simulator configuration.

Epsilon is explicitly a **numerical floor**, several orders above float64
roundoff at metre scale and far below a micrometre per sample. It prevents
division by zero without claiming to bound sensor, dynamics or integration
error. The value and sensitivity settings were recorded before evaluating
FBS policy outcomes. They were not chosen using collisions, completion or
StaleGuard performance.

| Epsilon (m/sample) | Changed dispatch decisions | Unsafe | Safe completion | Fallbacks |
|---:|---:|---:|---:|---:|
| 1e-12 — fixed primary | 0 | 128 | 59 | 0 |
| 1e-15 | 0 | 128 | 59 | 0 |
| 1e-9 | 0 | 128 | 59 | 0 |
| 1e-6 | 0 | 128 | 59 | 0 |

Sensitivity reuses each measured history and computation time, with verified
original/fallback trajectories. No new simulations or best-value selection
were used. All four epsilon settings produce the same floored TTLs as well
as the same dispatch decisions on this grid.

## Five-policy comparison

Each policy has N=209. The three mission categories are exclusive.

| Policy | Unsafe | Safe completion | Safe non-completion | Fallbacks |
|---|---:|---:|---:|---:|
| StaleGuard | 2 (0.96%) | 59 (28.23%) | 148 (70.81%) | 127 |
| No Recovery | 128 (61.24%) | 59 (28.23%) | 22 (10.53%) | 0 |
| Always-Recover | 0 | 19 (9.09%) | 190 (90.91%) | 190 |
| Use-Time Recheck | 1 (0.48%) | 55 (26.32%) | 153 (73.21%) | 130 |
| FBS | 128 (61.24%) | 59 (28.23%) | 22 (10.53%) | 0 |

| D (s) | FBS ready / 19 | Unsafe | Safe completion | Safe non-completion | Fallbacks |
|---:|---:|---:|---:|---:|---:|
| 0.0 | 0 | 0 | 19 | 0 | 0 |
| 0.1 | 19 | 1 | 16 | 2 | 0 |
| 0.2 | 19 | 2 | 12 | 5 | 0 |
| 0.3 | 19 | 6 | 5 | 8 | 0 |
| 0.4 | 19 | 12 | 3 | 4 | 0 |
| 0.5 | 19 | 14 | 3 | 2 | 0 |
| 0.6 | 19 | 17 | 1 | 1 | 0 |
| 0.7 | 19 | 19 | 0 | 0 | 0 |
| 0.8 | 19 | 19 | 0 | 0 | 0 |
| 0.9 | 19 | 19 | 0 | 0 | 0 |
| 1.0 | 19 | 19 | 0 | 0 | 0 |

FBS has no false-rejection completion losses because it never cancels.
All 128 unsafe flights had an available, unexpired FBS result. None is
attributable to unavailable FBS computation on this grid.

## Computation and TTL distribution

| Measured CPU work | Mean | Median | p95 | Minimum | Maximum |
|---|---:|---:|---:|---:|---:|
| Final approval update + TTL | 12.459 us | 10.901 us | 24.760 us | 6.122 us | 38.825 us |
| Earlier observation update | 5.146 us | 3.951 us | 11.077 us | 2.435 us | 34.818 us |
| Sum of earlier updates per flight | 40.088 us | 35.886 us | 108.389 us | 0 | 168.189 us |

There are 209 measured approval computations, one per executed FBS flight.
All 190 positive-delay dispatches have a ready result. The 19 zero-delay
dispatches fail open because their measured computation takes positive time.
Command application is never postponed.

Approval histories and TTL outputs are identical across all eleven future
dispatch delays for each seed. Over the 19 distinct approvals:

- 18 TTLs equal the cap: **210 steps = 4.2 s**.
- Seed 11 has **167 steps = 3.34 s**, from margin 1.106756577 m and volatility
  0.006626769 m/sample; the unrounded ratio is 167.013 steps.
- Mean TTL is **207.737 steps = 4.155 s**; median and p95 are **4.2 s**.

Every TTL exceeds the largest tested delay of 1.0 s. Thus every available
result passes the gate, explaining the exact match to No Recovery.

## Cold-start cases

Seeds **0, 1, 12, 15 and 18** approve at time zero. Each has exactly one
observation, no observed differences, v=0 and a capped TTL of 4.2 s.
No fictitious preflight history or extra settling time is introduced.

| FBS approval cohort | Executions | Unsafe | Safe completion | Safe non-completion | Fallbacks |
|---|---:|---:|---:|---:|---:|
| Cold start: 5 seeds | 55 | 32 | 11 | 12 | 0 |
| With history: 14 seeds | 154 | 96 | 48 | 10 | 0 |

The cohorts differ in scenario content; this is descriptive stratification,
not a causal estimate of the effect of cold initialization. Approvals with
history have 3–30 observations, including their approval sample.

## Interpretation and limitations

The locked FBS adaptation provided no safety benefit on this grid: its TTLs
all exceeded 1 s, so all original descents executed. The feature describes
current geometric separation, while unsafe entry depends on later descent
and continued forward motion. Small past feature changes, including the
zero-volatility cold starts, do not bound subsequent changes under a command.
This is an observed limitation of this feature/history adaptation, not a
general conclusion that the published FBS algorithm is ineffective.

The scalar omits command duration, command-induced motion and PID/dynamics
response. EMA volatility is not a worst-case bound. Numerical epsilon is not
a physical uncertainty allowance. The study has only 19 scenarios reused
across delays, finite monitoring/deadline horizons and deterministic noiseless
observations. Numerical sensitivity here does not validate real sensor-noise
robustness. The official grid contains no actual FBS expiration; expiration
and fallback integration were checked with deterministic unit executions.

The earlier [proposal](19_fbs_baseline_proposal.md) records the then-unresolved
availability rule. The user's implementation instruction supersedes its
fail-closed recommendation with fail-open. The cap and initialization are
locked UAV adaptations; their original artifact settings remain unverified.
Previous-policy computation times are retained from the earlier evaluation,
so the table is not a fresh simultaneous hardware benchmark of five policies.

## Implementation and validation

- `fbs.py`: scalar feature, EMA, frozen TTL and dispatch gate; no dynamics model.
- `staleguard_benefit.py --fbs`: all five Experiment B policies. Without the
  flag, existing policy configurations remain unchanged.
- `diagnostics/test_fbs.py`: seven new tests; **15 total tests passed**, covering
  feature boundaries, initialization, approval update, units, cap/floor,
  inclusive TTL/readiness, zero-age fail-open and absence of later cancellation.
- `diagnostics/validate_fbs.py`: resumable native FBS flights, independent
  prefix/EMA checks, epsilon sensitivity and existing-policy regression replays.

Every FBS flight matched its corresponding verified No Recovery control and
1,000 Hz physics trace exactly. All 209 approval observations matched both
the actual execution trace and the saved approval snapshot. Assertions checked
observation timing, frozen histories, reference application at scheduled
dispatch, original deadlines and no future-D dependence.

Twelve existing-policy replays cover all four policies at D=0, seed 7/D=0.4
and seed 11/D=0.6, using their prior measured prediction outputs/times.
Every full control/physics hash matches. An AST comparison also found the
entire original executor unchanged after removing the new optional FBS
argument and its guarded branches. Hash checks preserve 248 existing
scientific-source/input/result files, except the authorized executor edit.

The existing 2 GiB process-RSS maximum and 2 GiB host-available minimum were
enforced between flights, with one worker and one library thread. Recorded
worker RSS stayed at approximately **636 MiB**. No memory guard stopped the run.

```sh
# Native five-policy experiment; does not overwrite old result files.
python inspection_descent/staleguard_benefit.py --count 20 --experiment B --fbs

# Memory-guarded validation, reusing the saved four-policy comparison.
python inspection_descent/diagnostics/validate_fbs.py --stage run
python inspection_descent/diagnostics/validate_fbs.py --stage regression
python inspection_descent/diagnostics/validate_fbs.py --stage report

python -m unittest discover -s inspection_descent/diagnostics -p 'test*.py' -v
```

Evidence: [five-policy terminal report](../result/baseline_fbs.log),
[metrics and sensitivity](../result/fbs.json),
[209 actual flight records](../results/fbs/executions.jsonl),
[existing-policy replays](../results/fbs/existing_policy_replays.json),
[executor structure check](../results/fbs/executor_structure_check.json),
[preservation manifest](../results/fbs/preserved_files.json).
