# Proposal: FBS baseline for Experiment B

Status: design only. No implementation or simulations. Epsilon and the
unavailable-result rule require resolution before evaluation.

## Reference and artifact availability

Shraga et al., *Approved Too Late: Verdict Staleness in LLM-Guarded
Self-Adaptive Systems*, ACSOS 2026,
[Section III-D](https://arxiv.org/html/2608.26306#S3.SS4).

The inspected [arXiv source archive](https://arxiv.org/src/2608.26306)
contains manuscript sources and figures, but no executable reproduction code
or parameter configuration. A separate reproduction artifact could not be
located. The paper specifies the EMA coefficient but delegates epsilon, the
TTL cap, initialization and update order to that artifact. This proposal
preserves the published mechanism; it is not an exact artifact reproduction.

## Published mechanism

For a scalar feature f, signed safe-side margin m and recent volatility v,
FBS estimates a validity horizon in observation steps:

```text
v[t]     = 0.1 * abs(f[t] - f[t-1]) + 0.9 * v[t-1]
delta[t] = min(delta_max, max(m[t], 0) / max(v[t], epsilon))
```

An upstream-approved command passes only when its margin is positive and
its age K satisfies `K <= floor(delta[t])`. Otherwise, fallback applies.
The formula, coefficient 0.1, positive-margin requirement and inclusive TTL
comparison come from Section III-D. Recent volatility is not a worst-case
drift bound, so the TTL is a heuristic rather than a safety certificate.

## UAV-specific safety-margin feature

Use the observed position `(x, y, h)` and the existing unsafe-region geometry:

```text
f[t] = max(x_min - x,
           x - x_max,
           abs(y) - y_limit,
           h - unsafe_below_z)
m[t] = f[t]
```

The threshold is zero. A positive feature means the UAV is outside the unsafe
volume; a nonpositive feature means it is inside or on its boundary. This
sign convention matches the existing closed unsafe predicate exactly. The
feature expresses signed axis-aligned geometric clearance in metres, without
an added safety margin or fitted weights.

This scalar includes every geometric boundary, but cannot determine whether
executing the descent will subsequently cause unsafe entry. It omits command
effects, inertia, PID response and descent duration. Altitude clearance can
also dominate the maximum and remain nearly constant while horizontal
clearance shrinks. These are limitations of the chosen proxy, not reasons to
tune it against known failures.

## Approval-time information and parameters

Maintain the EMA from actual observations during the existing flight to
approval. Include the approval observation in the final update, then freeze
the margin, volatility and TTL. No later observations, delay-specific states,
oracle labels or model rollouts enter the decision.

Use the experiment's existing pending approval as `A = 1`; do not derive it
from calibration success labels, ground truth or StaleGuard predictions.

| Setting | Proposed treatment | Source and justification |
|---|---|---|
| Sampling | 50 Hz | UAV adaptation: existing controller observation frequency. Volatility is metres per control step; TTL is control steps. |
| EMA coefficient | 0.1 | Published value. At 50 Hz, its approximately 6.6-step half-life corresponds to approximately 132 ms. |
| Initialization | `v[0] = 0` | Proposed adaptation pending artifact recovery. Update only with actual observed differences; include the approval sample before computing TTL. |
| Maximum TTL | 210 steps = 4.2 s | Proposed adaptation: existing approval-to-deadline budget. Longer validity cannot affect this experiment; the cap does not depend on the future dispatch delay. |
| Epsilon | Unresolved; positive, in metres per sample | Recover the artifact value and its feature scaling, or justify a UAV value from independently established observation resolution/noise. Do not select it using collision outcomes. |

Some calibrated scenarios approve at time zero and therefore have no past
differences. Zero initialization is a prior, not measured evidence of
stability. Report history length and cold-start cases separately. Do not
invent preflight observations or advance the scenario to obtain history.

## Computation and scheduled dispatch

Measure FBS computation from the approval observation, including its final
feature/EMA update and TTL calculation. Report ongoing monitoring cost
separately. Freeze the result once computed.

At the unchanged scheduled dispatch, use the observation's grounding time:

```text
K = ceil((scheduled_dispatch_time - observation_time) / 0.02)
pass = upstream_approved and margin > 0 and K <= floor(delta)
```

The conservative age conversion follows the paper's live-clock convention.
Use exact control-step arithmetic for dispatches already on the control grid.
Computation consumes part of the observation-to-dispatch interval; do not add
it to dispatch delay again. Represent measured result availability within
the existing simulated timing scheme. Do not postpone dispatch, block PID,
freeze physics or cancel an already-dispatched command later.

For an available result that fails the gate, cancel the descent and invoke
the existing safety-only fallback: continue nominal forward flight at usual
altitude. Preserve the original command and fixed mission deadline.

The paper does not specify the unavailable-result scheduling rule. The
recommendation is fallback at scheduled dispatch, consistent with requiring
a completed passing gate. This is a declared scheduling adaptation and
differs from StaleGuard's fail-open behavior. At D=0, any positive computation
time makes the result unavailable. Report these cancellations separately
from TTL expiration. If matching Experiment B's fail-open behavior is chosen
instead, document that adaptation explicitly rather than silently treating
it as the published rule.

## Evaluation and unresolved decisions

Add FBS alongside all existing policies. Preserve calibrated seeds 0–19,
the existing infeasible-seed exclusion, D=0.0–1.0 s, T=4.2 s, dynamics,
recovery and all current metrics and prior results.

Also record margin, volatility, history length, TTL, computation time,
prediction readiness and rejection reason. Use ground truth only afterward
to assess unsafe executions and false rejections. Preserve the distinction
between operational mission outcomes and the paper's replay-based approval
expiry metric; they are not interchangeable.

Before implementation:

1. Recover the reproduction configuration if available; otherwise label
   initialization and the cap as UAV-specific choices.
2. Justify epsilon independently of the evaluated outcomes and freeze it
   before evaluation. Report any subsequent parameter sensitivity separately.
3. Resolve the unavailable-result rule, especially its effect at D=0.
4. Freeze the feature and all parameter choices without tuning against the
   known failures or selecting settings for favorable baseline rankings.

The proposal requires only scalar monitoring, an approval-time TTL record
and a dispatch gate. It introduces no dynamics model, future-state access,
new recovery behavior or changes to existing policy definitions.
