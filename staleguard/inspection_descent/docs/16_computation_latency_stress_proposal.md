# Experiment B: computation-latency stress proposal

Status: design only. No implementation or stress simulations are included.

Scientific objective: determine when fail-open dispatch causes unsafe command
execution because a correct rejecting TTL prediction is unavailable at dispatch.

Use a **uniform slowdown of recorded prediction runtimes**, with sweep points
determined by dispatch boundaries rather than chosen to trigger specific failures.

## 1. Variables and justified range

Use all 19 feasible calibrated scenarios, excluding seed 16, with
D = 0, 20, 40, 60, 80, 100 ms and T=4.2 s after approval.

Freeze each scenario's prediction and measured computation time `c_s` from the
[short-delay supplementary experiment](15_short_delay_latency.md). Define
stressed availability time as:

```text
L_s(alpha) = alpha * c_s
```

Apply the same slowdown factor `alpha` to every scenario. Start at `alpha = 1`,
the recorded timing baseline. Sweep the availability transitions `alpha = D/c_s`,
including equality and the interval immediately afterward. Equality remains
ready under the existing rule.

Stop just beyond the largest positive-delay transition—approximately **5.10×**
for this dataset—when every positive-delay prediction is unavailable. This
covers all possible availability changes on this schedule without selecting
seeds or arbitrary failure-producing penalties.

For illustration, seed 10/D=40 ms crosses its availability boundary at
approximately **2.04×**. That is a derived stress threshold, not measured
hardware performance.

## 2. Paired executions and counterfactuals

For every scenario, delay, and stress condition, compare:

- **Stressed StaleGuard:** unchanged fail-open dispatch with availability time
  `L_s(alpha)`.
- **On-time diagnostic counterfactual:** the identical stored prediction is
  available at dispatch; the existing TTL gate decides whether to cancel.

Both use the same command, dispatch event, initial conditions, and deadline.
Neither postpones dispatch.

Retain No Recovery and Always-Recover as the existing baselines. The on-time
counterfactual is an information-availability control, not a proposed
operational policy.

Preserve the existing predictor, calibrated scenarios, UAV dynamics, safety
predicates, safety-only fallback, and fail-open policy.

## 3. Metrics and causal attribution

Report readiness, original-command executions, unsafe entries, safe completions,
and fallback counts by D and `alpha`.

Count an unsafe execution as **latency-caused** only when:

- The stressed prediction was unavailable.
- Its stored TTL would reject at that D.
- Stressed execution is unsafe.
- The paired on-time execution is safe.

An unsafe command that the stored TTL would allow remains a **false-safe
prediction decision**, even when the prediction arrives late. Unsafe waiting
or unsuccessful fallback must be reported separately.

Also track false rejections and completion harm. “Correct prediction” here means
a correct rejection **at that dispatch delay**, not necessarily an accurate TTL
estimate overall.

## 4. Implementation complexity

One small supplementary driver should suffice. Preserve the predictor's existing
delay grid and all scientific code.

Reuse verified original-command and fallback traces; execute missing paired
trajectories once. Changing availability only selects between these trajectories,
so physics need not rerun for every slowdown value. Validate representative
boundary transitions through the native execution loop.

Run serially with the existing memory safeguards. A readiness/unsafe-rate figure
and a compact attribution table should fit the five-page workshop paper.

## 5. Scientific limitations

Values above 1× are **simulated latency stress**, not evidence that particular
hardware has those runtimes. Uniform scaling preserves runtime ordering and
does not represent jitter or concurrent workload precisely.

The experiment estimates conditional failure thresholds for nineteen
deterministic scenarios, not deployment failure probabilities. Repeated delays
are not independent scenarios. Keep the measured 1× results visually distinct,
and retain the [saved wider-grid results](14_corrected_experiment_b.md) as context
and checks that known prediction errors are not misclassified as latency failures.
