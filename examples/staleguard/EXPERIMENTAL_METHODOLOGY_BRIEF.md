# StaleGuard — factual brief for Section V: Experimental Methodology

**Purpose:** Supply ChatGPT web with the experimental facts needed to draft Section V of “StaleGuard: Approval-Time Safety Prediction for Delayed UAV Commands.” This is a self-contained account of the implementation and saved experiments, checked on **5 November 2026**. It is not a replacement for the paper’s results section.

The experiments ask whether waiting changes the safety of a previously approved UAV command, whether an approval-time model predicts that change, and whether different intervention policies preserve safety and mission completion. In Inspection-Descent, the aircraft flies forward and briefly descends to inspect a point before a restricted volume. In Geofence Turn, it patrols east, turns left, and continues north through a fixed checkpoint. The completed six-policy datasets contain **59,616 policy evaluations**; 5,236 Inspection-Descent records reuse verified identical trajectories. The separate Geofence early-recovery comparisons each contain **13,800 new physical simulations**. Prediction-accuracy artifacts currently cover development scenarios only. The writer must distinguish dispatch-time StaleGuard from StaleGuard-EarlyRecovery, retain mission-specific policy differences, and avoid claiming independent accuracy validation or embedded execution. Statistical uncertainty for the Geofence comparisons still requires a separate analysis if the paper will report it.

## How to use this brief

Draft the existing V.A, V.B and V.C subsections from the main account below. Use the technical appendix to obtain exact parameters, policy boundaries, source evidence and qualifications. Keep most implementation detail and numerical results out of Section V; Section IV already explains the model, while Section VI should present outcomes. Do not invent unexecuted experiments, measurements, tests or citations.

## V.A — Simulation platform and missions

Both missions use Safe-Control-Gym’s 3D quadrotor environment with PyBullet, the Crazyflie CF2X configuration and its cascaded PID controller. The controller updates at **50 Hz** and actual safety/completion observations run at **1,000 Hz**. Commands edit the reference while the simulated aircraft and unchanged controller determine the physical response. Prediction uses a separate lightweight closed-loop model with **100 Hz RK4 integration**, two integration steps per control update, warmed Numba kernels and preserved PID memory. Experiments are headless, CPU-only, serial and use one numerical thread. Initialization/inertial randomization and disturbances are disabled. Physical state and controller memory are assumed available; no AI planner, state estimator or real queue workload is executed.

In **Inspection-Descent**, the UAV flies east and executes an approved low-altitude request for a fixed duration. It must remain near a fixed inspection point for **0.18 s**, before **4.2 s after approval**, without entering a closed restricted volume. Its safety-only recovery cancels descent, retains forward flight and requests usual altitude. In **Geofence Turn**, the UAV patrols east, executes the fixed-radius 90° left course turn and continues north. It must spend **0.20 s** in the fixed checkpoint with velocity within **0.12 m/s** of the required northbound velocity, before a scenario-specific frozen deadline. Safety requires **0.05 m body clearance** from a virtual fence. Recovery cancels the turn and requests braking/hover through the same PID controller; it does not execute the northbound maneuver or retry completion.

Calibration uses only immediate No-Recovery execution, **D=0**. Inspection searches approval/duration candidates and maximizes safe inspection dwell, keeping the first tie. Geofence samples a characterized feasible speed/radius, freezes the checkpoint and deadline, measures a nominal straight-flight violation bound, then selects the **latest safe-completing approval** on a **0.20 s grid**, no earlier than **0.60 s**. Comparative delayed-policy outcomes do not enter calibration. This makes all retained immediate commands feasible but does not guarantee positive-delay tolerance.

Operational monitoring begins at reset. Inspection stops at the control endpoint containing its first unsafe event; Geofence monitors through the deadline even after completion or violation. Inspection’s prediction/accuracy safety horizon is separately **8.5 s from flight start**; it must not be described as the 4.2 s operational deadline. Geofence uses its frozen scenario deadline for both safety and completion.

## V.B — Baselines and experimental protocol

| Original six-policy experiment | Attempts / accepted | Delays | Evaluations |
|---|---:|---:|---:|
| Inspection, seeds 20–319 | 300 / 276 | Eleven: 0–1 s by 0.10 s | 18,216 |
| Geofence, processed seeds 22–327 | 306 / 300 | Twenty-three over 0–1 s | 41,400 |

Inspection development used seeds 0–19 (19 feasible); Geofence development used seeds 0–21 (20 feasible). Inspection retains its 24 rejections without replacement; Geofence stops when 300 scenarios have been accepted. The **59,616 total counts policy evaluations**: Inspection includes **5,236 verified identical-trajectory reuse records**, so this is not 59,616 newly simulated independent flights.

The original six are No Recovery, Always Recover, Fixed TTL, FBS, Use-Time Recheck and **dispatch-time StaleGuard**. The latter predicts once at approval, checks readiness/expiry only at dispatch, and executes unverified when prediction is unavailable. Use-Time Recheck predicts from the dispatch state, continues nominal flight during measured computation, and applies its stored decision on the first strictly later 50 Hz tick without rechecking. Fixed TTL is **0.30 s Inspection / 0.40 s Geofence**, with equality permitted. FBS freezes a margin/feature-volatility TTL at approval. Always Recover’s original timing differs across missions; the exact table in Appendix B must be respected.

Geofence predicts on **19 delays**: 0–0.20 s by 0.02 s, then 0.30–1.00 s by 0.10 s. The benchmark additionally dispatches at **0.24, 0.44, 0.64 and 0.84 s**, without adding predictor samples or interpolation. All policies receive identical scenario/delay tests. Actual future D is provided to the test scheduler, not the approval predictor. Rates weight these grid points equally, not a measured deployment delay distribution.

Three separate Geofence comparisons each contain **300 × 23 × 2 = 13,800 physical evaluations**: dispatch-time StaleGuard versus StaleGuard-EarlyRecovery; EarlyRecovery versus **0.20 s Fixed-EarlyRecovery**; and EarlyRecovery versus PPR. EarlyRecovery cancels while pending once its prediction is ready and TTL expired. PPR periodically predicts whether further nominal waiting followed by recovery would remain safe; it does not predict the queued turn’s validity. These follow-ups revisit already-examined evaluation scenarios and are exploratory. The abstract’s 13,800 figure specifically refers to the fixed-early comparison.

## V.C — Research questions and metrics

- **RQ1: How does delay affect safety and completion?** Report actual No-Recovery outcomes by delay, distinguish the two objectives and inspect non-monotonicity.
- **RQ2: How accurate and fast is approval prediction?** Compare raw predictions and TTL dispatch decisions separately against actual No-Recovery safety, ignoring readiness. Completed dedicated accuracy artifacts cover **209 Inspection and 380 Geofence development comparisons**, not the independent evaluation cohorts. Raw predictions exist only at directly evaluated delays. Report confusion counts/rates, observable grid-resolved TTL errors and warmed computation distributions.
- **RQ3: How do policies trade safety against completion?** Report mutually exclusive safe completion, safe non-completion and unsafe outcomes, plus recovery attempts/successes/failures, overall and by delay. Pair policies on the same seed/delay and state their denominators. Recovery success requires safety of the entire monitored flight; it does not imply completion.

Repeated delays are dependent observations: the independent statistical unit is the **scenario**, so 6,900 pairs contain 300 scenario clusters. The original Inspection analysis has saved scenario-clustered bootstrap intervals and paired tests; current Geofence summaries are descriptive and do not yet provide matching uncertainty estimates. Appendix D distinguishes performed analysis from recommendations.

Prediction latency is measured in wall-clock time and replayed on simulated control ticks. This is not demonstrated concurrent onboard execution. Count StaleGuard computation once per scenario, Use-Time once per flight, and PPR per check with separate flight totals. FBS’s primary timer covers its approval update/freeze, not its whole observation history. No policy’s safety-only recovery is assumed guaranteed safe. Before paper submission, resolve the naming, horizon, sample-count and evidence-scope qualifications listed in Appendix F.

---

# Technical appendix: exact facts and drafting qualifications

## A. Scenario generation and mission predicates

### A.1 Inspection-Descent

Scenario generation uses NumPy’s seeded default random generator. The aircraft starts at x=y=0, usual altitude, with the nominal eastward velocity ramp. Initial draws are:

| Quantity | Original sampling/construction |
|---|---|
| Speed | Uniform 0.4–0.8 m/s |
| Usual height | Uniform 1.15–1.40 m; enforce at least 0.10 m above unsafe-height threshold |
| Structure start in X | Uniform 0.8–1.5 m |
| Structure length in X | Uniform 0.3–0.6 m |
| Unsafe lateral half-width | 0.35 m |
| Unsafe-height threshold | Uniform 0.90–1.05 m |
| Requested low height | Uniform 0.60 m to unsafe-height threshold minus 0.10 m |
| Initial low-duration draw | Uniform 0.4–1.0 s; this is the search starting value, not a calibrated-duration cap |
| Inspection point | x = structure start − 0.5 m; y = 0; z = approved low height |
| Generated approval time | max(0.1 s, inspection_x/speed + 0.2 s − initial_low_duration/2) |

Calibration rounds to 50 Hz control ticks. Approval candidates descend by five ticks from the generated approval, appending zero if necessary. For each approval, low-duration candidates ascend by five ticks from the rounded initial duration, up to nominal structure-arrival tick minus approval tick. The search evaluates every candidate and maximizes safe consecutive inspection dwell, retaining the first on ties. The selected duration can exceed 1 s: the frozen independent commands actually range **0.8–2.4 s**. Selected approvals range **0–1.56 s**. These ranges describe the retained dataset, not new sampling distributions.

The unsafe predicate is the **closed** set:

\[
x_{min}\le x\le x_{max},\qquad |y|\le y_{limit},\qquad z\le z_{unsafe}.
\]

This evaluates the simulated position, not the Geofence body-sphere clearance. It is not a physical contact/collision detector.

Inspection eligibility uses **strict** bounds:

\[
|x-(x_{min}-0.5)|<0.12\text{ m},\quad |y|<0.20\text{ m},\quad
|z-z_{low}|<0.08\text{ m}.
\]

Eligibility must persist for at least **180 physics ticks = 0.18 s**, measured by elapsed ticks, before the operational deadline. There is **no additional velocity constraint** in this mission’s completion predicate. Position and dwell are proxies for inspection; the experiment does not simulate camera imagery or inspection quality.

The descent changes only the reference altitude over the approved duration. It preserves the existing forward position/velocity reference. Recovery requests usual altitude while preserving that forward motion; it does not brake, hover, stop at the inspection point, reset the PID, or retry descent.

**Two different horizons must be retained:**

- Operational calibration/benchmark deadline: approval + 4.2 s, on the existing rounded control-step clock. Operational safety monitoring begins at reset and stops at the first unsafe control endpoint or deadline.
- Original TTL prediction and dedicated accuracy truth: **8.5 s from flight start**, so remaining prediction horizon is 8.5 s minus approval time. Dedicated truth observes safety from approval through that endpoint and continues after a first unsafe event. It cannot be replaced by the shorter operational benchmark journal.

The Inspection predictor additionally screens nominal continuation through its whole remaining 8.5 s horizon. It saves waiting checkpoints during that continuation, returns INVALID_NOMINAL if it predicts an unsafe nominal path, and stops individual commanded rollouts on a predicted unsafe swept segment. Therefore the paper’s generic “waiting only until the largest delay” algorithm is not a literal description of this implementation.

### A.2 Geofence Turn

The mission schema is **northbound_checkpoint_v2**. It uses a single virtual vertical fence at x = fence_x, with the allowed region west of that plane. The course turn does not require the UAV yaw to rotate 90°; the shared PID’s desired yaw remains zero.

A previous actual-flight characterization supplies eight feasible speed–radius combinations:

| Speed (m/s) | Feasible radii (m) |
|---:|---|
| 0.4 | 0.2, 0.4, 0.6 |
| 0.6 | 0.2, 0.4, 0.6 |
| 0.8 | 0.4, 0.6 |

Choose speed uniformly from the three values, then radius uniformly from the feasible values for that speed. This is not a uniform draw over all eight pairs. Characterization is a reused input; it is not rerun for every calibrated scenario. The 0.15 m tracking threshold belongs to characterization, not to final per-scenario acceptance.

| Quantity | Sampling/construction before approval calibration |
|---|---|
| Height | Uniform 1.15–1.40 m |
| Initial X / Y | Uniform −0.15 to 0.15 m / −0.10 to 0.10 m |
| Fence X | Initial X + uniform 1.5–2.5 m |
| Checkpoint size | 0.60 m in X × 0.20 m in Y × 0.20 m in Z |
| Checkpoint Y center | Initial Y + radius + speed × 2 s |
| Checkpoint Z center | Usual height |
| Checkpoint X center | Uniform between the earliest reachable lane + half-width and the fence’s body-safe limit − half-width; reject if no such interval exists |
| Deadline duration T | round_to_20ms(πR/(2v) + 2 s + uniform[1,2] s) |

The earliest lane used for checkpoint geometry is initial_x + v(0.60−0.20) + R. After generation, the region, command and deadline stay fixed during every calibration candidate and dispatch-delay comparison. The checkpoint is never used to steer the turn or generate a connector. The frozen 300-scenario deadline durations range **3.56–6.34 s**.

At dispatch, the reference uses a quarter-circle of radius R, speed v and duration πR/(2v), anchored at the **current nominal reference row**, followed by straight northbound motion. The anchor is not the measured aircraft position. No path replanning, westward correction, checkpoint-following connector, or re-optimization occurs. Delaying dispatch shifts the lane east by nominal waiting progress while keeping approved parameters fixed.

The body radius is the enclosing sphere of the simulated CF2X collision cylinder:

\[
r_{body}=\sqrt{0.06^2+(0.025/2)^2}=0.0612882534\text{ m}.
\]

Safety requires:

\[
clearance(t)=x_{fence}-x(t)-r_{body}\ge0.05\text{ m}.
\]

Equivalently, the center must remain at or west of x_fence − 0.1112882534 m. The 5 cm margin is applied once. Equality is safe. This sphere approximates the simulator’s collision body and does not establish coverage of all real UAV propellers.

Checkpoint position bounds are inclusive. Eligibility additionally requires:

\[
\|\mathbf v(t)-(0,v,0)\|_2\le0.12\text{ m/s}.
\]

Eligibility must persist continuously for **200 physics ticks = 0.20 s** and finish by the original approval-relative T. The spatial tolerances already define the box; no extra inflation is applied. Hovering in place does not satisfy the northbound velocity requirement. Monitoring begins at reset and continues through approval + T regardless of earlier completion or violation.

The approval upper bound is the first actual nominal straight-flight safety violation, rounded upward to 0.20 s. The old configuration field approval_max_s=3.20 is **not** the search cap. Frozen accepted approvals range **1.2–4.8 s**. The first passing descending candidate maximizes nominal patrol distance v(approval_time−0.20 s). All six rejected evaluation seeds—**28, 115, 128, 175, 204, 282**—were geometric checkpoint rejections.

## B. Detailed protocols, policies, predictor and scheduling

### Frozen cohorts and completed experiments

| Experiment | Accepted scenarios | Dispatch delays per scenario | Policies | Policy evaluations | Scenario–delay groups |
|---|---:|---:|---:|---:|---:|
| Inspection-Descent original benchmark | 276 | 11 | Original six | 18,216 | 3,036 |
| Geofence Turn original benchmark | 300 | 23 | Original six | 41,400 | 6,900 |
| Geofence: dispatch-time SG vs SG-EarlyRecovery | 300 | 23 | 2 | 13,800 | 6,900 pairs |
| Geofence: SG-EarlyRecovery vs Fixed-EarlyRecovery | 300 | 23 | 2 | 13,800 | 6,900 pairs |
| Geofence: SG-EarlyRecovery vs PPR | 300 | 23 | 2 | 13,800 | 6,900 pairs |

The first two rows are the **59,616-policy-evaluation six-policy comparison**. The three recovery comparisons are separate experiments; they must not be silently pooled into that total or into each other. The abstract’s 13,800-evaluation fixed-threshold comparison is specifically the fourth row.

Inspection-Descent development attempted seeds 0–19: **19 accepted, one rejected**. Its original independent evaluation attempted exactly seeds **20–319: 276 accepted, 24 rejected (92.0% acceptance)**. Geofence development accepted 20 of 22 attempted seeds, using seeds 0–21. Its evaluation processed seeds **22–327: 300 accepted, six rejected (98.04% acceptance)**, stopping at the target of 300 accepted scenarios. Although its configured candidate pool had 500 seeds, only 306 were attempted. These are different sampling/stopping rules and must be described accurately.

The original six-policy evaluations used scenarios separate from their mission’s development cohort. The later Geofence recovery experiments revisit the same already-examined 300 evaluation scenarios and are **exploratory follow-up comparisons**, not a new untouched validation cohort. A smoke-test folder named “development” is not evidence that its contained seeds are development seeds; some recovery smoke tests use evaluation seed 22.

### Exact delay grids

**Inspection-Descent:** prediction and dispatch both use eleven delays, 0–1 s in 0.10 s increments.

**Geofence approval prediction:** nineteen delays:

~~~python
0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20,
0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00
~~~

**Geofence benchmark dispatch:** the above grid plus **0.24, 0.44, 0.64 and 0.84 s**, giving 23 delays. These four dispatch delays are off the predictor’s grid and assess the existing TTL decision between directly predicted points. They do not trigger additional approval rollouts or interpolation. Every dispatch delay aligns with the 50 Hz control loop.

Every accepted scenario is evaluated at every protocol delay under every selected policy. Comparisons are paired by **mission, seed and dispatch delay**. The unknown delay is systematically enumerated by the experiment, not sampled from a measured deployment distribution. Aggregate rates weight tested grid points equally; the denser sampling near zero in Geofence is not a uniform continuous-time delay distribution.

### The original six policies

| Policy identifier | Information and computation | Decision/action timing |
|---|---|---|
| no_recovery | No prediction | Execute the approved command at scheduled dispatch |
| always_recover | No prediction | **Geofence:** reject and brake/hover at dispatch, including D=0. **Inspection:** immediate D=0 dispatch takes precedence; if still pending, cancel at approval and continue nominal forward/usual-altitude flight |
| fixed_ttl | Fixed lifetime: **0.30 s Inspection; 0.40 s Geofence** | Check at dispatch. Equality is permitted; recover only when elapsed control steps are strictly greater than the lifetime |
| fbs | Observe actual 50 Hz safety features through approval; freeze a margin/EMA-volatility TTL | At dispatch, recover if the frozen result is available and expired; otherwise execute, including fail-open when unavailable |
| use_time_recheck | At dispatch, capture the current physical/PID state and predict one immediate command execution | Continue nominal flight during computation. Apply that stored decision on the first control tick strictly after computation finishes; no second recheck |
| staleguard | One approval-time safety sweep per scenario; no actual future D input | At dispatch, reject if the forecast is ready and its first-unsafe TTL has expired. If unavailable, execute on schedule with no later cancellation |

**The six-policy staleguard identifier is the dispatch-time, or passive, gate.** It does not recover proactively while pending. If the paper uses “StaleGuard” as the name of the overall predictor-plus-early-recovery proposal, explicitly distinguish this gate from **StaleGuard-EarlyRecovery** in the methodology and result labels. Do not describe all 59,616 original evaluations as testing pre-dispatch recovery.

The Inspection Always-Recover behavior is a historical protocol difference, not a dispatch-time unconditional baseline identical to Geofence’s. It explains why Inspection has 276 safe completions for that policy: its D=0 command executions. Retain this distinction rather than presenting the six policy implementations as identical across missions.

### Separate proactive-recovery comparisons

**StaleGuard-EarlyRecovery** uses the same approval predictor and expiry rule as the dispatch-time gate. On each control tick while the command is pending, it recovers when the forecast is ready and expired, permanently cancelling the command. If no intervention has occurred, it uses the original StaleGuard gate at dispatch. Forecasts arriving after dispatch cannot cancel an already-executed command.

**Fixed-EarlyRecovery** uses **0.20 s in the completed Geofence comparison**. It cancels a pending turn when that timeout is reached, and also rejects dispatch exactly at the timeout. This is a different intervention time and expiry boundary from Geofence’s 0.40 s dispatch-only Fixed TTL. The artifacts establish the frozen threshold; do not invent an optimality or public-preregistration claim for its selection. Inspection has a 0.30 s implementation, but no completed full Inspection recovery comparison is present in the canonical evaluation results.

**PPR (Periodic Predictive Recovery)** periodically predicts recoverability of continuing patrol until a future monitoring opportunity and only then braking/hovering. It does not predict queued-turn validity. Geofence uses a **0.20 s monitoring interval** and a **20 ms computation allowance**; strict tick scheduling gives a two-tick, 40 ms latency allowance, so each model rollout waits **0.24 s** before starting the same recovery. Actual measured check latency still controls availability. Checks do not overlap, monitoring starts at approval, and dispatch wins a same-tick tie. If dispatch occurs before intervention, PPR executes the turn without a new dispatch checker. PPR is a project-specific baseline, not a reproduction of Althoff, Black-Box Simplex or another published method.

PPR’s 20 ms computation allowance is an engineering assumption. Its source documents a prior 25-check profile on development seeds 0–4 (4.04 ms mean, 6.12 ms maximum), rather than a certified worst-case bound. The completed PPR journal reports 15,111 checks and zero measured bound overruns. This does not certify future hardware timing. Inspection’s separately implemented PPR uses a different 0.10 s interval and 10 ms allowance, but has no completed full evaluation comparison here; do not transfer Geofence outcomes to it.

Within a paired approval-prediction comparison, both StaleGuard variants share one frozen forecast and its measured duration per scenario. Separate benchmark runs measure their forecasts again. Their recorded safety profiles are identical across the four completed Geofence benchmark runs, but their measured runtimes differ and consequently can change readiness and outcomes. Compare policies using pairs within the same experiment, rather than mixing denominators or forecast timings across directories.




### B.1 Common physical/PID state

The stored physical array order is:

~~~text
[x, vx, y, vy, z, vz, roll, pitch, yaw, p, q, r]
~~~

PID memory contains the three position integrals, three previous Euler angles, and three attitude integrals. Predictive checkpoints preserve these arrays, physical state and reference-step information. Each hypothetical dispatch restores its corresponding checkpoint; PID history is not zeroed. The shared source is at root-level model/, not the now-removed inspection_descent/model/ path mentioned by some old documentation.

The existing memory guards require process RSS at most 2,048 MiB and at least 2,048 MiB available host memory. These are simulation safeguards, not measurements of an onboard memory/CPU budget.

Vehicle parameters come from SCG’s CF2X URDF and PID controller. Principal values are mass **0.027 kg**, arm length **0.0397 m**, inertia diagonal **(1.4e−5, 1.4e−5, 2.17e−5) kg·m²**, gravity **9.8 m/s²**, thrust coefficient **3.16e−10**, and moment coefficient **7.94e−12** in the simulator’s conventions. PID gains, saturation, motor mixing and internal-state handling are reused rather than optimized for a policy.

Inspection prediction checks straight position segments between 50 Hz states for intersection with the closed unsafe volume. Geofence prediction uses the minimum endpoint body clearance on each such segment. These are model-based sampled/swept checks; neither constitutes a formal continuous-time reachable-set guarantee. Actual PyBullet monitoring at 1,000 Hz is also a finite sampling resolution.

Geofence includes prior-flight minimum clearance in the approval snapshot and evaluates the required waiting prefix only up to the largest hypothetical delay before branching. Inspection instead performs its full nominal-continuation screening described above. Do not state that both implementations execute exactly the same Algorithm 1 loop without acknowledging that distinction.

### B.2 Expiry and computation readiness

Let w be elapsed control ticks since approval and t_p the stored measured prediction duration. StaleGuard readiness is inclusive:

\[
ready(w)=[t_p\le w/50].
\]

The first predicted unsafe delay becomes an integer expiry tick k_tau. A ready forecast rejects when **w ≥ k_tau**. If no unsafe tested delay is found, there is no predicted expiry **within the tested delay grid**; this is not infinite validity. Inspection INVALID_AT_APPROVAL and INVALID_NOMINAL are treated as expiry at zero by the gate. Expiration uses the first predicted failure even when a later direct prediction becomes safe again.

This first-failure rule is conservative relative to the evaluated grid. It is **not** a safety bound for every intervening continuous delay: a coarse interval can contain an unsafe off-grid dispatch before the next predicted unsafe grid point.

For Use-Time Recheck and each PPR check, availability is the first control tick strictly after completion:

\[
k_{available}=k_{observation}+\lfloor50t_{compute}\rfloor+1.
\]

Even a computation below 20 ms therefore incurs one control interval before application. The aircraft follows its nominal reference during that simulated interval. For Use-Time Recheck, a SAFE decision predicts execution at its dispatch snapshot, but the command is actually applied later without another state check. The reference at actual application determines where the delayed maneuver begins.

The simulator computes forecasts synchronously and then **replays measured computation latency on the simulated control clock**. It does not block simulated dispatch until a result is ready, and it does not demonstrate real concurrent flight-control/predictor operation. StaleGuard’s cached approval prediction is reused over the scenario’s delay sweep; file I/O and journal writing are not charged as onboard forecast latency. Geofence’s returned computation timer covers model construction and forecasting; its dispatch snapshot capture is outside the Use-Time timer, whereas Inspection’s Use-Time timer includes snapshot capture. These are recorded computation measurements, not interchangeable end-to-end onboard processing benchmarks.

### B.3 FBS adaptation and timing interpretation

FBS observes actual UAV states every control tick from reset through approval, inclusive. The initial volatility is zero. With safety feature f_k:

\[
v_k=0.1|f_k-f_{k-1}|+0.9v_{k-1},\qquad v_0=0.
\]

Geofence uses f = 0.05 − body_clearance and safe-side margin m = −f. Inspection directly uses the following positive-safe margin:

\[
m=\max(x_{min}-x,\ x-x_{max},\ |y|-y_{limit},\ z-z_{unsafe}).
\]

For either feature convention, volatility uses absolute successive changes. At approval, freeze:

\[
TTL_{FBS}=\left\lfloor\min\left(210,\frac{\max(m,0)}{\max(v,10^{-12})}\right)\right\rfloor.
\]

Volatility has units metres per control sample, so the ratio gives ticks. At dispatch, recover only if its approval computation is ready and either m ≤ 0 or w > TTL_FBS. A positive-margin TTL permits equality; m=0 expires even though the mission safety predicate itself includes its boundary. There is no dynamics rollout and no future-state input.

The benchmark’s FBS computation_s is the timed approval observation update plus freeze. It excludes the total cost of prior observations. Inspection retains history timers; Geofence records them only in diagnostic traces. Report any cumulative observation cost separately when available.

## C. Definitions for analysis and reporting

For N completed policy evaluations:

- Unsafe rate = number unsafe / N.
- Safe-completion rate = number safe and completed / N.
- Safe-noncompletion rate = number safe and not completed / N.
- Recovery-attempt rate = number with an actual fallback activation / N.
- Recovery successes = fallback activated and entire monitored flight safe.
- Recovery failures = fallback activated and any monitored violation.

Recovery successes + failures must equal attempts. Recovery success does not imply mission completion. Rates condition on accepted D=0-feasible scenarios. Report the attempted/accepted calibration counts separately; do not silently count rejected missions as safe, unsafe or completed operational trials.

For a reference policy A and alternative B on the same seed/delay:

| Paired metric | Definition |
|---|---|
| Completion benefit | A safely completes; B does not |
| Completion harm | B safely completes; A does not |
| Unsafe prevented | B unsafe; A safe |
| Unsafe introduced | A unsafe; B safe |
| Net completion benefit | Benefits − harms = A safe completions − B safe completions |
| Net unsafe reduction | Prevented − introduced = B unsafe − A unsafe |

Choose the reference explicitly from the protocol. Some historical metric keys retain earlier names; their name alone is not evidence of which two policies were compared.

For prediction accuracy:

| Metric | Numerator / denominator |
|---|---|
| Accuracy | (Correct-safe + correct-unsafe) / all classified comparisons |
| False-safe rate | Predicted SAFE but actually unsafe / all actually unsafe comparisons |
| False-rejection rate | Predicted UNSAFE but actually safe / all actually safe comparisons |

A false-safe rate is **not** the fraction of predicted-safe commands that fail. A false rejection concerns safety; missing a mission objective does not make the command unsafe. The four confusion counts must sum to the number of evaluated comparisons. Undefined rates are null, not zero.

TTL error compares the predicted first unsafe **tested dispatch delay** with the actual first unsafe **tested dispatch delay**, not the later physical time when a fence/volume violation occurs. It is grid-resolved, not an exact continuous-time threshold. Geofence reports absolute error where both first-unsafe delays are observed, and separately lists scenarios with no unsafe delay within 1 s and non-monotonic safety. Inspection additionally reports INVALID_NOMINAL, INVALID_AT_APPROVAL, ALWAYS_SAFE, FINITE_TTL and NON_MONOTONIC classifications; its TTL-error summary excludes invalid-at-approval/invalid-nominal profiles. Missing profiles are not manufactured from a TTL alone.

Timing summaries count one StaleGuard forecast per scenario, not once for each repeated delay. Mean, median and p95 are reported with sample counts. Geofence uses nearest-rank p95; Inspection retains NumPy’s interpolated percentile. Do not merge those quantiles as if computed by one common definition. Dispatch-to-action delay uses the actual execution/recovery tick minus scheduled dispatch tick. Pre-dispatch intervention is excluded from that nonnegative-delay distribution and instead reported by its recovery lead time. For PPR, retain check count, time per check, total per flight and timing-bound overruns; its per-check runtime is not its whole-flight prediction cost.

Time origins must also be interpreted consistently. Benchmark event steps count 50 Hz ticks from flight start; benchmark first_violation_s is flight-start time, while Geofence completion_s is approval-relative. Geofence accuracy stores first_violation_s relative to approval and retains its flight-start counterpart in first_violation_flight_s. Benchmark horizon_s is the final flight-start timestamp, not the duration after approval. Original converted Inspection completion timestamps and physical runtimes are unavailable/null. Do not subtract dispatch delays from those fields to manufacture a TTL or missing runtime.

## D. Statistical uncertainty and the limits of current analysis

The **original Inspection independent evaluation** already includes a saved, specified analysis:

- 10,000 scenario-clustered bootstrap replicates, analysis seed 1729, percentile 95% intervals.
- Each resampled feasible scenario carries all eleven delays, six policies and outcome measures together.
- Aggregate/per-delay outcome rates and paired StaleGuard-minus-baseline differences receive intervals.
- Calibration feasibility uses a Wilson 95% interval over all 300 attempted seeds.
- Exact two-sided paired sign tests use scenario-level differences, excluding ties; all ties give p=1. Aggregate differences use each scenario’s counts over its eleven delays.
- Holm correction jointly covers 180 tests: five alternatives × three metrics × twelve strata (aggregate plus eleven delays).
- Scenario-any-unsafe Wilson intervals accompany possibly degenerate zero-event bootstrap intervals. Timing analysis also has scenario-clustered mean intervals.

These results remain under independent_evaluation_20_319/main_result/, including summary.json, protocol.json, paired_comparisons.json and six_policy_report.log. The converted compact metrics.json supplies descriptive summaries; it does not replace all that inferential analysis.

The completed **Geofence** compact metrics provide descriptive counts/rates, computation summaries and paired comparisons. They do **not** currently contain a corresponding scenario-clustered bootstrap or multiplicity-adjusted significance analysis. Do not claim that those tests or confidence intervals were performed for Geofence. If the paper needs a unified uncertainty method, scenario-level paired resampling is a suitable next analysis, but its settings and results must be explicitly established rather than invented during drafting. This document did not execute that analysis.

The delay grid, feasible-case selection, simple two-mission scenario families, perfect-state/controller-memory availability and fixed model parameters limit generalization. Equal grid weighting estimates performance on the tested protocol, not a real scheduler’s delay distribution. Repeated deterministic seeds do not represent multiple real aircraft or deployment sites. Do not explain all differences as check-to-use latency merely because one policy acts later; causal attribution requires an appropriate same-decision immediate-application counterfactual.

## E. Numerical cross-checks for the paper writer

This section checks identification and denominators; it is not proposed text for Section V’s results.

### E.1 Original six-policy outcome counts

Each Inspection policy has 3,036 evaluations; each Geofence policy has 6,900.

| Policy | Inspection unsafe | Inspection safe completion | Geofence unsafe | Geofence safe completion |
|---|---:|---:|---:|---:|
| No Recovery | 1,841 | 930 | 2,549 | 1,692 |
| Always Recover | 0 | 276 | 500 | 0 |
| Fixed TTL | 139 | 819 | 1,239 | 1,692 |
| FBS | 1,841 | 930 | 2,549 | 1,692 |
| Use-Time Recheck | 63 | 864 | 671 | 1,393 |
| StaleGuard, dispatch-time gate | 22 | 917 | 600 | 1,686 |

The Inspection journal has **12,980 native executions plus 5,236 verified-reference-reuse records**. Reuse concerns identical nonpredictive trajectories for Always Recover and Fixed TTL, with native boundary checks and matching recorded control/physics hashes. The original main_result/analysis_metrics.json records checking all 5,236 references. Geofence’s 41,400 original benchmark records are physical flight simulations. Thus “59,616 policy evaluations” is accurate; “59,616 newly simulated independent flights” is not. The original comparison represents 54,380 native flight simulations plus the verified reuse records, excluding calibration and diagnostics.

### E.2 The abstract’s early-versus-fixed claim

Use only geofence_turn/results/evaluation/fixed_early_benchmark/ for these figures:

| Policy | Evaluations | Unsafe | Unsafe rate | Safe completions | Safe-completion rate |
|---|---:|---:|---:|---:|---:|
| StaleGuard-EarlyRecovery | 6,900 | 98 | 1.4203% | 1,684 | 24.4058% |
| Fixed-EarlyRecovery, 0.20 s | 6,900 | 315 | 4.5652% | 1,692 | 24.5217% |

These are 13,800 total physical evaluations forming 6,900 pairs over 300 scenarios. The safe-completion difference is **−8 / 6,900 = −0.11594 percentage points**, which rounds to the draft’s −0.12 percentage points. Do not divide either policy’s outcomes by 13,800. Do not describe this as an Inspection-Descent experiment or a fresh held-out cohort.

StaleGuard is remeasured in each independent benchmark run. For example, its Geofence original benchmark has 600 unsafe outcomes; the separately paired passive-versus-early run has 612 passive unsafe and 112 early unsafe; the fixed comparison has 98 early unsafe; the PPR comparison has 101 early unsafe. Their safety forecasts agree, but measured readiness differs. Use the correct within-run comparison instead of presenting these as one immutable StaleGuard outcome table.

### E.3 Dedicated prediction-accuracy evidence

| Measurement on development data | Inspection | Geofence |
|---|---:|---:|
| Scenarios / comparisons | 19 / 209 | 20 / 380 |
| Correct-safe | 79 | 219 |
| Correct-unsafe | 126 | 158 |
| False-safe | 2 | 1 |
| False-rejection | 2 | 2 |
| Raw accuracy | 98.0861% | 99.2105% |
| TTL-decision accuracy on this same grid | 98.0861% | 99.2105% |
| Observable TTL-error cases | 18 | 19 |
| Mean absolute TTL error | 0.01667 s | 0.003158 s |

Geofence had no actual safety loss observed through 1 s for seed 16; it is excluded from its finite-error denominator rather than extrapolated. Both accuracy experiments reuse compatible recorded actual No-Recovery truth. They do not simulate runtime recovery decisions. Agreement of raw and TTL counts on these grids does not establish off-grid accuracy.

### E.4 Recorded prediction timing examples

| Official saved experiment | Forecast samples | Mean / median / p95 (ms) |
|---|---:|---|
| Inspection original StaleGuard benchmark | 276 | 43.01 / 42.26 / 61.39 |
| Geofence original StaleGuard benchmark | 300 | 84.74 / 83.70 / 104.29 |
| Geofence fixed-early comparison, SG-EarlyRecovery | 300 | 80.44 / 78.64 / 106.53 |

These are warmed forecast timings from their own experiments, not a universal StaleGuard constant. Do not substitute the earlier 42 ms small development diagnostic for the 300-scenario Geofence benchmark. Geofence run metadata records Python 3.10.21 for the original benchmark/PPR run and Python 3.10.22 for the passive-versus-early/fixed-early runs; NumPy is 2.2.6. Recorded Linux kernel/platform strings also differ. The inspected compact provenance does not establish an exact CPU model or controlled cross-run hardware/load equivalence. Obtain that separately if the paper requires it. Do not infer an embedded performance guarantee from desktop wall-clock measurements.

## F. Alignment issues in the supplied LaTeX draft

These are factual qualifications for the writer; no LaTeX file was edited.

1. **Policy naming:** the proposed pre-dispatch policy is the staleguard_early_recovery implementation, whereas the six-policy staleguard implementation is dispatch-only. Explain this distinction explicitly.
2. **Evaluation count:** use “59,616 policy evaluations” or explain native simulation plus verified reuse. Treat the 13,800 fixed-early evaluations as a separate comparison.
3. **Horizon:** a fixed horizon is fixed for a scenario’s delay sweep. It is not a single common approval-relative duration across both missions. Inspection’s safety predictor ends at flight time 8.5 s, while its operational deadline is approval + 4.2 s. Geofence uses its frozen scenario T for safety and completion.
4. **Algorithm 1:** Geofence’s prefix algorithm agrees with waiting only to the largest candidate delay. Inspection additionally screens the whole nominal horizon and has invalid-nominal/early-termination handling. Either label the algorithm a conceptual abstraction and state this implementation difference, or revise its description later.
5. **State layout:** the mathematical vector in the draft uses position then velocity, while arrays interleave position and velocity. A mathematical permutation is acceptable if notation is consistent; do not claim it is the literal stored order.
6. **First-unsafe conservatism:** it is conservative over evaluated grid results, not guaranteed safe at untested continuous delays.
7. **Independent evidence:** the saved standalone accuracy results are development-only. Recovery comparisons revisit previously examined Geofence scenarios. Do not call either a new independent validation.
8. **Physical setting:** “border surveillance” motivates the Geofence mission, but the experiment models a fence plane and checkpoint, without a geographic border, surveillance sensor, mission-planning agent or real queue implementation.
9. **Incomplete statistical work:** retain original Inspection uncertainty where appropriate; do not claim new Geofence intervals/tests that are absent from saved analysis.

## G. Evidence, source ownership and reproducibility

All paths below are relative to examples/staleguard. They identify evidence; the narrative above does not require the web writer to open the repository.

| Facts | Canonical source/evidence |
|---|---|
| Shared dynamics, PID and checkpoint/restore | [model/lightweight.py](model/lightweight.py), [model/common.py](model/common.py), [model/scg.py](model/scg.py), [model/__init__.py](model/__init__.py) |
| Actual physics observation timing | [physics_monitor.py](physics_monitor.py) |
| Vehicle and SCG controller parameters | ../../safe_control_gym/envs/gym_pybullet_drones/assets/cf2x.urdf and ../../safe_control_gym/controllers/pid/pid.py |
| Inspection generator, reference edits, geometry, objective | [inspection_descent/scenario.py](inspection_descent/scenario.py) |
| Inspection ordered maximum-dwell calibration | [inspection_descent/calibration.py](inspection_descent/calibration.py), functions command_candidates, inspection_dwell_ticks and calibrate_scenarios |
| Inspection physical execution, readiness and recovery | [inspection_descent/simulation.py](inspection_descent/simulation.py), functions run_actual, execute, capture_approval and run_ground_truth |
| Inspection policy implementations | [inspection_descent/baselines/staleguard.py](inspection_descent/baselines/staleguard.py), [use_time_recheck.py](inspection_descent/baselines/use_time_recheck.py), [fbs.py](inspection_descent/baselines/fbs.py), [fixed_ttl.py](inspection_descent/baselines/fixed_ttl.py), [always_recover.py](inspection_descent/baselines/always_recover.py) |
| Inspection original completed protocol, inference and reuse | [protocol.json](inspection_descent/independent_evaluation_20_319/main_result/protocol.json), [summary.json](inspection_descent/independent_evaluation_20_319/main_result/summary.json), [analysis_metrics.json](inspection_descent/independent_evaluation_20_319/main_result/analysis_metrics.json), [six_policy_report.log](inspection_descent/independent_evaluation_20_319/main_result/six_policy_report.log) |
| Geofence generation, turn construction, clearance, checkpoint, recovery | [geofence_turn/scenario.py](geofence_turn/scenario.py) |
| Geofence approval search | [geofence_turn/calibration.py](geofence_turn/calibration.py), functions find_nominal_bound, approval_candidates, verify_calibration and calibrate_scenarios |
| Feasible turn pairs | [envelope.json](geofence_turn/support/logs/northbound_review/characterization/envelope.json) |
| Geofence actual monitoring and all policy action scheduling | [geofence_turn/simulation.py](geofence_turn/simulation.py), function execute |
| Geofence forecast and readiness | [geofence_turn/baselines/staleguard.py](geofence_turn/baselines/staleguard.py), functions predict_delays and staleguard_status |
| Geofence fixed early threshold and PPR target | [fixed_early_recovery.py](geofence_turn/baselines/fixed_early_recovery.py), [ppr.py](geofence_turn/baselines/ppr.py) |
| Delay grids and cached approval forecasts | Each mission’s baseline_benchmark/run.py and each experiment’s saved run.json |
| Outcome, timing and paired definitions | [Geofence metrics.py](geofence_turn/baseline_benchmark/metrics.py), [Inspection metrics.py](inspection_descent/baseline_benchmark/metrics.py) |
| Raw/TTL accuracy and original truth horizons | Each mission’s ttl_pred_accuracy/run.py and metrics.py, plus results/development/ttl_pred_accuracy/ |

The frozen calibration manifests are each mission’s results/evaluation/calibration/calibrated_scenarios.json. They preserve scenario parameters, approval choices and snapshot checksums. Inspection’s manifest references the original historical snapshots; Geofence stores snapshots alongside its calibration. Class-location handling and reviewed source relocations let readers validate unchanged old snapshot bytes without retaining duplicate source modules.

Each completed policy benchmark has **run.json, executions.jsonl and metrics.json**. The journal is authoritative; metrics are derived without rerunning PyBullet. Protocol and source/configuration provenance, approval snapshot checksums, cached forecasts and measured timing are recorded. Journals are incrementally flushed, duplicate keys rejected, and committed content checked by checksum for resumption. Historical Inspection conversion is explicitly labelled historical_schema_conversion and cannot be mixed with new physical records. Missing original flight runtimes and completion timestamps remain null. Source relocation does not rewrite historical hashes; old runs may fail current-source resume checks and require a fresh output.

For this brief, read-only checks verified the complete keys, journal checksums, outcome/recovery totals, frozen scenario parameters and every snapshot checksum for the two original benchmarks and all three completed Geofence recovery comparisons. No new calibration, prediction, flight, benchmark or statistical experiment was run. No scientific implementation or historical artifact was modified.

**Recommended factual wording for the main count:** “We compared six policies in 59,616 scenario–delay–policy evaluations across two simulated missions, retaining verified identical-trajectory reuse in the historical Inspection-Descent dataset. We separately compared proactive recovery policies in paired Geofence experiments.” State which of those separate comparisons is actually included in the paper.
