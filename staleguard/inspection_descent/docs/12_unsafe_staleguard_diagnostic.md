> Source relocation: this report preserves its historical findings and commands. Current code: `staleguard.py` → [staleguard.py](../baselines/staleguard.py); `ground_truth.py` → [run.py](../ttl_pred_accuracy/run.py). Old entry points have been removed.

# Three unsafe StaleGuard executions — Experiment B

All three saved flights were reproduced with identical outcomes and complete
control/physics trace hashes. Only seeds 9/D=0.3, 7/D=0.4, and 8/D=0.4 were
investigated. No experiment source, calibration, predicate, or prior result was
changed. Simulations ran serially with the existing memory guard and launch lock.

Times below are seconds from **mission start**, not approval or dispatch.
Actual entry means the first unsafe sample at the existing 1,000 Hz monitor;
continuous entry lies between that sample and the preceding millisecond.
"Predicted TTL" is the existing first-unsafe-delay cutoff, not last-safe delay.

| Seed | Delay | Predicted TTL | Predicted safety at D | Actual unsafe-entry time | Root cause |
|---:|---:|---:|---|---:|---|
| 9 | 0.3 s | 0.4 s | SAFE | 1.829 s | Temporal safety-check resolution: even the predicted path crosses the unsafe volume between checked states. |
| 7 | 0.4 s | 0.5 s | SAFE | 2.254 s | Model error: x is underpredicted by 4.507 mm at actual entry, reversing the order of structure entry and climb clearance. |
| 8 | 0.4 s | 0.5 s | SAFE | 2.091 s | Model error: x is underpredicted by 12.828 mm at actual entry, again predicting clearance before structure entry. |

## Commands and dispatch decisions

Commands are loaded unchanged from `calibrated_scenarios.json`. Each requests
low altitude for the given duration, then usual height, while continuing forward.
The JSON evidence also contains each complete calibrated scenario and geometry.

| Seed | Approval | Dispatch | Low height | Low duration | Usual height | Forward speed |
|---:|---:|---:|---:|---:|---:|---:|
| 9 | 0.20 s | 0.50 s | 0.881398 m | 1.12 s | 1.221704 m | 0.748100 m/s |
| 7 | 0.40 s | 0.80 s | 0.814042 m | 1.30 s | 1.374303 m | 0.650038 m/s |
| 8 | 0.06 s | 0.46 s | 0.729247 m | 1.36 s | 1.396819 m | 0.530789 m/s |

All classifications are `FINITE_TTL`. First predicted unsafe delays are exactly
the TTL values in the first table; last predicted safe delays are respectively
0.3, 0.4, and 0.4 s. The original recorded prediction durations were respectively
**41.919, 66.012, and 63.652 ms**, all shorter than their dispatch delays.

The dispatch gate therefore had a result and correctly evaluated expiry as
**15 >= 20: false**, **20 >= 25: false**, and **20 >= 25: false** in control
steps. It allowed the original commands. There is no late-result, equality,
floating-point comparison, or dispatch-decision error in these cases.

## Trajectory evidence

At actual entry, x must be at least `x_min` and z at most `z_limit`; x is also
within the structure's upper bound, and both actual and predicted y are within
the unchanged ±0.35 m limit. Values below are metres.

| Seed | x_min | z_limit | Actual (x, z) at entry | Predicted (x, z) at the same time |
|---:|---:|---:|---|---|
| 9 | 1.222204 | 1.007411 | (1.222244, 1.007298) | (1.224073, 1.007078) |
| 7 | 1.342980 | 0.945025 | (1.343247, 0.940677) | (1.338740, 0.940115) |
| 8 | 1.023098 | 1.030484 | (1.023519, 1.013644) | (1.010690, 1.013364) |

**Seed 9 — missed inter-sample entry.** Actual unsafe samples occur at 1.829 s
only; 1.828 and 1.830 s are safe. Diagnostic dense evaluations of the existing
model are unsafe at 1.827–1.829 s. At the adjacent 50 Hz checks, the model is
still before the structure at 1.820 s (x=1.216793), then above the unsafe height
at 1.840 s (z=1.013654). Even the intermediate 100 Hz endpoint at 1.830 s is
already clear (z=1.007683). Thus checking only the existing 10 ms integration
endpoints would also miss this case. The error is temporal collision detection,
not interpolation of the 0.1 s dispatch-delay grid: D=0.3 is explicitly simulated.

**Seed 7 — false-safe model geometry.** Actual unsafe samples span
2.254–2.257 s; 2.258 s is clear. The model predicts climbing above the unsafe
threshold at 2.259 s and first reaching the structure at 2.261 s. Its 4.507 mm
x lag at actual entry makes that entry appear 7 ms later. Predicted altitude
at actual entry is actually 0.562 mm lower than PyBullet, so optimistic altitude
is not the explanation. The false-safe decision comes from predicted x still
being outside the structure. Dense 1 ms model checks do not detect an entry.

**Seed 8 — false-safe model geometry.** Actual entry is 2.091 s, and execution
stops at 2.100 s while still unsafe; the exit time is therefore not measured.
The model predicts climb clearance at 2.107 s and structure entry at 2.114 s,
23 ms later than actual entry. At actual entry its z error is only -0.281 mm,
while its x lag is 12.828 mm. Again, the model predicts that x remains outside
until altitude is safe. Dense 1 ms model checks do not detect an entry.

These observations isolate the classification-changing horizontal prediction
error in seeds 7/8. They do not uniquely attribute its accumulation to a single
dynamics parameter or integration approximation.

## Why the TTL-accuracy log shows only one false-safe among these cases

The two experiment kinds currently observe safety at different temporal rates.
`ground_truth.py` checks saved **50 Hz** states, whereas the operational runner
checks **1,000 Hz** positions. The brief seed 9 and seed 7 entries fall entirely
between ground-truth samples. Consequently the saved calibrated accuracy log
reports SAFE/SAFE there, but SAFE/UNSAFE for seed 8, which is still unsafe at
2.100 s. This evaluation blind spot hides two failures; it does not cause the
operational dispatch decisions.

## Minimum corrections recommended, not implemented

1. For seed 9, check path segments for intersection with the unsafe volume
   between prediction samples, or use sufficiently fine event monitoring.
   Checking only the existing 100 Hz endpoints is insufficient.
2. For seeds 7/8, finer safety checks alone are insufficient. Accepted paths
   need a validated model-error allowance, or higher-fidelity verification with
   physics-step safety monitoring. These three selected failures do not justify
   a universal numerical margin or an arbitrary one-grid-step TTL reduction.
3. Align TTL ground-truth monitoring with the operational physics-step monitor
   before interpreting accuracy metrics as operational safety accuracy.

No dispatch-gate change is indicated. No correction was implemented.

## Reproduction and evidence

From `examples/staleguard`, using the SCG Python environment:

```sh
python inspection_descent/diagnostics/diagnose_unsafe_staleguard.py --output /tmp/staleguard-unsafe-replay
```

Output must be a new directory. The diagnostic reproduces the three saved
predictions, dispatch decisions, and exact physical traces. It verifies matching
approval states and predictive checkpoints. Partial RK4 evaluations add 1 ms
diagnostic samples without feeding them back into the rollout; every existing
20 ms model endpoint is asserted bit-for-bit identical to the native model.

- [Diagnostic source](../diagnostics/diagnose_unsafe_staleguard.py)
- [Numerical summary and near-entry samples](../results/unsafe_staleguard_diagnostic/summary.json)
- Full state/position arrays: `results/unsafe_staleguard_diagnostic/seed_{9,7,8}.npz`.
- Original inputs: `results/unknown_dispatch_delay/executions.jsonl`,
  `calibrated_scenarios.json`, and `result/ttl_accuracy_calibrated.log`.

The executed diagnostic verified unchanged hashes for 18 existing scientific
input, source, and result files. Existing experiment output was not overwritten.
