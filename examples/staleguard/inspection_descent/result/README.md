# Experiment results

These logs use the saved maximum-dwell calibrations for seeds 0–19.
Seed 16 is infeasible, so each experiment evaluates 19 scenarios.
All use delays from 0.0 to 1.0 s in 0.1 s steps.

| Experiment | Terminal output |
|---|---|
| TTL prediction accuracy on the calibrated commands | [ttl_accuracy_calibrated.log](ttl_accuracy_calibrated.log) |
| Experiment A, known delay: safety-only recovery | [baseline_safety_only.log](baseline_safety_only.log) |
| Experiment B, unknown delay: StaleGuard TTL checked at dispatch; safety-only recovery | [baseline_unknown_dispatch.log](baseline_unknown_dispatch.log) |
| Same comparison: recovery can stop and retry inspection | [baseline_mission_retry.log](baseline_mission_retry.log) |

The comparison experiments use T=4.2 s from command receipt. TTL accuracy keeps
its existing 8.5 s observation horizon. Fallback success in the safety-only
experiment means no unsafe entry, not completed inspection.

Run from `examples/staleguard`:

```sh
python inspection_descent/mission.py --calibrated --count 20
python inspection_descent/staleguard_benefit.py --count 20
python inspection_descent/staleguard_benefit.py --count 20 --experiment B
python inspection_descent/staleguard_benefit.py --count 20 --recovery mission-retry
```

The Experiment A and mission-retry logs remain unchanged copies of the earlier
completed runs. Experiment B is a separate execution; it does not replace A.
The TTL log uses calibrated commands. Older TTL outputs using generated
commands are preserved in `archive/`. This `result/` folder is not Git-ignored.
