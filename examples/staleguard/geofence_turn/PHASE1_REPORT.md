# Phase 1: a delayed left turn near a geofence

We changed the mission so that the UAV makes its approved left turn and then flies north through a fixed checkpoint. The previous path could turn back west to reach a target point; that artificial return has been removed. For illustration, at 0.6 m/s, a half-second wait moves a 0.4 m-radius turn 30 cm farther east. The radius and checkpoint stay unchanged, so the later flight may miss the checkpoint.

All five development cases completed safely when the turn was dispatched immediately. Their smallest body-to-fence gap was 8.55 cm, above the required 5 cm. In a separate flight, a 0.6 s delay caused both a checkpoint miss and a fence violation. The warmed safety predictor took a median 42.3 ms, but this was a **diagnostic benchmark, not an official mission-run timing measurement**. The controller also made small westward corrections after overshooting the turn. Next, we should measure prediction readiness during actual mission runs and compare predicted safety with a small set of delayed PyBullet flights before expanding calibration.

## What the experiment tests

The UAV accelerates east during its first 0.4 s and continues east while the command waits. When dispatched, it follows a quarter-circle to the left and then flies straight north at its approved speed and altitude. The turn starts at the planned patrol position for that execution time, rather than being repositioned using the measured aircraft position. A delay therefore moves the turn east; the path is never adjusted to recover a missed checkpoint.

Before trying approval times, we choose one turn radius and one checkpoint box. The box is 0.60 m wide east–west, 0.20 m long north–south, and 0.20 m high, and it fits inside the permitted fence boundary with the body-clearance requirement included. It lies approximately two seconds of northbound travel beyond the turn. To complete the mission, the UAV must remain inside for 0.20 s, with velocity within 0.12 m/s of the approved northward velocity, before its deadline from approval. A stationary hover does not qualify. These dimensions, tolerances and deadline allowances are experimental assumptions, not established UAV requirements.

The simulated body must stay at least 5 cm from the fence until the deadline, even after passing the checkpoint. We check the actual PyBullet position every millisecond, so a brief violation while the command is waiting makes the flight unsafe; a later turn cannot erase it.

Calibration asks how far east the UAV can patrol before an immediately executed turn becomes too late. With the radius, checkpoint and deadline fixed, we try approval times 0.2 s apart and select the latest one that completes safely. This search uses only immediate dispatch, written as D=0. StaleGuard forecasts and delayed-policy outcomes do not influence the choice.

## What we observed

| Measurement | Observed result | What it means |
|---|---|---|
| Turning tests | Eight of nine speed/radius pairs met the 15 cm tracking-error limit. At 0.8 m/s and radius 0.2 m, error reached 15.78 cm. | The tightest fast turn was excluded from scenario generation. |
| Immediate-dispatch development flights | All five attempted cases completed safely. Completion took 2.864–3.250 s after approval, before each case's own deadline of 3.98–4.98 s. | The corrected mission is feasible in this small sample. |
| Delayed flight, case 0 | A 0.6 s delay missed the checkpoint; minimum body clearance was −33.73 cm. | The modeled body crossed the fence. The flight failed despite keeping the same approved turn. |
| Diagnostic predictor timing | Fifty warmed full forecasts: median 42.3 ms, maximum 52.3 ms. First calls before warm-up took 155–361 ms in two runs. | Computation is in the requested range after initialization; this does not establish readiness during a mission. |

The generator processed ten seeds, but calibration stopped after the first five cases all passed, as planned. These results demonstrate feasibility; they are not a population success rate or a comparison of recovery policies.

## What remains and the next decision

The requested path never travels west, but the unchanged position controller corrects turn overshoot with **5.8–9.7 cm of actual westward motion**. Thus the path command no longer backtracks, but the measured aircraft motion is not strictly eastward-or-northward. The body-size approximation and mission tolerances also remain assumptions that need review before a larger study.

The timing benchmark reused saved approval states and excluded state collection and initialization. It called the real predictor, but did not measure the mission workflow used by Inspection-Descent's `mission.py`. Next, we should time forecasts during mission execution, preserve the measured wait for a result, and compare forecasts with actual delayed flights. A faster forecast cannot undo a violation before dispatch, so this benchmark alone does not establish that recovery will be timely or safe.

The [technical appendix](PHASE1_TECHNICAL_APPENDIX.md) contains the equations, individual cases, test evidence, assumptions, file changes, and reproduction commands.
