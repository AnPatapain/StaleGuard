# Underpass Inspection: positive-world-Y audit and fresh verification

The pre-edit audit found that the approved command already swerved toward
**positive world Y**, not negative Y. Its physical profile and pier geometry
were therefore retained. A positive-displacement parameter check, live world-Y
position/velocity readout and restored stock checkerboard floor make that intent
explicit. In the fresh immediate flight, the UAV finishes at **Y=+0.8478 m**;
at flight time 4.00 s it has **Y=+0.2523 m** and **Vy=+1.4468 m/s**. It completes
inspection safely with **0.1465 m** minimum pier clearance. The requested new
154-flight sweep measures command expiration at **0.14 s**, sideways recovery
safe through **0.94 s**, and a **+0.80 s** gap. Climb remains safe through 1 s,
so its expiration and exact gap remain unmeasured. Every fresh physical trajectory
matches its original counterpart exactly; the original pilot and reports remain
unchanged. These are repeated deterministic measurements of an already-positive
command, not evidence of a physical sign reversal. Next, use the world-coordinate
readout when inspecting direction and extend the climb-start range before making
claims about its recovery expiration.

## Mission and success conditions

The aircraft approaches a bridge at deck height, dives beneath it, spends at
least 0.18 s in the fixed inspection region, then moves sideways around a pier.
The same prescribed displacement begins from the measured dispatch position;
the obstacles stay fixed. Safe inspection completion requires the dwell and
no body-clearance violation anywhere in the full flight. Climb and sideways
recovery abandon inspection while continuing forward above or beside the deck.

Every flight runs to 8.8 s from reset, or 8 s after approval. Safety is observed
at 1,000 Hz against the original deck/pier volumes and unchanged 0.05 m body
clearance requirement. Structures remain visual-only; clearance violations are
detected numerically rather than by their contact forces.

## What the direction audit actually found

| Actual immediate-dive measurement | Original flight | Fresh flight |
|---|---:|---:|
| Y at flight time 4.00 s | +0.2523 m | +0.2523 m |
| Vy at flight time 4.00 s | +1.4468 m/s | +1.4468 m/s |
| Maximum Y | +1.0413 m | +1.0413 m |
| Final Y | +0.8478 m | +0.8478 m |

The original commanded lateral displacement was already +0.85 m. During the
fresh physical swerve, Y rises from −0.0032 m to +1.0411 m. The few millimetres
of initial negative tracking drift do not describe the commanded swerve's
direction. The aircraft overshoots, then corrects toward the requested level;
its minimum Vy is −0.6205 m/s at flight time 4.84 s. That correction explains
negative velocity readings during settling. All 51 delayed dives finish on
the positive-Y side of their dispatch anchors.

The pier remains centered at **X=2.55 m, Y=0**, with its original dimensions.
No geometry adjustment or threshold change was needed. The fixed camera is
unchanged; screen direction is a projection, while the live readout explicitly
reports physical world Y and Vy.

## Fresh physical feasibility and timing results

| Flight | Safe throughout? | Inspection completed? | Minimum body clearance |
|---|---|---|---:|
| Nominal | No; deck front | No | −0.2605 m |
| Immediate dive | Yes | Yes | 0.1465 m |
| Immediate climb | Yes | No | 0.4388 m |
| Immediate sideways | Yes | No | 0.3747 m |
| Dive delayed 0.40 s | No; pier | Yes | −0.0885 m |

| Quantity, approval-relative | Fresh observed result |
|---|---|
| First unsafe dive delay, t_cmd | **0.14 s**; safe through 0.12 s |
| Climb expiration, t_rec_climb | **Not bracketed**; latest safe tested start 1.00 s |
| Latest safe sideways start, t_rec_sideways | **0.94 s**; first unsafe 0.96 s |
| Climb gap | **Undefined**; no expiration observed |
| Sideways gap | **+0.80 s** |

The full tested safety profiles are dive safe at 0–0.12 s and unsafe at
0.14–1.00 s; climb safe at every tested start; sideways safe at 0–0.94 s and
unsafe at 0.96–1.00 s. No unsafe-to-safe reversal was observed. These are grid
measurements at 20 ms spacing, not exact continuous-time guarantees. All 51 dive
flights complete raw inspection dwell, but only 7 safely complete the whole flight.

The renewed results support a pre-dispatch opportunity for the specified sideways
recovery. They still do not demonstrate dispatch-time climb failure within 1 s.
The single fixed scenario, ideal state access, provisional inspection/maneuver
settings and untested prediction readiness limit broader conclusions. No predictive
policy was implemented or evaluated. Full file changes, validation evidence and
commands are in the [technical appendix](TECHNICAL_APPENDIX.md).
