# Underpass Inspection: physical feasibility pilot

The new seed-0 mission physically flies a UAV beneath a bridge, inspects a fixed
region, and swerves around a pier. The same maneuver starts from the aircraft's
measured position whenever dispatch occurs. In 154 actual PyBullet flights,
the first unsafe command delay was **0.14 s**, caused by inadequate pier clearance.
Sideways recovery was safe through **0.94 s** and unsafe from **0.96 s**, giving a
**0.80 s measured deadline gap**. Climb recovery stayed safe through the entire
0–1 s grid, so its expiration and exact gap remain unmeasured. Immediate dive
completed safely; both immediate recoveries were safe; straight flight violated
deck clearance. All four modes were inspected in actual GUI window screenshots,
and GUI trajectories exactly matched headless execution. This supports a physical
early-intervention opportunity for the specified sideways recovery, but does not
demonstrate dispatch-time climb failure within the tested range. One geometry,
ideal state access, provisional maneuver settings, and visual-only obstacles limit
the result. Next, extend only the climb-start range with the same geometry to
measure its boundary before introducing any predictive policy.

## What the UAV does and what counts as success

The UAV accelerates to an eastward reference speed of 0.6 m/s at altitude 1.35 m.
Approval occurs 0.8 s into flight. The bridge front is at X=1.50 m, and its slab
occupies altitudes 1.15–1.55 m, directly across the nominal flight. The approved
command smoothly drops 0.60 m, flies under the deck through a fixed inspection
region, then moves 0.85 m sideways to avoid a pier centered at X=2.55 m, Y=0.
The climb and sideways alternatives cancel inspection and keep flying forward
above or beside the deck, respectively. Their definitions are evaluated separately.

Inspection requires 0.18 s of continuous presence inside the fixed XYZ region.
Safe completion additionally requires no clearance violation anywhere in the
flight. Safety requires at least 0.05 m between the existing UAV body sphere and
either structural volume. Structures exert no PyBullet contact forces: entering
their clearance envelope is detected numerically. Thus unsafe flights can continue
through a visible structure, allowing the whole fixed horizon to be evaluated.
Every flight is observed at 1,000 Hz from reset to 8.8 s, or 8 s after approval.

## Observed physical feasibility

| Flight | Safe for whole flight? | Inspection dwell completed? | Minimum body clearance |
|---|---|---|---:|
| Nominal straight flight | No; deck front | No | −0.2605 m |
| Immediate dive | Yes | Yes, at approval +2.190 s | 0.1465 m |
| Immediate climb | Yes | No | 0.4388 m |
| Immediate sideways | Yes | No | 0.3747 m |
| Dive delayed by 0.40 s | No; pier | Yes, before violation | −0.0885 m |

Nominal flight first violates clearance at approval +1.726 s. Delaying the dive
by 0.40 s moves its initiation point 0.2661 m east, while the same pier and
inspection region remain fixed. That command completes inspection but later
violates pier clearance at approval +3.259 s. This is a measured separation
between inspection completion and whole-flight safety, not a prediction.

## Observed deadlines on the 20 ms grid

| Quantity | Measured result | Interpretation |
|---|---:|---|
| First unsafe dive delay, t_cmd | **0.14 s** | Safe through 0.12 s; first violation is at the pier |
| Latest safe sideways start, t_rec_sideways | **0.94 s** | Unsafe at 0.96–1.00 s; deck-front clearance loss |
| Sideways gap | **+0.80 s** | 0.94 − 0.14 s |
| Climb expiration, t_rec_climb | **Not established** | Every tested start, including 1.00 s, is safe |
| Exact climb gap | **Undefined** | A safe climb is demonstrated 0.86 s after command expiry |

The complete observed safety profiles are dive safe at 0–0.12 s and unsafe at
0.14–1.00 s; sideways safe at 0–0.94 s and unsafe at 0.96–1.00 s; climb safe at
every tested start. These are tested grid points, not continuous-time guarantees.
No unsafe-to-safe reversal was observed. All 51 dive flights completed inspection
dwell, while only 7 safely completed the whole flight. All 44 unsafe dives first
violated pier clearance; no deck-front or underside dive failure appeared here.
Climb never inspected; sideways never inspected.

## Meaning, limitations, and next decision

At a 1.00 s dispatch, both executing the dive and starting sideways recovery are
unsafe, while the same sideways recovery started earlier is safe. The mission
therefore provides a genuine measured early-intervention opportunity for that
specified recovery. Climbing at dispatch remains safe throughout this range,
so this pilot does **not** establish that early intervention is necessary for
the primary climb recovery. No negative gap or reversed deadline ordering was
observed in this single scenario; no claim is made about other geometries.

No predictive policy, computation readiness, or forecast error was evaluated.
Geometry, maneuver timings and inspection tolerances are provisional assumptions;
the first chosen geometry passed the basic checks without parameter searching.
The body sphere does not establish coverage of real propellers, disturbances and
state-estimation error are absent, and reference timing can interact with physical
tracking transients. Internal maneuver joins are smooth; measured-position
anchoring introduces up to 1.94 cm of reference repositioning at initiation,
without moving the physical state or resetting the PID.

The recommended next experiment is to bracket climb expiration beyond 1 s while
retaining this geometry and the same safety threshold. It was not run in Phase 1.
Implementation, validation details and reproduction commands are in the separate
[technical appendix](TECHNICAL_APPENDIX.md).
