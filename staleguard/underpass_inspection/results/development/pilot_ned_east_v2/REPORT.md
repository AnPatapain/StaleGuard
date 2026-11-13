# Underpass Inspection: North-East-Down physical correction

The approved swerve and independent sideways recovery now move East in the
right-handed NED frame: **negative PyBullet world Y**. The aircraft still flies
North toward the same bridge, descends to inspect underneath, then turns East
around the unchanged pier. The immediate dive safely completes inspection with
**0.1407 m** minimum body clearance and finishes at **world Y=−0.8454 m**, or
**NED East=+0.8454 m**. Actual GUI captures and camera projections confirm that
the swerve bends lower-right, matching the sketch. The fresh **154-flight** pilot
measures a first unsafe dive delay of **0.12 s** and last safe sideways start
of **0.92 s**, giving a **+0.80 s** gap. Both thresholds are 20 ms earlier than
before. Climb remains safe through 1 s, leaving its expiration and exact gap
unmeasured. All earlier artifacts are preserved. No geometry, safety margin,
controller or timing parameter was tuned. This single-scenario result establishes
the requested physical/visual direction and a sideways intervention window;
next, bracket climb expiration beyond the present grid before evaluating any
predictive policy.

## Mission, coordinates, and success

The UAV approaches the bridge at cruising altitude. Its dispatch-relative command
dives below the slab, spends at least 0.18 s inside a fixed inspection region,
then swerves around the pier and continues forward. Delayed dispatch shifts that
same maneuver relative to the fixed structures. Climb and sideways recovery
cancel inspection and continue above or beside the deck, respectively.

North is world +X, East is world −Y, and Down is world −Z. This changes the
physical lateral requests while leaving PyBullet's Z-up dynamics intact. The
deck, pier centered at X=2.55 m/Y=0, and inspection region are symmetric about
world Y=0, so no geometry mirroring or repositioning was necessary.

Safe inspection completion requires the dwell and no body-clearance violation
anywhere in the full flight. Every run is monitored at 1,000 Hz from reset to
flight time 8.8 s, or 8 s after approval. The same 0.05 m required clearance and
existing body sphere are used. Structures remain visual-only: clearance violations
are detected numerically, rather than by obstacle contact forces.

## Observed direction and physical feasibility

| Immediate-dive measurement | Before: positive world Y | New NED East version |
|---|---:|---:|
| Final world Y | +0.8478 m | **−0.8454 m** |
| World Vy at flight 4.00 s | +1.4468 m/s | **−1.4317 m/s** |
| NED East at flight 4.00 s | −0.2523 m | **+0.2569 m** |

During the new lateral phase, actual world Y decreases by **1.0457 m**, while
world X increases by 0.7018 m and altitude changes by only 0.0070 m. This is
physical lateral motion, not an extra vertical dip. Its actual projected movement
is rightward by 16.1% of the viewport width and downward by 3.7% of its height.
The unchanged camera shows North upper-right, East lower-right and Down downward.

| Fresh flight | Safe throughout? | Inspection completed? | Minimum body clearance |
|---|---|---|---:|
| Nominal | No; deck front | No | −0.2605 m |
| Immediate dive | Yes | Yes | **0.1407 m** |
| Immediate climb | Yes | No | 0.4388 m |
| Immediate East recovery | Yes | No | 0.3721 m |
| Dive delayed 0.40 s | No; pier | Yes | −0.0915 m |

## Fresh timing sweep

| Approval-relative measurement | New NED version | Previous world +Y version |
|---|---|---|
| First unsafe dive delay, t_cmd | **0.12 s** | 0.14 s |
| Climb expiration | **Not bracketed; safe through 1.00 s** | Not bracketed |
| Last safe sideways start, t_rec_sideways | **0.92 s** | 0.94 s |
| First unsafe sideways start | **0.94 s** | 0.96 s |
| Sideways recovery gap | **+0.80 s** | +0.80 s |
| Exact climb gap | **Undefined** | Undefined |

Dive is safe at tested delays 0–0.10 s and unsafe at 0.12–1.00 s. Sideways recovery
is safe at 0–0.92 s and unsafe at 0.94–1.00 s. Every tested climb start is safe.
No unsafe-to-safe reversal was observed. These are measurements on a 20 ms grid,
not exact continuous-time boundaries. All 51 dives complete raw inspection dwell,
but only **6/51** safely complete the whole flight, versus 7/51 previously.

The changed flights are not exact physical mirrors, despite symmetric obstacle
geometry. We used the new measured outcomes and did not tune away their differences.
The results support early intervention for the specified Eastward recovery, while
dispatch-time climb failure is still absent from the tested range. Prediction
error and readiness have not been evaluated. Provisional maneuver/inspection
settings, ideal state access and one deterministic scenario limit broader claims.
Files, validation evidence, screenshots and commands are in the separate
[technical appendix](TECHNICAL_APPENDIX.md).
