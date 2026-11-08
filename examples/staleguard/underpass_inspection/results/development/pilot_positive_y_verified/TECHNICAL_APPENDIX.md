# Positive-world-Y verification: technical appendix

## Exact implementation changes

| Modified file, relative to underpass_inspection | Change |
|---|---|
| `scenario.py` | Document existing +0.85 m as positive world Y and reject nonpositive `swerve_y`; displacement equation and every default parameter unchanged |
| `simulation.py` | Forward measured world Y/Vy to GUI drawing only; no physical/safety/control change |
| `support/visualization.py` | Remove the floor appearance override; display actual world Y/Vy; use the fresh verified pilot for saved-path overlays |
| `pilot.py` | Add actual +Y smoke checks/measurements, mark `positive_world_y_v1` verification protocol, use a distinct default sweep output |
| `support/test_pilot.py` | Test positive-world-Y displacement/velocity and rejection of a negative displacement |
| `README.md` | Current results, checkerboard floor, direction readout and reproduction instructions |

There was no physical sign reversal: the audit first checked both the generator
and archived actual trajectories. The generator already used
`position_y = +0.85 * smooth_phase`, `velocity_y = +0.85 * phase_derivative`.
It still does. This avoids making an unnecessary change that would instead send
the approved command toward negative world Y. The validation guard changes which
custom parameter values are accepted, not the default physical trajectory.

The pier is unchanged: X=[2.43,2.67], Y=[−0.15,0.15], Z=[0,1.15] m. All deck,
inspection, timing, recovery and body-clearance parameters are unchanged. The
camera remains distance 3.4 m, yaw −35°, pitch −20°, target [2.2,0.25,1.0] m.
Colors, labels, sketch-axis inset and `--overlay-all` remain. The inset's
`Y_display=-Y_world` is not applied to any physical state or reference.

## Executed validation

- Five focused unit tests passed. A separate five-flight smoke run passed all
  **41 checks**, including safe D=0 completion, safe immediate recoveries, unsafe
  nominal continuation, unchanged state/PID at initiation, relative references,
  fixed geometry, full regular 1 kHz observations and actual positive-Y motion.
- A fresh serial sweep executed **154 physical flights**: one nominal and 51
  starts for each of dive, climb and sideways. Initial smoke-gate flights inside
  that matrix were not duplicated. Every flight contains 8,801 actual 1 kHz
  positions and 441 full control observations through the same endpoint.
- Independent vectorized recomputation of all clearances, first violations,
  inspection dwell and outcomes passed. Grid/key completeness and current source
  provenance also passed. See `validation.json`.
- All 154 fresh physical arrays, full-state arrays and reference arrays match
  the original arrays exactly. Safety, completion, dwell completion time, first
  violation, per-obstacle minima and physical hashes match exactly as well. See
  `direction_audit.json`. This agreement was checked after rerunning, not assumed.
- Original pilot files were compared with pre-edit checksums and preserved.
  No original report, result, source hash or screenshot was rewritten. New data
  live in `results/development/pilot_positive_y_verified/`; this directory name
  denotes a fresh verification, not a new physical scenario.
- The requested five GUI replays use dive D=0, dive D=0.4, climb start=0.5,
  sideways start=0.5, and live D=0 dive with all recorded paths overlaid. Their
  actual trajectories/references/outcomes match the fresh headless data exactly.
  Stock floor visual/texture/material data are captured before and after scene
  styling and required to remain identical. See `gui_validation.json`.

The stock checkerboard is now visible. No call removes its texture, changes its
color/opacity or changes its geometry. Screenshots are actual PyBullet client
window captures, including an immediate-dive frame at flight 4.00 s where the
readout shows world Y=+0.252 m and Vy=+1.447 m/s. No trajectory was manufactured,
smoothed, or redrawn outside the simulator.

## Artifacts and measured direction

The four primary files are `summary.json`, `flights.jsonl`, `per_delay.csv` and
`trajectories.npz`. The summary stores the complete scenario, protocol, sources,
smoke checks and measured grid profiles. The archive retains actual physical
positions/times at 1 kHz, actual 12-state observations at 50 Hz and references.

`direction_audit.json` records before/after equivalence, original-file preservation
and actual world-Y measurements. Physical timestamps are flight-start-relative;
subtract approval time 0.8 s for approval-relative times. With D=0, the prescribed
swerve begins at flight 3.4 s and ends at 4.6 s. Measured Y at those endpoints is
−0.0031718447 m and +1.0411100984 m. Final Y is +0.8478390006 m; peak actual Vy is
+1.8222138818 m/s. Negative Vy during later overshoot correction is retained in
both the data and the live readout, not clipped or reinterpreted as commanded −Y.

The new measured deadline summary is 0.14 s first unsafe dive, 0.94 s last safe
sideways start, 0.96 s first unsafe sideways start, +0.80 s sideways gap, and
climb expiration/exact gap null because every tested climb start is safe. The
full tested safe/unsafe intervals remain in `summary.json`; no interpolation or
monotonicity assumption is used to invent an unmeasured deadline.

## GUI screenshots

- [Positive world Vy during the immediate swerve](gui/dive_positive_vy.png)
- [Immediate dive](gui/dive_0.png)
- [Delayed dive](gui/dive_delayed.png)
- [Climb](gui/climb.png)
- [Sideways recovery](gui/sideways.png)
- [All paths at flight start](gui/overlay_start.png)
- [All paths after the selected flight](gui/overlay.png)

## Commands from examples/staleguard, with scg active

```bash
python underpass_inspection/pilot.py --mode dive --delay 0 --gui --overlay-all --hold 10
```

The overlay uses the newly verified physical flights. Default GUI repeats create
numbered output folders, preserving earlier playbacks. To inspect the other
requested cases, use `--mode dive --delay 0.4`, `--mode climb --start 0.5`, or
`--mode sideways --start 0.5`, with `--gui --hold 10`.

Reproduce the fresh timing sweep in a new output directory:

```bash
python underpass_inspection/pilot.py --sweep --out /tmp/underpass_positive_y_reproduction
```

Recompute validation without flights:

```bash
python -m unittest underpass_inspection.support.test_pilot -v
python underpass_inspection/support/validate_results.py underpass_inspection/results/development/pilot_positive_y_verified
```

No shared model, PID, existing mission, safety predicate, completion criterion,
body radius, clearance threshold, or control/physics frequency was modified.
