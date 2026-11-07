# Figure 3 trajectory validation

Actual SCG/PyBullet flights reproduce the eastbound patrol, the two safety-only braking/hover recoveries, and (when exported) the unmodified No-Recovery turn. Neither recovery executes the northbound turn or completes the checkpoint. No plotting, interpolation, or coordinate smoothing was performed.

Status: **READY**.

## Recorded versus replayed physical outcomes

| Policy | Quantity | Archived | Replay |
|---|---|---:|---:|
| staleguard_early_recovery | Recovery (flight seconds) | 1.9 | 1.9 |
| staleguard_early_recovery | Command application (flight seconds) | None | None |
| staleguard_early_recovery | First violation (flight seconds) | None | None |
| staleguard_early_recovery | Minimum body clearance (m) | 0.3659709348814321 | 0.3659709348814321 |
| staleguard_early_recovery | Completed | False | False |
| staleguard_early_recovery | Safe | True | True |
| staleguard_early_recovery | Prediction computation (ms) | 65.92215085402131 | 65.92215085402131 |
| use_time_recheck | Recovery (flight seconds) | 2.32 | 2.32 |
| use_time_recheck | Command application (flight seconds) | None | None |
| use_time_recheck | First violation (flight seconds) | 2.552 | 2.552 |
| use_time_recheck | Minimum body clearance (m) | 0.03893330070225935 | 0.03893330070225935 |
| use_time_recheck | Completed | False | False |
| use_time_recheck | Safe | False | False |
| use_time_recheck | Prediction computation (ms) | 3.947183024138212 | 3.6437729722820222 |
| no_recovery | Recovery (flight seconds) | None | None |
| no_recovery | Command application (flight seconds) | 2.3 | 2.3 |
| no_recovery | First violation (flight seconds) | 2.511 | 2.511 |
| no_recovery | Minimum body clearance (m) | -0.2720176519091804 | -0.2720176519091804 |
| no_recovery | Completed | False | False |
| no_recovery | Safe | False | False |
| no_recovery | Prediction computation (ms) | None | None |

## Configuration, sources, and checks

- Frozen seed 22, dispatch delay 0.50 s, TURN_LEFT_90 radius 0.40 m, speed 0.80 m/s.
- Controller 50 Hz; unchanged physics/safety monitor 1,000 Hz. The full monitored flight includes pre-approval flight and continues after a violation until the original deadline.
- Every approval snapshot matches exactly: physical state, PID memory, remaining reference, command, configuration, and prior clearance.
- All exported physical-state prefixes through approval match exactly.
- First-violation XYZ is captured from the existing monitor callback, at its actual physics tick; it is not interpolated from the 50 Hz CSV.
- The observation wrapper does not change actions, physics steps, prediction timing measurement, or policy decisions. The original monitor is restored after each flight.
- Memory guards, the simulation lock, CPU-only execution, and single numerical threads remain active.
- Journal byte counts/checksums, calibration checksum, snapshot checksum, configuration, and scientific sources were verified before replay. Source relocations require the existing pinned relocation checks.
- Simulator hash differences were reviewed against Git: new policy branches preserve the common physical execution and Use-Time decision scheduling. The exporter pins the reviewed current and archived versions.

- **staleguard_early_recovery source:** `geofence_turn/results/evaluation/fixed_early_benchmark/` (`run.json`, `executions.jsonl`, `metrics.json`).
- **use_time_recheck source:** `geofence_turn/results/evaluation/baseline_benchmark/` (`run.json`, `executions.jsonl`, `metrics.json`).
- **no_recovery source:** `geofence_turn/results/evaluation/baseline_benchmark/` (`run.json`, `executions.jsonl`, `metrics.json`).
- **staleguard_early_recovery:** 323 physical samples; clearance error 0 m; discrepancies: [].
- **use_time_recheck:** 323 physical samples; clearance error 0 m; discrepancies: [].
- **no_recovery:** 323 physical samples; clearance error 0 m; discrepancies: [].
  Final-state maximum absolute errors: {'final_position': 0.0, 'final_velocity': 0.0}.

## No-Recovery physical trajectory

- Command executes at 2.300 s, XYZ [1.5570917853533062, 0.030667698261513315, 1.1999266054185003] m.
- First violation: {'time_flight_s': 2.511, 'physics_tick': 2511, 'position_m': [1.7248697168249916, 0.034912854201658176, 1.2002533011778331]}. XYZ is the actual center position, sampled at 1,000 Hz.
- Final position: [1.9428407792580713, 3.116831983388693, 1.2007699811142327] m; final velocity: [0.0011188651450374518, 0.7900037414485602, -0.00021033034284234425] m/s.
- Full flight: 323 physical states at 50 Hz, from 0.00 through 6.44 s; no samples after the violation are removed.

| Coordinate | Minimum (m) | Maximum (m) |
|---|---:|---:|
| X | -0.123432488022 | 2.046628554561 |
| Y | 0.030634401783 | 3.116831983389 |
| Z | 1.198512817291 | 1.202331809914 |

At the last 50 Hz sample before the violation (2.500 s), the velocity heading is 4.860052 degrees north of east and the UAV has moved 0.003449316 m north since command application. These are physical-state measurements, not reference-path estimates.

The final velocity heading is 89.918853 degrees north of east. The maximum altitude departure from the frozen nominal height is 0.002507965 m. The CSV retains 197 control samples after the first violation.

The coordinate and velocity changes show a meaningful left-curving segment that starts before the first violation and continues northward afterward. They do not establish an exact 90-degree physical turn or a perfect circular arc. No-Recovery has negative body clearance; its intrusion must be distinguished from Use-Time's positive-clearance breach of the 5 cm requirement.

The No-Recovery extension replayed exactly one physical flight. The previous two CSVs, their policy metadata and prediction timings, and the geometry were preserved. Historical benchmark artifacts were not modified.

## Interpretation and limitations

The two source experiments were separate runs. Their Python patch versions and Linux kernels differ and are retained in figure3_events.json. Use-Time computation is measured again; its duration may differ without changing the first subsequent 50 Hz application tick. EarlyRecovery uses its archived forecast and archived 65.922 ms duration, without recomputing TTL.

Prediction finish and availability times are derived from the simulator timing model, not timestamps of concurrent onboard computation. Prediction expiry is a dispatch-delay threshold, not the physical violation time. Angular rates p/q/r are rad/s; roll/pitch/yaw are radians.

The physical fence and the safe-center boundary are different planes. Use-Time violates the required 5 cm clearance while retaining positive minimum body clearance. No-Recovery, when included, has negative body clearance. The figure must distinguish these outcomes. This comparison illustrates recovery timing; it does not isolate computation latency as the sole cause of the unsafe flight.

## Regenerate

From `examples/staleguard`, in the `scg` environment:

```bash
python figures/figure3_data/reproduce_trajectories.py --overwrite
```

`--overwrite` replaces only the exporter outputs. Omit it for a fresh output, or supply `--out PATH`. The default regenerates all three policies. To add only No-Recovery while preserving the other exports, use `python figures/figure3_data/reproduce_trajectories.py --policies no_recovery` (add `--overwrite` only to replace an existing No-Recovery export). Only selected policies are replayed; no benchmark is launched. Each CSV uses full-precision numeric values and can be loaded directly by PGFPlots with `col sep=comma, x=x, y=y, z=z`.

## Preparation history

This extension used two No-Recovery simulation attempts: the initial replay matched the archive but a policy-selection metadata bug prevented saving its in-memory trace. After the bug was fixed and a focused metadata regression check passed, the user explicitly authorized one replacement No-Recovery replay. The replacement produced the exported data and matched all required archived measurements. Neither previously validated recovery policy was rerun.
