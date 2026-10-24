# Geofence artifact simplification

Default calibration now produces one scenario JSON and the necessary approval snapshots; a baseline comparison produces only `executions.jsonl`, `metrics.json` and `run.json`. The flight remains the same: a delayed UAV command starts its approved turn farther east and then continues north toward the original checkpoint. This task changed how results are saved and summarized, not that motion or any policy decision. All 20 existing development cases were copied into the compact layout with byte-identical snapshots, retaining the original 20/22 acceptance count. Forty focused tests passed. The metrics command processed 80 archived flight records and reproduced their existing outcome and recovery-attempt counts. No scenarios were recalibrated and no new mission flights were run. Optional detailed traces remain available. The next step is to use the compact inputs and outputs for the existing benchmark; the missing baselines and TTL-accuracy experiment remain unfinished.

## What changed

Calibration keeps accepted parameters, configuration, provenance, acceptance statistics and resume state together. Its default output has no candidate journals, per-scenario configuration files or trajectory files. A search interrupted during a scenario retains that scenario's candidate cache; completed searches discard it. Detailed candidates and traces require `--diagnostics`.

The benchmark appends the complete existing flight-result dictionary without renaming or filtering its fields. One run JSON preserves source/configuration checks, the fixed protocol, journal integrity and approval forecasts with their original computation times. Resume skips committed flights and rejects duplicates or incompatible inputs. It can remove an unfinished final write while preserving complete records.

Metrics are rebuilt from the flight journal and fixed protocol, with no simulation. They cover outcomes, recovery success/failure, timing, readiness, per-delay results and paired StaleGuard/Use-Time Recheck benefits and harms. Missing measurements remain null; incomplete pairs are counted and excluded from paired results. StaleGuard timing counts one cached approval forecast per scenario; Use-Time Recheck counts its recorded per-flight checks.

## Focused validation

| Check | Observed result |
|---|---|
| Existing development inputs | All **20** load in both historical and compact layouts; snapshot bytes unchanged |
| Acceptance statistics | **20 accepted / 22 attempted**, including two rejections, preserved |
| Saved-result compatibility | **80 legacy records** reproduce all previous outcome/fallback counts; **16 northbound records** also match their recorded counts |
| Calibration fixture | Same five approvals from 33 recorded candidate outcomes; no new calibration flights |
| Resume fixtures | No duplicate flights; cached prediction timing preserved; altered protocol/source/committed data rejected |
| Metric fixtures | Benefit/harm, recovery failure, missing timing, partial pairs and rates checked |
| Regression tests | **40 passed**, plus a final repeat of the six artifact-specific tests |
| Historical preservation | All **634** pre-existing archive files checked byte-for-byte |
| New mission flights / recalibrations | **0 / 0** |

The simulation, baseline implementations and scientific scenario functions are unchanged. Resource guards, prediction timers and action-step scheduling remain active. No new mission-run timing claim is made. Historical connector records were used only to validate file-schema compatibility, not as current mission-performance evidence.

The [README](../../README.md#workflow-and-artifacts) documents the exact files and commands. The [validation log](../results/artifact_simplification/validation.log) and [derived historical metrics](../results/artifact_simplification/metrics.json) contain the supporting checks. Existing scientific data and historical reports remain in their archive locations.
