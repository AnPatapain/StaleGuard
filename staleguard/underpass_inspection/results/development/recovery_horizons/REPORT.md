# Underpass recovery-aware development validation

Two new policies let the UAV abandon a pending inspection before its predicted sideways escape becomes unsafe. RecoveryAware considers command safety and sideways recovery; DualHorizon also considers safe inspection completion. Both use the same unchanged 19-delay prediction grid and independently timed forecasts. In development seed 1, a command queued until 1 s instead triggers an actual Eastward recovery at 0.60 s, safely abandoning inspection. Across 12 development scenarios, optimization roughly halved three-profile computation to **142 ms median / 170 ms p95**. All 18 main smoke flights were safe; 14 completed inspection. Prediction errors remain: one false-safe command prediction and one false-safe recovery prediction in 228 comparisons each. Completion and safety profiles coincide in these development cases, so the policies naturally made identical smoke decisions. The benchmark is operationally prepared, but prediction does not guarantee recovery safety. Next, run only the 2,600 new-policy flights and merge them with the unchanged seven-policy results; label that comparison exploratory.

The UAV continues North while its approved command is queued. The predictor follows that nominal flight to each candidate start, then independently simulates the original dive-and-swerve and the Eastward sideways escape. Physical success means inspection dwell before the fixed deadline with no clearance violation anywhere in the flight. Recovery abandons inspection. The bridge, pier, PID, 50 Hz control and 1,000 Hz physical safety monitoring are unchanged.

| Development evidence | Observed result |
|---|---:|
| Frozen scenarios used for prediction validation | 12 / 12 |
| Candidate delays per profile | 19, unchanged, through 1 s |
| Optimized versus original command-profile disagreements | 0 / 228 |
| Compiled versus reference terminal state / PID maximum difference | 0 / 0 |
| Exact versus broad-phase clearance diagnostic disagreements | 0 / 10,000 |
| Physical command predictions: false-safe / false-reject | 1 / 1, out of 228 |
| Physical safe-completion predictions: false-safe / false-reject | 1 / 1, out of 228 |
| Physical recovery predictions: false-safe / false-reject | 1 / 0, out of 228 |
| Main physical smoke: 3 scenarios × 3 delays × 2 policies | 18 flights |
| Main smoke: safe / safe inspection / recovery | 18 / 14 / 4 |
| Main smoke matching independent physical trajectory hashes and outcomes | 18 / 18 |
| RecoveryAware versus DualHorizon smoke action differences | 0 / 9 paired cases |
| Focused tests | 58 passed |

The physical comparison labels above come from archived independent development simulations, not from the online predictor. Both policies recovered at 0.60 s in seed 1 and 0.40 s in seed 6 for dispatch at 1 s. All sampled predicted safety and safe-completion labels coincide across these 12 scenarios. Synthetic tests establish that DualHorizon can differ when completion becomes infeasible earlier; no difference was manufactured in the physical mission.

| Warmed computation diagnostic | Median ms | p95 ms |
|---|---:|---:|
| Original command-only StaleGuard | 142.4 | 151.6 |
| Three-profile reference, full continuations | 285.9 | 293.6 |
| Compiled full continuations | 179.6 | 185.8 |
| Compiled, stop after safety failure | 158.7 | 178.6 |
| Chosen: compiled, early stop, exact clearance broad phase | **142.0** | **169.8** |

Each row uses 36 timings (three repeats on every development scenario). The chosen method is **2.01× faster in median** than the equivalent unoptimized three-profile reference. It is roughly the original command-only median cost, with a higher p95; this is not a claim of twice the speed of original StaleGuard. All waiting, both continuations, completion checks, packing, extraction and cleanup count toward atomic readiness. Recovery is computed first, but partial results never advance a policy's availability.

Four additional development flights checked dispatch at 2 s. Both policies recovered safely at 0.60 s in seed 1. In seed 3, all sampled profiles through 1 s had no predicted expiration, so both admitted the command at 2 s under the unchanged gate. Both then violated deck-front clearance at approval +2.11 s. This unfavorable outcome demonstrates the finite prediction horizon; no new rejection rule was added. The full eight-second observation horizon still applies.

Interruption/resumption repaired an intentionally uncommitted tail and completed all 18 unique main-smoke records. A metrics-only merge combined those with 63 archived records into an 81-record nine-policy smoke comparison. Validation confirmed identical metrics regeneration and rejected duplicates, incompatible physical protocols and changed source provenance. All **1,058 pre-existing result artifacts** and the five original baseline/predictor source modules remain byte-for-byte unchanged. No existing policy was rerun; no new held-out policy flight was executed.

Remaining limits matter: sampled boundaries are not exact deadlines or guarantees between samples; the model still differs from PyBullet; readiness can miss short recovery windows; and warmed offline computation replay is not a live asynchronous deployment measurement. Inspection residence is evaluated on the model's unchanged 100 Hz integration segments, while physical truth is monitored at 1,000 Hz. The old 100 scenarios have already been examined, so the eventual nine-policy comparison is exploratory. Operational preparation is complete; superiority and real-world safety are unestablished. Reproduction, hashes, source changes and detailed rules are in the [technical appendix](TECHNICAL_APPENDIX.md).
