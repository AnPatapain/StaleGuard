# Task 1 technical appendix

This appendix records the experiment definitions, individual results and reproduction commands. The [main report](REPORT.md) explains the mission and research decision.

## Protocol and scope

- Mission schema: `northbound_checkpoint_v2`; original `Config()` unchanged.
- Reused cases: seeds 0–4, read from the original frozen development dataset; snapshots were not rewritten.
- New generation order: seeds 5–21 inclusive, stopping after 15 additional accepted cases. Both rejections (5 and 13) are retained. All generated new seeds were attempted; there is no unused generated suffix in the denominator.
- Calibration uses the original `calibration.py` and `calibrate()` selection function. Batch 1 attempted 15 seeds and accepted 13; batch 2 attempted two and accepted two. Neither delayed flight results nor policy predictions entered calibration.
- The approval search upper bound remains the actual nominal fence-violation probe. Its existing override of the legacy default upper bound permits approvals at 3.4 and 3.6 s; no search parameter was changed.
- Delay steps at 50 Hz: 0–10 inclusive, then 15, 20, 25, 30, 35, 40, 45, 50. These are exactly the requested 19 delays; there is no dispatch rounding error.
- Policy: No Recovery only. Full initial-state replay, no snapshot teleport, no recovery or replanning, and monitoring through approval + T even after a violation or completion.
- Safety and checkpoint conditions use the original 1000 Hz physics monitor. The diagnostic additionally reads actual position/velocity at each observed tick to distinguish box misses from insufficient dwell. These reads do not alter actions or state.
- One CPU simulation worker and one numerical thread. Original 2048 MiB RSS and minimum-available-memory guards remain active. Calibration subprocesses retain ≤20 generated seeds and ≤200 trials per invocation; evaluation uses 133, 133 and 114 flights, each below its 160-trial/900-second cap. No guard stopped this run.

## Metric definitions

| Metric | Calculation |
|---|---|
| Last safe-completable delay | Maximum tested D with `safe` and `completed` both true |
| First unsafe delay | Minimum tested D with body clearance below 0.05 m at any monitored time |
| First completion loss | Minimum tested D with the checkpoint position, velocity and continuous 0.20 s dwell objective not completed before T, independently of safety |
| First safe-completion loss | Minimum tested D where safety and completion do not both hold; used to classify the first limiting factor |
| Predispatch violation | A violation at or before dispatch under the existing monitor's inclusive tick convention; table gives the first such D |
| Non-monotonic outcome | A Boolean outcome is false at one tested D and true again at a larger tested D; calculated separately for safety, completion and their conjunction |
| Positive-delay completion rate | Scenarios with any safe completion at D > 0 divided by 20 |
| Completion rate at D ≥ 0.1 s | Scenarios with any safe completion at a tested D ≥ 0.1 divided by 20 |
| Acceptance | 20 accepted / 22 generated and attempted overall; 15/17 for new seeds, including geometry rejection |

A blank first-unsafe result means no violation through the largest tested delay (1.00 s), not an unbounded safety guarantee. Reported thresholds are discrete observations; no interpolation or monotonicity assumption is used. The two rates count scenarios once even if several delays succeed. No baseline decision or scientific threshold was changed to smooth a boundary.

## Individual scenario results

Seeds 0–4 are reused; all others are newly calibrated. Times are seconds. “—” means not observed through D=1.00 s. In the final column, C means checkpoint completion was lost and regained; no scenario regained safety or safe completion after loss.

| Seed | Approval | Last safe completion D | First unsafe D | First checkpoint loss D | First predispatch violation D | Non-monotonic |
|---:|---:|---:|---:|---:|---:|---|
| 0 | 2.00 | 0.12 | 0.14 | 0.14 | 1.00 | — |
| 1 | 1.80 | 0.10 | 0.50 | 0.12 | — | — |
| 2 | 1.80 | 0.14 | 0.30 | 0.16 | 0.90 | — |
| 3 | 1.40 | 0.02 | 0.04 | 0.06 | 0.70 | — |
| 4 | 1.40 | 0.16 | 0.50 | 0.18 | — | — |
| 6 | 2.80 | 0.00 | 0.70 | 0.02 | — | — |
| 7 | 1.40 | 0.08 | 0.10 | 0.14 | 1.00 | — |
| 8 | 1.80 | 0.00 | 0.70 | 0.02 | — | — |
| 9 | 2.60 | 0.02 | 0.06 | 0.04 | — | — |
| 10 | 1.20 | 0.18 | 0.60 | 0.20 | — | — |
| 11 | 3.40 | 0.00 | 0.04 | 0.02 | 0.70 | C |
| 12 | 1.80 | 0.08 | 0.90 | 0.10 | — | — |
| 14 | 2.80 | 0.18 | 1.00 | 0.20 | — | — |
| 15 | 1.40 | 0.02 | 0.50 | 0.04 | — | — |
| 16 | 1.60 | 0.04 | — | 0.06 | — | — |
| 17 | 1.20 | 0.16 | 0.18 | 0.30 | — | — |
| 18 | 2.00 | 0.06 | 0.60 | 0.08 | — | — |
| 19 | 2.00 | 0.14 | 0.30 | 0.16 | — | — |
| 20 | 1.40 | 0.00 | 0.50 | 0.02 | — | — |
| 21 | 3.60 | 0.12 | 0.14 | 0.14 | — | — |

## Results at every tested delay

The first three outcome columns partition the 20 cases. “Checkpoint complete” also includes unsafe flights that completed the objective. Predispatch violations are a subset of unsafe flights.

| D (s) | Safe complete | Safe incomplete | Unsafe | Checkpoint complete | Predispatch violation |
|---:|---:|---:|---:|---:|---:|
| 0.00 | 20 | 0 | 0 | 20 | 0 |
| 0.02 | 16 | 4 | 0 | 16 | 0 |
| 0.04 | 13 | 5 | 2 | 15 | 0 |
| 0.06 | 12 | 5 | 3 | 12 | 0 |
| 0.08 | 11 | 6 | 3 | 11 | 0 |
| 0.10 | 9 | 7 | 4 | 10 | 0 |
| 0.12 | 8 | 8 | 4 | 9 | 0 |
| 0.14 | 6 | 8 | 6 | 6 | 0 |
| 0.16 | 4 | 10 | 6 | 4 | 0 |
| 0.18 | 2 | 11 | 7 | 3 | 0 |
| 0.20 | 0 | 13 | 7 | 1 | 0 |
| 0.30 | 0 | 11 | 9 | 0 | 0 |
| 0.40 | 0 | 11 | 9 | 0 | 0 |
| 0.50 | 0 | 7 | 13 | 0 | 0 |
| 0.60 | 0 | 5 | 15 | 0 | 0 |
| 0.70 | 0 | 3 | 17 | 0 | 2 |
| 0.80 | 0 | 3 | 17 | 0 | 2 |
| 0.90 | 0 | 2 | 18 | 0 | 3 |
| 1.00 | 0 | 1 | 19 | 0 | 5 |

## Non-monotonic completion and boundary sensitivity

Seed 11 completes at D=0. At D=0.02 s its longest qualifying interval inside the box is 0.144 s, so it fails the 0.20 s dwell while remaining safe (minimum clearance 0.0542622044 m). At D=0.04 s it qualifies for 0.509 s and completes, but minimum clearance falls to 0.0499860460 m. It then loses completion again at D=0.06 s. This is a completion reversal, not a safe-completion reversal. The 0.04 s safety result is only 0.0000139540 m below the predefined threshold and should be treated as a sensitive simulation boundary.

Unmodified `simulation.execute()` was rerun at D=0.02 and 0.04 without the extra observations. Every saved scientific result field, excluding wall-clock runtime, matched the diagnostic journal exactly. Both control traces are retained together in `validation_traces.npz`. The 34 focused tests pass; preservation tests rechecked all 326 original log/data files, 486 moved archive files and the shared model files. Every sweep flight also reproduces its saved approval state, PID state, command and remaining reference exactly. The five reused D=0 flights match their original complete result records except runtime.

## Work performed

| Work | New actual mission flights |
|---|---:|
| Additional calibration: 121 candidate trials + 15 nominal probes + 15 D=0 replays | 151 |
| Requested 20 × 19 No-Recovery sweep | 380 |
| Diagnostic-observer preflight | 1 |
| Uninstrumented seed-11 confirmations | 2 |
| Total | **534** |

The original five calibration runs were reused, not rerun. Focused unit tests include model predictions and recorded-fixture checks; these are not additional actual mission-flight outcomes. No 300-scenario calibration, six-policy benchmark, StaleGuard timing experiment or new baseline implementation was performed.

## Files and provenance

All new outputs are in this single task directory. There are no 380 individual trajectory files.

| Item | Contents |
|---|---|
| `REPORT.md`, `TECHNICAL_APPENDIX.md` | Main findings and this technical record |
| `scenario_metrics.csv`, `per_delay.csv`, `summary.json` | Per-case, per-delay and aggregate measurements |
| `flights.jsonl` | All 380 actual sweep results, including additional checkpoint observations |
| `calibrated_scenarios.json` | Combined 20-case index, snapshot checksums and original/new calibration source hashes |
| `calibration_01/`, `calibration_02/` | Original calibration CLI outputs for the 17 new attempted seeds |
| `metadata.json`, `protocol.json`, `resume_sessions.jsonl` | Source/environment, fixed protocol and invocation records |
| `validation.json`, `validation.log`, `validation_traces.npz` | Checks, focused test log and two confirmation traces |
| `analyze.py` | Recompute tables from the journal without simulation; its hash is in `summary.json` |
| `finished.json` | Final task-completion record |

The only new executable research-support driver is [support/delay_tolerance.py](../../delay_tolerance.py). Scientific source functions were not edited. Runtime provenance is in `metadata.json`; the unchanged vehicle-model hashes and original snapshot hashes are in `protocol.json`. The exact flight-journal and analysis-source hashes are in `summary.json`. Historical data is loaded with the existing schema, configuration, snapshot-checksum, command and reference checks; no compatibility check was bypassed.

## Reproduction

Run from `examples/staleguard` in the SCG environment, with a fresh output directory. The first command redoes only the 15 additional accepted calibrations; the five original cases are loaded from the frozen development dataset. Each evaluation command is separately bounded and skips already journaled flights after checking source provenance. Keep source files unchanged when resuming.

```sh
SCG_PYTHON=/home/kean/miniconda3/envs/scg/bin/python
export PYTHONDONTWRITEBYTECODE=1
export NUMBA_CACHE_DIR=/tmp/staleguard-refactor-numba
OUT=geofence_turn/support/results/delay_tolerance_reproduction_new
$SCG_PYTHON geofence_turn/support/delay_tolerance.py calibrate --out "$OUT"
$SCG_PYTHON geofence_turn/support/delay_tolerance.py evaluate --out "$OUT" --start-index 0 --count 7
$SCG_PYTHON geofence_turn/support/delay_tolerance.py evaluate --out "$OUT" --start-index 7 --count 7
$SCG_PYTHON geofence_turn/support/delay_tolerance.py evaluate --out "$OUT" --start-index 14 --count 6
```

To regenerate the current result tables without flying anything:

```sh
$SCG_PYTHON geofence_turn/support/results/delay_tolerance_20/analyze.py
```

The latter reads the 380-flight journal and includes validation-flight counts if a validation record is present. Its directory is resolved relative to its own file. Copy it into another complete run directory to analyze that run; confirmation flights are not assumed when no validation record exists.
