# Supporting tools and preserved evidence

The research scripts live one level above this folder. This folder groups the
checks, reports and stored files that support them.

| Item | Purpose |
|---|---|
| `delay_tolerance.py` | Task 1: unchanged calibration of 15 additional accepted cases and bounded No-Recovery delay sweeps |
| `readiness.py` | Time the existing saved-snapshot forecasts; no mission-run accuracy experiment |
| `diagnostics.py` | Plot saved actual flights; does not run a new experiment |
| `validation.py` | Audit stored calibration and mission results |
| `tests/` | Unit tests, focused physics checks and scientific-equivalence checks |
| `logs/` | Original calibration inputs, snapshots, scientific run records and console logs |
| `results/` | Recorded refactor comparisons and focused validation outputs |
| `reports/` | Phase 1 report, technical appendix and refactor report |
| `layout_reorganization.json` | Old/new paths and SHA256 hashes for all 486 moved archive files |

The logs contain research data, including the five calibrated development cases;
they are not disposable diagnostic output. Their contents and the existing results
were preserved byte for byte. Old absolute paths inside historical metadata are
kept unchanged; use the [research README](../README.md) for current commands.

Run supporting tools from `the repository root` with the existing SCG Python:

```sh
python -m staleguard.geofence_turn.support.readiness --out staleguard/geofence_turn/support/results/readiness_new --repeats 1 --max-trials 12 --max-seconds 120
python -m staleguard.geofence_turn.support.validation
python -m staleguard.geofence_turn.support.diagnostics --logs staleguard/geofence_turn/support/logs/northbound_review
python -m unittest discover -s staleguard/geofence_turn/support/tests -v
```

The plot command writes under the selected `--logs` root; choose a copied dataset
if you want to keep its plot files unchanged. Tests use temporary output folders.
These plotting, readiness and validation checks add no baseline or mission evaluation.

The requested Task 1 delay-tolerance experiment is documented in its
[report](results/delay_tolerance_20/REPORT.md) and
[technical appendix](results/delay_tolerance_20/TECHNICAL_APPENDIX.md).
It reuses the five original cases, adds 15 accepted cases under the unchanged
calibration rule, and records 380 actual flights in one journal. Its reproduction
commands and resource limits are in the appendix. The diagnostic adds no baseline
and does not run a six-policy comparison.

The cleanup passed **38 focused tests**, including three fresh physics flights.
The immediate and delayed control traces match the pre-refactor records exactly;
55 live forecast outcomes and all five saved approval times remain unchanged.
The [test log](results/refactor_equivalence/support_layout_checks/tests.log) and
[dataset audit](results/refactor_equivalence/support_layout_checks/dataset_audit.json)
record these checks. No scenarios were recalibrated and no baseline was added.

Current calibration and benchmark outputs use the compact conventions under
`staleguard/geofence_turn/results/development/` and `staleguard/geofence_turn/results/evaluation/`.
This support folder retains historical artifacts and optional diagnostics.
See the [artifact simplification report](reports/ARTIFACT_SIMPLIFICATION_REPORT.md)
and the [research README](../README.md#workflow-and-artifacts) for the current files.
