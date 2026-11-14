# Stage 2 validation report

## Status and Git safety

The minimal package and all six local artifact reconstructions are implemented and validated. External data distribution and interactive GUI verification remain outstanding; neither is represented as completed.

- Stage 1 revision: `5a4d6fb37f610cd45055843685a75dbdb15741df`.
- Stage 2 branch: `refactor/staleguard-reproducibility`.
- No Stage 2 commit, push, merge, reset or history rewrite was performed.
- Working tree: 4,677 tracked deletions and 23 tracked modifications, plus new canonical data, reproduction modules and documentation. Changes are unstaged.
- Original source, results, development files and removed upstream examples/tests were preserved locally in `.stage2_archive/` before removal. This ignored backup is excluded from installation and is not a runtime dependency.
- User `staleguard/tree.txt` remains byte-identical. It was already tracked at Stage 1; it is excluded from wheels and Git export using package configuration and `.gitattributes`.

## Final source structure

```text
.
├── README.md
├── VALIDATION.md
├── LICENSE
├── CITATION.cff
├── pyproject.toml
├── requirements-reproduction.txt
├── .gitignore
├── .gitattributes
├── safe_control_gym/             # Original simulator, controllers and assets
├── staleguard/
│   ├── __init__.py
│   ├── io.py                    # Minimal metadata/I/O and historical pickle loader
│   ├── physics_monitor.py
│   ├── visualize.py
│   ├── model/                   # Six essential predictor/adapter modules
│   ├── inspection_descent/      # Scenario, calibration, simulation, storage, baselines
│   ├── geofence_turn/
│   ├── underpass_inspection/
│   └── reproduce/
│       ├── __init__.py
│       ├── common.py
│       ├── run_experiments.py
│       ├── fig2.py
│       ├── fig3.py
│       ├── table2.py
│       ├── table3.py
│       ├── fig4.py
│       └── fig5.py
├── data/                        # Immutable primary files plus manifest/README
│   ├── manifest.json
│   ├── README.md
│   ├── inspection_descent/
│   ├── geofence_turn/
│   └── underpass_inspection/
└── outputs/                     # Ignored regenerated artifacts and validation evidence
```

There are 49 retained StaleGuard Python files: 38 retained originals and 11 new modules. There were 94 original files; 56 were removed. No dedicated tests or support directories remain. The vendored simulator source/assets have no Git diff. Its unrelated controller implementations remain untouched rather than risking scientific dependency changes.

## Cleanup and provenance

Removed active Always Recover, Fixed TTL and PPR policies; unused alternative predictor `model/pybullet.py`; old benchmark/table coordinators; pilots; diagnostic/support scripts; source archives; source-relocation adapters; development documentation, tests and unrelated upstream examples. Old policy rows remain intact in immutable journals. Exactly eight published policies are available through the new runner.

Essential physical/profile calculations and the pending-command clock are retained in the mission modules and `model/mission_evaluation.py`. Essential GUI code was extracted from Underpass support into `visualize.py`. Shared artifact loading/calculation is in `reproduce/common.py`. There are no remaining support-directory dependencies.

Historical source-hash enforcement, relocation translation tables and legacy resumption acceptance rules were removed. Original hashes, revisions, configurations, timings and metadata remain byte-identical in primary data. Historical relocation mappings are preserved as manifest metadata, not executable compatibility logic.

`io.SnapshotReader` is the only historical deserialization adapter: it maps old `experiment`, `geofence_turn.mission`, mission and model class paths to retained package modules. It does not rewrite pickle bytes or recreate obsolete module hierarchies.

Data reconstruction checks existence, byte lengths, SHA-256, original journal metadata, unique keys, complete scenario/delay grids, configuration agreement and prediction/physical joins. Physical reuse checks require matching action/timing and physical outcome fields. Verified reuse references: 17,888 Inspection and 31,969 Geofence records. New output metadata records Git revision, current source fingerprints, inputs, dependencies and configuration, without enforcing obsolete source hashes.

Because Stage 2 is not committed, output metadata records the Stage 1 HEAD plus the actual Stage 2 source fingerprints. Installed non-Git packages record a null Git revision and actual source fingerprints.

## Canonical primary evidence

All 1,489 primary files (534,950,138 bytes) are present locally. Every canonical file was copied without rewriting and SHA-256 checked against its source; the final full sweep passed. No primary dataset is missing locally. Full file-by-file paths, sizes and hashes are in `data/manifest.json`. Dataset totals below exclude the small separately referenced Inspection origin manifest where applicable.

| Original source | Canonical destination | Bytes | Execution rows | Prediction rows | Snapshots |
|---|---|---:|---:|---:|---:|
| `staleguard/inspection_descent/results/evaluation/calibration` | `data/inspection_descent/calibration` | 12,127,122 | — | — | 276 |
| `staleguard/inspection_descent/results/evaluation/baseline_benchmark` | `data/inspection_descent/benchmark` | 19,496,410 | 18216 | — | 0 |
| `staleguard/inspection_descent/results/evaluation/supplementary_feasibility_v1` | `data/inspection_descent/feasibility` | 7,768,827 | 6072 | 552 | 0 |
| `staleguard/inspection_descent/results/evaluation/three_opportunity_benchmark_v1` | `data/inspection_descent/additional_policies/three_opportunity` | 37,552,600 | 12144 | 0 | 0 |
| `staleguard/inspection_descent/results/evaluation/always_recover_corrected_v1` | `data/inspection_descent/additional_policies/recovery_sources` | 6,254,314 | 3036 | 0 | 0 |
| `staleguard/geofence_turn/results/evaluation/calibration` | `data/geofence_turn/calibration` | 7,963,946 | — | — | 300 |
| `staleguard/geofence_turn/results/evaluation/baseline_benchmark` | `data/geofence_turn/benchmark` | 40,704,182 | 41400 | — | 0 |
| `staleguard/geofence_turn/results/evaluation/supplementary_feasibility_v1` | `data/geofence_turn/feasibility` | 15,501,175 | 11400 | 600 | 0 |
| `staleguard/geofence_turn/results/evaluation/three_opportunity_benchmark_v1` | `data/geofence_turn/additional_policies/three_opportunity` | 45,840,045 | 13800 | 0 | 0 |
| `staleguard/geofence_turn/results/evaluation/always_recover_corrected_v1` | `data/geofence_turn/additional_policies/recovery_sources` | 14,785,219 | 6900 | 0 | 0 |
| `staleguard/geofence_turn/results/evaluation/fixed_early_benchmark` | `data/geofence_turn/additional_policies/fixed_early` | 13,341,865 | 13800 | — | 0 |
| `staleguard/underpass_inspection/results/evaluation/calibration` | `data/underpass_inspection/calibration` | 52,404,477 | — | — | 865 |
| `staleguard/underpass_inspection/results/evaluation/baseline_benchmark_v2` | `data/underpass_inspection/benchmark` | 185,620,527 | 46800 | 1200 | 0 |
| `staleguard/underpass_inspection/results/evaluation/supplementary_feasibility_v1` | `data/underpass_inspection/feasibility` | 34,038,207 | 15200 | 0 | 0 |
| `staleguard/underpass_inspection/results/development/pilot_ned_east_v2` | `data/underpass_inspection/representative_flights` | 41,307,101 | — | — | 0 |

Retained evidence includes execution/prediction journals, original run/progress manifests, calibrated scenarios and pickle snapshots, approval arrays, measured trajectories and indispensable calibration envelopes/origin metadata. Recorded computation times are retained, not remeasured for the paper. Canonical data contain no regenerable `metrics.json`, tables, plotting CSVs, reports or images.

Underpass calibration retains its original 865 accepted candidates; the benchmark selects exactly the original 400 evaluation scenarios. The representative Figure 2 flights are the recorded seed-0 pilot flights and are explicitly separate from the evaluation cohort.

**Distribution limitation:** `data/underpass_inspection/benchmark/executions.jsonl` is 176,132,260 bytes and is gitignored because it exceeds GitHub's ordinary per-file limit. SHA-256: `8692aa2d9b1bb7cfb8b111743f92aff0c329862292862ee7e84639e5bac8f2e6`. It is preserved locally, but a public external copy must be distributed with this manifest. No download location or DOI has been invented. A checkout without this journal cannot reconstruct the affected artifacts.

## Artifact reconstruction

All six commands were executed successfully from archived primary records. None reads old derived metrics/tables/plotting CSVs as scientific input. Old derived files were used only as comparison references before removal from the released source.

| Artifact | Primary data complete locally? | Regenerated? | Matches final paper? |
|---|---|---|---|
| Figure 2 — trajectories | Yes | PDF/PNG, arrays, CSV, scenario/events | Actual four trajectory hashes and original events match; presentation rebuilt |
| Figure 3 — physical feasibility | Yes | Both panels and per-scenario data | All 400 profiles and six ordering categories verified |
| Table II — prediction/readiness | Yes | CSV/Markdown and label/boundary/runtime exports | Nine property rows and three readiness rows match |
| Table III — eight policies | Yes | CSV/Markdown and paired comparisons | All 24 mission-policy rows match |
| Figure 4 — Underpass delay curves | Yes | PDF/PNG and per-delay CSVs | All eight policies agree with reconstructed Table III |
| Figure 5 — Underpass trade-off | Yes | PDF/PNG and policy CSV | All eight policies agree; overlapping values are not offset |

Reference comparisons passed: 99 prediction/readiness fields, 168 policy count fields, 144 paired scalar fields and 24 bootstrap confidence intervals. Confidence intervals use the original 2,000 whole-scenario bootstrap resamples with seed 2027. Source fingerprints in all six output manifests match the final 49 source files.

Full StaleGuard counts:

| Mission | Unsafe | Safe completions |
|---|---:|---:|
| Inspection | 8 / 3,036 | 913 / 3,036 |
| Geofence | 44 / 6,900 | 1,656 / 6,900 |
| Underpass | 182 / 5,200 | 2,900 / 5,200 |

Original recorded runtime/readiness:

| Mission | Median / p95 (ms, rounded) | Ready by dispatch |
|---|---:|---:|
| Inspection | 41.24 / 55.28 | 2,760 / 3,036 |
| Geofence | 96.38 / 141.10 | 5,273 / 6,900 |
| Underpass | 149.00 / 167.44 | 3,547 / 5,200 |

Inspection recovery has 276 censored boundaries and no observed recovery loss. Its undefined boundary errors and conditional false-feasible rates remain blank/N/M, not zero. Geofence has 124 observed physical recovery losses, Underpass 243; no recorded forecast finishes at/after those sampled losses. This does not establish continuous-time recovery safety.

Underpass physical ordering: command first 148, recovery first 40, same sampled loss 22, command loss only 108, recovery loss only 33, neither observed 49.

Figure 2 verifies recorded nominal, safe immediate inspection, late 0.4 s inspection, and safe 0.5 s recovery arrays against their original flight hashes. No flight was newly replayed for that figure; original geometry and event times were retained.

## Commands executed and reproducibility

Validated environment: Python 3.10.21, Linux, original vendored PyBullet simulator. Exact installed dependency versions are in `requirements-reproduction.txt`. The configured project supports Python >=3.10; other Python/dependency combinations were not validated.

Installation was tested without network/dependency replacement using:

```bash
python -m pip install --no-deps --no-build-isolation --target /tmp/staleguard-final-verified-install .
```

The resulting wheel imported outside the source tree, included required simulator assets, and regenerated Table II from `/tmp`; both result CSVs exactly match the source-tree reconstruction. This verifies a real installed package, not only imports from the checkout. A clean dependency installation from the network was not attempted in the restricted environment.

Run these independent reconstruction commands from the repository root using the installed scientific environment:

```bash
python -m staleguard.reproduce.fig2   --data-root data --out outputs/fig2
python -m staleguard.reproduce.fig3   --data-root data --out outputs/fig3
python -m staleguard.reproduce.table2 --data-root data --out outputs/table2
python -m staleguard.reproduce.table3 --data-root data --out outputs/table3
python -m staleguard.reproduce.fig4   --data-root data --out outputs/fig4
python -m staleguard.reproduce.fig5   --data-root data --out outputs/fig5
```

Each output directory includes `metadata.json`; figures include PDF/PNG and supporting data; tables include CSV/Markdown. All new outputs are outside read-only canonical data. `data/README.md` provides the full-file integrity command.

Bounded fresh runs executed (each used a previously empty directory):

```bash
python -m staleguard.reproduce.run_experiments --data-root data --mission inspection_descent --experiment benchmark --seeds 20 --policies no_recovery staleguard_dual_horizon --delays 0 --max-records 2 --workers 1 --out outputs/runs/inspection-smoke
python -m staleguard.reproduce.run_experiments --data-root data --mission geofence_turn --experiment benchmark --seeds 22 --policies no_recovery staleguard_dual_horizon --delays 0 --max-records 2 --workers 1 --out outputs/runs/geofence-smoke
python -m staleguard.reproduce.run_experiments --data-root data --mission underpass_inspection --experiment feasibility --seeds 1002 --delays 0 --max-records 2 --workers 2 --out outputs/runs/underpass-smoke
python -m staleguard.reproduce.run_experiments --data-root data --mission geofence_turn --experiment prediction --seeds 22 --max-records 1 --workers 1 --out outputs/runs/final-prediction
python -m staleguard.reproduce.run_experiments --data-root data --mission inspection_descent --experiment calibration --seeds 20 --max-records 1 --out outputs/runs/inspection-calibration
python -m staleguard.reproduce.run_experiments --data-root data --mission geofence_turn --experiment calibration --seeds 22 --max-records 1 --out outputs/runs/geofence-calibration
python -m staleguard.reproduce.run_experiments --data-root data --mission underpass_inspection --experiment calibration --seeds 1002 --max-records 1 --out outputs/runs/underpass-calibration
```

These directories now exist: the fresh runner intentionally rejects a nonempty output directory. Use a new `--out` for another physical run. The artifact reconstruction commands can regenerate their derived outputs. No full evaluation was rerun.

## Scientific equivalence and limitations

Before removal, 60 focused existing tests passed. Temporary verification drivers outside the released package compared the pre-cleanup source with the cleaned implementation using one frozen case per mission and all eight policies at the same selected delay: 24 physical executions. Three-profile predictions, terminal prediction/controller states and available physical trajectory hashes/final states matched exactly. Calibration selection for seeds 20/22/1002 matched the original frozen scenario definitions.

Use-Time Recheck includes measured wall-clock computation; independent live runs can finish at different control steps. Its native pre/post implementations were additionally compared with identical deterministic timer increments for all three missions; decisions and physical outputs matched. Geofence native Fixed EarlyRecovery was likewise checked against its original implementation. These timer controls were limited to temporary validation and did not alter production code or archived timings. A coordinator/native cancellation metadata difference was not misreported as a physical difference.

Bounded fresh simulations verified approval capture, snapshot loading, dispatch, recovery, full prediction, calibration and two-process execution. This is bounded equivalence evidence, not exhaustive physical replay of every frozen scenario. Prediction equations, PID/dynamics, mission geometry, safety/completion predicates, recovery maneuvers, grids and the final Dual-horizon rule were not intentionally changed. No parameter tuning or historical evidence repair was performed.

GUI `--help`, all mission entry-point construction and headless Underpass scene creation passed. The ground's physical collision geometry remained unchanged; extra presentation geometry has no collision shapes. Interactive PyBullet capture was attempted for 30 seconds but the sandbox cannot access X11 `:0` (`xwininfo` also failed). No interactive capture was produced; interactive rendering remains unverified on a usable desktop display. Run `python -m staleguard.visualize --paper-figure --overlay-all` there.

The source retains original numeric behavior within the checked cases. It does not claim onboard validation, formal safety guarantees, exact wall-clock reproducibility on other hardware, or exhaustive cross-environment determinism.

## Validation evidence

Generated local evidence (ignored with `outputs/`):

- `outputs/data_inventory.csv`: dataset mapping, sizes and record counts.
- `outputs/data_integrity.json`: final source/canonical hash sweep.
- `outputs/artifact_validation.json`: reference comparison totals.
- `outputs/scientific_equivalence.json`: bounded numerical/physical comparisons.
- `outputs/gui-check/status.json` and `gui.log`: attempted interactive GUI verification.
- `outputs/installed-table2/`: reconstruction using the installed wheel outside the source tree.
- `outputs/runs/`: fresh bounded simulation/calibration observations, distinctly separate from archived evidence.

## Retained Python source inventory

- `staleguard/__init__.py`
- `staleguard/geofence_turn/__init__.py`
- `staleguard/geofence_turn/baselines/fbs.py`
- `staleguard/geofence_turn/baselines/fixed_early_recovery.py`
- `staleguard/geofence_turn/baselines/recovery_horizons.py`
- `staleguard/geofence_turn/baselines/staleguard.py`
- `staleguard/geofence_turn/baselines/use_time_recheck.py`
- `staleguard/geofence_turn/calibration.py`
- `staleguard/geofence_turn/scenario.py`
- `staleguard/geofence_turn/simulation.py`
- `staleguard/geofence_turn/storage.py`
- `staleguard/inspection_descent/__init__.py`
- `staleguard/inspection_descent/baselines/fbs.py`
- `staleguard/inspection_descent/baselines/recovery_horizons.py`
- `staleguard/inspection_descent/baselines/staleguard.py`
- `staleguard/inspection_descent/baselines/use_time_recheck.py`
- `staleguard/inspection_descent/calibration.py`
- `staleguard/inspection_descent/scenario.py`
- `staleguard/inspection_descent/simulation.py`
- `staleguard/inspection_descent/storage.py`
- `staleguard/io.py`
- `staleguard/model/__init__.py`
- `staleguard/model/common.py`
- `staleguard/model/feasibility_profiles.py`
- `staleguard/model/lightweight.py`
- `staleguard/model/mission_evaluation.py`
- `staleguard/model/scg.py`
- `staleguard/physics_monitor.py`
- `staleguard/reproduce/__init__.py`
- `staleguard/reproduce/common.py`
- `staleguard/reproduce/fig2.py`
- `staleguard/reproduce/fig3.py`
- `staleguard/reproduce/fig4.py`
- `staleguard/reproduce/fig5.py`
- `staleguard/reproduce/run_experiments.py`
- `staleguard/reproduce/table2.py`
- `staleguard/reproduce/table3.py`
- `staleguard/underpass_inspection/__init__.py`
- `staleguard/underpass_inspection/baselines/fbs.py`
- `staleguard/underpass_inspection/baselines/fixed_early_recovery.py`
- `staleguard/underpass_inspection/baselines/policies.py`
- `staleguard/underpass_inspection/baselines/recovery_horizons.py`
- `staleguard/underpass_inspection/baselines/staleguard.py`
- `staleguard/underpass_inspection/baselines/use_time_recheck.py`
- `staleguard/underpass_inspection/calibration.py`
- `staleguard/underpass_inspection/scenario.py`
- `staleguard/underpass_inspection/simulation.py`
- `staleguard/underpass_inspection/storage.py`
- `staleguard/visualize.py`

## Removed original StaleGuard Python files

- `staleguard/_relocation.py`
- `staleguard/figures/figure3_data/reproduce_trajectories.py`
- `staleguard/geofence_turn/baseline_benchmark/metrics.py`
- `staleguard/geofence_turn/baseline_benchmark/run.py`
- `staleguard/geofence_turn/baselines/always_recover.py`
- `staleguard/geofence_turn/baselines/fixed_ttl.py`
- `staleguard/geofence_turn/baselines/ppr.py`
- `staleguard/geofence_turn/support/diagnostics.py`
- `staleguard/geofence_turn/support/results/delay_tolerance_20/analyze.py`
- `staleguard/geofence_turn/support/tests/__init__.py`
- `staleguard/geofence_turn/support/tests/test_integration.py`
- `staleguard/geofence_turn/support/tests/test_mission.py`
- `staleguard/geofence_turn/support/tests/test_new_baselines.py`
- `staleguard/geofence_turn/support/tests/test_ppr.py`
- `staleguard/geofence_turn/support/tests/test_prediction.py`
- `staleguard/geofence_turn/support/tests/test_refactor_equivalence.py`
- `staleguard/geofence_turn/support/validation.py`
- `staleguard/geofence_turn/support/visualize_geofence_case.py`
- `staleguard/geofence_turn/ttl_pred_accuracy/metrics.py`
- `staleguard/geofence_turn/ttl_pred_accuracy/run.py`
- `staleguard/inspection_descent/baseline_benchmark/metrics.py`
- `staleguard/inspection_descent/baseline_benchmark/run.py`
- `staleguard/inspection_descent/baselines/always_recover.py`
- `staleguard/inspection_descent/baselines/fixed_early_recovery.py`
- `staleguard/inspection_descent/baselines/fixed_ttl.py`
- `staleguard/inspection_descent/baselines/ppr.py`
- `staleguard/inspection_descent/support/test_fbs.py`
- `staleguard/inspection_descent/support/test_fixed_ttl.py`
- `staleguard/inspection_descent/support/test_temporal_safety.py`
- `staleguard/inspection_descent/support/test_use_time_recheck.py`
- `staleguard/inspection_descent/support/test_workflow.py`
- `staleguard/inspection_descent/ttl_pred_accuracy/metrics.py`
- `staleguard/inspection_descent/ttl_pred_accuracy/run.py`
- `staleguard/model/pybullet.py`
- `staleguard/plot_dispatch_delay_effect.py`
- `staleguard/tables/aggregate_results.py`
- `staleguard/tables/complete_experiments.py`
- `staleguard/tables/tables_test_complete.py`
- `staleguard/underpass_inspection/baseline_benchmark/metrics.py`
- `staleguard/underpass_inspection/baseline_benchmark/run.py`
- `staleguard/underpass_inspection/pilot.py`
- `staleguard/underpass_inspection/support/deadline_sweep.py`
- `staleguard/underpass_inspection/support/feasibility_sweep.py`
- `staleguard/underpass_inspection/support/paper_figure3.py`
- `staleguard/underpass_inspection/support/profile_recovery_horizons.py`
- `staleguard/underpass_inspection/support/supplementary_analysis.py`
- `staleguard/underpass_inspection/support/test_baselines.py`
- `staleguard/underpass_inspection/support/test_benchmark.py`
- `staleguard/underpass_inspection/support/test_development.py`
- `staleguard/underpass_inspection/support/test_pilot.py`
- `staleguard/underpass_inspection/support/test_recovery_horizons.py`
- `staleguard/underpass_inspection/support/test_supplementary.py`
- `staleguard/underpass_inspection/support/validate_baseline_benchmark.py`
- `staleguard/underpass_inspection/support/validate_results.py`
- `staleguard/underpass_inspection/support/visualization.py`
- `staleguard/underpass_inspection/support/visualize_policy_comparison.py`

## Modified original files (excluding deletions)

- `.gitignore`
- `CITATION.cff`
- `README.md`
- `pyproject.toml`
- `staleguard/__init__.py`
- `staleguard/geofence_turn/__init__.py`
- `staleguard/geofence_turn/baselines/fixed_early_recovery.py`
- `staleguard/geofence_turn/baselines/recovery_horizons.py`
- `staleguard/geofence_turn/calibration.py`
- `staleguard/geofence_turn/simulation.py`
- `staleguard/geofence_turn/storage.py`
- `staleguard/inspection_descent/__init__.py`
- `staleguard/inspection_descent/calibration.py`
- `staleguard/inspection_descent/simulation.py`
- `staleguard/inspection_descent/storage.py`
- `staleguard/model/common.py`
- `staleguard/model/mission_evaluation.py`
- `staleguard/model/scg.py`
- `staleguard/underpass_inspection/__init__.py`
- `staleguard/underpass_inspection/baselines/policies.py`
- `staleguard/underpass_inspection/calibration.py`
- `staleguard/underpass_inspection/simulation.py`
- `staleguard/underpass_inspection/storage.py`

New modules: `staleguard/io.py`, `staleguard/reproduce/__init__.py`, `staleguard/reproduce/common.py`, `staleguard/reproduce/fig2.py`, `staleguard/reproduce/fig3.py`, `staleguard/reproduce/fig4.py`, `staleguard/reproduce/fig5.py`, `staleguard/reproduce/run_experiments.py`, `staleguard/reproduce/table2.py`, `staleguard/reproduce/table3.py`, `staleguard/visualize.py`.
