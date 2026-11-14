# Immutable primary experimental data

`manifest.json` lists each source path, canonical destination, byte length and
SHA-256. Files were copied byte-for-byte: JSON formatting, journal ordering,
recorded runtimes, pickle bytes and arrays are unchanged. Historical source
hashes are metadata; they are not enforced against the cleaned Python source.

| Mission | Scenarios | Policy delays | Prediction delays |
|---|---:|---:|---:|
| Inspection Descent | 276 | 11 | 11 |
| Geofence Turn | 300 | 23 | 19 |
| Underpass Inspection | 400 | 13 | 19 |

Each mission has `calibration/`, `benchmark/` and `feasibility/`. Inspection and
Geofence also have `additional_policies/`, including historical recovery records
referenced by physical reuse mappings. Immutable journals can contain policies
absent from the final paper; Table III selects only the final eight.

`underpass_inspection/representative_flights/` holds Figure 2's actual seed-0
nominal, immediate-inspection, 0.4 s inspection and 0.5 s sideways-recovery flights.
This is not a Geofence trajectory dataset. Underpass calibration retains the 865
accepted candidates; its benchmark manifest selects the actual 400 evaluation scenarios.

## Required evidence

- `executions.jsonl`: physical outcomes and policy/action records.
- `predictions.jsonl`: recorded feasibility labels and original runtimes.
- `run.json`, `progress.json`: original configurations, seeds, numerical environment and hashes.
- Calibration manifests and snapshots: approved scenarios, physical/PID states and references.
- `.npz` files: approval arrays and measured trajectories.
- Geofence's calibration envelope and Inspection's referenced historical input manifest.

Old calibration JSONs can contain historical relative paths. The canonical
manifest explicitly maps snapshots. `staleguard.io.SnapshotReader` translates the
few historical Python class names during deserialization, without modifying data.
There is no historical Python source-hash compatibility enforcement.

## Distribution

All data are present locally. No external download URL or DOI has been assigned.
`underpass_inspection/benchmark/executions.jsonl` is 176,132,260 bytes, above
GitHub's ordinary per-file limit, and is gitignored. It must be distributed
separately with this manifest before an external checkout can reproduce all artifacts.
Missing raw records cannot be replaced by `metrics.json`.

The local `.stage2_archive/` preserves original/development files, is gitignored,
is excluded from installation, and is not used by reproduction commands.
Derived metrics, tables, figures and reports belong under `outputs/`, not `data/`.

Each artifact command validates all primary files it uses. To check everything:

```bash
python - <<'CHECK'
import json, hashlib
from pathlib import Path
root = Path('data')
manifest = json.loads((root / 'manifest.json').read_text())
for name, expected in manifest['files'].items():
    path = root / name
    assert path.stat().st_size == expected['bytes'], name
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    assert digest.hexdigest() == expected['sha256'], name
print('Verified', len(manifest['files']), 'immutable primary files')
CHECK
```
