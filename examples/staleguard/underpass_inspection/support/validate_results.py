"""Independent array-based checks of saved actual flights; no simulation runs."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
import hashlib
import json
import numpy as np


def validate(folder):
    folder = Path(folder)
    summary = json.loads((folder/'summary.json').read_text())
    rows = [json.loads(line) for line in (folder/'flights.jsonl').read_text().splitlines()]
    source_root = Path(__file__).resolve().parents[4]
    checks = dict(complete=summary['status'] == 'complete',
                  count_matches=len(rows) == summary['flight_count'],
                  no_duplicate_flights=len({r['trajectory_key'] for r in rows}) == len(rows),
                  sources_match=all(hashlib.sha256((source_root/name).read_bytes()).hexdigest() == digest
                                    for name, digest in summary['source_sha256'].items()))
    s = summary['scenario']
    errors = []
    with np.load(folder/'trajectories.npz') as archive:
        for row in rows:
            key = row['trajectory_key']
            physics = archive[key+'__physics']
            t, positions = physics[:, 0], physics[:, 1:]
            try:
                assert len(t) == row['horizon_step']*20+1
                np.testing.assert_allclose(t, np.arange(len(t))/1000, atol=1e-12, rtol=0)
                assert t[-1] == row['horizon_flight_s']
                assert hashlib.sha256(physics.tobytes()).hexdigest() == row['physics_sha256']
                distances = []
                # Independent vector form: centered box / half-extents, rather
                # than the scalar lower/upper implementation used during flight.
                for box in summary['obstacle_boxes']:
                    lo, hi = np.array(box['lower']), np.array(box['upper'])
                    q = np.abs(positions-(lo+hi)/2)-(hi-lo)/2
                    distance = np.linalg.norm(np.maximum(q, 0), axis=1)+np.minimum(np.max(q, axis=1), 0)
                    distances.append(distance-summary['body_radius_m'])
                distances = np.array(distances)
                np.testing.assert_allclose(np.min(distances), row['min_clearance_m'], atol=1e-12, rtol=0)
                bad = np.flatnonzero(np.min(distances, axis=0) < s['clearance_m'])
                assert row['safe'] == (len(bad) == 0)
                if len(bad):
                    assert bad[0] == row['first_violation']['physics_tick']
                    assert summary['obstacle_boxes'][int(np.argmin(distances[:, bad[0]]))]['name'] == row['first_violation']['obstacle']
                mask = ((t >= s['approval_s'])
                        & np.all(positions >= s['inspection_lower'], axis=1)
                        & np.all(positions <= s['inspection_upper'], axis=1))
                edges = np.diff(np.r_[False, mask, False].astype(int))
                starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
                dwell = int(max(ends-starts-1, default=0))
                assert row['inspection_completed'] == (dwell >= round(1000*s['inspection_dwell_s']))
                assert row['safe_completion'] == (row['safe'] and row['inspection_completed'])
                assert positions[-1, 0] > s['deck_back']
                if row['mode'] != 'nominal':
                    assert row['pid_unchanged_at_initiation'] and row['physical_state_unchanged_at_initiation']
                assert row['geometry_unchanged']
            except (AssertionError, ValueError) as error:
                errors.append(dict(flight=key, error=str(error)))
    checks['all_saved_physics_and_outcomes_match'] = not errors
    if summary['sweep_grid_s'] is not None:
        expected = {(m, d) for m in ('dive', 'climb', 'sideways') for d in summary['sweep_grid_s']}
        observed = {(r['mode'], r['start_s']) for r in rows if r['mode'] != 'nominal'}
        checks['full_requested_grid'] = expected == observed
    return dict(passed=all(checks.values()), checks=checks, errors=errors,
                inspected_flights=len(rows), reran_physics=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    result = validate(args.folder)
    (args.folder/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    if not result['passed']:
        raise SystemExit(1)
