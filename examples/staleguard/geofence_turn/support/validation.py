"""Audit the small northbound calibration artifacts; runs no new flights."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import numpy as np
from ..scenario import BODY_RADIUS_M, Config, HZ, SCHEMA, scenario_from_dict


def validate(logs):
    logs = Path(logs)
    dev = logs/'development'
    config = Config()
    attempts = [json.loads(x) for x in (dev/'attempts.jsonl').read_text().splitlines()]
    candidate_count = accepted = rejected = 0
    for attempt in attempts:
        if 'generator_rejection' in attempt['generated']:
            assert not attempt['accepted'] and attempt['candidate_count'] == 0
            rejected += 1
            continue
        s = scenario_from_dict(attempt['generated'])
        assert float(s.fence_x-s.command.checkpoint_upper[0]-BODY_RADIUS_M) >= config.d_min_m-1e-12
        directory = dev/f'seed_{s.seed}'
        rows = [json.loads(x) for x in (directory/'candidates.jsonl').read_text().splitlines()]
        candidate_count += len(rows)
        assert len(rows) == attempt['candidate_count']
        distances = [row['nominal_patrol_distance_m'] for row in rows]
        assert distances == sorted(set(distances), reverse=True)
        nominal = json.loads((directory/'nominal_bound.json').read_text())
        assert rows[0]['approval_s'] >= nominal['first_violation_s']
        assert all(row['rejected_because'] for row in rows[:-1])
        for row in rows:
            assert row['command'] == attempt['generated']['command']
            outcome = row['outcome']
            assert outcome['safe_completion'] == (outcome['safe'] and outcome['completed'])
            assert outcome['physics_samples'] == outcome['expected_physics_samples']
            if outcome['completed']:
                assert outcome['completion_s'] <= s.deadline_s
        if not attempt['accepted']:
            assert rows[-1]['rejected_because']
            rejected += 1
            continue
        accepted += 1
        assert not rows[-1]['rejected_because']
        assert rows[-1]['approval_s'] == attempt['selected']['approval_s']
        assert hashlib.sha256((directory/'approval.pkl').read_bytes()).hexdigest() == attempt['snapshot_sha256']
        assert json.loads((directory/'configuration.json').read_text()) == asdict(config)
        selected = scenario_from_dict(attempt['selected'])
        result = json.loads((directory/'d0_replay.json').read_text())
        assert result['safe_completion'] and result['min_clearance_m'] >= config.d_min_m
        assert result['completion_s'] <= selected.deadline_s
        trace = np.load(directory/'d0_trace.npz')['controls']
        assert np.min(np.diff(trace[:,13])) >= -1e-12
        assert np.min(trace[:,14]) >= 0
        arc_end = round(selected.approval_s*HZ) + int(np.ceil(selected.command.arc_s*HZ))
        np.testing.assert_allclose(trace[arc_end:,13], trace[arc_end,13], atol=1e-12)
        np.testing.assert_allclose(trace[arc_end:,14], 0, atol=1e-12)
        np.testing.assert_allclose(trace[arc_end:,16], selected.speed, atol=1e-12)
        assert result['min_clearance_m'] <= np.min(selected.fence_x-trace[:,1]-BODY_RADIUS_M)+1e-6
    return dict(schema=SCHEMA, accepted=accepted, rejected=rejected, attempted=len(attempts),
                audited_candidates=candidate_count, baseline_flights=0, status='PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--logs', default=str(Path(__file__).resolve().parent/'logs/northbound_review'))
    print(json.dumps(validate(parser.parse_args().logs), indent=2))
