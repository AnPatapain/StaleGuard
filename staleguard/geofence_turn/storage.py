"""Minimal mission snapshot checks and record I/O; historical source hashes are metadata only."""

from pathlib import Path
import numpy as np
from staleguard.io import write_json, append_json, file_hash, save_snapshot, metadata, load_pickle

ROOT = Path(__file__).resolve().parent


def check_approval_snapshot(actual, expected):
    for name in ('state', 'remaining_reference'):
        np.testing.assert_array_equal(getattr(actual, name), getattr(expected, name))
    for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
        np.testing.assert_array_equal(
            getattr(actual.pid_state, name), getattr(expected.pid_state, name)
        )
    for name in (
        'horizon_steps',
        'approval_time',
        'command',
        'fence_x',
        'config',
        'prefix_min_clearance_m',
    ):
        if getattr(actual, name) != getattr(expected, name):
            raise ValueError(f'Approval snapshot differs: {name}')


from dataclasses import asdict
from .scenario import SCHEMA

DEFAULT_ENVELOPE = ROOT.parent.parent / 'data/geofence_turn/calibration/envelope.json'


def open_calibration(output, seeds, target, envelope_path, config, resume=False):
    output = Path(output)
    if resume or output.exists() and any(output.iterdir()):
        raise FileExistsError('Use a fresh calibration output')
    (output / 'snapshots').mkdir(parents=True, exist_ok=True)
    return dict(
        format='compact_calibration_v1',
        schema=SCHEMA,
        config=asdict(config),
        provenance=metadata(config),
        scenarios=[],
        statistics={},
        calibration=dict(
            status='running',
            settings=dict(
                seeds=list(seeds), target=target, envelope_sha256=file_hash(envelope_path)
            ),
            attempts=[],
        ),
    )


def save_progress(output, data):
    attempted = len(data['calibration']['attempts'])
    accepted = len(data['scenarios'])
    data['statistics'] = dict(
        generated=len(data['calibration']['settings']['seeds']),
        attempted=attempted,
        accepted=accepted,
        rejected=attempted - accepted,
        acceptance_rate=accepted / attempted if attempted else None,
    )
    write_json(Path(output) / 'calibrated_scenarios.json', data)
