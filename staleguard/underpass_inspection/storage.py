"""Minimal mission snapshot checks and record I/O; historical source hashes are metadata only."""

from pathlib import Path
from dataclasses import asdict
import json
import numpy as np
from staleguard.io import write_json, append_json, file_hash, save_snapshot, metadata, load_pickle

ROOT = Path(__file__).resolve().parent


def canonical(value):
    return json.loads(json.dumps(value))


def check_approval_snapshot(actual, expected):
    def item(value, name):
        return value[name] if isinstance(value, dict) else getattr(value, name)

    for name in ('state', 'remaining_reference'):
        np.testing.assert_array_equal(item(actual, name), item(expected, name))
    for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
        np.testing.assert_array_equal(
            getattr(item(actual, 'pid_state'), name), getattr(item(expected, 'pid_state'), name)
        )
    for name in ('horizon_steps', 'approval_time'):
        if item(actual, name) != item(expected, name):
            raise ValueError(f'Approval snapshot differs: {name}')
    if canonical(asdict(item(actual, 'scenario'))) != canonical(asdict(item(expected, 'scenario'))):
        raise ValueError('Approval scenario differs')
