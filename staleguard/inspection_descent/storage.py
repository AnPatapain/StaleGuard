"""Mission snapshot validation and record I/O."""

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
        'approval_time',
        'horizon_steps',
        'low_height',
        'low_steps',
        'box_x_min',
        'box_x_max',
        'box_y_limit',
        'unsafe_below_z',
    ):
        if getattr(actual, name) != getattr(expected, name):
            raise ValueError(f'Approval snapshot differs: {name}')


def check_memory():
    """Original 2048 MiB process cap and 2048 MiB available-system reserve."""
    status = Path('/proc/self/status').read_text().splitlines()
    rss = int(next(line.split()[1] for line in status if line.startswith('VmRSS:'))) / 1024
    memory = Path('/proc/meminfo').read_text().splitlines()
    available = (
        int(next(line.split()[1] for line in memory if line.startswith('MemAvailable:'))) / 1024
    )
    if rss > 2048 or available < 2048:
        raise RuntimeError(
            f'Memory budget reached: process={rss:.0f} MiB, available={available:.0f} MiB'
        )
    return rss, available
