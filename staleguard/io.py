"""Primary record I/O and isolated compatibility for historical pickle names."""

from pathlib import Path
from dataclasses import asdict, is_dataclass
from importlib.metadata import version
import hashlib, json, pickle, platform, subprocess, sys


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


class SnapshotReader(pickle.Unpickler):
    def find_class(self, module, name):
        if module == 'experiment' and name in ('Scenario', 'ApprovalSnapshot'):
            module = 'staleguard.inspection_descent.scenario'
        elif module == 'geofence_turn.mission' and name in ('Scenario', 'Config', 'Command'):
            module = 'staleguard.geofence_turn.scenario'
        elif module.split('.')[0] in (
            'model',
            'physics_monitor',
            'inspection_descent',
            'geofence_turn',
            'underpass_inspection',
        ):
            module = 'staleguard.' + module
        return super().find_class(module, name)


def load_pickle(path):
    with Path(path).open('rb') as f:
        return SnapshotReader(f).load()


def write_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(
        json.dumps(
            value,
            indent=2,
            allow_nan=False,
            default=lambda x: (
                x.tolist() if hasattr(x, 'tolist') else asdict(x) if is_dataclass(x) else str(x)
            ),
        )
        + '\n'
    )


def append_json(path, row):
    with Path(path).open('a') as f:
        f.write(json.dumps(row, allow_nan=False) + '\n')
        f.flush()


def save_snapshot(path, snapshot):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(pickle.dumps(snapshot, protocol=pickle.HIGHEST_PROTOCOL))
    return file_hash(path)


def metadata(config=None):
    try:
        revision = subprocess.check_output(
            ['git', '-C', str(Path(__file__).resolve().parent.parent), 'rev-parse', 'HEAD'],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    return dict(
        git_revision=revision,
        python=sys.version,
        platform=platform.platform(),
        packages={
            n: version(n)
            for n in ('numpy', 'scipy', 'numba', 'pybullet', 'gymnasium', 'matplotlib', 'casadi')
        },
        config=asdict(config) if is_dataclass(config) else config,
    )
