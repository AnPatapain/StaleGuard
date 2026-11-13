"""Read-only compatibility for frozen snapshots and recorded source paths."""
from pathlib import Path
import hashlib
import json
import pickle

ROOT = Path(__file__).resolve().parent
REPOSITORY = ROOT.parent
LEGACY_PACKAGES = {'model', 'inspection_descent', 'geofence_turn', 'underpass_inspection', 'physics_monitor'}

class SnapshotUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.split('.')[0] in LEGACY_PACKAGES:
            module = 'staleguard.' + module
        return super().find_class(module, name)

def load_snapshot(path):
    with Path(path).open('rb') as stream:
        return SnapshotUnpickler(stream).load()

def source_path(path):
    """Resolve archived checkout paths without rewriting their manifests."""
    path = Path(path).resolve()
    marker = '/examples/staleguard/'
    if marker in path.as_posix():
        return ROOT / path.as_posix().split(marker, 1)[1]
    if not path.exists() and '/safe_control_gym/' in path.as_posix():
        return REPOSITORY / 'safe_control_gym' / path.as_posix().split('/safe_control_gym/', 1)[1]
    return path

def source_matches(path, expected, hash_file=None):
    """Accept exact bytes or an explicitly pinned root-relocation pair."""
    path = source_path(path)
    if not path.is_file():
        return False
    actual = hash_file(path) if hash_file is not None else hashlib.sha256(path.read_bytes()).hexdigest()
    if actual == expected:
        return True
    try:
        name = path.relative_to(ROOT).as_posix()
    except ValueError:
        return False
    manifest = json.loads((ROOT / 'source_root_relocation.json').read_text())
    entry = manifest['sources'].get(name, {})
    return entry.get('before_sha256') == expected and entry.get('after_sha256') == actual
