"""Preserve the locked E01 evidence and reconstruct its tables without simulation."""

from contextlib import contextmanager
import hashlib
import csv
import json
from pathlib import Path, PurePosixPath
import tarfile

from staleguard.io import file_hash
from .protocol import ROOT, REPO, atomic_json

FORMAT = 'staleguard-robustness-archive-v1'
RELATIVE = Path('underpass_inspection/robustness_v1')
TABLES = tuple(f'table_E01_{letter}.{suffix}' for letter in 'ABCD' for suffix in ('csv', 'md'))


def location(data_root):
    return Path(data_root).resolve() / RELATIVE


def verify_folder(folder, extract_to=None):
    manifest = json.loads((folder / 'manifest.json').read_text())
    if manifest['format'] != FORMAT or not manifest['locked']:
        raise ValueError('Unsupported or unlocked robustness archive')
    for name, entry in manifest['files'].items():
        path = folder / name
        if path.stat().st_size != entry['bytes'] or file_hash(path) != entry['sha256']:
            raise ValueError('Archived file is missing or corrupted: ' + name)
    seen = set()
    with tarfile.open(folder / 'records.tar.gz', mode='r|gz') as archive:
        first = archive.next()
        if first is None or first.name != 'ARCHIVE_INDEX.json' or not first.isfile():
            raise ValueError('Archive is missing its record index')
        contents = archive.extractfile(first).read()
        if hashlib.sha256(contents).hexdigest() != manifest['record_index_sha256']:
            raise ValueError('Archive record index checksum differs')
        expected = json.loads(contents)['files']
        for member in iter(archive.next, None):
            name = member.name
            relative = PurePosixPath(name)
            if (
                not member.isfile()
                or relative.is_absolute()
                or '..' in relative.parts
                or name not in expected
                or name in seen
            ):
                raise ValueError('Unexpected or unsafe archive member: ' + name)
            entry = expected[name]
            if member.size != entry['bytes']:
                raise ValueError('Archived record length differs: ' + name)
            stream = archive.extractfile(member)
            destination = None
            if extract_to is not None:
                path = extract_to / name
                path.parent.mkdir(parents=True, exist_ok=True)
                destination = path.open('wb')
            sha = hashlib.sha256()
            try:
                for block in iter(lambda: stream.read(1048576), b''):
                    sha.update(block)
                    if destination is not None:
                        destination.write(block)
            finally:
                if destination is not None:
                    destination.close()
            if sha.hexdigest() != entry['sha256']:
                raise ValueError('Archived record checksum differs: ' + name)
            seen.add(name)
    if seen != set(expected) or len(seen) != manifest['record_files']:
        raise ValueError('Archive is missing primary records')
    return manifest


def verify_archive(data_root):
    folder = location(data_root)
    manifest = verify_folder(folder)
    print('Verified', manifest['record_files'], 'archived records and all locked reference files')
    return manifest


@contextmanager
def analysis_root(path):
    from . import analysis

    previous = analysis.ROOT
    analysis.ROOT = path
    try:
        yield analysis
    finally:
        analysis.ROOT = previous


def reproduce_archive(data_root, out):
    if out is None:
        raise ValueError('Provide a fresh --out directory for the reconstructed robustness tables')
    out = Path(out).resolve()
    if out.is_relative_to(Path(data_root).resolve()) or out == ROOT or out.is_relative_to(ROOT):
        raise ValueError('--out must be outside immutable data and the locked original experiment')
    if out.exists() and any(out.iterdir()):
        raise ValueError('Use an empty --out directory; existing results will not be overwritten')
    out.mkdir(parents=True, exist_ok=True)
    folder = location(data_root)
    manifest = verify_folder(folder, extract_to=out)
    with analysis_root(out) as analysis:
        # Labels embedded in the locked tables are part of the archived presentation.
        # Current policy names remain independent of those immutable reference bytes.
        with (folder / 'table_E01_B.csv').open() as stream:
            labels = {row['policy']: row['label'] for row in csv.DictReader(stream)}
        analysis.summarize(labels=labels)
    matches = {}
    for name in TABLES:
        matches[name] = file_hash(out / name) == manifest['files'][name]['sha256']
    if not all(matches.values()):
        raise ValueError(
            'Reconstructed tables differ from the locked paper values: ' + str(matches)
        )
    atomic_json(
        out / 'ARCHIVE_REPRODUCTION.json',
        dict(
            status='COMPLETE',
            simulations_run=0,
            predictions_recomputed=0,
            table_byte_matches=matches,
            archive_manifest_sha256=file_hash(folder / 'manifest.json'),
            records_verified=manifest['record_files'],
        ),
    )
    print('All four CSV/Markdown tables match the locked paper values exactly:', out)
