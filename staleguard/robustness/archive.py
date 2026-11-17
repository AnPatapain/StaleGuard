"""Preserve the locked E01 evidence and reconstruct its tables without simulation."""

from contextlib import contextmanager
import gzip
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile

from staleguard.io import file_hash
from .protocol import ROOT, REPO, atomic_json

FORMAT = 'staleguard-robustness-archive-v1'
RELATIVE = Path('underpass_inspection/robustness_v1')
TABLES = tuple(f'table_E01_{letter}.{suffix}' for letter in 'ABCD' for suffix in ('csv', 'md'))


def location(data_root):
    return Path(data_root).resolve() / RELATIVE


def selected_records(source):
    original = json.loads((source / 'manifest.json').read_text())
    files = {name: dict(entry) for name, entry in original['primary_files'].items()}
    additional = [
        'manifest.json',
        'specification.json',
        'specification.sha256',
        'selection.json',
        'implementation.json',
        'simulator_assets.json',
        'PILOT.json',
        'progress.json',
        'verification/TEST_RESULTS.json',
        'verification/tests.log',
        'verification/RESUME_PILOT.json',
        'verification/INTERRUPTION_RESULTS.json',
    ]
    additional.extend(str(p.relative_to(source)) for p in (source / 'execution').glob('*.json'))
    additional.extend(
        str(p.relative_to(source))
        for p in (source / 'nominal_audit').rglob('*')
        if p.is_file() and p.suffix != '.log' and '__pycache__' not in p.parts
    )
    for name in additional:
        path = source / name
        if path.is_file():
            files[name] = dict(bytes=path.stat().st_size, sha256=file_hash(path))
    for name, entry in files.items():
        path = source / name
        if path.stat().st_size != entry['bytes'] or file_hash(path) != entry['sha256']:
            raise ValueError('Locked source record differs from its manifest: ' + name)
    return dict(sorted(files.items()))


def freeze_archive(data_root):
    source = ROOT
    target = location(data_root)
    if target.exists():
        verify_archive(data_root)
        print('The existing locked archive is valid; nothing was overwritten:', target)
        return
    validation = json.loads((source / 'VALIDATION.json').read_text())
    if (
        validation['status'],
        validation['outcomes'],
        validation['forecasts'],
        validation['h2_scenario_conditions'],
    ) != ('COMPLETE', 24960, 320, 320):
        raise ValueError('Only the complete, validated E01 evaluation can be archived')
    if json.loads((source / 'progress.json').read_text()).get('running'):
        raise ValueError('Cannot freeze an experiment that is still running')
    files = selected_records(source)
    index = json.dumps(dict(format=FORMAT, files=files), indent=2).encode() + b'\n'
    index_sha = hashlib.sha256(index).hexdigest()
    staging = target.with_name(target.name + '.copying')
    if staging.exists():
        raise ValueError(
            'An unfinished archive copy exists; inspect it before retrying: ' + str(staging)
        )
    staging.mkdir(parents=True)
    with (staging / 'records.tar.gz').open('wb') as stream:
        with gzip.GzipFile(
            fileobj=stream, mode='wb', filename='', mtime=0, compresslevel=6
        ) as zipped:
            with tarfile.open(fileobj=zipped, mode='w|') as archive:
                header = tarfile.TarInfo('ARCHIVE_INDEX.json')
                header.size = len(index)
                header.mode = 0o444
                archive.addfile(header, io.BytesIO(index))
                for name, entry in files.items():
                    header = tarfile.TarInfo(name)
                    header.size = entry['bytes']
                    header.mode = 0o444
                    with (source / name).open('rb') as original:
                        archive.addfile(header, original)
    copied = {}
    references = list(TABLES) + ['REPORT.md', 'VALIDATION.json']
    references += ['nominal_audit/REPORT.md', 'nominal_audit/SUMMARY.json']
    for name in references:
        destination = staging / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / name, destination)
        if file_hash(destination) != file_hash(source / name):
            raise ValueError('Copy verification failed: ' + name)
        copied[name] = dict(
            bytes=destination.stat().st_size,
            sha256=file_hash(destination),
            source=str((source / name).relative_to(REPO)),
        )
    bundle = staging / 'records.tar.gz'
    copied['records.tar.gz'] = dict(
        bytes=bundle.stat().st_size,
        sha256=file_hash(bundle),
        source='Byte-preserving bundle of the original per-record files',
    )
    spec = json.loads((source / 'specification.json').read_text())
    manifest = dict(
        format=FORMAT,
        locked=True,
        experiment='E01',
        source='outputs/robustness_v1',
        mission=spec['mission'],
        scenarios=64,
        conditions=spec['conditions'],
        policies=spec['policies'],
        policy_delays=spec['policy_delays'],
        prediction_delays=spec['prediction_delays'],
        selected_seeds=spec['selected_seeds'],
        outcomes=24960,
        forecasts=320,
        record_index_sha256=index_sha,
        record_files=len(files),
        original_record_bytes=sum(entry['bytes'] for entry in files.values()),
        source_manifest_sha256=file_hash(source / 'manifest.json'),
        original_specification_sha256=file_hash(source / 'specification.json'),
        original_data_manifest_sha256=spec['data_manifest_sha256'],
        validation=validation,
        files=copied,
        exclusions=[
            'Caches, sockets, locks, process IDs, console logs and duplicate JSONL exports'
        ],
        table_role='Locked paper reference values; reconstruction uses the underlying recorded evidence',
    )
    atomic_json(staging / 'manifest.json', manifest)
    verify_folder(staging)
    staging.rename(target)
    print('Archived and byte-verified:', target)
    print(
        'Record files:',
        len(files),
        'compressed bytes:',
        bundle.stat().st_size if bundle.exists() else (target / 'records.tar.gz').stat().st_size,
    )


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
        analysis.summarize()
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
