"""Small data-integrity, analysis and output helpers; no archived metrics input."""

from pathlib import Path
from collections import Counter, defaultdict
from functools import cached_property
import argparse, csv, hashlib, json, platform, subprocess, sys
from importlib.metadata import version
import numpy as np
from staleguard.underpass_inspection.baselines.staleguard import first_ready_step
from staleguard.io import file_hash
from staleguard.policies import POLICIES, LABELS

MISSIONS = ('inspection_descent', 'geofence_turn', 'underpass_inspection')

PROPERTIES = {
    'command': 'command_safe',
    'completion': 'command_complete',
    'recovery': 'recovery_safe',
}
COUNTS = dict(zip(MISSIONS, (276, 300, 400)))


def write_json(path, value):
    Path(path).write_text(
        json.dumps(
            value,
            indent=2,
            allow_nan=False,
            default=lambda x: (
                x.item()
                if isinstance(x, np.generic)
                else x.tolist() if isinstance(x, np.ndarray) else str(x)
            ),
        )
        + '\n'
    )


def write_csv(path, rows):
    rows = list(rows)
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--data-root', type=Path, default=Path('data'))
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.resolve().is_relative_to(args.data_root.resolve()):
        parser.error('--out must be outside immutable data')
    args.out.mkdir(parents=True, exist_ok=True)
    return args, Data(args.data_root)


class Data:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.manifest = json.loads((self.root / 'manifest.json').read_text())
        self.used = {}
        self.cache = {}
        if self.manifest['format'] != 'staleguard-primary-data-v1':
            raise ValueError('Unsupported data manifest')

    def file(self, relative):
        path = self.root / relative
        info = self.manifest['files'][relative]
        if relative not in self.used:
            if (
                not path.is_file()
                or path.stat().st_size != info['bytes']
                or file_hash(path) != info['sha256']
            ):
                raise ValueError('Missing/corrupted primary data: ' + str(path))
            self.used[relative] = info['sha256']
        return path

    def entry(self, name):
        return self.manifest['datasets'][name]

    def read_json(self, name, filename):
        return json.loads(self.file(self.entry(name)['files'][filename]).read_text())

    def run(self, name):
        r = self.read_json(name, 'run.json')
        if 'progress.json' in self.entry(name)['files']:
            r.update(self.read_json(name, 'progress.json'))
        if r.get('status') not in ('complete', 'SUCCESS'):
            raise ValueError('Incomplete dataset: ' + name)
        if r.get('config_sha256') and r['config_sha256'] != file_hash(
            self.file(self.entry(name)['files']['run.json'])
        ):
            raise ValueError('Original run manifest hash differs')
        for filename, prefix in (
            ('executions.jsonl', 'journal'),
            ('predictions.jsonl', 'prediction'),
        ):
            if filename in self.entry(name)['files']:
                record = self.manifest['files'][self.entry(name)['files'][filename]]
                if prefix + '_sha256' in r and record['sha256'] != r[prefix + '_sha256']:
                    raise ValueError('Original journal hash differs')
                if prefix + '_bytes' in r and record['bytes'] != r[prefix + '_bytes']:
                    raise ValueError('Original journal length differs')
        return r

    def records(self, name, filename='executions.jsonl'):
        cache_key = name, filename
        if cache_key not in self.cache:
            p = self.file(self.entry(name)['files'][filename])
            rows = [json.loads(line) for line in p.open()]
            expected = self.entry(name).get(filename + '_records')
            if expected is not None and expected != len(rows):
                raise ValueError('Record count mismatch: ' + name)
            keys = set()
            for row in rows:
                if filename == 'predictions.jsonl':
                    key = row['policy'], row['seed']
                elif name.endswith('/feasibility'):
                    key = (
                        row['seed'],
                        row.get('kind', row.get('mode')),
                        round(
                            (
                                row['start_s']
                                if 'start_s' in row and row.get('mode') in ('dive', 'sideways')
                                else row['delay_s']
                            )
                            * 50
                        ),
                    )
                elif 'event' in row:
                    key = row['seed'], row['event']['policy'], round(row['delay_s'] * 50)
                else:
                    key = row['trajectory_key']
                if key in keys:
                    raise ValueError('Duplicate scientific key: ' + str((name, key)))
                keys.add(key)
            self.cache[cache_key] = rows
        return self.cache[cache_key]

    def forecasts(self, mission, policy='staleguard_dual_horizon'):
        name = mission + ('/benchmark' if mission == 'underpass_inspection' else '/feasibility')
        rows = self.records(name, 'predictions.jsonl')
        result = {}
        for row in rows:
            if row['policy'] == policy:
                if str(row['seed']) in result:
                    raise ValueError('Duplicate forecast')
                result[str(row['seed'])] = row['forecast']
        if len(result) != COUNTS[mission]:
            raise ValueError('Forecast cohort mismatch')
        return result

    def policy_records(self, mission):
        names = [mission + '/benchmark']
        if mission == 'geofence_turn':
            names.append(mission + '/additional_policies/fixed_early')
        if mission != 'underpass_inspection':
            names.append(mission + '/additional_policies/three_opportunity')
        rows = [r for name in names for r in self.records(name) if r['event']['policy'] in POLICIES]
        run = self.run(mission + '/benchmark')
        scenarios = run['protocol']['scenarios']
        seeds = sorted(s['seed'] for s in scenarios)
        delays = run['protocol']['delays']
        canonical = {s['seed']: s for s in scenarios}
        for name in names:
            if {s['seed']: s for s in self.run(name)['protocol']['scenarios']} != canonical:
                raise ValueError('Policy scenario configuration differs')
        expected = {(s, round(d * 50), p) for s in seeds for d in delays for p in POLICIES}
        seen = set()
        for r in rows:
            k = r['seed'], round(r['delay_s'] * 50), r['event']['policy']
            if k in seen or k not in expected:
                raise ValueError('Duplicate/unexpected policy execution: ' + str(k))
            seen.add(k)
            if r['safe_completion'] != (r['safe'] and r['completed']):
                raise ValueError('Inconsistent outcome')
        if len(seeds) != COUNTS[mission] or seen != expected:
            raise ValueError('Incomplete policy grid')
        return rows, seeds, delays

    def physical(self, mission):
        rows = self.records(mission + '/feasibility')
        forecasts = self.forecasts(mission)
        grid = next(iter(forecasts.values()))['grid_s']
        seeds = sorted(map(int, forecasts))
        truth = {}
        source = self.run(mission + '/benchmark')['protocol']['scenarios']
        continuations = self.run(mission + '/feasibility')['protocol']['scenarios']
        if {s['seed']: s for s in source} != {s['seed']: s for s in continuations}:
            raise ValueError('Physical scenario configuration differs')
        for r in rows:
            kind = (
                ('command' if r['mode'] == 'dive' else 'recovery')
                if mission == 'underpass_inspection'
                else r['kind']
            )
            delay = r['start_s'] if mission == 'underpass_inspection' else r['delay_s']
            key = r['seed'], kind, round(delay * 50)
            if key in truth:
                raise ValueError('Duplicate physical continuation')
            if r['safe_completion'] != (
                r['safe'] and r.get('completed', r.get('inspection_completed'))
            ):
                raise ValueError('Inconsistent physical labels')
            truth[key] = r
        if set(truth) != {
            (s, k, round(d * 50)) for s in seeds for k in ('command', 'recovery') for d in grid
        }:
            raise ValueError('Physical grid incomplete')
        return forecasts, truth, grid, seeds

    def verify_reuse(self, mission):
        names = [
            n
            for n in self.manifest['datasets']
            if n.startswith(mission + '/') and 'executions.jsonl' in self.entry(n)['files']
        ]
        aliases = {self.entry(n)['source'].removeprefix('staleguard/'): n for n in names}
        indices = {}
        for name in names:
            index = defaultdict(list)
            for r in self.records(name):
                index[r['seed'], round(r['delay_s'] * 50), r['event']['policy']].append(r)
            indices[name] = index
        count = 0
        for name in names:
            for r in self.records(name):
                ps = r.get('physical_source')
                if not ps:
                    continue
                origin = ps.get('journal', mission + '/results/evaluation/baseline_benchmark')
                origin = origin.removeprefix('staleguard/').removesuffix('/executions.jsonl')
                if origin not in aliases:
                    raise ValueError('Missing physical reuse source: ' + origin)
                candidates = indices[aliases[origin]][
                    ps['seed'], round(ps['delay_s'] * 50), ps['policy']
                ]

                def action(x):
                    e = x['event']
                    return (
                        ('recovery', e['fallback_step'])
                        if e.get('fallback_step') is not None
                        else (
                            ('command', e['application_step'])
                            if e.get('application_step') is not None
                            else None
                        )
                    )

                matches = [
                    x
                    for x in candidates
                    if action(x) == action(r)
                    and all(
                        x.get(f) == r.get(f)
                        for f in ('safe', 'completed', 'safe_completion', 'horizon_s')
                    )
                ]
                if not matches:
                    raise ValueError(
                        'Physical continuation not equivalent: '
                        + str((name, r['seed'], r['delay_s'], r['event']['policy']))
                    )
                for field in (
                    'physics_sha256',
                    'control_sha256',
                    'min_clearance_m',
                    'final_position',
                    'final_velocity',
                ):
                    if field in r and not any(x.get(field) == r[field] for x in matches):
                        raise ValueError('Reused physics differs: ' + field)
                count += 1
        return count


def metadata(data, out, artifact, **extra):
    from staleguard.io import metadata as runtime_metadata

    write_json(
        Path(out) / 'metadata.json',
        dict(
            artifact=artifact,
            evidence='immutable archived observations; no simulation',
            **runtime_metadata(),
            input_sha256=data.used,
            data_manifest_sha256=file_hash(data.root / 'manifest.json'),
            **extra,
        ),
    )


def plot_style():
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            'font.family': 'DejaVu Serif',
            'font.size': 9,
            'axes.spines.top': False,
            'axes.spines.right': False,
            'pdf.fonttype': 42,
            'savefig.bbox': 'tight',
        }
    )
    return plt


def save_figure(fig, out, name):
    fig.savefig(Path(out) / (name + '.pdf'))
    fig.savefig(Path(out) / (name + '.png'), dpi=250)


def table_text(out, name, rows):
    fields = list(rows[0])
    lines = ['| ' + ' | '.join(fields) + ' |', '| ' + ' | '.join('---' for _ in fields) + ' |']
    for row in rows:
        lines.append(
            '| '
            + ' | '.join('N/M' if row.get(k) is None else str(row.get(k, '')) for k in fields)
            + ' |'
        )
    (Path(out) / (name + '.md')).write_text('\n'.join(lines) + '\n')


def ratio(a, b):
    return None if not b else a / b


def boundary(flags, grid):
    i = next((i for i, v in enumerate(flags) if not v), None)
    return dict(
        first_loss_s=None if i is None else grid[i],
        last_safe_prefix_s=grid[-1] if i is None else None if i == 0 else grid[i - 1],
        initially_infeasible=i == 0,
        right_censored=i is None,
        non_monotonic=any(not a and b for a, b in zip(flags, flags[1:])),
        labels=flags,
    )


def profile_metrics(forecasts, truth, grid, seeds):
    result = {}
    physical = {}
    errors_by_seed = {}
    for seed in seeds:
        physical[seed] = {
            p: boundary(
                [
                    truth[seed, 'recovery' if p == 'recovery' else 'command', round(d * 50)][
                        'safe' if p != 'completion' else 'safe_completion'
                    ]
                    for d in grid
                ],
                grid,
            )
            for p in PROPERTIES
        }
    for prop, field in PROPERTIES.items():
        counts = Counter()
        errors = []
        details = []
        initial = Counter()
        censored = Counter()
        nonmono = Counter()
        scenarios = Counter()
        for seed in seeds:
            f = forecasts[str(seed)]
            assert [round(r['delay_s'] * 50) for r in f['results']] == [round(d * 50) for d in grid]
            pred = boundary([r[field] for r in f['results']], grid)
            act = physical[seed][prop]
            q = Counter(
                'tp' if p and t else 'tn' if not p and not t else 'fp' if p else 'fn'
                for p, t in zip(pred['labels'], act['labels'])
            )
            counts.update(q)
            scenarios['false_feasible'] += q['fp'] > 0
            scenarios['false_reject'] += q['fn'] > 0
            initial['predicted'] += pred['initially_infeasible']
            initial['physical'] += act['initially_infeasible']
            censored['predicted'] += pred['right_censored']
            censored['physical'] += act['right_censored']
            nonmono['predicted'] += pred['non_monotonic']
            nonmono['physical'] += act['non_monotonic']
            error = (
                None
                if pred['first_loss_s'] is None or act['first_loss_s'] is None
                else round(pred['first_loss_s'] - act['first_loss_s'], 10)
            )
            if error is not None:
                errors.append(error)
            details.append(
                dict(seed=seed, predicted=pred, physical=act, sampled_boundary_error_s=error)
            )
        total = sum(counts.values())
        assert total == len(seeds) * len(grid)
        # Keep optional errors null when no finite sampled boundary pair exists.
        result[prop] = dict(
            status='COMPLETE',
            scenarios=len(seeds),
            physical_comparisons=total,
            counts={k: counts[k] for k in ('tp', 'tn', 'fp', 'fn')},
            accuracy=ratio(counts['tp'] + counts['tn'], total),
            false_feasible_rate=ratio(counts['fp'], counts['fp'] + counts['tn']),
            false_reject_rate=ratio(counts['fn'], counts['fn'] + counts['tp']),
            error_scenarios=dict(scenarios),
            boundary_pairs=len(errors),
            boundary_mae_s=float(np.mean(np.abs(errors))) if errors else None,
            boundary_median_absolute_error_s=float(np.median(np.abs(errors))) if errors else None,
            initial_cases=dict(initial),
            right_censored=dict(censored),
            non_monotonic=dict(nonmono),
            per_scenario=details,
        )
    # All forecasts, not just policy-selected continuations, are timed once per seed.
    runtimes = np.array([forecasts[str(seed)]['computation_s'] for seed in seeds])
    readiness = Counter()
    readiness_rows = []
    for seed in seeds:
        r = physical[seed]['recovery']
        ready = first_ready_step(forecasts[str(seed)]['computation_s']) / 50
        if r['first_loss_s'] is None:
            state = 'recovery_loss_unobserved'
        elif ready >= r['first_loss_s']:
            state = 'late_at_or_after_first_unsafe_sample'
        elif r['last_safe_prefix_s'] is not None and ready <= r['last_safe_prefix_s']:
            state = 'ready_by_last_safe_sample'
        else:
            state = 'ready_inside_unresolved_boundary_bracket'
        readiness[state] += 1
        readiness_rows.append(
            dict(
                seed=seed,
                ready_s=ready,
                classification=state,
                physical_recovery_first_loss_s=r['first_loss_s'],
            )
        )
    observed = len(seeds) - readiness['recovery_loss_unobserved']
    return (
        result,
        dict(
            timed_forecasts=len(seeds),
            runtime_median_s=float(np.median(runtimes)),
            runtime_p95_s=float(np.quantile(runtimes, 0.95)),
            observed_physical_recovery_boundaries=observed,
            late_recovery_loss_rate_over_observed=ratio(
                readiness['late_at_or_after_first_unsafe_sample'], observed
            ),
            ready_by_last_safe_sample_rate_over_observed=ratio(
                readiness['ready_by_last_safe_sample'], observed
            ),
            physical_readiness=dict(readiness),
            physical_readiness_per_scenario=readiness_rows,
        ),
        physical,
    )


def compact(row):
    e = row['event']
    a = row.get('approval_step', e['scheduled_dispatch_step'] - round(row['delay_s'] * 50))
    return dict(
        seed=row['seed'],
        delay_s=round(row['delay_s'], 2),
        policy=e['policy'],
        safe=row['safe'],
        safe_completion=row['safe_completion'],
        completed=row['completed'],
        application_step=e['application_step'],
        fallback_step=e['fallback_step'],
        approval_step=a,
        first_violation_s=row.get('first_violation_s'),
        kind=row.get('execution_kind', 'archived_physical_flight'),
        physical_source=row.get('physical_source'),
        canceled=e.get('command_canceled'),
        physics_sha256=row.get('physics_sha256'),
    )


def group_metrics(rows):
    n = len(rows)
    recoveries = [r for r in rows if r['fallback_step'] is not None]
    origins = Counter(r['kind'] for r in rows)
    sources = {
        (
            r['physical_source'].get('journal'),
            r['physical_source']['seed'],
            r['physical_source']['delay_s'],
            r['physical_source']['policy'],
            r['physical_source'].get('kind')
            or (
                'recovery'
                if r['fallback_step'] is not None
                else 'command' if r['application_step'] is not None else 'unapplied'
            ),
        )
        for r in rows
        if r['physical_source']
    }
    # Inspection stops at its first physical violation. A precomputed clock log
    # can describe a later planned cancellation that was never physically reached.
    explicit = sum(
        r['canceled'] is True and r['fallback_step'] is not None and r['application_step'] is None
        for r in rows
    )
    inferred = sum(
        r['canceled'] is None and r['fallback_step'] is not None and r['application_step'] is None
        for r in rows
    )
    return dict(
        case_count=n,
        unsafe=sum(not r['safe'] for r in rows),
        safe_completion=sum(r['safe_completion'] for r in rows),
        unsafe_rate=sum(not r['safe'] for r in rows) / n,
        safe_completion_rate=sum(r['safe_completion'] for r in rows) / n,
        recovery_attempts=len(recoveries),
        recovery_failures=sum(not r['safe'] for r in recoveries),
        early_recoveries=sum(
            r['fallback_step'] < r['approval_step'] + round(r['delay_s'] * 50) for r in recoveries
        ),
        command_cancellations=explicit + inferred,
        explicit_cancellations=explicit,
        inferred_cancellations=inferred,
        planned_cancellations_not_physically_reached=sum(
            r['canceled'] is True and r['fallback_step'] is None for r in rows
        ),
        new_physical_flights=origins['new_physical_flight'],
        source_kind_counts=dict(origins),
        unique_reused_physical_source_records=len(sources),
    )


def paired(rows, seeds, delays):
    indexed = {(r['seed'], round(r['delay_s'] * 50), r['policy']): r for r in rows}
    out = {}
    decisions = {}
    for other in (
        'staleguard_early_recovery',
        'staleguard_recovery_aware',
        'fixed_early_recovery',
        'use_time_recheck',
    ):
        if not all(
            (s, round(d * 50), p) in indexed
            for s in seeds
            for d in delays
            for p in ('staleguard_dual_horizon', other)
        ):
            out[other] = dict(status='MISSING_PAIRED_CELLS')
            continue
        counts = Counter()
        vectors = []
        action_scenarios = set()
        outcome_scenarios = set()
        for seed in seeds:
            uv = []
            cv = []
            for d in delays:
                a = indexed[seed, round(d * 50), 'staleguard_dual_horizon']
                b = indexed[seed, round(d * 50), other]
                counts['pairs'] += 1
                counts['unsafe_prevented'] += not b['safe'] and a['safe']
                counts['unsafe_introduced'] += b['safe'] and not a['safe']
                counts['safe_completions_preserved'] += (
                    a['safe_completion'] and b['safe_completion']
                )
                counts['safe_completions_sacrificed'] += (
                    b['safe_completion'] and not a['safe_completion']
                )
                counts['safe_completions_gained'] += (
                    a['safe_completion'] and not b['safe_completion']
                )
                change = (a['application_step'], a['fallback_step']) != (
                    b['application_step'],
                    b['fallback_step'],
                )
                counts['different_actions'] += change
                if change:
                    action_scenarios.add(seed)
                if (a['safe'], a['safe_completion']) != (b['safe'], b['safe_completion']):
                    outcome_scenarios.add(seed)
                uv.append(int(not b['safe']) - int(not a['safe']))
                cv.append(int(a['safe_completion']) - int(b['safe_completion']))
            vectors.append([np.mean(uv), np.mean(cv)])
        v = np.asarray(vectors)
        rng = np.random.default_rng(2027)
        samples = v[rng.integers(0, len(seeds), (2000, len(seeds)))].mean(axis=1)
        out[other] = dict(
            status='COMPLETE',
            **dict(counts),
            scenarios=len(seeds),
            different_action_scenarios=len(action_scenarios),
            different_outcome_scenarios=len(outcome_scenarios),
            net_unsafe_rate_reduction=float(v[:, 0].mean()),
            net_safe_completion_change=float(v[:, 1].mean()),
            unsafe_reduction_ci95=np.quantile(samples[:, 0], [0.025, 0.975]).tolist(),
            completion_change_ci95=np.quantile(samples[:, 1], [0.025, 0.975]).tolist(),
            bootstrap=dict(unit='whole scenario', resamples=2000, seed=2027),
        )
    return out
