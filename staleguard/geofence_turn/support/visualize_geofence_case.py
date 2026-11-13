"""Replay one frozen Geofence Turn case and plot actual PyBullet flight traces.

Run with --show for a rotatable Matplotlib scene. Paths are unfiltered 50 Hz
physical positions; clearance/outcomes/event times come from the 1 kHz monitor.
No approval forecast is recomputed, and no 3D violation point is interpolated.
"""
from pathlib import Path
import sys


from staleguard import geofence_turn # Set numerical threads and Matplotlib cache before imports.
import argparse
import fcntl
import hashlib
import json
import numpy as np

from staleguard.geofence_turn.scenario import Config, HZ, PHYSICS_HZ, POS, clearance, scenario_dict
from staleguard.geofence_turn.simulation import execute
from staleguard.geofence_turn.storage import (ROOT, load_calibrated_scenarios, file_hash,
                                   metadata, no_recovery_hash, check_approval_snapshot)
from staleguard.geofence_turn.baselines.staleguard import LightweightPredictiveModel
from staleguard.model.scg import load_scg_platform_params

POLICIES = ('no_recovery', 'use_time_recheck', 'staleguard_early_recovery')
COLORS = ('#59636e', '#cb6224', '#16805c')


def archived_case(journal, scenario, snapshot_path, delay, policies, config):
    """Validate scientific inputs and journal integrity before using saved data."""
    journal = journal / 'executions.jsonl' if journal.is_dir() else journal
    saved = json.loads(journal.with_name('run.json').read_text())
    provenance, current = saved['provenance'], metadata(config)
    for field in ('schema','config','numpy','numerical_threads','cpu_only','simulation_workers'):
        if provenance[field] != current[field]:
            raise ValueError(f'Archived numerical environment/config differs: {field}')
    if provenance['python'].split()[0].split('.')[:2] != current['python'].split()[0].split('.')[:2]:
        raise ValueError('Python major/minor version differs')
    for field in ('python','platform'):
        if provenance[field] != current[field]:
            print(f'Archive {journal.parent.name}: {field} differs; checking actual replay, not assuming equivalence.\n'
                  f'  archived: {provenance[field]}\n  current:  {current[field]}')
    for name in ('__init__.py','scenario.py','baselines/staleguard.py','baselines/use_time_recheck.py'):
        if provenance['source_hashes'][name] != current['source_hashes'][name]:
            raise ValueError(f'Archived mission/predictor source differs: {name}')
    for name, checksum in {**provenance['shared_source_hashes'], **provenance['environment_source_hashes']}.items():
        # Archived EarlyRecovery was run in another checkout; retain the exact
        # shared-source content checks while resolving its local package path.
        path = Path(name)
        if path.is_absolute() and 'inspection_descent' in path.parts:
            path = Path(*path.parts[path.parts.index('inspection_descent'):])
        if file_hash(ROOT.parent/path) != checksum:
            raise ValueError(f'Archived physics/controller source differs: {name}')
    # The existing No-Recovery projection prunes other policies, but retains
    # this newly added, unused policy import. Remove ONLY that import in the
    # comparison text; require exact equality with the historical physical AST.
    # Neither scientific source nor historical provenance is edited.
    source = (ROOT/'simulation.py').read_text().replace('from .baselines import fixed_early_recovery\n','')
    if no_recovery_hash(source) != provenance['no_recovery_sha256']:
        raise ValueError('Archived No-Recovery physics/monitor AST differs')
    protocol = saved['protocol']
    expected = json.loads(json.dumps(scenario_dict(scenario)))
    if (protocol['control_hz'] != HZ
            or expected not in protocol['scenarios']
            or delay not in protocol['delays']
            or protocol['snapshot_sha256'][str(scenario.seed)] != file_hash(snapshot_path)):
        raise ValueError(f'Archived mission/snapshot/protocol differs: {journal}')
    raw = journal.read_bytes()
    if (len(raw) != saved['journal_bytes']
            or hashlib.sha256(raw).hexdigest() != saved['journal_sha256']):
        raise ValueError(f'Archived journal checksum differs: {journal}')
    rows = {}
    for line in raw.splitlines():
        row = json.loads(line)
        policy = row['event']['policy']
        if row['seed'] == scenario.seed and row['delay_s'] == delay and policy in policies:
            if policy in rows:
                raise ValueError(f'Duplicate archived flight: {policy}')
            rows[policy] = row
    if set(rows) != set(policies):
        raise ValueError(f'Missing archived flights: {set(policies)-set(rows)}')
    return saved, rows


def replay(args):
    config = Config()
    case = next((c for c in load_calibrated_scenarios(args.calibration, config)
                 if c[0].seed == args.seed), None)
    if case is None:
        raise ValueError(f'Seed {args.seed} is absent from calibration')
    scenario, snapshot, snapshot_path = case
    _, archived = archived_case(args.baseline_results, scenario, snapshot_path,
                               args.delay, POLICIES[:2], config)
    saved, early = archived_case(args.early_results, scenario, snapshot_path,
                                args.delay, POLICIES[2:], config)
    archived.update(early)
    prediction = saved['predictions'][str(args.seed)]
    for field in ('expiry_steps', 'computation_s'):
        if prediction[field] != early[POLICIES[2]]['event'][field]:
            raise ValueError(f'Frozen forecast differs from archived event: {field}')

    platform = load_scg_platform_params()
    # Warm the shared Numba kernel with ONE dynamics step, not a new TTL forecast.
    model = LightweightPredictiveModel(*platform)
    try:
        ctrl, obs, info, _ = model.start(snapshot)
        model.step(ctrl.select_action(obs, info), obs, info)
    finally:
        model.close({})

    flights = {}
    print('\nActual PyBullet replay; 50 Hz paths / 1000 Hz safety monitor')
    print('Policy                         safe complete  clearance(m)  first violation(s)  action step  archive')
    for policy in POLICIES:
        actual, approval, trace = execute(scenario, args.delay, config=config,
            policy=policy, received=prediction if policy == POLICIES[2] else None,
            platform=platform, save_trace=True)
        check_approval_snapshot(approval, snapshot)
        assert trace.ndim == 2 and trace.shape[1] == 25  # step + 12 actual + 12 reference
        assert np.array_equal(trace[:, 0], np.arange(len(trace)))
        assert actual['physics_samples'] == (len(trace)-1)*(PHYSICS_HZ//HZ)+1
        old = archived[policy]
        action = lambda r: (r['event']['application_step'], r['event']['fallback_step'])
        same = (all(actual[k] == old[k] for k in
                    ('safe', 'completed', 'safe_completion', 'completion_s', 'first_violation_s',
                     'pre_dispatch_violation_s', 'physics_samples', 'horizon_s'))
                and action(actual) == action(old)
                and actual['event']['scheduled_dispatch_step'] == old['event']['scheduled_dispatch_step']
                and np.isclose(actual['min_clearance_m'], old['min_clearance_m'], rtol=0, atol=1e-10))
        if not same and policy != 'use_time_recheck':
            raise ValueError(f'Deterministic replay differs from archive: {policy}')
        status = 'MATCH' if same else 'DIFFERS (shown as replayed)'
        step = next((s for s in action(actual) if s is not None), None)
        print(f'{policy:30s} {str(actual["safe"]):5s} {str(actual["completed"]):8s} '
              f'{actual["min_clearance_m"]:12.9f} {str(actual["first_violation_s"]):>19s} {str(step):>12s}  {status}')
        if policy == 'use_time_recheck':
            now, before = actual['event']['computation_s'], old['event']['computation_s']
            print(f'  Recheck computation: replay {1000*now:.3f} ms; archived {1000*before:.3f} ms; '
                  f'difference {1000*(now-before):+.3f} ms. Recovery steps: {action(actual)[1]} / {action(old)[1]}.')
            if not same:
                print('  Archived outcome:', {k: old[k] for k in ('safe', 'completed', 'min_clearance_m', 'first_violation_s')})
        flights[policy] = dict(result=actual, xyz=trace[:, 1+np.array(POS)], time=trace[:, 0]/HZ)
    approval_step = round(scenario.approval_s*HZ)
    for flight in flights.values():
        np.testing.assert_array_equal(flight['xyz'][:approval_step+1], flights[POLICIES[0]]['xyz'][:approval_step+1])
    safety_x = float(clearance(0., scenario.fence_x))-config.d_min_m
    print(f'Frozen seed={scenario.seed}, radius={scenario.command.radius:g} m, speed={scenario.speed:g} m/s; '
          f'fence X={scenario.fence_x:.9f} m; UAV-center safety limit X={safety_x:.9f} m.')
    print('All approval states/PID memory/remaining references match the frozen snapshot exactly.\n')
    return scenario, config, prediction, flights


def make_figure(scenario, config, flights):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from mpl_toolkits.mplot3d import proj3d

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8,
        'axes.labelsize': 8, 'xtick.labelsize': 7, 'ytick.labelsize': 7,
        'pdf.fonttype': 42, 'svg.fonttype': 'none', 'axes.linewidth': .65,
        'savefig.facecolor': 'white'})
    fig = plt.figure(figsize=(7.16, 5.65), facecolor='white')
    ax = fig.add_axes([.015, .065, .97, .92], projection='3d', computed_zorder=False)

    xyz = np.concatenate([f['xyz'] for f in flights.values()])
    lower, upper = np.array(scenario.command.checkpoint_lower), np.array(scenario.command.checkpoint_upper)
    lo = np.minimum(xyz.min(axis=0), lower)
    hi = np.maximum(xyz.max(axis=0), upper)
    bounds = [(lo[0]-.15, max(hi[0], scenario.fence_x)+.20),
              (lo[1]-.23, hi[1]+.25), (0., hi[2]+.38)]
    fence = scenario.fence_x
    safety = float(clearance(0., fence))-config.d_min_m  # fence - body radius - d_min

    def plane(x, color, alpha, dashed=False):
        y0, y1 = bounds[1]; z0, z1 = bounds[2]
        vertices = [(x,y0,z0),(x,y1,z0),(x,y1,z1),(x,y0,z1)]
        ax.add_collection3d(Poly3DCollection([vertices], facecolor=color, alpha=alpha,
                            edgecolor='none', zorder=1))
        q = np.array(vertices+[vertices[0]])
        ax.plot(*q.T, color=color, lw=.85, ls='--' if dashed else '-', alpha=.8, zorder=2)

    plane(fence, '#b8454b', .11)
    plane(safety, '#a44b58', .025, dashed=True)
    corners = np.array([[x,y,z] for x in (lower[0],upper[0])
                        for y in (lower[1],upper[1]) for z in (lower[2],upper[2])])
    faces = [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]
    ax.add_collection3d(Poly3DCollection([corners[f] for f in faces],
                        facecolor='#559fc8', edgecolor='#3e82a9', alpha=.12, linewidths=.65, zorder=2))
    approval = round(scenario.approval_s*HZ)
    for policy, color in zip(POLICIES, COLORS):
        points = flights[policy]['xyz']
        ax.plot(*points[approval:].T, color=color, lw=1.9, zorder=4,
                ls='-' if policy != 'no_recovery' else (0,(5,1.8)))
    common = flights[POLICIES[0]]['xyz'][:approval+1]
    ax.plot(*common.T, color='#3f4852', lw=1.65, zorder=4)
    ax.set(xlim=bounds[0], ylim=bounds[1], zlim=bounds[2],
           xlabel='East, X (m)', ylabel='North, Y (m)', zlabel='Altitude, Z (m)')
    ax.set_box_aspect([b-a for a,b in bounds], zoom=1.02)
    ax.view_init(elev=25, azim=-61)
    ax.tick_params(pad=0, length=2)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis.pane.set_edgecolor('#e6e9eb')
        axis._axinfo['grid'].update(color='#e5e8eb', linewidth=.45)
    ax.set_zticks([0., .4, .8, 1.2, 1.6])
    fig.canvas.draw()

    def callout(point, text, location, color, marker=None):
        if marker:
            ax.scatter(*point, s=32, marker=marker, color=color, edgecolor='white', linewidth=.65, depthshade=False, zorder=6)
        x, y, _ = proj3d.proj_transform(*point, ax.get_proj())
        ax.annotate(text, xy=(x,y), xycoords='data', xytext=location, textcoords='axes fraction',
                    color=color, fontsize=7.5, ha='left', va='center',
                    bbox=dict(boxstyle='round,pad=.22', facecolor='white', alpha=.94, edgecolor='none'),
                    arrowprops=dict(arrowstyle='-', color=color, lw=.7, shrinkA=3, shrinkB=3), zorder=8)

    callout(common[-1], f'Approval\n{scenario.approval_s:.2f} s', (.08,.47), '#38434e', 'o')
    early = flights[POLICIES[2]]['result']['event']['fallback_step']
    late = flights[POLICIES[1]]['result']['event']['fallback_step']
    if early is not None:
        callout(flights[POLICIES[2]]['xyz'][early], f'Early braking\n{early/HZ:.2f} s', (.12,.68), COLORS[2], 'D')
    if late is not None:
        callout(flights[POLICIES[1]]['xyz'][late], f'Late braking\n{late/HZ:.2f} s', (.68,.42), COLORS[1], 's')
    dispatch = flights['no_recovery']['result']['event']['scheduled_dispatch_step']
    # These two policies share the same physical state before dispatch. Early
    # recovery has already diverged; this is not a dispatch marker on its path.
    assert dispatch == flights['use_time_recheck']['result']['event']['scheduled_dispatch_step']
    np.testing.assert_array_equal(flights['no_recovery']['xyz'][dispatch],
                                  flights['use_time_recheck']['xyz'][dispatch])
    ax.plot(*flights['no_recovery']['xyz'][dispatch], marker='|', ms=9, mew=1.1,
            color='#38434e', linestyle='None', zorder=7)
    callout(flights['no_recovery']['xyz'][dispatch],
            f'Scheduled dispatch\n{dispatch/HZ:.2f} s', (.53,.28), '#38434e')
    callout((fence,bounds[1][1]*.88,bounds[2][1]), 'Geofence', (.76,.87), '#9c363f')
    callout((safety,bounds[1][0],bounds[2][1]), '5 cm body-clearance limit', (.08,.88), '#994452')
    callout((lower[0],upper[1],upper[2]), 'Target objective', (.38,.77), '#327b9e')
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=22)
    parser.add_argument('--delay', type=float, default=.50)
    parser.add_argument('--calibration', type=Path, default=ROOT/'results/evaluation/calibration')
    parser.add_argument('--baseline-results', type=Path, default=ROOT/'results/evaluation/baseline_benchmark')
    parser.add_argument('--early-results', type=Path, default=ROOT/'results/evaluation/early_recovery_benchmark')
    parser.add_argument('--out', type=Path, default=ROOT/'support/results/geofence_figure')
    parser.add_argument('--show', action='store_true', help='Open the rotatable 3D scene after saving')
    args = parser.parse_args()
    if not np.isfinite(args.delay) or args.delay < 0 or not np.isclose(args.delay*HZ,round(args.delay*HZ)):
        parser.error('--delay must be a nonnegative 50 Hz control-grid delay')
    import matplotlib
    matplotlib.use('TkAgg' if args.show else 'Agg')
    with (ROOT/'support/logs/.simulation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        scenario, config, _, flights = replay(args)
    figure = make_figure(scenario, config, flights)
    args.out.mkdir(parents=True, exist_ok=True)
    stem = f'geofence_seed_{args.seed}_delay_{args.delay:.2f}_scheduled_dispatch'
    for suffix in ('png','pdf'):
        path = args.out/f'{stem}.{suffix}'
        # mplot3d may omit an outward-facing axis label from its tight bounds.
        figure.savefig(path, dpi=400, bbox_inches='tight', pad_inches=.07,
                       bbox_extra_artists=[axis.label for axis in
                           (figure.axes[0].xaxis, figure.axes[0].yaxis, figure.axes[0].zaxis)])
        print(f'Saved {path}')
    if args.show:
        import matplotlib.pyplot as plt
        from matplotlib.text import Annotation
        # Projection-dependent event callouts are for the paper view only; hide
        # them when rotating so they cannot point to stale screen coordinates.
        def rotate(event):
            if event.inaxes is figure.axes[0]:
                for artist in figure.axes[0].texts:
                    if isinstance(artist, Annotation):
                        artist.set_visible(False)
                figure.canvas.draw_idle()
        figure.canvas.mpl_connect('button_press_event', rotate)
        plt.show()


if __name__ == '__main__':
    main()
