"""Figure 2: actual recorded frozen Underpass trajectories, not Geofence flights."""

from .common import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def main():
    args, data = arguments(__doc__)
    name = 'underpass_inspection/representative_flights'
    entry = data.entry(name)
    summary = data.read_json(name, 'summary.json')
    rows = data.records(name, 'flights.jsonl')
    scenario = summary['scenario']
    selected = {}
    events = {}
    export = []
    styles = {
        'nominal': ('nominal', None, '#62676c', '--'),
        'safe_inspection': ('dive', 0.0, '#168c40', '-'),
        'late_inspection': ('dive', 0.4, '#d82727', '-'),
        'safe_recovery': ('sideways', 0.5, '#ff8800', '-'),
    }
    archive = np.load(data.file(entry['files']['trajectories.npz']))
    for label, (mode, start, color, linestyle) in styles.items():
        matches = [r for r in rows if r['mode'] == mode and r['start_s'] == start]
        if len(matches) != 1:
            raise ValueError('Missing/duplicate representative flight: ' + label)
        row = matches[0]
        key = row['trajectory_key']
        physics = archive[key + '__physics']
        digest = hashlib.sha256(physics.tobytes()).hexdigest()
        if digest != row['physics_sha256']:
            raise ValueError('Representative trajectory hash mismatch')
        for kind in ('physics', 'control', 'reference'):
            selected[label + '__' + kind] = archive[key + '__' + kind]
        events[label] = dict(
            source='archived actual PyBullet flight; no new replay',
            seed=scenario['seed'],
            mode=mode,
            start_approval_s=start,
            approval_step=row['approval_step'],
            initiation_step=row['initiation_step'],
            safe=row['safe'],
            safe_completion=row['safe_completion'],
            first_violation=row['first_violation'],
            physics_sha256=digest,
        )
        export.extend(
            dict(
                trajectory=label,
                flight_s=float(v[0]),
                approval_s=float(v[0] - scenario['approval_s']),
                x_m=float(v[1]),
                y_m=float(v[2]),
                z_m=float(v[3]),
            )
            for v in physics
        )
    np.savez_compressed(args.out / 'trajectories.npz', **selected)
    write_csv(args.out / 'trajectories.csv', export)
    write_json(args.out / 'scenario.json', scenario)
    write_json(args.out / 'events.json', events)
    plt = plot_style()
    fig = plt.figure(figsize=(8.5, 4.8))
    ax = fig.add_subplot(111, projection='3d')
    for box in summary['obstacle_boxes']:
        lo = np.array(box['lower'])
        hi = np.array(box['upper'])
        faces = []
        for axis in range(3):
            other = [i for i in range(3) if i != axis]
            for side in (0, 1):
                points = []
                for a, b in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    v = lo.copy()
                    v[axis] = (lo, hi)[side][axis]
                    v[other[0]] = (lo, hi)[a][other[0]]
                    v[other[1]] = (lo, hi)[b][other[1]]
                    points.append(v)
                faces.append(points)
        ax.add_collection3d(
            Poly3DCollection(
                faces,
                facecolor='#bec5cb',
                edgecolor='#80878c',
                linewidth=0.5,
                alpha=0.28 if box['name'] == 'deck' else 0.7,
            )
        )
    labels = [
        'Nominal',
        'Safe inspection (D=0)',
        'Late inspection (D=0.4 s)',
        'Safe recovery (+0.5 s)',
    ]
    for (label, (_, _, color, linestyle)), legend in zip(styles.items(), labels):
        points = selected[label + '__physics'][::10]
        ax.plot(
            points[:, 1], points[:, 2], points[:, 3], linestyle, color=color, lw=1, label=legend
        )
    ax.set(xlabel='North (m)', ylabel='World Y (m)', zlabel='Altitude (m)')
    ax.view_init(elev=20, azim=-55)
    ax.set_box_aspect((3, 1.5, 1.3), zoom=0.85)
    ax.tick_params(labelsize=7)
    ax.xaxis.label.set_size(8)
    ax.yaxis.label.set_size(8)
    ax.zaxis.label.set_size(8)
    ax.set_yticks([-1.0, -0.5, 0.0, 0.5])
    ax.set_zticks([0.0, 0.5, 1.0, 1.5])
    ax.set_zlabel('')
    ax.text2D(0.84, 0.86, 'Altitude (m)', transform=ax.transAxes, fontsize=8)
    fig.legend(
        *ax.get_legend_handles_labels(), loc='lower center', ncol=2, frameon=False, fontsize=8
    )
    fig.subplots_adjust(bottom=0.17, right=0.9)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((0.98, 0.98, 0.98, 1))
        axis._axinfo['grid']['color'] = (0.85, 0.85, 0.85, 0.4)
    save_figure(fig, args.out, 'fig2')
    metadata(
        data,
        args.out,
        'Figure 2',
        scenario_seed=scenario['seed'],
        trajectories=events,
        newly_replayed=0,
        coordinate_system='original Z-up world coordinates; NED East = -world Y',
    )


if __name__ == '__main__':
    main()
