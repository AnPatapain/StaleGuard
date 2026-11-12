"""Export the archived Underpass pilot (seed 0) as vector PDF + editable SVG.

Save this file to underpass_inspection/support/paper_figure3.py.
It reads validated physical trajectories; it does not run or alter the simulator.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from underpass_inspection.scenario import Scenario, SCENARIO_VERSION

DEFAULT_RUN = ROOT / f'results/development/pilot_{SCENARIO_VERSION}'
COLORS = {
    'nominal': '#5C636B',
    'safe_inspection': '#269A38',
    'late_inspection': '#DA2A2A',
    'safe_recovery': '#F17E12',
}
CASES = (
    ('nominal', 'nominal', None, 'Nominal waiting'),
    ('safe_inspection', 'dive', 0.0, 'Safe inspection ($D=0$)'),
    ('late_inspection', 'dive', 0.4, 'Late inspection ($D=0.4$ s; unsafe)'),
    ('safe_recovery', 'sideways', 0.5, 'Safe sideways recovery (+0.5 s)'),
)


def load_flights(folder):
    """Read only the frozen pilot's checksum-verified PyBullet trajectories."""
    summary = json.loads((folder / 'summary.json').read_text())
    scenario = Scenario(**summary['scenario'])
    rows = [json.loads(line) for line in (folder / 'flights.jsonl').read_text().splitlines()]
    flights = {}
    with np.load(folder / 'trajectories.npz') as archive:
        for key, mode, start, label in CASES:
            row = next((r for r in rows if r['mode'] == mode and
                        (r['start_s'] is None if start is None else
                         r['start_s'] is not None and abs(r['start_s'] - start) < 1e-9)), None)
            if row is None:
                raise ValueError(f'Missing archived flight: {mode}, start={start}')
            physical = archive[row['trajectory_key'] + '__physics']
            if hashlib.sha256(physical.tobytes()).hexdigest() != row['physics_sha256']:
                raise ValueError(f'Trajectory checksum mismatch: {key}')
            flights[key] = (physical, row, label)

    if scenario.seed != 0:
        raise ValueError(f'This figure is specified for pilot seed 0, found {scenario.seed}')
    if not flights['safe_inspection'][1]['safe_completion']:
        raise ValueError('The archived immediate inspection must be a safe completion')
    if flights['late_inspection'][1]['safe']:
        raise ValueError('The archived delayed inspection must be unsafe')
    if not flights['safe_recovery'][1]['safe']:
        raise ValueError('The archived sideways recovery must be safe')
    return scenario, flights


def faces_of(box):
    """Six exact polygonal faces of an axis-aligned scenario box."""
    x0, y0, z0 = box.lower
    x1, y1, z1 = box.upper
    a, b, c, d = (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)
    e, f, g, h = (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)
    return [[a,b,c,d], [e,f,g,h], [a,b,f,e], [b,c,g,f], [c,d,h,g], [d,a,e,h]]


def camera_projection(elev, azim):
    """Orthographic 3D-to-2D projection; North/East/Down remain physical XYZ."""
    theta, phi = np.deg2rad([azim, elev])
    right = np.array([-np.sin(theta), np.cos(theta), 0.])
    up = np.array([-np.sin(phi)*np.cos(theta), -np.sin(phi)*np.sin(theta),
                   np.cos(phi)])
    eye = np.array([np.cos(phi)*np.cos(theta),
                    np.cos(phi)*np.sin(theta), np.sin(phi)])

    def project(points):
        points = np.asarray(points)
        return np.column_stack((points @ right, points @ up))

    return project, eye


def draw_box(ax, box, facecolor, edgecolor, alpha, project, eye):
    """Exact 3D box surfaces as editable, depth-sorted vector polygons."""
    faces = faces_of(box)
    ordered = sorted(faces, key=lambda face: np.mean(np.asarray(face) @ eye))
    for face in ordered:
        ax.add_patch(Polygon(project(face), closed=True, facecolor=facecolor,
                             edgecolor=edgecolor, lw=.75, alpha=alpha, zorder=2))


def draw_ned_inset(fig, project):
    """Vector compass aligned with the exact 3D projection."""
    inset = fig.add_axes([.045, .70, .25, .17])
    inset.set_xlim(0, 1)
    inset.set_ylim(0, 1)
    inset.axis('off')
    inset.text(.02, .99, 'NED frame', va='top', fontsize=10, color='#343C45')
    axes = [
        ('N (+X)', (1, 0, 0), '#B64C32'),
        ('E (-Y)', (0, -1, 0), '#268749'),
        ('D (-Z)', (0, 0, -1), '#3462AE'),
    ]
    vectors = [project([direction])[0] for _, direction, _ in axes]
    maximum = max(np.linalg.norm(v) for v in vectors)
    base = np.array([.19, .46])
    for (label, _, color), vector in zip(axes, vectors):
        end = base + vector * (.39 / maximum)
        inset.annotate('', xy=end, xytext=base,
                       arrowprops=dict(arrowstyle='-|>', color=color, lw=1.6,
                                       shrinkA=0, shrinkB=0, mutation_scale=10))
        inset.text(end[0] + .025, end[1], label, fontsize=8.4, color=color,
                   ha='left', va='center')


def make_figure(scenario, flights, at_after_approval=1.0, elev=22.0, azim=-125.0):
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 10,
        'pdf.fonttype': 42,
        'svg.fonttype': 'none',  # Keep text editable in Inkscape.
        'savefig.facecolor': 'white',
    })
    project, eye = camera_projection(elev, azim)
    fig = plt.figure(figsize=(9.1, 4.9), facecolor='white')
    ax = fig.add_axes([.035, .15, .93, .73])
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')

    # Quiet physical floor grid, generated as vectors rather than a bitmap.
    floor_points = []
    for x in np.arange(0, 5.8, .5):
        xy = project([[x, -1.5, 0], [x, .8, 0]])
        ax.plot(xy[:, 0], xy[:, 1], color='#E5EAF0', lw=.48, zorder=0)
        floor_points.extend(xy.tolist())
    for y in np.arange(-1.5, .81, .5):
        xy = project([[0, y, 0], [5.5, y, 0]])
        ax.plot(xy[:, 0], xy[:, 1], color='#E5EAF0', lw=.48, zorder=0)
        floor_points.extend(xy.tolist())

    deck, pier = scenario.obstacles
    draw_box(ax, deck, '#D3DBE3', '#8493A0', .29, project, eye)
    draw_box(ax, pier, '#A99889', '#75655B', .73, project, eye)
    draw_box(ax, scenario.inspection, '#7ECCAA', '#25836C', .22, project, eye)

    projected = []
    for key, _, _, _ in CASES:
        full, row, _ = flights[key]
        # Original 1 kHz physical positions displayed at 50 Hz; no smoothing.
        xy = project(full[::20, 1:4])
        projected.append(xy)
        ax.plot(xy[:, 0], xy[:, 1],
                color=COLORS[key], lw=2.2 if key != 'nominal' else 1.35,
                ls='--' if key == 'nominal' else '-', zorder=5)

    # Position marker, not an enlarged representation of the physical drone.
    full = flights['safe_inspection'][0]
    instant = scenario.approval_s + at_after_approval
    if not full[0, 0] <= instant <= full[-1, 0]:
        raise ValueError('--at is outside the recorded flight time')
    index = np.argmin(np.abs(full[:, 0] - instant))
    xy = project(full[index:index+1, 1:4])[0]
    ax.scatter(*xy, s=42, color='#202A35', edgecolors='white',
               linewidths=.8, zorder=8)

    # Frame the true 3D content, not an arbitrary padded 3D viewport.
    box_xyz = np.asarray([pt for box in (deck, pier, scenario.inspection)
                          for face in faces_of(box) for pt in face])
    visible = np.vstack(projected + [project(box_xyz), np.asarray(floor_points)])
    mins, maxs = visible.min(axis=0), visible.max(axis=0)
    span = maxs - mins
    ax.set_xlim(mins[0] - .045*span[0], maxs[0] + .045*span[0])
    ax.set_ylim(mins[1] - .14*span[1], maxs[1] + .14*span[1])

    fig.text(.055, .963, 'UNDERPASS INSPECTION', fontsize=13.5,
             weight='bold', color='#202A35', va='top')
    fig.text(.055, .923, 'Actual PyBullet trajectories  |  frozen scenario seed 0',
             fontsize=9, color='#53606E', va='top')

    handles = [Line2D([0], [0], color=COLORS[key],
                      ls='--' if key == 'nominal' else '-', lw=2.4, label=label)
               for key, _, _, label in CASES]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .018),
               ncol=2, frameon=False, columnspacing=2.5,
               handlelength=2.8, fontsize=9)
    draw_ned_inset(fig, project)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=DEFAULT_RUN,
                        help='Directory with frozen pilot summary.json and trajectories.npz')
    parser.add_argument('--out', type=Path, default=Path('underpass_fig3_vector'),
                        help='Output filename without extension')
    parser.add_argument('--at', type=float, default=1.0,
                        help='Mark aircraft position this many seconds after approval')
    parser.add_argument('--elev', type=float, default=22.0)
    parser.add_argument('--azim', type=float, default=-125.0)
    args = parser.parse_args()
    scenario, flights = load_flights(args.run)
    fig = make_figure(scenario, flights, args.at, args.elev, args.azim)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    for extension in ('pdf', 'svg'):
        destination = args.out.with_suffix('.' + extension)
        fig.savefig(destination, format=extension, bbox_inches='tight', pad_inches=.07)
        print('Saved:', destination)
    plt.close(fig)
    print('Scenario seed:', scenario.seed, '(original frozen pilot; not v2 evaluation)')


if __name__ == '__main__':
    main()