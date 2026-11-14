"""Actual PyBullet flight visualization. Display objects never affect collision or control."""

from pathlib import Path
from itertools import product
from dataclasses import asdict
import argparse, json, hashlib, os, time, re, shutil, subprocess
import numpy as np
import pybullet as p
from staleguard.underpass_inspection.scenario import Box, POS, BODY_RADIUS_M, WORLD_TO_NED

COLORS = {
    'nominal': (0.36, 0.39, 0.42),
    'dive': (0.10, 0.60, 0.22),
    'climb': (0.12, 0.32, 0.87),
    'sideways': (0.0, 0.65, 0.75),
}
PAPER_COLORS = {
    'nominal': (0.36, 0.39, 0.42),
    'safe_inspection': (0.10, 0.60, 0.22),
    'late_inspection': (0.86, 0.16, 0.13),
    'safe_recovery': (1.0, 0.50, 0.05),
}
DEFAULT_CAMERA = dict(distance=3.4, yaw=-35.0, pitch=-20.0, target=[2.2, 0.25, 1.0])
PAPER_CAMERA = dict(distance=2.7, yaw=-35.0, pitch=-20.0, target=[1.85, 0.1, 1.0])


def paper_mode():
    return os.environ.get('STALEGUARD_PAPER_FIGURE') == '1'


def polyline(points, client, color, width=1.0, dashed=False):
    """Join actual samples directly. Dashed lines omit segments, never move samples."""
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        if not dashed or i % 6 < 3:
            p.addUserDebugLine(a, b, color, width, physicsClientId=client)


def wire_box(box, client, color, inflation=0.0):
    lo, hi = np.asarray(box.lower) - inflation, np.asarray(box.upper) + inflation
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        for sides in product((0, 1), repeat=2):
            a, b = lo.copy(), lo.copy()
            for i, side in zip(other, sides):
                a[i] = b[i] = (lo, hi)[side][i]
            b[axis] = hi[axis]
            p.addUserDebugLine(a, b, color, 0.6, physicsClientId=client)


def visual_box(box, color, client):
    lo, hi = np.asarray(box.lower), np.asarray(box.upper)
    shape = p.createVisualShape(
        p.GEOM_BOX, halfExtents=(hi - lo) / 2, rgbaColor=color, physicsClientId=client
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=shape,
        basePosition=(lo + hi) / 2,
        physicsClientId=client,
    )


def deck_faces(box, client):
    """Exact existing face coordinates, separate translucent materials; no contacts."""
    lo, hi = np.asarray(box.lower), np.asarray(box.upper)
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        for side in (0, 1):
            vertices = []
            for a, b in ((0, 0), (1, 0), (1, 1), (0, 1)):
                vertex = lo.copy()
                vertex[axis] = (lo, hi)[side][axis]
                vertex[other[0]] = (lo, hi)[a][other[0]]
                vertex[other[1]] = (lo, hi)[b][other[1]]
                vertices.append(vertex.tolist())
            color = (
                (0.70, 0.74, 0.77, 0.82)
                if axis == 0 and side == 0
                else (
                    (0.64, 0.69, 0.72, 0.68)
                    if axis == 2 and side == 0
                    else (0.91, 0.93, 0.94, 0.88) if axis == 2 else (0.79, 0.82, 0.84, 0.80)
                )
            )
            shape = p.createVisualShape(
                p.GEOM_MESH,
                vertices=vertices,
                indices=[0, 1, 2, 0, 2, 3, 2, 1, 0, 3, 2, 0],
                rgbaColor=color,
                physicsClientId=client,
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=shape,
                physicsClientId=client,
            )


def draw_reference(reference, client, mode):
    if paper_mode():
        return
    color = tuple(0.62 + 0.38 * c for c in COLORS[mode])
    polyline(reference[:, POS], client, color, 0.8, dashed=mode == 'nominal')


def draw_ground(plane, client, center):
    """Restyle the existing ground's visual material and add a faint local grid."""
    if plane is None:
        return
    ground = np.full(3, 250 / 255)  # #FAFAFA
    grid = np.full(3, 218 / 255)  # #DADADA
    # A light translucent material blends the shaded floor into the white
    # background without a contrasting horizon; its collision is untouched.
    p.changeVisualShape(
        plane,
        -1,
        textureUniqueId=-1,
        rgbaColor=[*ground, 0.06],
        specularColor=[0.0, 0.0, 0.0],
        physicsClientId=client,
    )
    height = p.getBasePositionAndOrientation(plane, physicsClientId=client)[0][2] + 0.001
    cx, cy = np.floor(np.asarray(center) / 0.5) * 0.5
    extent = 10.0
    vertices, indices = [], []
    # Thin flat strips respect depth, unlike debug lines drawn over obstacles.
    for offset in np.arange(-extent, extent + 0.25, 0.5):
        for axis in (0, 1):
            strip = [
                [offset + dx, y, 0.0]
                for dx, y in (
                    (-0.003, -extent),
                    (0.003, -extent),
                    (0.003, extent),
                    (-0.003, extent),
                )
            ]
            if axis:
                strip = [[y, x, z] for x, y, z in strip]
            i = len(vertices)
            vertices.extend(strip)
            indices.extend((i, i + 1, i + 2, i, i + 2, i + 3, i + 2, i + 1, i, i + 3, i + 2, i))
    shape = p.createVisualShape(
        p.GEOM_MESH,
        vertices=vertices,
        indices=indices,
        rgbaColor=[*grid, 0.55],
        physicsClientId=client,
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=shape,
        basePosition=[cx, cy, height],
        physicsClientId=client,
    )


def draw_scene(s, client, plane=None):
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=client)
    p.configureDebugVisualizer(rgbBackground=[1.0, 1.0, 1.0], physicsClientId=client)
    camera = PAPER_CAMERA if paper_mode() else DEFAULT_CAMERA
    p.resetDebugVisualizerCamera(
        camera['distance'], camera['yaw'], camera['pitch'], camera['target'], physicsClientId=client
    )
    draw_ground(plane, client, camera['target'][:2])
    deck_faces(s.obstacles[0], client)
    # Presentation-only footing: no collision shape or mission safety predicate.
    pier = s.obstacles[1]
    ground_z = (
        p.getBasePositionAndOrientation(plane, physicsClientId=client)[0][2]
        if plane is not None
        else -0.05
    )
    base = Box(
        'visual_base',
        (pier.lower[0] - 0.18, pier.lower[1] - 0.22, ground_z),
        (pier.upper[0] + 0.18, pier.upper[1] + 0.22, pier.lower[2] + 0.04),
    )
    visual_box(base, (0.76, 0.80, 0.83, 1.0), client)
    wire_box(base, client, (0.47, 0.51, 0.55))
    visual_box(pier, (0.67, 0.69, 0.71, 1.0), client)
    visual_box(s.inspection, (0.3, 0.75, 0.42, 0.045), client)
    wire_box(s.obstacles[0], client, (0.47, 0.51, 0.55))
    wire_box(pier, client, (0.47, 0.51, 0.55))
    wire_box(s.inspection, client, (0.44, 0.70, 0.49))
    for box in s.obstacles:
        # Conservative guide; decisions still use exact sphere-to-box distance.
        wire_box(box, client, (0.79, 0.81, 0.83), BODY_RADIUS_M + s.clearance_m)


def paper_quadrotor(drone, client):
    """Larger display mesh only; the physical UAV and its collision stay intact."""
    visual = next(
        v
        for v in p.getVisualShapeData(drone, physicsClientId=client)
        if v[1] == -1 and v[2] == p.GEOM_MESH
    )
    shape = p.createVisualShape(
        p.GEOM_MESH,
        fileName=visual[4].decode(),
        meshScale=(1.6 * np.asarray(visual[3])).tolist(),
        visualFramePosition=visual[5],
        visualFrameOrientation=visual[6],
        rgbaColor=[0.12, 0.15, 0.18, 1.0],
        physicsClientId=client,
    )
    position, orientation = p.getBasePositionAndOrientation(drone, physicsClientId=client)
    display = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=shape,
        basePosition=position,
        baseOrientation=orientation,
        physicsClientId=client,
    )
    p.changeVisualShape(
        display, -1, textureUniqueId=-1, specularColor=[0.0, 0.0, 0.0], physicsClientId=client
    )
    p.changeVisualShape(drone, -1, rgbaColor=[1.0, 1.0, 1.0, 0.0], physicsClientId=client)
    return display


def recorded_paths(s, mode, start_s, paper=False):
    from staleguard.reproduce.common import Data

    data = Data(os.environ.get('STALEGUARD_DATA_ROOT', 'data'))
    name = 'underpass_inspection/representative_flights'
    summary = data.read_json(name, 'summary.json')
    if json.loads(json.dumps(asdict(s))) != summary['scenario']:
        raise ValueError('Recorded overlays require frozen representative seed 0')
    rows = data.records(name, 'flights.jsonl')
    result = {}
    with np.load(data.file(data.entry(name)['files']['trajectories.npz'])) as archive:
        for label, (mode, delay) in {
            'nominal': ('nominal', None),
            'safe_inspection': ('dive', 0.0),
            'late_inspection': ('dive', 0.4),
            'safe_recovery': ('sideways', 0.5),
        }.items():
            row = next(r for r in rows if r['mode'] == mode and r['start_s'] == delay)
            points = archive[row['trajectory_key'] + '__physics']
            if hashlib.sha256(points.tobytes()).hexdigest() != row['physics_sha256']:
                raise ValueError('Recorded trajectory changed')
            result[label] = dict(points=points[::20, 1:].copy(), start_s=delay)
    return result


class FlightView:
    def __init__(self, s, env, mode, start_s, overlay_all=False, overlay_paths=None):
        self.client, self.drone, self.mode, self.start_s = (
            env.PYB_CLIENT,
            env.DRONE_ID,
            mode,
            start_s,
        )
        self.paper = paper_mode()
        self.capture = os.environ.get('STALEGUARD_PAPER_CAPTURE')
        self.capture_at = float(os.environ.get('STALEGUARD_PAPER_AT', '0'))
        self.captured = False
        draw_scene(s, self.client, env.PLANE_ID)
        self.quad = paper_quadrotor(self.drone, self.client) if self.paper else None
        overlay = (
            overlay_paths
            if overlay_paths is not None
            else recorded_paths(s, mode, start_s, self.paper) if overlay_all else {}
        )
        for name, trace in overlay.items():
            polyline(
                trace['points'], self.client, PAPER_COLORS[name], 2.8, dashed=name == 'nominal'
            )

    def event(self, label, position, color):
        if not self.paper:
            p.addUserDebugText(label, position, color, textSize=0.8, physicsClientId=self.client)

    def approval(self, position):
        self.event('Approval', position, (0.3, 0.3, 0.3))

    def initiation(self, position, reference):
        self.event(self.mode + ' start', position, COLORS[self.mode])
        draw_reference(reference, self.client, self.mode)

    def refresh(self, *args):
        pass

    def update(self, a, b, step, active, unsafe, world_y, world_vy):
        if self.paper:
            position, orientation = p.getBasePositionAndOrientation(
                self.drone, physicsClientId=self.client
            )
            p.resetBasePositionAndOrientation(
                self.quad, position, orientation, physicsClientId=self.client
            )
        else:
            p.addUserDebugLine(
                a,
                b,
                (0.86, 0.16, 0.13) if unsafe else COLORS[self.mode if active else 'nominal'],
                2.5,
                physicsClientId=self.client,
            )
        if self.capture and not self.captured and (step + 1) / 50 >= self.capture_at:
            save_camera(self.capture, self.client)
            self.captured = True


def save_camera(path, client):
    time.sleep(0.15)
    if shutil.which('xwininfo') and shutil.which('import'):
        try:
            lines = subprocess.check_output(
                ['xwininfo', '-root', '-tree'], text=True, stderr=subprocess.DEVNULL
            ).splitlines()
            window = next(
                re.search(r'0x[0-9a-fA-F]+', line).group()
                for line in reversed(lines)
                if 'Bullet Physics ExampleBrowser' in line
            )
            subprocess.run(
                ['import', '-window', window, str(path)], check=True, stderr=subprocess.DEVNULL
            )
            return
        except (subprocess.SubprocessError, StopIteration, AttributeError):
            pass
    from PIL import Image

    camera = p.getDebugVisualizerCamera(physicsClientId=client)
    image = p.getCameraImage(
        1200,
        800,
        camera[2],
        camera[3],
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
        physicsClientId=client,
    )
    Image.fromarray(np.asarray(image[2], dtype=np.uint8).reshape(800, 1200, 4)).save(path)


def main():
    from staleguard.reproduce.common import Data, MISSIONS, POLICIES
    from staleguard.reproduce.run_experiments import load_cases

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', choices=MISSIONS, default='underpass_inspection')
    parser.add_argument('--data-root', type=Path, default=Path('data'))
    parser.add_argument('--seed', type=int)
    parser.add_argument('--mode', choices=tuple(COLORS), default='dive')
    parser.add_argument('--delay', type=float, default=0.0)
    parser.add_argument('--start', type=float, default=0.0)
    parser.add_argument('--policy', choices=POLICIES)
    parser.add_argument('--paper-figure', action='store_true')
    parser.add_argument('--overlay-all', action='store_true')
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--paper-at', type=float, default=1.8)
    parser.add_argument('--hold', type=float, default=2.0)
    parser.add_argument('--playback-speed', type=float, default=0.5)
    args = parser.parse_args()
    data = Data(args.data_root)
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        parser.error(
            'A desktop display is required; use reproduce.fig2 for headless recorded trajectories'
        )
    os.environ['STALEGUARD_DATA_ROOT'] = str(args.data_root.resolve())
    if args.mission == 'underpass_inspection':
        from staleguard.underpass_inspection.scenario import Scenario
        from staleguard.underpass_inspection.simulation import execute

        s = (
            Scenario()
            if args.seed in (None, 0)
            else next(s for seed, s, snap in load_cases(data, args.mission) if seed == args.seed)
        )
        if args.paper_figure:
            os.environ['STALEGUARD_PAPER_FIGURE'] = '1'
            os.environ['STALEGUARD_PAPER_AT'] = str(s.approval_s + args.paper_at)
            if args.capture:
                os.environ['STALEGUARD_PAPER_CAPTURE'] = str(args.capture.resolve())
        execute(
            s,
            args.mode,
            args.delay if args.mode == 'dive' else args.start,
            gui=True,
            playback_speed=args.playback_speed,
            hold_s=args.hold,
            capture=None if args.paper_figure else args.capture,
            overlay_all=args.overlay_all or args.paper_figure,
        )
    else:
        from importlib import import_module
        from staleguard.model.scg import load_scg_platform_params
        from staleguard.model.mission_evaluation import clock_action

        cases = load_cases(data, args.mission)
        seed, s, snap = next(r for r in cases if args.seed is None or r[0] == args.seed)
        policy = args.policy or 'no_recovery'
        received = (
            data.forecasts(args.mission, policy)[str(seed)]
            if policy in ('staleguard_recovery_aware', 'staleguard_dual_horizon')
            else data.run(args.mission + '/benchmark').get('predictions', {}).get(str(seed))
        )
        kwargs = dict(
            delay=args.delay,
            policy=policy,
            received=received,
            platform=load_scg_platform_params(),
            gui=True,
        )
        if (
            policy in ('staleguard_recovery_aware', 'staleguard_dual_horizon')
            or args.mission == 'inspection_descent'
            and policy == 'fixed_early_recovery'
        ):
            action, _ = clock_action(
                policy,
                snap,
                args.delay,
                received if policy != 'fixed_early_recovery' else None,
                horizon_steps=210 if args.mission == 'inspection_descent' else snap.horizon_steps,
            )
            kwargs.update(policy='no_recovery', forced_action=action)
        if args.mission == 'inspection_descent':
            kwargs.update(snapshot=snap, seed=seed)
        import_module('staleguard.' + args.mission + '.simulation').execute(s, **kwargs)


if __name__ == '__main__':
    main()
