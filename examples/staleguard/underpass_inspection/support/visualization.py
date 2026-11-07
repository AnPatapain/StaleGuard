"""PyBullet drawing only. World states, references and scientific geometry stay Z-up."""
from dataclasses import asdict
from itertools import product
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import time
import numpy as np
import pybullet as p

from underpass_inspection.scenario import POS, BODY_RADIUS_M, SCENARIO_VERSION, WORLD_TO_NED

COLORS = {'nominal': (.36, .39, .42), 'dive': (.10, .60, .22),
          'climb': (.12, .32, .87), 'sideways': (0., .65, .75)}
DEFAULT_CAMERA = dict(distance=3.4, yaw=-35., pitch=-20., target=[2.2, .25, 1.0])
PILOT = Path(__file__).resolve().parents[1]/f'results/development/pilot_{SCENARIO_VERSION}'


def polyline(points, client, color, width=1., dashed=False):
    """Join actual samples directly. Dashed lines omit segments, never move samples."""
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        if not dashed or i % 6 < 3:
            p.addUserDebugLine(a, b, color, width, physicsClientId=client)


def wire_box(box, client, color, inflation=0.):
    lo, hi = np.asarray(box.lower)-inflation, np.asarray(box.upper)+inflation
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        for sides in product((0, 1), repeat=2):
            a, b = lo.copy(), lo.copy()
            for i, side in zip(other, sides):
                a[i] = b[i] = (lo, hi)[side][i]
            b[axis] = hi[axis]
            p.addUserDebugLine(a, b, color, .6, physicsClientId=client)


def visual_box(box, color, client):
    lo, hi = np.asarray(box.lower), np.asarray(box.upper)
    shape = p.createVisualShape(p.GEOM_BOX, halfExtents=(hi-lo)/2,
                                rgbaColor=color, physicsClientId=client)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                      baseVisualShapeIndex=shape, basePosition=(lo+hi)/2,
                      physicsClientId=client)


def deck_faces(box, client):
    """Exact existing face coordinates, separate translucent materials; no contacts."""
    lo, hi = np.asarray(box.lower), np.asarray(box.upper)
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        for side in (0, 1):
            vertices = []
            for a, b in ((0, 0), (1, 0), (1, 1), (0, 1)):
                vertex = lo.copy(); vertex[axis] = (lo, hi)[side][axis]
                vertex[other[0]] = (lo, hi)[a][other[0]]
                vertex[other[1]] = (lo, hi)[b][other[1]]
                vertices.append(vertex.tolist())
            color = ((.86, .52, .18, .20) if axis == 0 and side == 0 else
                     (.95, .72, .40, .17) if axis == 2 and side == 0 else
                     (.83, .76, .61, .07) if axis == 2 else (.87, .69, .43, .07))
            shape = p.createVisualShape(p.GEOM_MESH, vertices=vertices,
                indices=[0, 1, 2, 0, 2, 3, 2, 1, 0, 3, 2, 0],
                rgbaColor=color, physicsClientId=client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                              baseVisualShapeIndex=shape, physicsClientId=client)


def draw_reference(reference, client, mode):
    color = tuple(.62+.38*c for c in COLORS[mode])
    polyline(reference[:, POS], client, color, .8, dashed=mode == 'nominal')


def draw_scene(s, client, plane=None):
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=client)
    p.configureDebugVisualizer(rgbBackground=[.98, .985, .99], physicsClientId=client)
    camera = DEFAULT_CAMERA
    p.resetDebugVisualizerCamera(camera['distance'], camera['yaw'], camera['pitch'],
                                 camera['target'], physicsClientId=client)
    # Leave the stock SCG/PyBullet checkerboard plane completely untouched.
    deck_faces(s.obstacles[0], client)
    visual_box(s.obstacles[1], (.43, .32, .26, .83), client)
    visual_box(s.inspection, (.3, .75, .42, .045), client)
    wire_box(s.obstacles[0], client, (.70, .48, .24))
    wire_box(s.obstacles[1], client, (.49, .37, .30))
    wire_box(s.inspection, client, (.44, .70, .49))
    for box in s.obstacles:
        # Conservative guide; decisions still use exact sphere-to-box distance.
        wire_box(box, client, (.90, .78, .76), BODY_RADIUS_M+s.clearance_m)


def recorded_paths(s, mode, start_s):
    """Load physical samples from the unchanged frozen scenario; never predict paths."""
    summary = json.loads((PILOT/'summary.json').read_text())
    if json.loads(json.dumps(asdict(s))) != summary['scenario']:
        raise ValueError('--overlay-all requires the frozen pilot scenario; custom geometry is not compatible')
    rows = [json.loads(line) for line in (PILOT/'flights.jsonl').read_text().splitlines()]
    starts = {'nominal': None, 'dive': 0., 'climb': .5, 'sideways': .5}
    if mode != 'nominal':
        starts[mode] = start_s
    result = {}
    with np.load(PILOT/'trajectories.npz') as archive:
        for name, delay in starts.items():
            row = next((r for r in rows if r['mode'] == name and r['start_s'] == delay), None)
            if row is None:
                raise ValueError(f'No recorded {name} flight at +{delay}s for comparison overlay')
            physics = archive[row['trajectory_key']+'__physics']
            if hashlib.sha256(physics.tobytes()).hexdigest() != row['physics_sha256']:
                raise ValueError('Recorded overlay trajectory checksum mismatch')
            result[name] = dict(points=physics[::20, 1:].copy(), start_s=delay,
                                physics_sha256=row['physics_sha256'])
    return result


class FlightView:
    """Small GUI drawing helper. Every live point comes from the running simulator."""
    def __init__(self, s, env, mode, start_s, overlay_all=False):
        self.client, self.mode, self.start_s = env.PYB_CLIENT, mode, start_s
        self.s = s
        self.world_y, self.world_vy = float(env.state[2]), float(env.state[3])
        self.overlay = recorded_paths(s, mode, start_s) if overlay_all else None
        self.text_ids, self.line_ids, self.events = {}, {}, {}
        self.frame, self.camera_key = None, None
        draw_scene(s, self.client, env.PLANE_ID)
        if self.overlay:
            for name, path in self.overlay.items():
                polyline(path['points'], self.client, COLORS[name], 2.8, dashed=name == 'nominal')

    def event(self, label, position, color):
        self.events[label] = np.asarray(position).copy()
        p.addUserDebugPoints([position], [color], pointSize=6, physicsClientId=self.client)

    def approval(self, position):
        self.event('approval', position, (.22, .25, .28))

    def initiation(self, position, reference):
        self.event('start', position, COLORS[self.mode])
        draw_reference(reference, self.client, self.mode)

    def screen_point(self, x, y):
        point = self.frame @ np.array([2*x-1, 2*y-1, .9, 1.])
        return point[:3]/point[3]

    def text(self, key, text, x, y, color=(.22, .25, .29), size=1.):
        self.text_ids[key] = p.addUserDebugText(text, self.screen_point(x, y), color,
            textSize=size, replaceItemUniqueId=self.text_ids.get(key, -1), physicsClientId=self.client)

    def hud_line(self, key, a, b, color):
        self.line_ids[key] = p.addUserDebugLine(self.screen_point(*a), self.screen_point(*b),
            color, 1.5, replaceItemUniqueId=self.line_ids.get(key, -1), physicsClientId=self.client)

    def refresh(self, flight_s, active, unsafe):
        camera = p.getDebugVisualizerCamera(physicsClientId=self.client)
        view = np.asarray(camera[2]).reshape(4, 4, order='F')
        projection = np.asarray(camera[3]).reshape(4, 4, order='F')
        camera_key = tuple(camera[:2])+tuple(camera[2])+tuple(camera[3])
        if camera_key != self.camera_key:
            self.camera_key = camera_key
            self.frame = np.linalg.inv(projection @ view)
            self.text('title', 'UNDERPASS INSPECTION  |  '+self.mode.upper()+'  |  NED', .035, .955, size=1.2)
            if self.overlay:
                for i, (name, path) in enumerate(self.overlay.items()):
                    suffix = '' if path['start_s'] is None else f' +{path["start_s"]:.2f}s'
                    self.text('legend'+name, name.title()+suffix, .035+i*.23, .17, COLORS[name])
                self.text('style', 'Comparison: saved physical flights | thick = actual, thin = selected reference',
                          .035, .115, size=.9)
            else:
                self.text('style', 'Thick: actual flight   |   thin: reference   |   dashed gray: nominal',
                          .035, .17, size=.9)
            self.text('approval', f'Approval: flight +{self.s.approval_s:.2f}s (gray point)', .035, .06, size=.9)
            if self.mode != 'nominal':
                self.text('start', f'{self.mode.title()} start: approval +{self.start_s:.2f}s (colored point)',
                          .43, .06, COLORS[self.mode], size=.9)
            # NED basis in the actual camera: North=+world X, East=-world Y,
            # Down=-world Z. The two sign flips preserve right-handedness.
            self.text('inset', 'NED coordinate frame', .035, .80, size=.85)
            origin = np.array([.06, .70])
            for axis, direction, color in zip(('+X North (forward)', '+Y East (right)', '+Z Down'), WORLD_TO_NED,
                    ((.7, .3, .2), (.25, .55, .35), (.25, .40, .7))):
                end = origin+(view[:2, :3] @ direction)*np.array([.085, .105])
                self.hud_line('axis'+axis, origin, end, color)
                self.text('axislabel'+axis, axis, end[0]+.007, end[1], color, size=.8)
            self.text('mapping', 'North / East / Down', .035, .535, size=.75)
        current = self.mode if active or self.mode == 'nominal' else 'nominal / queued'
        status = 'UNSAFE' if unsafe else 'safe so far'
        self.text('status', f'{current.upper()} | flight {flight_s:.2f}s | approval {flight_s-self.s.approval_s:+.2f}s | {status}',
                  .035, .895, (.82, .18, .15) if unsafe else (.23, .36, .29), size=1.)
        self.text('world_y', f'NED East: {-self.world_y:+.3f} m | V_E: {-self.world_vy:+.3f} m/s | world Y: {self.world_y:+.3f} m',
                  .035, .225, size=.9)

    def update(self, a, b, step, active, unsafe, world_y, world_vy):
        self.world_y, self.world_vy = float(world_y), float(world_vy)
        name = self.mode if active else 'nominal'
        color = (.86, .16, .13) if unsafe else COLORS[name]
        if name != 'nominal' or step % 6 < 3:
            p.addUserDebugLine(a, b, color, 3.4, physicsClientId=self.client)
        if step % 5 == 0:
            self.refresh((step+1)/50, active, unsafe)


def save_camera(path, client):
    """Capture the actual GUI window including text/paths; portable camera fallback."""
    time.sleep(.15)  # Let the GUI thread finish the last text/camera update.
    if shutil.which('xwininfo') and shutil.which('import'):
        try:
            tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, stderr=subprocess.DEVNULL)
            windows = [line for line in tree.splitlines() if 'Bullet Physics ExampleBrowser' in line]
            window = re.search(r'0x[0-9a-fA-F]+', windows[-1]).group()
            subprocess.run(['import', '-window', window, str(path)], check=True, stderr=subprocess.DEVNULL)
            return
        except (subprocess.SubprocessError, IndexError, AttributeError):
            pass
    from PIL import Image
    camera = p.getDebugVisualizerCamera(physicsClientId=client)
    width, height = 1200, 800
    data = p.getCameraImage(width, height, camera[2], camera[3],
                            renderer=p.ER_BULLET_HARDWARE_OPENGL, physicsClientId=client)
    Image.fromarray(np.asarray(data[2], dtype=np.uint8).reshape(height, width, 4)).save(path)
    print('Saved camera render; this platform cannot capture GUI text/debug overlays.', flush=True)
