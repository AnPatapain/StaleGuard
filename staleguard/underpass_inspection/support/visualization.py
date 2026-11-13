"""PyBullet drawing only. World states, references and scientific geometry stay Z-up."""
from dataclasses import asdict
from itertools import product
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import time
import numpy as np
import pybullet as p

from staleguard.underpass_inspection.scenario import Box, POS, BODY_RADIUS_M, SCENARIO_VERSION, WORLD_TO_NED

COLORS = {'nominal': (.36, .39, .42), 'dive': (.10, .60, .22),
          'climb': (.12, .32, .87), 'sideways': (0., .65, .75)}
DEFAULT_CAMERA = dict(distance=3.4, yaw=-35., pitch=-20., target=[2.2, .25, 1.0])
PAPER_CAMERA = dict(distance=2.7, yaw=-35., pitch=-20., target=[1.85, .1, 1.0])
PAPER_COLORS = {'nominal': (.36, .39, .42),
                'safe_inspection': (.10, .60, .22),
                'late_inspection': (.86, .16, .13),
                'safe_recovery': (1., .50, .05)}
PILOT = Path(__file__).resolve().parents[1]/f'results/development/pilot_{SCENARIO_VERSION}'


def paper_mode():
    return os.environ.get('STALEGUARD_PAPER_FIGURE') == '1'


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
            color = ((.70, .74, .77, .82) if axis == 0 and side == 0 else
                     (.64, .69, .72, .68) if axis == 2 and side == 0 else
                     (.91, .93, .94, .88) if axis == 2 else (.79, .82, .84, .80))
            shape = p.createVisualShape(p.GEOM_MESH, vertices=vertices,
                indices=[0, 1, 2, 0, 2, 3, 2, 1, 0, 3, 2, 0],
                rgbaColor=color, physicsClientId=client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                              baseVisualShapeIndex=shape, physicsClientId=client)


def draw_reference(reference, client, mode):
    if paper_mode():
        return
    color = tuple(.62+.38*c for c in COLORS[mode])
    polyline(reference[:, POS], client, color, .8, dashed=mode == 'nominal')


def draw_ground(plane, client, center):
    """Restyle the existing ground's visual material and add a faint local grid."""
    if plane is None:
        return
    ground = np.full(3, 250/255)  # #FAFAFA
    grid = np.full(3, 218/255)  # #DADADA
    # A light translucent material blends the shaded floor into the white
    # background without a contrasting horizon; its collision is untouched.
    p.changeVisualShape(plane, -1, textureUniqueId=-1,
                        rgbaColor=[*ground, .06], specularColor=[0., 0., 0.],
                        physicsClientId=client)
    height = p.getBasePositionAndOrientation(plane, physicsClientId=client)[0][2] + .001
    cx, cy = np.floor(np.asarray(center)/.5)*.5
    extent = 10.
    vertices, indices = [], []
    # Thin flat strips respect depth, unlike debug lines drawn over obstacles.
    for offset in np.arange(-extent, extent+.25, .5):
        for axis in (0, 1):
            strip = [[offset+dx, y, 0.] for dx, y in
                     ((-.003, -extent), (.003, -extent),
                      (.003, extent), (-.003, extent))]
            if axis:
                strip = [[y, x, z] for x, y, z in strip]
            i = len(vertices)
            vertices.extend(strip)
            indices.extend((i, i+1, i+2, i, i+2, i+3,
                            i+2, i+1, i, i+3, i+2, i))
    shape = p.createVisualShape(p.GEOM_MESH, vertices=vertices, indices=indices,
                                rgbaColor=[*grid, .55], physicsClientId=client)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                      baseVisualShapeIndex=shape, basePosition=[cx, cy, height],
                      physicsClientId=client)


def draw_scene(s, client, plane=None):
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=client)
    p.configureDebugVisualizer(rgbBackground=[1., 1., 1.], physicsClientId=client)
    camera = PAPER_CAMERA if paper_mode() else DEFAULT_CAMERA
    p.resetDebugVisualizerCamera(camera['distance'], camera['yaw'], camera['pitch'],
                                 camera['target'], physicsClientId=client)
    draw_ground(plane, client, camera['target'][:2])
    deck_faces(s.obstacles[0], client)
    # Presentation-only footing: no collision shape or mission safety predicate.
    pier = s.obstacles[1]
    ground_z = (p.getBasePositionAndOrientation(plane, physicsClientId=client)[0][2]
                if plane is not None else -.05)
    base = Box('visual_base', (pier.lower[0]-.18, pier.lower[1]-.22, ground_z),
               (pier.upper[0]+.18, pier.upper[1]+.22, pier.lower[2]+.04))
    visual_box(base, (.76, .80, .83, 1.), client)
    wire_box(base, client, (.47, .51, .55))
    visual_box(pier, (.67, .69, .71, 1.), client)
    visual_box(s.inspection, (.3, .75, .42, .045), client)
    wire_box(s.obstacles[0], client, (.47, .51, .55))
    wire_box(pier, client, (.47, .51, .55))
    wire_box(s.inspection, client, (.44, .70, .49))
    for box in s.obstacles:
        # Conservative guide; decisions still use exact sphere-to-box distance.
        wire_box(box, client, (.79, .81, .83), BODY_RADIUS_M+s.clearance_m)


def recorded_paths(s, mode, start_s, paper=False):
    """Load physical samples from the unchanged frozen scenario; never predict paths."""
    summary = json.loads((PILOT/'summary.json').read_text())
    if json.loads(json.dumps(asdict(s))) != summary['scenario']:
        raise ValueError('--overlay-all requires the frozen pilot scenario; custom geometry is not compatible')
    rows = [json.loads(line) for line in (PILOT/'flights.jsonl').read_text().splitlines()]

    if paper:
        starts = {'nominal': ('nominal', None),
                  'safe_inspection': ('dive', 0.),
                  'late_inspection': ('dive', .4),
                  'safe_recovery': ('sideways', .5)}
    else:
        starts = {'nominal': ('nominal', None),
                  'dive': ('dive', 0.),
                  'climb': ('climb', .5),
                  'sideways': ('sideways', .5)}
        if mode != 'nominal':
            starts[mode] = (mode, start_s)

    result = {}
    with np.load(PILOT/'trajectories.npz') as archive:
        for name, (flight_mode, delay) in starts.items():
            row = next((r for r in rows if r['mode'] == flight_mode and r['start_s'] == delay), None)
            if row is None:
                raise ValueError(f'No recorded {flight_mode} flight at +{delay}s for comparison overlay')
            physics = archive[row['trajectory_key']+'__physics']
            if hashlib.sha256(physics.tobytes()).hexdigest() != row['physics_sha256']:
                raise ValueError('Recorded overlay trajectory checksum mismatch')
            result[name] = dict(points=physics[::20, 1:].copy(), start_s=delay,
                                physics_sha256=row['physics_sha256'])
    return result


def replay_paths(s, mode, start_s):
    """GUI comparison for a calibrated case: four actual headless flights only.

    Does not write into the scientific deadline journal or use future predictions.
    The same frozen scenario and original schedules feed the existing simulator.
    """
    from staleguard.underpass_inspection.simulation import execute
    from staleguard.underpass_inspection.calibration import release_memory
    starts = {'nominal': 0., 'dive': 0., 'climb': .5, 'sideways': .5}
    if mode != 'nominal':
        starts[mode] = start_s
    paths = {}
    for name, start in starts.items():
        release_memory()
        row, data = execute(s, name, start)
        paths[name] = dict(points=data['physics'][::20, 1:].copy(),
                           start_s=None if name=='nominal' else start,
                           physics_sha256=row['physics_sha256'])
    return paths


def paper_quadrotor(drone, client):
    """Larger display mesh only; the physical UAV and its collision stay intact."""
    visual = next(v for v in p.getVisualShapeData(drone, physicsClientId=client)
                  if v[1] == -1 and v[2] == p.GEOM_MESH)
    shape = p.createVisualShape(p.GEOM_MESH, fileName=visual[4].decode(),
        meshScale=(1.6*np.asarray(visual[3])).tolist(),
        visualFramePosition=visual[5], visualFrameOrientation=visual[6],
        rgbaColor=[.12, .15, .18, 1.], physicsClientId=client)
    position, orientation = p.getBasePositionAndOrientation(drone, physicsClientId=client)
    display = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=shape, basePosition=position, baseOrientation=orientation,
        physicsClientId=client)
    p.changeVisualShape(display, -1, textureUniqueId=-1,
                        specularColor=[0., 0., 0.], physicsClientId=client)
    p.changeVisualShape(drone, -1, rgbaColor=[1., 1., 1., 0.], physicsClientId=client)
    return display


class FlightView:
    """Small GUI drawing helper. Every live point comes from the running simulator."""
    def __init__(self, s, env, mode, start_s, overlay_all=False, overlay_paths=None):
        self.client, self.mode, self.start_s = env.PYB_CLIENT, mode, start_s
        self.s = s
        self.world_y, self.world_vy = float(env.state[2]), float(env.state[3])
        self.paper = paper_mode()
        self.drone = env.DRONE_ID
        self.paper_capture = os.environ.get('STALEGUARD_PAPER_CAPTURE') if self.paper else None
        self.paper_capture_at = float(os.environ.get('STALEGUARD_PAPER_AT', '0'))
        self.paper_captured = False
        self.overlay = ((overlay_paths if overlay_paths is not None else
                         recorded_paths(s, mode, start_s, paper=self.paper))
                        if overlay_all else None)
        self.text_ids, self.line_ids, self.events = {}, {}, {}
        self.frame, self.camera_key = None, None
        draw_scene(s, self.client, env.PLANE_ID)
        self.quad_display = paper_quadrotor(self.drone, self.client) if self.paper else None
        if self.overlay:
            for name, path in self.overlay.items():
                color = PAPER_COLORS[name] if self.paper else COLORS[name]
                polyline(path['points'], self.client, color,
                         2.8 if self.paper else 2.2, dashed=name == 'nominal')

    def event(self, label, position, color):
        if self.paper:
            return
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
        if self.paper:
            return
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
        if self.paper:
            position, orientation = p.getBasePositionAndOrientation(
                self.drone, physicsClientId=self.client)
            p.resetBasePositionAndOrientation(self.quad_display, position, orientation,
                                               physicsClientId=self.client)
            if (self.paper_capture and not self.paper_captured
                    and (step+1)/50 >= self.paper_capture_at):
                self.paper_captured = True
                save_camera(self.paper_capture, self.client)
            return
        name = self.mode if active else 'nominal'
        color = (.86, .16, .13) if unsafe else COLORS[name]
        if name != 'nominal' or step % 6 < 3:
            p.addUserDebugLine(a, b, color, 2.5, physicsClientId=self.client)
        if step % 5 == 0:
            self.refresh((step+1)/50, active, unsafe)


# Fraction of the captured height cut away after labeling (paper figure only).
PAPER_CROP_TOP, PAPER_CROP_BOTTOM = .14, .17


def label_paper_capture(path, client):
    """Add sharp 2D text after capture, then crop empty space; never change simulator geometry or trajectories."""
    from PIL import Image, ImageDraw, ImageFont

    image = Image.open(path).convert('RGB')
    width, height = image.size
    scale = max(.8, min(width/1000, height/800))
    top = round(PAPER_CROP_TOP*height)
    bottom = height-round(PAPER_CROP_BOTTOM*height)  # new bottom edge of the image

    def font(size):
        try:
            return ImageFont.truetype('DejaVuSans.ttf', round(size*scale))
        except OSError:
            return ImageFont.load_default()

    draw = ImageDraw.Draw(image, 'RGBA')
    # Sharp NED coordinate inset, oriented using the actual PyBullet camera.
    camera = p.getDebugVisualizerCamera(physicsClientId=client)
    view = np.asarray(camera[2]).reshape(4, 4, order='F')

    draw.text((34, round(.21 * height)), 'NED coordinate frame',
              font=font(15), fill=(45, 52, 60, 255))

    origin = np.array([.075 * width, .29 * height])
    axes = (
        ('+X North (forward)', WORLD_TO_NED[0], (180, 75, 50, 255)),
        ('+Y East (right)', WORLD_TO_NED[1], (55, 140, 85, 255)),
        ('+Z Down', WORLD_TO_NED[2], (55, 95, 180, 255)),
    )

    for label, direction, color in axes:
        projected = view[:2, :3] @ direction
        end = origin + np.array([
            projected[0] * .085 * width,
            -projected[1] * .105 * height
        ])

        draw.line((*origin, *end), fill=color, width=max(2, round(3 * scale)))
        draw.text((end[0] + 7, end[1] - 9), label,
                  font=font(14), fill=color)

    # Legend sits just above the new bottom edge.
    draw.rectangle((18, bottom-112, width-18, bottom-15),
                   fill=(255, 255, 255, 235))

    labels = [
        ('nominal', 'Nominal', 34, bottom-94),
        ('safe_inspection', 'Safe inspection (D=0)', width//2, bottom-94),
        ('late_inspection', 'Late inspection (D=0.4 s)', 34, bottom-51),
        ('safe_recovery', 'Safe recovery (+0.5 s)', width//2, bottom-51)
    ]
    for name, label, x, y in labels:
        color = tuple(round(255*c) for c in PAPER_COLORS[name])+(255,)
        if name == 'nominal':
            draw.line((x, y+11, x+12, y+11), fill=color, width=3)
            draw.line((x+19, y+11, x+32, y+11), fill=color, width=3)
        else:
            draw.line((x, y+11, x+32, y+11), fill=color, width=3)
        draw.text((x+43, y), label, font=font(17),
                  fill=(35, 42, 48, 255))

    image.crop((0, top, width, bottom)).save(path)


def save_camera(path, client):
    """Capture the actual GUI window including text/paths; portable camera fallback."""
    time.sleep(.15)  # Let the GUI thread finish the last text/camera update.
    if shutil.which('xwininfo') and shutil.which('import'):
        try:
            tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, stderr=subprocess.DEVNULL)
            windows = [line for line in tree.splitlines() if 'Bullet Physics ExampleBrowser' in line]
            window = re.search(r'0x[0-9a-fA-F]+', windows[-1]).group()
            subprocess.run(['import', '-window', window, str(path)], check=True, stderr=subprocess.DEVNULL)
            if paper_mode():
                label_paper_capture(path, client)
            return
        except (subprocess.SubprocessError, IndexError, AttributeError):
            pass
    if paper_mode():
        raise RuntimeError(
            'Paper capture needs an X11 window capture to preserve trajectory overlays. '
            'Install xwininfo/ImageMagick import, or capture the GUI window manually.'
        )
    from PIL import Image
    camera = p.getDebugVisualizerCamera(physicsClientId=client)
    width, height = 1200, 800
    data = p.getCameraImage(width, height, camera[2], camera[3],
                            renderer=p.ER_BULLET_HARDWARE_OPENGL, physicsClientId=client)
    Image.fromarray(np.asarray(data[2], dtype=np.uint8).reshape(height, width, 4)).save(path)
    print('Saved camera render; this platform cannot capture GUI text/debug overlays.', flush=True)
