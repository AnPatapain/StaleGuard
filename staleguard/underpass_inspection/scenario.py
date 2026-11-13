"""Fixed bridge/pier geometry and dispatch-relative, smooth reference edits.

All default geometry, maneuver durations and inspection bounds are provisional
pilot design assumptions, not measured vehicle limits or optimized policy inputs.
The body sphere and 5 cm margin are the existing Geofence conventions.

Mission coordinates are right-handed North-East-Down (NED):
N=x_world, E=-y_world, D=-z_world. PyBullet state, physics, references and safety
geometry remain in the original Z-up world. Positive East therefore requests
negative world-Y displacement; positive Down is negative world Z.
"""
from dataclasses import asdict, dataclass, replace
import math
import numpy as np

from staleguard.model import CTRL_FREQ_HZ as HZ, PYB_FREQ_HZ as PHYSICS_HZ
# Enclosing sphere of SCG cf2x collision cylinder (r=.06, length=.025 m).
BODY_RADIUS_M = math.hypot(.06, .025 / 2)

POS = [0, 2, 4]
VEL = [1, 3, 5]
MODES = ('nominal', 'dive', 'climb', 'sideways')
SCENARIO_VERSION = 'ned_east_v2'
WORLD_TO_NED = np.diag([1., -1., -1.])
DEVELOPMENT_RANGES = dict(speed=[.55, .65], approval_s=[.70, .90],
                          deck_front=[1.35, 1.65], pier_x=[2.35, 3.25])


def world_to_ned(vector):
    """Position/linear velocity conversion only; no simulator state is modified."""
    return np.asarray(vector) @ WORLD_TO_NED.T


@dataclass(frozen=True)
class Box:
    name: str
    lower: tuple
    upper: tuple

    def __post_init__(self):
        if (len(self.lower) != 3 or len(self.upper) != 3
                or not np.all(np.isfinite([*self.lower, *self.upper]))
                or np.any(np.asarray(self.lower) >= self.upper)):
            raise ValueError('A box needs three finite, ordered bounds')


@dataclass(frozen=True)
class Scenario:
    version: str = SCENARIO_VERSION
    seed: int = 0
    speed: float = .6
    cruise_z: float = 1.35
    ramp_s: float = .4
    approval_s: float = .8
    observation_s: float = 8.0  # Fixed duration after approval, for every mode/start.
    clearance_m: float = .05
    deck_front: float = 1.5
    deck_back: float = 4.0
    deck_half_width: float = .55
    deck_bottom: float = 1.15
    deck_top: float = 1.55
    pier_x: float = 2.55
    pier_y: float = 0.
    pier_half_x: float = .12
    pier_half_y: float = .15
    dive_drop: float = .60
    dive_s: float = 1.30
    swerve_start_s: float = 2.60
    swerve_s: float = 1.20
    swerve_y: float = .85  # Positive NED East magnitude; applied along negative world Y.
    climb_rise: float = .70
    climb_s: float = 1.20
    sideways_y: float = 1.0  # Independent recovery's positive NED East magnitude.
    sideways_s: float = 1.30
    inspection_lower: tuple = (1.55, -.18, .65)
    inspection_upper: tuple = (1.85, .18, .85)
    inspection_dwell_s: float = .18

    def __post_init__(self):
        if self.version != SCENARIO_VERSION:
            raise ValueError('Use the current NED scenario version; preserve old world-Y pilots separately')
        if not isinstance(self.seed, int) or self.seed < 0:
            raise ValueError('A frozen scenario needs a nonnegative integer seed')
        for name, value in asdict(self).items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f'Nonfinite scenario parameter: {name}')
        if (min(self.speed, self.ramp_s, self.observation_s, self.dive_s,
                self.swerve_s, self.swerve_y, self.climb_s, self.sideways_s, self.sideways_y,
                self.dive_drop, self.climb_rise, self.pier_half_x,
                self.pier_half_y, self.inspection_dwell_s) <= 0
                or self.clearance_m < 0 or self.approval_s < self.ramp_s
                or self.swerve_start_s < self.dive_s
                or not self.deck_bottom < self.cruise_z < self.deck_top
                or not self.deck_front < self.pier_x < self.deck_back):
            raise ValueError('Invalid pilot geometry or maneuver parameters')
        self.obstacles
        self.inspection

    @property
    def obstacles(self):
        return (Box('deck', (self.deck_front, -self.deck_half_width, self.deck_bottom),
                    (self.deck_back, self.deck_half_width, self.deck_top)),
                Box('pier', (self.pier_x-self.pier_half_x,
                             self.pier_y-self.pier_half_y, 0.),
                    (self.pier_x+self.pier_half_x,
                     self.pier_y+self.pier_half_y, self.deck_bottom)))

    @property
    def inspection(self):
        return Box('inspection', tuple(self.inspection_lower), tuple(self.inspection_upper))


def generate_scenario(seed):
    """One independent frozen-distribution draw; no outcomes or candidate search.

    Speed/approval/front are modest variations around seed 0. Pier X is drawn
    independently in an absolute common interior segment behind all inspection
    regions. Even its furthest rear edge fits the shortest sampled deck. These
    provisional ranges do not guarantee D=0 feasibility. Only approval is rounded
    to the existing 20 ms clock. The inspection box moves with the slab; its size,
    altitude and dwell stay fixed. Every obstacle is frozen before any flight.
    """
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError('Use a nonnegative integer seed; calibration enforces dataset splits')
    if seed == 0:
        return Scenario()
    rng = np.random.default_rng(seed)
    speed = float(rng.uniform(*DEVELOPMENT_RANGES['speed']))
    approval = round(float(rng.uniform(*DEVELOPMENT_RANGES['approval_s']))*HZ)/HZ
    front = float(rng.uniform(*DEVELOPMENT_RANGES['deck_front']))
    pier = float(rng.uniform(*DEVELOPMENT_RANGES['pier_x']))
    base = Scenario()
    shift = front-base.deck_front
    return replace(base, seed=seed, speed=speed, approval_s=approval,
                   deck_front=front, deck_back=base.deck_back+shift, pier_x=pier,
                   inspection_lower=(base.inspection_lower[0]+shift, *base.inspection_lower[1:]),
                   inspection_upper=(base.inspection_upper[0]+shift, *base.inspection_upper[1:]))


def box_signed_distance(position, box):
    """Euclidean point-to-box distance outside; negative nearest-face depth inside."""
    # Scalar arithmetic avoids allocating arrays at every 1 kHz observation.
    q = [max(box.lower[i]-position[i], position[i]-box.upper[i]) for i in range(3)]
    return math.sqrt(sum(max(v, 0.)**2 for v in q)) + min(max(q), 0.)


def body_clearances(position, obstacles):
    return {box.name: box_signed_distance(position, box)-BODY_RADIUS_M for box in obstacles}


def violation_part(position, box):
    """Geometric diagnostic at the first violating sample; no causal inference."""
    if box.name == 'pier':
        return 'pier'
    outside = []
    for axis in range(3):
        if position[axis] < box.lower[axis]:
            outside.append(('front', 'side', 'underside')[axis])
        elif position[axis] > box.upper[axis]:
            outside.append(('back', 'side', 'top')[axis])
    return 'deck_' + ('_'.join(outside) if outside else 'interior')


def in_inspection(position, s):
    return all(lo <= value <= hi for value, lo, hi in
               zip(position, s.inspection_lower, s.inspection_upper))


def smooth_step(t, start, duration):
    """Quintic displacement and its derivative; zero endpoint velocity/acceleration."""
    u = np.clip((np.asarray(t)-start)/duration, 0., 1.)
    return u**3*(10-15*u+6*u*u), 30*u*u*(1-u)**2/duration


def make_nominal(n, s):
    t = np.arange(n)/HZ
    ramp = np.minimum(t, s.ramp_s)
    reference = np.zeros((n, 12))
    reference[:, 0] = s.speed*(ramp*ramp/(2*s.ramp_s)+np.maximum(t-s.ramp_s, 0))
    reference[:, 1] = s.speed*np.minimum(t/s.ramp_s, 1.)
    reference[:, 4] = s.cruise_z
    return reference


def displacement(t, s, mode):
    """Scenario-fixed shape. No obstacle coordinates or future outcomes are read."""
    t = np.asarray(t, dtype=float)
    position = np.zeros((t.size, 3))
    velocity = np.zeros_like(position)
    position[:, 0] = s.speed*t
    velocity[:, 0] = s.speed
    if mode == 'dive':
        f, df = smooth_step(t, 0., s.dive_s)
        position[:, 2], velocity[:, 2] = -s.dive_drop*f, -s.dive_drop*df
        f, df = smooth_step(t, s.swerve_start_s, s.swerve_s)
        position[:, 1], velocity[:, 1] = -s.swerve_y*f, -s.swerve_y*df
    elif mode == 'climb':
        f, df = smooth_step(t, 0., s.climb_s)
        position[:, 2], velocity[:, 2] = s.climb_rise*f, s.climb_rise*df
    elif mode == 'sideways':
        f, df = smooth_step(t, 0., s.sideways_s)
        position[:, 1], velocity[:, 1] = -s.sideways_y*f, -s.sideways_y*df
    else:
        raise ValueError(mode)
    return position, velocity


def apply_maneuver(reference, step, state, s, mode):
    """Anchor the unchanged displacement shape to the measured initiation position.

    Internal joins are C2 (position, velocity and acceleration continuous). The
    requested actual-state anchor can differ from the previous nominal reference
    by its tracking error; that dispatch splice is measured and reported. The
    physical state and PID memory are never overwritten.
    """
    changed = reference.copy()
    dp, velocity = displacement(np.arange(len(reference)-step)/HZ, s, mode)
    changed[step:, POS] = state[POS]+dp
    changed[step:, VEL] = velocity
    return changed
