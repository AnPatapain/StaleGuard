"""Frozen command, deterministic references, and physical mission predicates."""

from dataclasses import dataclass, asdict
import math
import numpy as np

HZ = 50
PHYSICS_HZ = 1000
RAMP_S = 0.4  # Existing nominal acceleration ramp.
SCHEMA = "northbound_checkpoint_v2"
# Enclosing sphere of SCG cf2x.urdf collision cylinder (r=.06, length=.025 m).
BODY_RADIUS_M = math.hypot(0.06, 0.025 / 2)
POS = [0, 2, 4]
VEL = [1, 3, 5]


@dataclass(frozen=True)
class Config:
    # Provisional physical requirements, frozen before the characterization/run.
    d_min_m: float = 0.05
    position_tolerance_m: float = 0.10
    velocity_tolerance_mps: float = 0.12
    dwell_s: float = 0.20
    # Provisional characterization acceptance bound, not a safety margin.
    tracking_limit_m: float = 0.15
    checkpoint_width_m: float = 0.60  # Provisional fixed east-west extent.
    checkpoint_north_travel_s: float = 2.0  # Preselect north distance = speed * 2 s.
    approval_min_s: float = 0.60
    approval_max_s: float = 3.20
    approval_grid_s: float = 0.20
    max_rss_mb: int = 2048
    min_available_mb: int = 2048


@dataclass(frozen=True)
class Command:
    radius: float
    speed: float
    checkpoint_lower: tuple  # Fixed world-frame AABB, inclusive position bounds.
    checkpoint_upper: tuple
    direction: str = "TURN_LEFT_90"

    def __post_init__(self):
        if (
            self.radius <= 0
            or self.speed <= 0
            or self.direction != "TURN_LEFT_90"
            or len(self.checkpoint_lower) != 3
            or len(self.checkpoint_upper) != 3
            or not np.all(
                np.isfinite(
                    [self.radius, self.speed, *self.checkpoint_lower, *self.checkpoint_upper]
                )
            )
            or np.any(np.asarray(self.checkpoint_lower) >= self.checkpoint_upper)
        ):
            raise ValueError("Invalid frozen turn command")
        object.__setattr__(self, "checkpoint_lower", tuple(self.checkpoint_lower))
        object.__setattr__(self, "checkpoint_upper", tuple(self.checkpoint_upper))

    @property
    def arc_s(self):
        return math.pi * self.radius / (2 * self.speed)


@dataclass(frozen=True)
class Scenario:
    seed: int
    speed: float
    height: float
    initial_x: float
    initial_y: float
    fence_x: float
    deadline_s: float  # relative to approval, identical for every policy
    command: Command
    approval_s: float = 0.60


def scenario_dict(s):
    return dict(schema=SCHEMA, **asdict(s))


def scenario_from_dict(row):
    row = dict(row)
    if row.pop("schema", None) != SCHEMA:
        raise ValueError("Incompatible geofence scenario: regenerate for northbound_checkpoint_v2")
    row["command"] = Command(**row["command"])
    return Scenario(**row)


def make_forward_trajectory(n, s):
    times = np.arange(n) / HZ
    ramp = np.minimum(times, RAMP_S)
    x = s.initial_x + s.speed * (ramp**2 / (2 * RAMP_S) + np.maximum(times - RAMP_S, 0))
    ref = np.zeros((n, 12))
    ref[:, 0] = x
    ref[:, 1] = s.speed * np.minimum(times / RAMP_S, 1)
    ref[:, 2] = s.initial_y
    ref[:, 4] = s.height
    return ref


def turn_sample(t, anchor, cmd):
    """Quarter-circle, then straight +Y forever. The checkpoint never steers it.

    The dispatch anchor is the nominal reference row, as in the original reference
    editing contract. No goal following, lateral correction, or route search.
    """
    x0, y0, z0 = np.asarray(anchor)[POS]
    if t <= cmd.arc_s:
        theta = max(t, 0) * cmd.speed / cmd.radius
        return (
            np.array([x0 + cmd.radius * np.sin(theta), y0 + cmd.radius * (1 - np.cos(theta)), z0]),
            np.array([cmd.speed * np.cos(theta), cmd.speed * np.sin(theta), 0]),
        )
    end = np.array([x0 + cmd.radius, y0 + cmd.radius, z0])
    velocity = np.array([0.0, cmd.speed, 0.0])
    return end + velocity * (t - cmd.arc_s), velocity


def apply_turn_command(reference, start_step, command):
    if not hasattr(command, "checkpoint_lower") or not hasattr(command, "checkpoint_upper"):
        raise ValueError("Legacy connector command: regenerate the northbound dataset")
    if not 0 <= start_step < len(reference):
        raise ValueError("Dispatch outside reference")
    anchor = reference[start_step].copy()
    if not np.allclose(anchor[VEL], [command.speed, 0, 0], atol=1e-10):
        raise ValueError("Turn must start after the +X acceleration ramp")
    changed = reference.copy()
    # Vectorized analytic samples of the same C1 path, with no endpoint connector.
    times = np.arange(len(changed) - start_step) / HZ
    theta = np.minimum(times * command.speed / command.radius, math.pi / 2)
    changed[start_step:, 0] = anchor[0] + command.radius * np.sin(theta)
    changed[start_step:, 1] = command.speed * np.cos(theta)
    changed[start_step:, 2] = (
        anchor[2]
        + command.radius * (1 - np.cos(theta))
        + command.speed * np.maximum(times - command.arc_s, 0)
    )
    changed[start_step:, 3] = command.speed * np.sin(theta)
    changed[start_step:, 4] = anchor[4]
    changed[start_step:, 5] = 0
    # Explicit zero avoids a residual cos(pi/2) east velocity on the north leg.
    changed[start_step + np.flatnonzero(times >= command.arc_s), 1] = 0
    return changed


def clearance(x, fence_x):
    """Signed BODY-to-plane distance. Apply d_min separately, exactly once."""
    return fence_x - np.asarray(x) - BODY_RADIUS_M


def in_checkpoint(position, velocity, command, config):
    """Pass through the frozen region while tracking the approved north velocity.

    Position bounds already include their named spatial tolerances; do not inflate
    the region again here. A stationary safety fallback is not a valid north pass.
    """
    position = np.asarray(position)
    return (
        np.all(position >= command.checkpoint_lower)
        and np.all(position <= command.checkpoint_upper)
        and np.linalg.norm(np.asarray(velocity) - [0.0, command.speed, 0.0])
        <= config.velocity_tolerance_mps
    )


def safety_only_reference(reference, step, state):
    """Cancel the turn and request hover at current position; never retry mission.

    Uses the existing position PID. A step to zero reference velocity asks it to
    brake; actual stopping clearance is measured, not assumed safe.
    """
    changed = reference.copy()
    changed[step:, POS] = state[POS]
    changed[step:, VEL] = 0
    return changed


def generate_scenario(seed, envelope, config=Config()):
    """Sample once; no delayed-policy outcomes enter this function.

    Speeds and radii are measured grid values, not an extrapolated continuous
    turning envelope. Remaining quantities are independent draws subject to the
    explicitly documented geometry/deadline construction below.
    """
    feasible = [r for r in envelope['results'] if r['feasible']]
    if not feasible:
        raise ValueError('No empirically feasible turning pair')
    if envelope['config'] != asdict(config):
        raise ValueError('Envelope/config mismatch; characterize again')
    rng = np.random.default_rng(seed)
    speeds = sorted(set(r['speed'] for r in feasible))
    speed = float(rng.choice(speeds))
    radii = sorted(set(r['radius'] for r in feasible if r['speed'] == speed))
    radius = float(rng.choice(radii))
    height = float(rng.uniform(1.15, 1.40))
    initial_x = float(rng.uniform(-0.15, 0.15))
    initial_y = float(rng.uniform(-0.10, 0.10))
    # Same provisional patrol-space envelope as the original development run.
    fence_x = initial_x + float(rng.uniform(1.5, 2.5))
    # A fixed-width checkpoint must fit wholly inside the body-safe half-space,
    # and overlap lanes reachable by a turn after the acceleration ramp. This is
    # a geometric sampling constraint, not an approval-time or policy adjustment.
    width = config.checkpoint_width_m
    earliest_lane = initial_x + speed * (config.approval_min_s - RAMP_S / 2) + radius
    latest_center = fence_x - BODY_RADIUS_M - config.d_min_m - width / 2
    earliest_center = earliest_lane + width / 2
    if earliest_center > latest_center:
        raise ValueError(
            f'checkpoint_geometry: seed={seed} speed={speed} radius={radius} '
            f'fence_x={fence_x} earliest_center={earliest_center} '
            f'latest_center={latest_center}; no fixed-width region fits'
        )
    center_x = float(rng.uniform(earliest_center, latest_center))
    center_y = initial_y + radius + speed * config.checkpoint_north_travel_s
    tolerance = config.position_tolerance_m
    command = Command(
        radius,
        speed,
        (center_x - width / 2, center_y - tolerance, height - tolerance),
        (center_x + width / 2, center_y + tolerance, height + tolerance),
    )
    # Freeze the deadline on the existing 50 Hz grid before any calibration.
    deadline = (
        round(
            (command.arc_s + config.checkpoint_north_travel_s + float(rng.uniform(1.0, 2.0))) * HZ
        )
        / HZ
    )
    return Scenario(int(seed), speed, height, initial_x, initial_y, fence_x, deadline, command)
