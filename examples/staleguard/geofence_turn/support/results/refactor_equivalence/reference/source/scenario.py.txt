"""Seeded development generator and latest-feasible D=0 calibration only."""
from dataclasses import asdict, replace
import numpy as np
from .mission import (BODY_RADIUS_M, Config, Command, Scenario, RAMP_S, HZ, scenario_dict)


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
    initial_x = float(rng.uniform(-.15, .15))
    initial_y = float(rng.uniform(-.10, .10))
    # Same provisional patrol-space envelope as the original development run.
    fence_x = initial_x + float(rng.uniform(1.5, 2.5))
    # A fixed-width checkpoint must fit wholly inside the body-safe half-space,
    # and overlap lanes reachable by a turn after the acceleration ramp. This is
    # a geometric sampling constraint, not an approval-time or policy adjustment.
    width = config.checkpoint_width_m
    earliest_lane = initial_x + speed*(config.approval_min_s-RAMP_S/2) + radius
    latest_center = fence_x - BODY_RADIUS_M - config.d_min_m - width/2
    earliest_center = earliest_lane + width/2
    if earliest_center > latest_center:
        raise ValueError(f'checkpoint_geometry: seed={seed} speed={speed} radius={radius} '
                         f'fence_x={fence_x} earliest_center={earliest_center} '
                         f'latest_center={latest_center}; no fixed-width region fits')
    center_x = float(rng.uniform(earliest_center, latest_center))
    center_y = initial_y + radius + speed*config.checkpoint_north_travel_s
    tolerance = config.position_tolerance_m
    command = Command(radius, speed,
                      (center_x-width/2, center_y-tolerance, height-tolerance),
                      (center_x+width/2, center_y+tolerance, height+tolerance))
    # Freeze the deadline on the existing 50 Hz grid before any calibration.
    deadline = round((command.arc_s + config.checkpoint_north_travel_s
                      + float(rng.uniform(1., 2.)))*HZ)/HZ
    return Scenario(int(seed), speed, height, initial_x, initial_y, fence_x, deadline, command)


def approval_candidates(s, config=Config(), latest_approval_s=None):
    lo = round(config.approval_min_s*HZ)
    hi = round((config.approval_max_s if latest_approval_s is None else latest_approval_s)*HZ)
    stride = round(config.approval_grid_s*HZ)
    if lo < round(RAMP_S*HZ) or stride < 1:
        raise ValueError('Invalid post-ramp calibration grid')
    # Exhaustively descending finite grid; first feasible is its LATEST member.
    return [replace(s, approval_s=step/HZ) for step in range(hi, lo-1, -stride)]


def rejection_reasons(result):
    reasons = []
    if not result['safe']:
        reasons.append('body_clearance')
    if not result['completed']:
        reasons.append('checkpoint_deadline_velocity_or_dwell')
    return reasons


def calibrate(s, evaluate, config=Config(), on_candidate=None, latest_approval_s=None):
    """evaluate(candidate) returns the real D=0 simulation result and snapshot."""
    rows = []
    for candidate in approval_candidates(s, config, latest_approval_s):
        result, snapshot, trace = evaluate(candidate)
        reasons = rejection_reasons(result)
        row = dict(approval_s=candidate.approval_s, command=scenario_dict(candidate)['command'],
                   nominal_patrol_distance_m=candidate.speed*(candidate.approval_s-RAMP_S/2),
                   outcome=result, rejected_because=reasons)
        rows.append(row)
        if on_candidate is not None:
            on_candidate(row)
        if not reasons:
            return candidate, snapshot, rows
    return None, None, rows
