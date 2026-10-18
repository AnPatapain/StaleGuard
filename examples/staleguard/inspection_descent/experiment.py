"""Shared definition of the inspection-descent experiment."""

from dataclasses import dataclass

import numpy as np

from model.common import PIDState, STATE_VX, STATE_X, STATE_Y, STATE_Z


# Reproducible experiment settings.
CTRL_FREQ_HZ = 50
PYB_FREQ_HZ = 1000
MODEL_FREQ_HZ = 100

EPISODE_SECONDS = 9
MEASURE_SECONDS = 8.5

DELAY_GRID = tuple(np.arange(0.0, 1.01, 0.1)) # Both experiments: 0.0–1.0 s, inclusive.
SIM_SEED = 42

ACCELERATION_SECONDS = 0.4 # Before Quadrotor reaches its stable speed, it needs to ramp during 0.4s from 0 → stable speed.
INSPECTION_STANDOFF = 0.5 # Distance on x that Quad need to keep from structure for inspection (mettres).


@dataclass
class Scenario:
    # Nominal flight.
    speed: float
    usual_height: float

    # Structure.
    box_x_min: float
    box_x_max: float
    box_y_limit: float
    unsafe_below_z: float

    # Approved command.
    approval_time: float
    low_height: float
    low_seconds: float


@dataclass(frozen=True)
class ApprovalSnapshot:
    """All information available to StaleGuard at approval time."""

    state: np.ndarray
    pid_state: PIDState

    # Index 0 is approval time.
    remaining_reference: np.ndarray

    low_height: float
    low_steps: int

    box_x_min: float
    box_x_max: float
    box_y_limit: float
    unsafe_below_z: float

    approval_time: float
    horizon_steps: int

def inspection_x(scenario: Scenario):
    return scenario.box_x_min - INSPECTION_STANDOFF


def generate_scenario(seed):
    rng = np.random.default_rng(seed)

    speed = rng.uniform(0.4, 0.8)
    usual_height = rng.uniform(1.15, 1.40)

    structure_start = rng.uniform(0.8, 1.5) # x direction
    structure_length = rng.uniform(0.3, 0.6) # x direction
    structure_width = 0.35 # y direction
    unsafe_below_z = rng.uniform(0.90, 1.05)

    # Nominal requested height must be above the unsafe threshold.
    if usual_height <= unsafe_below_z + 0.10:
        usual_height = unsafe_below_z + 0.10

    # The dip command goes below the unsafe threshold, so timing can matter.
    low_height = rng.uniform(0.60, unsafe_below_z - 0.10)
    low_seconds = rng.uniform(0.4, 1.0)

    scan_x = structure_start - INSPECTION_STANDOFF
    scan_time = scan_x/speed + ACCELERATION_SECONDS/2 # account for duration that Quad ramp from 0 to stable speed.
    approval_time = max(0.1, scan_time - low_seconds/2)

    return Scenario(
        speed=speed,
        usual_height=usual_height,
        box_x_min=structure_start,
        box_x_max=structure_start + structure_length,
        box_y_limit=structure_width,
        unsafe_below_z=unsafe_below_z,
        approval_time=approval_time,
        low_height=low_height,
        low_seconds=low_seconds,
    )


def make_forward_trajectory(env, scenario):
    n = env.X_GOAL.shape[0]
    dt = 1.0 / env.CTRL_FREQ
    times = np.arange(n) * dt
    speeds = scenario.speed * np.minimum(times / ACCELERATION_SECONDS, 1.0)

    trajectory = np.zeros_like(env.X_GOAL)
    # Trapezoidal integration is exact for this piecewise-linear velocity ramp.
    # At sample k, position integrates velocity up to k, not through k + 1.
    trajectory[:, STATE_X] = (np.cumsum(speeds) - speeds / 2) * dt
    trajectory[:, STATE_VX] = speeds
    trajectory[:, STATE_Z] = scenario.usual_height
    return trajectory


def apply_descent_command(reference, start_step, low_steps, low_height):
    """Return a copy of the reference with the approved descent applied."""
    changed = reference.copy()
    end = min(start_step + low_steps, len(changed))
    changed[start_step:end, STATE_Z] = low_height
    return changed


def unsafe(state, geometry):
    x, y, z = state[STATE_X], state[STATE_Y], state[STATE_Z]
    return (
        geometry.box_x_min <= x <= geometry.box_x_max
        and abs(y) <= geometry.box_y_limit
        and z <= geometry.unsafe_below_z
    )


def classify_delay_results(delays, results, nominal_safe=True):
    """Classify safety outcomes across dispatch delays."""
    if not nominal_safe:
        return {
            "classification": "INVALID_NOMINAL",
            "results": None,
            "first_bad_delay": None,
            "last_safe_delay": None,
        }

    if not delays or len(delays) != len(results):
        raise ValueError("delays and results must have equal, nonzero length")

    if results[0] != "SAFE":
        return {
            "classification": "INVALID_AT_APPROVAL",
            "results": results,
            "first_bad_delay": delays[0],
            "last_safe_delay": None,
        }

    first_bad_index = next(
        (i for i, status in enumerate(results) if status != "SAFE"),
        None,
    )

    if first_bad_index is None:
        return {
            "classification": "ALWAYS_SAFE",
            "results": results,
            "first_bad_delay": None,
            "last_safe_delay": delays[-1],
        }

    monotonic = all(status != "SAFE" for status in results[first_bad_index:])
    return {
        "classification": "FINITE_TTL" if monotonic else "NON_MONOTONIC",
        "results": results,
        "first_bad_delay": delays[first_bad_index],
        "last_safe_delay": delays[first_bad_index - 1],
    }
