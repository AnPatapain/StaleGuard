"""Official D=0-only maximum-dwell calibration, also used by diagnostics.

Use exactly the original planner's finite candidate grid. Among candidates with
no unsafe entry at D=0, maximize consecutive inspection dwell measured at the
existing physics rate. Keep the first candidate in the original order on ties.
Require the existing inspection dwell threshold; otherwise return None.
No positive delay, prediction, or recovery outcome participates in selection.
"""

import os

# Manual calibration runs simulations serially, with one numerical-library thread.
if __name__ == "__main__":
    for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[variable] = "1"

from dataclasses import asdict, replace
import json

from experiment import ACCELERATION_SECONDS, CTRL_FREQ_HZ, DELAY_GRID, PYB_FREQ_HZ, generate_scenario, inspection_x
from mission import capture_approval
from model.common import STATE_SIZE, STATE_X, STATE_Y, STATE_Z
from staleguard_benefit import CALIBRATION_FILE, MISSION_DEADLINE_S, SCAN_DWELL_S, in_inspection_region, run_actual


def command_candidates(scenario):
    """Preserve the original planner's candidate grid, bounds, and order.

    The generated low duration is a starting value, not an operational cap.
    The reference returns to usual height by nominal structure arrival on the
    existing rounded control-step grid.
    """
    step = round((DELAY_GRID[1] - DELAY_GRID[0]) * CTRL_FREQ_HZ)
    approval_steps = list(range(round(scenario.approval_time * CTRL_FREQ_HZ), -1, -step))
    if 0 not in approval_steps:
        approval_steps.append(0)
    original_low_steps = round(scenario.low_seconds * CTRL_FREQ_HZ)
    structure_step = round(
        (scenario.box_x_min / scenario.speed + ACCELERATION_SECONDS / 2) * CTRL_FREQ_HZ
    )
    for approval in approval_steps:
        for low_steps in range(original_low_steps, structure_step - approval + 1, step):
            yield replace(
                scenario, approval_time=approval / CTRL_FREQ_HZ,
                low_seconds=low_steps / CTRL_FREQ_HZ,
            )


def inspection_dwell_ticks(physics_trace, scenario):
    """Score elapsed consecutive dwell using the existing inspection predicate."""
    state = [0.0] * STATE_SIZE
    scan_x = inspection_x(scenario)
    entered = None
    longest = 0
    for expected_tick, (tick, position) in enumerate(physics_trace):
        assert tick == expected_tick
        state[STATE_X], state[STATE_Y], state[STATE_Z] = position
        if in_inspection_region(state, scenario, scan_x):
            if entered is None:
                entered = tick
            longest = max(longest, tick - entered)
        else:
            entered = None
    return longest


def plan_max_dwell_command(scenario, on_candidate=None):
    """Return the D=0-safe maximum-dwell command, or None if inspection is infeasible.

    on_candidate(rank, command, outcome) is an optional audit hook; its return
    value is ignored. It cannot influence the objective, safety filter, or ties.
    """
    best = None
    best_ticks = -1
    required_ticks = round(SCAN_DWELL_S * PYB_FREQ_HZ)
    for rank, candidate in enumerate(command_candidates(scenario), start=1):
        snapshot = capture_approval(candidate)
        physics_trace = []
        completed, unsafe_entry, recovery_started, safe_completion = run_actual(
            candidate, snapshot,
            dispatch_step=round(candidate.approval_time * CTRL_FREQ_HZ),
            recover_step=0,
            deadline_step=round((candidate.approval_time + MISSION_DEADLINE_S) * CTRL_FREQ_HZ),
            recover=False,
            physics_trace=physics_trace,
        )
        dwell_ticks = inspection_dwell_ticks(physics_trace, candidate)
        assert not recovery_started
        assert safe_completion == (completed and not unsafe_entry)
        assert completed == (dwell_ticks >= required_ticks)
        if not unsafe_entry and dwell_ticks > best_ticks:
            best, best_ticks = candidate, dwell_ticks
        if on_candidate is not None:
            on_candidate(rank, candidate, {
                "inspection_completed": completed,
                "unsafe_entry": unsafe_entry,
                "recovery_started": recovery_started,
                "safe_completion": safe_completion,
                "best_dwell_ticks": dwell_ticks,
            })
    return best if best_ticks >= required_ticks else None


def calibrate_scenarios(seeds):
    """Run the D=0 search for each seed and collect results for the JSON file."""
    results = {}
    for seed in seeds:
        scenario = generate_scenario(seed)
        best_safe_dwell = None

        def record(rank, candidate, outcome):
            nonlocal best_safe_dwell
            if not outcome["unsafe_entry"]:
                dwell = outcome["best_dwell_ticks"]
                best_safe_dwell = dwell if best_safe_dwell is None else max(best_safe_dwell, dwell)

        command = plan_max_dwell_command(scenario, on_candidate=record)
        results[str(seed)] = {
            "generated_scenario": asdict(scenario),
            "command": None if command is None else asdict(command),
            "best_safe_dwell_ticks": best_safe_dwell,
        }
        status = "infeasible" if command is None else "feasible"
        print(f"Seed {seed}: {status}; best safe D=0 dwell = {best_safe_dwell} physics ticks", flush=True)
    return results


if __name__ == "__main__":
    results = calibrate_scenarios(range(20))
    CALIBRATION_FILE.write_text(json.dumps(results, indent=2) + "\n")
    print(f"Saved seeds 0–19 to {CALIBRATION_FILE}")
