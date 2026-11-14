"""Original calibration search and acceptance criteria; fresh outputs only."""

from pathlib import Path
import sys
from staleguard import inspection_descent
import argparse
from dataclasses import asdict, replace
import fcntl
import json
from staleguard.inspection_descent.scenario import (
    generate_scenario,
    configuration,
    ACCELERATION_SECONDS,
    CTRL_FREQ_HZ,
    DELAY_GRID,
    PYB_FREQ_HZ,
    inspection_x,
    in_inspection_region,
    SCAN_DWELL_S,
    MISSION_DEADLINE_S,
)
from staleguard.model.common import STATE_SIZE, STATE_X, STATE_Y, STATE_Z
from staleguard.inspection_descent.simulation import execute, capture_approval, run_actual
from staleguard.inspection_descent.storage import (
    ROOT,
    check_memory,
    metadata,
    save_snapshot,
    write_json,
    file_hash,
)


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
                scenario,
                approval_time=approval / CTRL_FREQ_HZ,
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


def calibrate_scenarios(seeds, output, resume=False):
    """Keep the official candidate order, maximum dwell and first-on-ties rule."""
    output = Path(output)
    target = output / 'calibrated_scenarios.json'
    seeds = list(seeds)
    if target.exists() or resume:
        raise FileExistsError('Use a fresh output directory')
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(output)
    output.mkdir(parents=True, exist_ok=True)
    data = dict(
        format='inspection_calibration_v1',
        origin='physical_calibration',
        config=configuration(),
        provenance=metadata(),
        seeds=seeds,
        scenarios=[],
        attempted_seeds=[],
        rejected_seeds=[],
        status='partial',
        statistics=dict(attempted=0, accepted=0, rejected=0),
    )
    write_json(target, data)  # An interrupted first seed can restart safely.
    for seed in seeds:
        if seed in data['attempted_seeds']:
            continue
        check_memory()
        scenario = generate_scenario(seed)
        selected = None
        selected_snapshot = None
        best_dwell = None
        # Original candidate order; keep the first candidate on equal dwell.
        for candidate in command_candidates(scenario):
            snapshot = capture_approval(candidate)
            physics_trace = []
            completed, unsafe_entry, _, safe_completion = run_actual(
                candidate,
                snapshot,
                round(candidate.approval_time * CTRL_FREQ_HZ),
                0,
                round((candidate.approval_time + MISSION_DEADLINE_S) * CTRL_FREQ_HZ),
                False,
                physics_trace=physics_trace,
            )
            dwell = inspection_dwell_ticks(physics_trace, candidate)
            assert completed == (dwell >= round(SCAN_DWELL_S * PYB_FREQ_HZ))
            assert safe_completion == (completed and not unsafe_entry)
            if not unsafe_entry and (best_dwell is None or dwell > best_dwell):
                selected, selected_snapshot, best_dwell = candidate, snapshot, dwell
            check_memory()
        if best_dwell is None or best_dwell < round(SCAN_DWELL_S * PYB_FREQ_HZ):
            selected = None
        if selected is not None:
            snapshot = selected_snapshot
            verified, _, _ = execute(selected, snapshot=snapshot, seed=seed)
            if not verified['safe_completion']:
                raise ValueError('Selected D=0 command failed verification')
            path = output / f'snapshots/seed_{seed}.pkl'
            checksum = save_snapshot(path, snapshot)
            data['scenarios'].append(
                dict(
                    seed=seed,
                    scenario=asdict(selected),
                    generated_scenario=asdict(scenario),
                    best_safe_dwell_ticks=best_dwell,
                    approval_snapshot=str(path.relative_to(output)),
                    snapshot_sha256=checksum,
                    d0_verification=verified,
                )
            )
        else:
            data['rejected_seeds'].append(seed)
        data['attempted_seeds'].append(seed)
        data['statistics'] = dict(
            attempted=len(data['attempted_seeds']),
            accepted=len(data['scenarios']),
            rejected=len(data['rejected_seeds']),
        )
        data['status'] = 'complete' if len(data['attempted_seeds']) == len(seeds) else 'partial'
        write_json(target, data)
        print(
            f'seed={seed}: {"accepted" if selected is not None else "rejected"}; '
            f'{data["statistics"]}',
            flush=True,
        )
    return data
