"""Sweep dispatch delay for the existing calibrated command, without recovery.

From the repository root, using the SCG environment:
    python examples/staleguard/inspection_descent/diagnostics/no_recovery_diagnostic.py

T is relative to approval, as in staleguard_benefit.py. Calibration runs once
per scenario; the same approved command is then used at every dispatch delay.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import csv
from dataclasses import asdict

from experiment import CTRL_FREQ_HZ, DELAY_GRID, generate_scenario
from mission import capture_approval
from staleguard_benefit import SCENARIOS, run_actual
from first_feasible import plan_first_feasible_command as plan_feasible_command


MISSION_DEADLINE_S = 4.2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "results" / "no_recovery_delay_sweep.csv",
        help="Destination for one row per scenario and delay.",
    )
    args = parser.parse_args()

    calibrated = []
    for scenario_id in range(SCENARIOS):
        scenario = plan_feasible_command(generate_scenario(scenario_id))
        if scenario is None:
            raise RuntimeError(f"Scenario {scenario_id}: no feasible original command found.")
        calibrated.append((scenario_id, scenario, capture_approval(scenario)))

    fields = [
        "scenario_id", "delay_s", "deadline_s", "classification",
        "inspection_completed", "unsafe_entry", "recovery_started", "safe_completion",
        "dispatch_step", "deadline_step",
        *asdict(calibrated[0][1]),
    ]
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()

        print(f"No Recovery | T = {MISSION_DEADLINE_S:.1f}s | N = {SCENARIOS}")
        print("A: safe, inspection completed | B: safe, inspection failed | C: unsafe")
        print(f"{'D (s)':>5} {'A':>4} {'B':>4} {'C':>4} {'Total':>6}")
        print("-" * 27)
        for delay in DELAY_GRID:
            counts = {"A": 0, "B": 0, "C": 0}
            for scenario_id, scenario, snapshot in calibrated:
                dispatch_step = round((snapshot.approval_time + float(delay)) * CTRL_FREQ_HZ)
                deadline_step = round((snapshot.approval_time + MISSION_DEADLINE_S) * CTRL_FREQ_HZ)
                inspection, unsafe_entry, recovery_started, safe_completion = run_actual(
                    scenario, snapshot,
                    dispatch_step=dispatch_step,
                    recover_step=0,
                    deadline_step=deadline_step,
                    recover=False,
                )
                assert not recovery_started
                assert safe_completion == (inspection and not unsafe_entry)
                classification = "C" if unsafe_entry else "A" if inspection else "B"
                assert (classification == "A") == safe_completion
                counts[classification] += 1
                writer.writerow({
                    "scenario_id": scenario_id,
                    "delay_s": f"{delay:.1f}",
                    "deadline_s": MISSION_DEADLINE_S,
                    "classification": classification,
                    "inspection_completed": inspection,
                    "unsafe_entry": unsafe_entry,
                    "recovery_started": recovery_started,
                    "safe_completion": safe_completion,
                    "dispatch_step": dispatch_step,
                    "deadline_step": deadline_step,
                    **asdict(scenario),
                })

            total = sum(counts.values())
            assert total == SCENARIOS, (delay, counts)
            csv_file.flush()
            print(f"{delay:5.1f} {counts['A']:4d} {counts['B']:4d} {counts['C']:4d} {total:6d}")

    print(f"Verified A+B+C={SCENARIOS} at all {len(DELAY_GRID)} delays.")
    print(f"CSV: {args.csv}")


if __name__ == "__main__":
    main()
