"""Historical first-feasible calibration, retained only for reproducibility."""

from experiment import CTRL_FREQ_HZ
from max_dwell_calibration import command_candidates
from mission import capture_approval
from staleguard_benefit import DEADLINES, run_actual


def plan_first_feasible_command(scenario):
    for candidate in command_candidates(scenario):
        snapshot = capture_approval(candidate)
        _, _, _, success = run_actual(
            candidate, snapshot,
            dispatch_step=round(candidate.approval_time * CTRL_FREQ_HZ),
            recover_step=0,
            deadline_step=round((candidate.approval_time + min(DEADLINES)) * CTRL_FREQ_HZ),
            recover=False,
        )
        if success:
            return candidate
    return None
