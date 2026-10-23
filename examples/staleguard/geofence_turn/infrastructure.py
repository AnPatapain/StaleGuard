"""Reuse existing vehicle/PID models without copying or changing Mission A.

The legacy infrastructure uses top-level imports and lives in Mission A's model/
folder. This compatibility path exposes that existing infrastructure; no Mission
A calibration, runner, geometry or policy logic is used by this mission.
"""
import sys
from pathlib import Path

LEGACY = Path(__file__).resolve().parents[1] / "inspection_descent"
if str(LEGACY) not in sys.path:
    sys.path.append(str(LEGACY))

from model.common import capture_pid_state, restore_pid_state, PIDState  # noqa: E402
from model.lightweight import LightweightPredictiveModel  # noqa: E402
from model.scg import load_scg_platform_params  # noqa: E402
