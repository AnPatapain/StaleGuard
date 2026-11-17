"""E01: locked Underpass robustness evaluation, separate from the paper benchmark."""

from pathlib import Path
import os

os.environ['NUMBA_CACHE_DIR'] = str(
    Path(__file__).resolve().parents[2] / 'outputs/robustness_v1/.cache/numba'
)
os.environ['MPLCONFIGDIR'] = str(
    Path(__file__).resolve().parents[2] / 'outputs/robustness_v1/.cache/matplotlib'
)
