import numpy as np

CTRL_FREQ_HZ = 50 # PID runs 50 steps/second
PYB_FREQ_HZ = 1000  # Physics env evolves 1000 steps/second
MODEL_FREQ_HZ = 100 # Staleguard model predicts env evolution 100 steps/second

EPISODE_SECONDS = 6 # Experiment duration
MEASURE_SECONDS = 5.5 # Only this portion of episode is evaluated (0.5s serves as margin)

DELAY_GRID = tuple(np.arange(0.0, 1.51, 0.1)) # 0.0, 0.1, 0.2, ..., 1.5

SIM_SEED = 42 # Reproducible simulator reset