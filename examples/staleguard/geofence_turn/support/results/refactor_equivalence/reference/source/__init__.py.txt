"""Independent geofence emergency-turn mission; no Mission A behavior changes."""
import os

# Serial, CPU-only execution: set before importing numerical libraries.
for _name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
              "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMBA_NUM_THREADS"):
    os.environ[_name] = "1"
os.environ["CUDA_VISIBLE_DEVICES"] = ""

os.environ.setdefault("MPLCONFIGDIR", "/tmp/staleguard-geofence-matplotlib")
