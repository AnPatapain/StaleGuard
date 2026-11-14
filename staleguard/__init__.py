"""StaleGuard UAV prediction, assurance and reproducibility; original CPU/thread limits."""

import os

for name in (
    'OPENBLAS_NUM_THREADS',
    'OMP_NUM_THREADS',
    'MKL_NUM_THREADS',
    'NUMEXPR_NUM_THREADS',
    'VECLIB_MAXIMUM_THREADS',
    'NUMBA_NUM_THREADS',
):
    os.environ[name] = '1'
os.environ['CUDA_VISIBLE_DEVICES'] = ''
