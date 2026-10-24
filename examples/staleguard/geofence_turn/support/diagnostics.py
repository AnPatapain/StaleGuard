"""Render saved northbound development measurements; runs no simulations."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from ..scenario import BODY_RADIUS_M, HZ, Config


def render(logs):
    logs = Path(logs)
    out = logs/'diagnostics'
    out.mkdir(exist_ok=True)
    summary = json.loads((logs/'development/summary.json').read_text())
    selected = summary['scenarios'][0]['selected']
    seed = selected['seed']
    trace = np.load(logs/f'development/seed_{seed}/d0_trace.npz')['controls']
    fig, ax = plt.subplots(1,2,figsize=(10,4),constrained_layout=True)
    ax[0].plot(trace[:,1],trace[:,3],label='Actual PyBullet center')
    ax[0].plot(trace[:,13],trace[:,15],'--',label='Quarter-circle + straight north')
    ax[0].axvline(selected['fence_x'],color='black',label='Fence')
    ax[0].axvline(selected['fence_x']-BODY_RADIUS_M-Config().d_min_m,color='tab:red',ls=':',
                  label='Center limit: body + d_min')
    lower,upper = selected['command']['checkpoint_lower'],selected['command']['checkpoint_upper']
    ax[0].add_patch(Rectangle(lower[:2],upper[0]-lower[0],upper[1]-lower[1],
                              fill=False,color='tab:green',label='Fixed checkpoint region'))
    ax[0].set(xlabel='X east (m)',ylabel='Y north (m)',title=f'Seed {seed}: D=0')
    ax[0].axis('equal')
    ax[0].legend(fontsize=7)
    ax[1].plot(trace[:,0]/HZ-selected['approval_s'],selected['fence_x']-trace[:,1]-BODY_RADIUS_M)
    ax[1].axhline(Config().d_min_m,color='tab:red',ls=':',label='Required d_min')
    ax[1].set(xlabel='Time since approval (s)',ylabel='Body clearance (m)',title='Actual states (50 Hz plot; 1 kHz monitor)')
    ax[1].legend(fontsize=8)
    fig.savefig(out/'northbound_checkpoint.png',dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--logs',default=str(Path(__file__).resolve().parent/'logs/northbound_review'))
    render(parser.parse_args().logs)
