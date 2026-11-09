"""Plot retained actual policy flights; optionally replay one recorded action in GUI.

GUI playback repeats the recorded maneuver and physical start time, so a fresh
wall-clock measurement cannot silently retime the comparison. It is explanatory
playback of an already executed policy, never a new benchmark policy result.
"""
from pathlib import Path
import sys
if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
import fcntl
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from underpass_inspection.storage import ROOT,load_inputs,read_run,iter_committed
from underpass_inspection.scenario import Scenario
from underpass_inspection.simulation import execute
from underpass_inspection.baselines.policies import POLICIES


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,default=ROOT/'results/development/baseline_benchmark')
    p.add_argument('--seed',type=int)
    p.add_argument('--delay',type=float,default=.92)
    p.add_argument('--out',type=Path)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--policy',choices=POLICIES,default='staleguard_early_recovery')
    p.add_argument('--playback-speed',type=float,default=.5)
    p.add_argument('--hold',type=float,default=10.)
    p.add_argument('--capture',type=Path)
    a=p.parse_args();run=read_run(a.run)
    seed=a.seed if a.seed is not None else run['protocol']['representative_seeds'][0]
    s=Scenario(**next(v for v in run['protocol']['scenarios'] if v['seed']==seed))
    rows=[r for r in iter_committed(a.run,run) if r['seed']==seed and r['delay_s']==a.delay]
    if not rows:raise ValueError('No retained comparison at this seed/delay')
    figure,axes=plt.subplots(1,2,figsize=(12,5.5))
    for box,color in zip(s.obstacles,('#cba976','#80664e')):
        axes[0].add_patch(Rectangle((box.lower[0],box.lower[2]),box.upper[0]-box.lower[0],
            box.upper[2]-box.lower[2],color=color,alpha=.25))
        axes[1].add_patch(Rectangle((box.lower[0],-box.upper[1]),box.upper[0]-box.lower[0],
            box.upper[1]-box.lower[1],color=color,alpha=.20))
    colors=plt.get_cmap('tab10')
    with np.load(a.run/'representative_trajectories.npz') as archive:
        for i,row in enumerate(rows):
            policy=row['event']['policy'];prefix=f'seed{seed}_{round(a.delay*50):03d}_{policy}'
            raw=archive[prefix+'__physics']
            if hashlib.sha256(raw.tobytes()).hexdigest()!=row['physics_sha256']:
                raise ValueError('Saved physical trajectory hash mismatch')
            color=colors(POLICIES.index(policy));label=f'{policy} ({"safe" if row["safe"] else "unsafe"})'
            axes[0].plot(raw[:,1],raw[:,3],color=color,label=label,lw=1.6)
            axes[1].plot(raw[:,1],-raw[:,2],color=color,lw=1.6)
            step=row['initiation_step']
            if step is not None:
                q=raw[step*20];axes[0].scatter(q[1],q[3],color=color,s=20)
                axes[1].scatter(q[1],-q[2],color=color,s=20)
            if row['first_violation']:
                q=raw[row['first_violation']['physics_tick']]
                axes[0].scatter(q[1],q[3],color='red',marker='x',s=30)
                axes[1].scatter(q[1],-q[2],color='red',marker='x',s=30)
    axes[0].set(xlabel='North / world X (m)',ylabel='Altitude / world Z up (m)',title='Physical altitude profile')
    axes[1].set(xlabel='North / world X (m)',ylabel='East / negative world Y (m)',title='Physical Eastward avoidance')
    for ax in axes:ax.grid(alpha=.2)
    axes[0].legend(loc='lower left',fontsize=7)
    figure.suptitle(f'Actual PyBullet policy flights — seed {seed}, scheduled dispatch +{a.delay:.2f} s after approval')
    figure.text(.5,.015,'Dots: actual maneuver starts. Red crosses: first clearance violation. Coincident paths are not offset or smoothed.',ha='center',fontsize=8)
    figure.tight_layout(rect=(0,.035,1,.94))
    target=a.out or a.run/f'comparison_seed{seed}_{round(a.delay*50):03d}.png'
    target.parent.mkdir(parents=True,exist_ok=True);figure.savefig(target,dpi=160);plt.close(figure)
    print(f'Saved actual-flight comparison: {target}',flush=True)
    if a.gui:
        row=next(r for r in rows if r['event']['policy']==a.policy)
        case=next(case for case in load_inputs(a.run,run) if case[0].seed==seed)
        if case[0].__dict__!=s.__dict__:raise ValueError('GUI calibration differs from the recorded scenario')
        with (ROOT/'support/.simulation.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            actual,_=execute(s,mode=row['mode'],start_s=row['start_s'] or 0.,gui=True,
                playback_speed=a.playback_speed,hold_s=a.hold,capture=a.capture)
        if actual['physics_sha256']!=row['physics_sha256']:
            raise ValueError('GUI physical replay differs from recorded policy flight')
        print('GUI repeats the recorded policy action; physical trajectory hash matches.',flush=True)


if __name__=='__main__':main()
