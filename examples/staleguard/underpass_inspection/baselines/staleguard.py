"""Geofence's two-stage approval forecast with Underpass references/3D clearance.

Shared LightweightPredictiveModel, checkpoint/PID restoration, fixed candidate
grid, first-invalid extraction and readiness rule are retained. No dispatch
schedule or offline physical deadline is an input. World coordinates stay Z-up.
"""
import math
import time
import numpy as np
from numba import njit
from model.lightweight import LightweightPredictiveModel
from ..scenario import HZ, POS, BODY_RADIUS_M, apply_maneuver


# Frozen prediction grid; distinct from the 51-point physical dispatch grid.
TTL_GRID = (
    0., .02, .04, .06, .08, .10, .12, .14, .16, .18, .20,
    .30, .40, .50, .60, .70, .80, .90, 1.,
)


@njit(cache=True)
def segment_box_clearance(a, b, lower, upper):
    """Minimum signed point/box distance on a straight segment, minus body R.

    Outside: squared Euclidean distance is quadratic between slab crossings.
    Inside: maximum nearest-face depth occurs at an endpoint or intersection
    of two affine face-distance functions. No expanded rectangular approximation
    replaces the rounded Euclidean body-clearance envelope.
    """
    velocity = b-a
    breaks = np.empty(8)
    breaks[0], breaks[1] = 0., 1.
    count = 2
    enter, leave = 0., 1.
    intersects = True
    for j in range(3):
        if velocity[j] != 0.:
            t0 = (lower[j]-a[j])/velocity[j]
            t1 = (upper[j]-a[j])/velocity[j]
            enter = max(enter, min(t0,t1)); leave = min(leave,max(t0,t1))
            for t in (t0,t1):
                if 0. < t < 1.:
                    breaks[count] = t; count += 1
        elif a[j] < lower[j] or a[j] > upper[j]:
            intersects = False
    if intersects and enter <= leave:
        c = np.empty(6); v = np.empty(6)
        for j in range(3):
            c[2*j], v[2*j] = a[j]-lower[j], velocity[j]
            c[2*j+1], v[2*j+1] = upper[j]-a[j], -velocity[j]
        depth = 0.
        for i in range(6):
            for k in range(i+1,6):
                if v[i] != v[k]:
                    t = (c[k]-c[i])/(v[i]-v[k])
                    if enter <= t <= leave:
                        depth = max(depth,np.min(c+v*t))
        depth = max(depth,np.min(c+v*enter),np.min(c+v*leave))
        return -depth-BODY_RADIUS_M
    ordered = np.sort(breaks[:count])
    minimum = math.inf
    for i in range(len(ordered)-1):
        left, right = ordered[i], ordered[i+1]
        middle = (left+right)/2
        quadratic, linear = 0., 0.
        for j in range(3):
            x = a[j]+velocity[j]*middle
            if x < lower[j] or x > upper[j]:
                edge = lower[j] if x < lower[j] else upper[j]
                quadratic += velocity[j]**2
                linear += velocity[j]*(a[j]-edge)
        optimum = min(right,max(left,-linear/quadratic)) if quadratic else left
        for t in (left,optimum,right):
            square = 0.
            for j in range(3):
                x = a[j]+velocity[j]*t
                distance = max(lower[j]-x,x-upper[j],0.)
                square += distance*distance
            minimum = min(minimum,square)
    return math.sqrt(minimum)-BODY_RADIUS_M


def obstacle_arrays(s):
    return tuple((np.array(b.lower),np.array(b.upper)) for b in s.obstacles)


def segment_clearance(previous, current, boxes):
    a, b = previous[POS], current[POS]
    return min(float(segment_box_clearance(a,b,lo,hi)) for lo,hi in boxes)


def predict_delays(snapshot, platform, delays=TTL_GRID):
    """Waiting checkpoints once; same relative dive from each predicted state."""
    if (not delays or len(set(delays))!=len(delays) or list(delays)!=sorted(delays)
            or any(not math.isfinite(d) or d<0 or round(d*HZ)>=snapshot.horizon_steps
                   or not math.isclose(d*HZ,round(d*HZ),abs_tol=1e-9) for d in delays)):
        raise ValueError('Use sorted distinct candidate delays on the control grid')
    began_wall=time.time();began=time.perf_counter(); checkpoints={}
    model=LightweightPredictiveModel(*platform)
    s=snapshot.scenario; boxes=obstacle_arrays(s)
    try:
        ctrl,obs,info,reference=model.start(snapshot)
        wanted={round(d*HZ) for d in delays}
        minimum=min(snapshot.prefix_min_clearance_m,segment_clearance(obs,obs,boxes))
        for step in range(max(wanted)+1):
            if step in wanted:
                checkpoints[step]=(model.save_checkpoint(ctrl,obs,info),minimum)
            if step==max(wanted):
                break
            action=ctrl.select_action(obs,info);previous=obs
            obs,_,info=model.step(action,obs,info)
            minimum=min(minimum,segment_clearance(previous,obs,boxes))
        results=[]
        for delay in delays:
            step=round(delay*HZ); checkpoint,minimum=checkpoints[step]
            obs,info=model.restore_checkpoint(ctrl,checkpoint)
            changed=apply_maneuver(reference,step,obs,s,'dive')
            model.set_reference(ctrl,changed)
            for _ in range(step,snapshot.horizon_steps):
                action=ctrl.select_action(obs,info);previous=obs
                obs,_,info=model.step(action,obs,info)
                minimum=min(minimum,segment_clearance(previous,obs,boxes))
            results.append(dict(delay_s=float(delay),safe=bool(minimum>=s.clearance_m),
                                min_clearance_m=minimum))
        bad=next((r for r in results if not r['safe']),None)
        expiry=None if bad is None else round(bad['delay_s']*HZ)
        elapsed=time.perf_counter()-began
        return dict(results=results,expiry_steps=expiry,grid_s=list(delays),
            right_censored=bad is None,
            non_monotonic=any(not a['safe'] and b['safe'] for a,b in zip(results,results[1:])),
            computation_s=elapsed,wall_started_unix_s=began_wall,wall_finished_unix_s=time.time())
    finally:
        model.close(checkpoints)


def staleguard_status(received, waited_steps):
    ready = received['computation_s'] <= waited_steps/HZ
    expired = received['expiry_steps'] is not None and waited_steps >= received['expiry_steps']
    return ready, ready and expired


def first_ready_step(computation_s):
    """Match the readiness comparison exactly, including floating-point ties."""
    step=math.ceil(computation_s*HZ)
    while step and computation_s<=(step-1)/HZ:step-=1
    while computation_s>step/HZ:step+=1
    return step
