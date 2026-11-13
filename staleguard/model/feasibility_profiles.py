"""Three opportunity rollouts for mission adapters, using the existing PID/RK4.

The decision state machine is reused directly from Underpass. This module only
adapts the original plane/closed-volume predicates and completion regions. All
properties cover the unchanged operational horizon; no PyBullet labels enter.
"""
import time
import math
import numpy as np
from numba import njit
from staleguard.model.lightweight import LightweightPredictiveModel,pid_step,rk4
from staleguard.underpass_inspection.baselines.recovery_horizons import kernel_parameters,_boundary,policy_plans

@njit(cache=True)
def safe_segment(a,b,kind,g):
    if kind==0: # Geofence: body clearance from the plane, equality is safe.
        return min(g[0]-a[0]-g[1],g[0]-b[0]-g[1])>=g[2]
    enter=0.;leave=1.
    for j in range(3):
        x=a[2*j];v=b[2*j]-x;lo=g[2*j];hi=g[2*j+1]
        if v==0.:
            if x<lo or x>hi:return True
        else:
            t0=(lo-x)/v;t1=(hi-x)/v
            enter=max(enter,min(t0,t1));leave=min(leave,max(t0,t1))
            if enter>leave:return True
    return False

@njit(cache=True)
def residence(a,b,lo,hi,target,tolerance,speed_required,strict,dwell,complete,dt,required):
    enter=0.;leave=1.
    for j in range(3):
        x=a[2*j];v=b[2*j]-x
        if v==0.:
            if (x<=lo[j] or x>=hi[j]) if strict else (x<lo[j] or x>hi[j]):return 0.,complete
        else:
            t0=(lo[j]-x)/v;t1=(hi[j]-x)/v
            enter=max(enter,min(t0,t1));leave=min(leave,max(t0,t1))
    if speed_required:
        qa=0.;qb=0.;qc=-tolerance*tolerance
        for j in range(3):
            x=a[2*j+1]-target[j];v=b[2*j+1]-a[2*j+1]
            qa+=v*v;qb+=2*x*v;qc+=x*x
        if qa==0.:
            if qc>0:return 0.,complete
        else:
            discriminant=qb*qb-4*qa*qc
            if discriminant<0:return 0.,complete
            root=math.sqrt(max(0.,discriminant))
            enter=max(enter,(-qb-root)/(2*qa));leave=min(leave,(-qb+root)/(2*qa))
    if enter>leave:return 0.,complete
    if enter>0.:dwell=0.
    dwell+=(leave-enter)*dt
    complete=complete or dwell+1e-12>=required
    if leave<1.:dwell=0.
    return dwell,complete

@njit(cache=True)
def advance(state,memory,reference,step,pid,quad,kind,g,lo,hi,target,tolerance,speed_required,strict,dwell,complete,required,deadline,inspect):
    previous=state
    action=pid_step(state,reference[step],memory[0],memory[1],memory[2],*pid)
    for sub in range(2):
        before=state;state=rk4(state,action,.01,*quad)
        if inspect and not complete:
            dt=min(.01,max(0.,deadline-step/50.-sub*.01))
            if dt>0:
                end=state if dt>=.01 else before+(state-before)*(dt/.01)
                dwell,complete=residence(before,end,lo,hi,target,tolerance,speed_required,strict,dwell,complete,dt,required)
    return state,safe_segment(previous,state,kind,g),dwell,complete

@njit(cache=True)
def waiting(state,memory,reference,steps,pid,quad,kind,g,lo,hi,target,tolerance,speed_required,strict,required,deadline,prefix_safe):
    states=np.empty((len(steps),12));memories=np.empty((len(steps),3,3));prefixes=np.empty(len(steps),np.bool_)
    dwells=np.empty(len(steps));completions=np.empty(len(steps),np.bool_)
    index=0;dwell=0.;complete=False
    for step in range(steps[-1]+1):
        if step==steps[index]:
            states[index]=state;memories[index]=memory;prefixes[index]=prefix_safe;dwells[index]=dwell;completions[index]=complete
            index+=1
            if index==len(steps):break
        state,safe,dwell,complete=advance(state,memory,reference,step,pid,quad,kind,g,lo,hi,target,tolerance,speed_required,strict,dwell,complete,required,deadline,True)
        prefix_safe=prefix_safe and safe
    return states,memories,prefixes,dwells,completions

@njit(cache=True)
def continuation(state,memory,reference,start,pid,quad,kind,g,lo,hi,target,tolerance,speed_required,strict,required,deadline,prefix_safe,dwell,complete,inspect,early_exit):
    safe=prefix_safe
    if early_exit and not safe:return False,False,state,memory
    for step in range(start,len(reference)-1):
        state,current,dwell,complete=advance(state,memory,reference,step,pid,quad,kind,g,lo,hi,target,tolerance,speed_required,strict,dwell,complete,required,deadline,inspect)
        safe=safe and current
        if early_exit and not safe:return False,False,state,memory
    return safe,safe and complete,state,memory


def finalize(profiles,grid):
    """Extract sampled boundaries and build the shared intervention timers."""
    boundaries={name:_boundary(profiles,field) for name,field in
        (('command','command_safe'),('completion','command_complete'),('recovery','recovery_safe'))}
    result=dict(results=profiles,boundaries=boundaries,grid_s=list(grid),expiry_steps=boundaries['command']['first_invalid_steps'])
    result['policy_plans']=policy_plans(result)
    return result


def predict(snapshot,platform,adapter,*,early_exit=True,diagnostics=False):
    began=time.perf_counter();wall=time.time()
    pid,quad=kernel_parameters(platform);grid=adapter['grid'];steps=np.array([round(d*50) for d in grid],np.int64)
    reference=np.asarray(snapshot.remaining_reference[:adapter['horizon_steps']+1],float).copy()
    assert len(reference)==adapter['horizon_steps']+1
    memory=np.array([snapshot.pid_state.integral_pos_e,snapshot.pid_state.last_rpy,snapshot.pid_state.integral_rpy_e],float)
    args=(pid,quad,adapter['kind'],np.array(adapter['geometry']),np.array(adapter['lower']),np.array(adapter['upper']),
        np.array(adapter['target']),adapter['tolerance'],adapter['speed_required'],adapter['strict'],adapter['dwell_s'],adapter['deadline_s'])
    states,memories,prefixes,dwells,completes=waiting(np.array(snapshot.state,float),memory,reference,steps,*args,adapter['prefix_safe'])
    rows=[dict(delay_s=float(d)) for d in grid];traces=[]
    # Recovery first; publication remains atomic after all continuations/cleanup.
    for recovery in (True,False):
        for i,step in enumerate(steps):
            changed=adapter['reference'](reference,int(step),states[i],recovery)
            safe,complete,state,mem=continuation(states[i].copy(),memories[i].copy(),changed,int(step),*args,
                prefixes[i],dwells[i],completes[i],not recovery,early_exit)
            if recovery:rows[i]['recovery_safe']=bool(safe)
            else:rows[i].update(safe=bool(safe),command_safe=bool(safe),command_complete=bool(complete))
            if diagnostics:traces.append(dict(recovery=recovery,delay_s=float(grid[i]),state=state.tolist(),memory=mem.tolist()))
    result=finalize(rows,grid)
    result.update(computation_s=time.perf_counter()-began,wall_started_unix_s=wall,wall_finished_unix_s=time.time(),
        publication='Atomic; all profiles, boundary extraction and reference construction included',
        horizon_s=adapter['deadline_s'],model_hz=100,control_hz=50)
    if diagnostics:result['diagnostics']=traces
    return result
