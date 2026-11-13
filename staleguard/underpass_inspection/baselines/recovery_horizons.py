"""Shared command/completion/recovery prediction using unchanged PID/RK4 kernels.

Safety uses exactly the original swept control-endpoint box predicate. Inspection
residence uses swept box entry/exit on 100 Hz RK4 segments, not PyBullet labels.
Unsafe continuations may stop: whole-flight safety and safe completion can never
be restored after a violation. No monotonicity assumption skips candidate delays.
"""
import time
import math
import numpy as np
from numba import njit
from staleguard.model import MODEL_FREQ_HZ
from staleguard.model.lightweight import LightweightPredictiveModel,pid_step,rk4,control_step
from .staleguard import TTL_GRID,segment_box_clearance,segment_clearance,obstacle_arrays
from ..scenario import HZ,POS,BODY_RADIUS_M,apply_maneuver


def kernel_parameters(platform):
    q,p=platform
    pid=(q.mass,p.control_dt,p.kf,p.gravity,p.p_for,p.i_for,p.d_for,p.p_tor,p.i_tor,p.d_tor,
         p.pwm2rpm_scale,p.pwm2rpm_const,p.min_pwm,p.max_pwm,p.pos_integral_limit,
         p.z_integral_limit,p.attitude_integral_limit,p.roll_pitch_integral_limit,p.torque_limit,
         p.mixer_matrix,p.target_yaw)
    quad=(q.mass,q.inertia,q.inertia_inv,q.arm_length,q.gamma,q.gravity)
    return pid,quad


@njit(cache=True)
def inspection_segment(a,b,lower,upper,dwell,complete,dt,required):
    enter=0.;leave=1.
    for j in range(3):
        x=a[2*j];v=b[2*j]-x
        if v==0.:
            if x<lower[j] or x>upper[j]:return 0.,complete
        else:
            t0=(lower[j]-x)/v;t1=(upper[j]-x)/v
            enter=max(enter,min(t0,t1));leave=min(leave,max(t0,t1))
    if enter>leave:return 0.,complete
    if enter>0.:dwell=0.
    dwell+=(leave-enter)*dt
    complete=complete or dwell+1e-12>=required
    if leave<1.:dwell=0.
    return dwell,complete


@njit(cache=True)
def segment_safe(a,b,boxes,clearance,broadphase):
    pa=np.array([a[0],a[2],a[4]]);pb=np.array([b[0],b[2],b[4]])
    # A separating coordinate is a sufficient Euclidean clearance certificate.
    # Near its boundary or rounded corners the exact original checker is used.
    for box in boxes:
        lower,upper=box[0],box[1];separated=False
        if broadphase:
            radius=BODY_RADIUS_M+clearance
            for j in range(3):
                if min(pa[j],pb[j])>upper[j]+radius+1e-12 or max(pa[j],pb[j])<lower[j]-radius-1e-12:
                    separated=True;break
        if not separated and segment_box_clearance(pa,pb,lower,upper)<clearance:return False
    return True


@njit(cache=True)
def continuation(state,memory,reference,start,pid,quad,boxes,clearance,lower,upper,required,
                 prefix_safe,dwell,complete,early_exit,broadphase,inspect):
    state=state.copy();memory=memory.copy();safe=prefix_safe;steps=0
    if early_exit and not safe:return False,False,state,memory,steps
    for step in range(start,len(reference)-1):
        previous=state
        action=pid_step(state,reference[step],memory[0],memory[1],memory[2],*pid)
        if inspect and not complete:
            for _ in range(MODEL_FREQ_HZ//HZ):
                before=state
                state=rk4(state,action,1./MODEL_FREQ_HZ,*quad)
                dwell,complete=inspection_segment(before,state,lower,upper,dwell,complete,1./MODEL_FREQ_HZ,required)
        else:
            state=control_step(state,action,1./MODEL_FREQ_HZ,MODEL_FREQ_HZ//HZ,*quad)
        steps+=1
        safe=safe and segment_safe(previous,state,boxes,clearance,broadphase)
        if early_exit and not safe:return False,False,state,memory,steps
    return safe,safe and complete,state,memory,steps


def _boundary(results,field):
    bad=next((i for i,r in enumerate(results) if not r[field]),None)
    return dict(first_invalid_steps=None if bad is None else round(results[bad]['delay_s']*HZ),
        last_safe_prefix_steps=None if bad==0 else round(results[bad-1]['delay_s']*HZ) if bad is not None else None,
        last_tested_safe_steps=max((round(r['delay_s']*HZ) for r in results if r[field]),default=None),
        right_censored=bad is None,
        non_monotonic=any(not a[field] and b[field] for a,b in zip(results,results[1:])))


def completion_expiry_steps(rows):
    """First sample after the last feasible completion interval, if observed."""
    last_success=max((i for i,row in enumerate(rows) if row['command_complete']),default=-1)
    # The next sample closes the final feasible interval. A feasible final
    # sample leaves the completion cutoff unobserved beyond the grid.
    return round(rows[last_success+1]['delay_s']*HZ) if last_success+1<len(rows) else None


def policy_plans(forecast):
    """Build intervention timers on the sampled grid from the saved labels."""
    rows=forecast['results']
    completion=completion_expiry_steps(rows)
    recovery=forecast['boundaries']['recovery']
    recovery_lost=recovery['first_invalid_steps'] is not None
    last_recovery=recovery['last_safe_prefix_steps']
    recovery_trigger=(last_recovery or 0) if recovery_lost else None
    verified_through=last_recovery if recovery_lost else round(rows[-1]['delay_s']*HZ)
    shared=[('command_expiry',forecast['expiry_steps']),('recovery_prefix_end',recovery_trigger)]
    plans={}
    for name,triggers in (('staleguard_recovery_aware',shared),
                          ('staleguard_dual_horizon',shared+[('completion_expiry',completion)])):
        step=min((value for _,value in triggers if value is not None),default=None)
        plans[name]=dict(trigger_steps=step,reasons=[reason for reason,value in triggers if value is not None and value==step],
                         recovery_verified_through_steps=verified_through)
    return plans


def predict(snapshot,platform,*,backend='compiled',early_exit=True,broadphase=True,diagnostics=False):
    """Atomic publication: all profiles/extraction/cleanup count toward readiness."""
    began_wall=time.time();began=time.perf_counter()
    if backend not in ('compiled','reference'):raise ValueError(backend)
    pid,quad=kernel_parameters(platform);s=snapshot.scenario
    python_boxes=obstacle_arrays(s)
    boxes=np.asarray([[b.lower,b.upper] for b in s.obstacles],dtype=float)
    lower=np.array(s.inspection_lower);upper=np.array(s.inspection_upper)
    model=LightweightPredictiveModel(*platform);checkpoints={};profiles=[];trace_checks=[]
    try:
        ctrl,obs,info,reference=model.start(snapshot)
        wanted={round(d*HZ) for d in TTL_GRID};prefix=min(snapshot.prefix_min_clearance_m,segment_clearance(obs,obs,python_boxes))>=s.clearance_m
        dwell=0.;complete=False
        for step in range(max(wanted)+1):
            if step in wanted:
                checkpoints[step]=(model.save_checkpoint(ctrl,obs,info),prefix,dwell,complete)
            if step==max(wanted):break
            action=ctrl.select_action(obs,info);previous=obs
            for _ in range(model.substeps):
                before=obs;obs=rk4(obs,action,model.dt,*quad)
                dwell,complete=inspection_segment(before,obs,lower,upper,dwell,complete,model.dt,s.inspection_dwell_s)
            info={'current_step':step+1}
            prefix=prefix and segment_clearance(previous,obs,python_boxes)>=s.clearance_m
        recovery_finished=None
        # Recovery first is recorded, but policies do not consume partial results.
        for mode in ('sideways','dive'):
            for i,delay in enumerate(TTL_GRID):
                step=round(delay*HZ);checkpoint,prefix,dwell,complete=checkpoints[step]
                obs,info=model.restore_checkpoint(ctrl,checkpoint)
                changed=apply_maneuver(reference,step,obs,s,mode)
                if backend=='compiled':
                    memory=np.array([checkpoint.pid_state.integral_pos_e,checkpoint.pid_state.last_rpy,checkpoint.pid_state.integral_rpy_e])
                    safe,completed,end,mem,nsteps=continuation(obs,memory,changed,step,pid,quad,boxes,s.clearance_m,
                        lower,upper,s.inspection_dwell_s,prefix,dwell,complete,early_exit,broadphase,mode=='dive')
                else:
                    model.set_reference(ctrl,changed);safe=prefix;nsteps=0
                    if not early_exit or safe:
                        for tick in range(step,snapshot.horizon_steps):
                            action=ctrl.select_action(obs,{'current_step':tick});previous=obs
                            for _ in range(model.substeps):
                                before=obs;obs=rk4(obs,action,model.dt,*quad)
                                dwell,complete=inspection_segment(before,obs,lower,upper,dwell,complete,model.dt,s.inspection_dwell_s)
                            nsteps+=1;safe=safe and segment_clearance(previous,obs,python_boxes)>=s.clearance_m
                            if early_exit and not safe:break
                    completed=safe and complete;end=obs
                    mem=np.array([ctrl.integral_pos_e,ctrl.last_rpy,ctrl.integral_rpy_e])
                if mode=='sideways':profiles.append(dict(delay_s=delay,recovery_safe=bool(safe)))
                else:profiles[i].update(safe=bool(safe),command_safe=bool(safe),command_complete=bool(completed))
                trace=dict(mode=mode,delay_s=delay,control_steps=int(nsteps))
                if diagnostics:trace.update(terminal_state=end.tolist(),terminal_pid=mem.tolist())
                trace_checks.append(trace)
            if mode=='sideways':recovery_finished=time.perf_counter()-began
    finally:model.close(checkpoints)
    boundaries={name:_boundary(profiles,field) for name,field in
        (('command','command_safe'),('completion','command_complete'),('recovery','recovery_safe'))}
    result=dict(results=profiles,boundaries=boundaries,grid_s=list(TTL_GRID),
        expiry_steps=boundaries['command']['first_invalid_steps'],
        non_monotonic=boundaries['command']['non_monotonic'],
        any_profile_non_monotonic=any(v['non_monotonic'] for v in boundaries.values()),
        right_censored=boundaries['command']['right_censored'],backend=backend,
        early_exit=early_exit,broadphase=broadphase,rollouts=trace_checks,
        recovery_profile_finished_s=recovery_finished,
        completion_model='Continuous residence on piecewise-linear 100 Hz RK4 segments; same inspection box/dwell/horizon',
        publication='Atomic: all profiles and boundaries ready together; partial recovery results are not consumed')
    result['policy_plans']=policy_plans(result)
    result['computation_s']=time.perf_counter()-began
    result['wall_started_unix_s']=began_wall;result['wall_finished_unix_s']=time.time()
    return result
