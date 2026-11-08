"""Seven single-command policies. This object never receives a dispatch schedule.

The physical driver announces dispatch only when it occurs. Pending timers run
from elapsed approval time. Decision rules and timers live in this mission; the physical predictor uses
the shared lightweight dynamics model.
"""
import math
import time
from . import fbs as freshness, fixed_early_recovery
from .staleguard import staleguard_status
from .use_time_recheck import first_step_after_check
from ..scenario import HZ
from .fbs import safety_feature
from .use_time_recheck import predict_immediate

POLICIES=('no_recovery','always_recover','fbs','use_time_recheck','staleguard',
          'staleguard_early_recovery','fixed_early_recovery')
APPROVAL_POLICIES=('staleguard','staleguard_early_recovery')


class PendingCommand:
    def __init__(self, name, scenario, received=None):
        if name not in POLICIES:
            raise ValueError(name)
        if name in APPROVAL_POLICIES and received is None:
            raise ValueError('Approval-only forecast required')
        self.name,self.scenario,self._forecast=name,scenario,received
        self.approval=round(scenario.approval_s*HZ)
        self.phase='before_approval';self._action=None;self.ready_announced=False
        self.previous=None;self.volatility=0.;self.features=[];self.observation_costs=[]
        self.events=[]
        self.event=dict(policy=name,application_step=None,fallback_step=None,
            early_recovery=False,prediction_ready=None,command_rejected=False,
            command_canceled=False,dispatch_observed_step=None)

    def log(self, kind, step, **data):
        self.events.append(dict(kind=kind,control_step=step,flight_s=step/HZ,
                                approval_s=(step-self.approval)/HZ,**data))

    def observe(self, step, state):
        if self.name!='fbs' or step>self.approval:
            return
        began=time.perf_counter()
        feature=safety_feature(state,self.scenario)
        self.volatility=freshness.update_volatility(feature,self.previous,self.volatility)
        self.previous=feature
        if step==self.approval:
            self.event['prediction']=freshness.freeze_prediction(feature,self.volatility)
        elapsed=time.perf_counter()-began
        self.features.append(feature);self.observation_costs.append(elapsed)
        if step==self.approval:
            self.event.update(computation_s=elapsed,observation_step=step,cold_start=step==0,
                observation_steps=list(range(step+1)),features_m=self.features,
                observation_update_s=self.observation_costs)

    def approve(self):
        self.phase='pending';self.log('approval',self.approval)
        if self.name in APPROVAL_POLICIES:
            runtime=self._forecast['computation_s']
            self.event.update(computation_s=runtime,
                prediction_start_flight_s=self.approval/HZ,
                prediction_completion_flight_s=self.approval/HZ+runtime,
                prediction_wall_started_unix_s=self._forecast.get('wall_started_unix_s'),
                prediction_wall_finished_unix_s=self._forecast.get('wall_finished_unix_s'),
                expiry_steps=self._forecast['expiry_steps'])
            self.log('prediction_started',self.approval)
        if self.name=='fixed_early_recovery':
            self.event['fixed_early_ttl_s']=fixed_early_recovery.FIXED_EARLY_TTL_S
        if self.name=='always_recover':
            self._start('sideways',self.approval,'approval')

    def announce_readiness(self, step):
        if self.name not in APPROVAL_POLICIES or self.ready_announced:
            return
        if self._forecast['computation_s'] <= (step-self.approval)/HZ:
            self.ready_announced=True
            self.event['prediction_ready_step']=step
            self.log('prediction_ready',step)

    def _start(self, action, step, reason):
        if self.phase not in ('pending','checking'):
            raise ValueError('A command/recovery cannot be restarted or overridden')
        self._action=action
        self.phase='recovery' if action=='sideways' else 'command'
        self.event['fallback_step' if action=='sideways' else 'application_step']=step
        if action=='sideways':
            self.event.update(command_rejected=True,command_canceled=True,recovery_reason=reason)
        self.log('recovery_started' if action=='sideways' else 'command_started',step,
                 maneuver=action,reason=reason)

    def dispatch(self, step, snapshot, platform):
        # Called by the driver ONLY at the actual event, never with future age.
        self.event['dispatch_observed_step']=step
        if self.name in APPROVAL_POLICIES:
            self.event['prediction_ready_at_dispatch']=self.ready_announced
        self.log('command_dispatch',step,canceled=self.phase=='recovery')
        if self.phase!='pending':
            self.log('dispatch_ignored',step)
            return
        waited=step-self.approval;recover=False
        if self.name in APPROVAL_POLICIES:
            ready,expired=staleguard_status(self._forecast,waited)
            self.event['prediction_ready']=ready;recover=expired
        elif self.name=='fbs':
            status=freshness.dispatch_status(self.event['prediction'],self.event['computation_s'],waited)
            self.event.update(status,prediction_ready=status['ready'],prediction_ready_at_dispatch=status['ready']);recover=status['recover']
        elif self.name=='fixed_early_recovery':
            recover=fixed_early_recovery.dispatch_recovery_due(waited)
        elif self.name=='use_time_recheck':
            began_wall=time.time();began=time.perf_counter()
            result=predict_immediate(snapshot,platform)
            runtime=time.perf_counter()-began
            self.phase='checking'
            self.event.update(check=result,computation_s=runtime,prediction_ready=False,prediction_ready_at_dispatch=False,
                check_state=snapshot.state.tolist(),check_started_step=step,
                prediction_start_flight_s=step/HZ,prediction_completion_flight_s=step/HZ+runtime,
                prediction_wall_started_unix_s=began_wall,prediction_wall_finished_unix_s=time.time(),
                decision_step=first_step_after_check(step,runtime))
            self.log('recheck_started',step)
            return
        self._start('sideways' if recover else 'dive',step,'dispatch')

    def tick(self, step):
        if self.phase=='checking':
            if step>=self.event['decision_step']:
                self.event['prediction_ready']=True
                self.log('recheck_ready',step)
                self._start('dive' if self.event['check']['safe'] else 'sideways',step,'recheck')
            return
        if self.phase!='pending':
            return
        waited=step-self.approval
        if self.name=='staleguard_early_recovery':
            ready,expired=staleguard_status(self._forecast,waited)
            if ready and expired:
                self.event['prediction_ready']=True
                self.log('predicted_ttl_reached',step)
                self._start('sideways',step,'predicted_expiry')
        elif self.name=='fixed_early_recovery' and fixed_early_recovery.pending_recovery_due(waited):
            self.log('fixed_timer_expired',step)
            self._start('sideways',step,'fixed_timer')

    def take_action(self):
        action=self._action;self._action=None
        return action

    def finish(self, scheduled_dispatch_step, total_step):
        # Scheduling metadata is attached AFTER the flight; it is never an input
        # to approve(), tick(), dispatch() or the predictor.
        event=self.event.copy()
        event['scheduled_dispatch_step']=scheduled_dispatch_step
        event['early_recovery']=(event['fallback_step'] is not None and
                                 event['fallback_step']<scheduled_dispatch_step)
        event['recovery_at_dispatch']=event['fallback_step']==scheduled_dispatch_step
        event['prediction_available_before_dispatch']=(
            self._forecast['computation_s']<= (scheduled_dispatch_step-self.approval)/HZ
            if self.name in APPROVAL_POLICIES else None)
        if self.name in APPROVAL_POLICIES:
            expiry=self._forecast['expiry_steps']
            event['prediction_available_at_expiry']=(None if expiry is None else
                self._forecast['computation_s']<=expiry/HZ)
            event['prediction_ready_control_step']=self.approval+math.ceil(self._forecast['computation_s']*HZ)
        event['events']=self.events+[dict(kind='observation_horizon_end',control_step=total_step,
            flight_s=total_step/HZ,approval_s=(total_step-self.approval)/HZ)]
        return event
