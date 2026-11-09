"""Two exploratory policies; existing seven policy implementations are untouched.

Dispatch wins a timer tie. RecoveryAware uses the existing command-safety gate at
actual dispatch. DualHorizon adds completion rejection only when recovery remains
inside its conservatively verified prefix. Proactive recovery uses the earliest
command-expiry / last-safe recovery-prefix trigger (plus completion for Dual).
A missed recovery prefix never makes an otherwise safe command inadmissible.
When command safety is already expired, the unchanged fallback is best-effort.
All-safe profiles are censored, not fabricated deadlines at the grid endpoint.
"""
from .policies import PendingCommand
from .staleguard import staleguard_status
from ..scenario import HZ

POLICIES=('staleguard_recovery_aware','staleguard_dual_horizon')


class HorizonCommand(PendingCommand):
    def __init__(self,name,scenario,received=None):
        if name not in POLICIES:raise ValueError(name)
        # Reuse lifecycle, readiness, cancellation and event recording exactly.
        super().__init__('staleguard_early_recovery',scenario,received)
        self.variant=name;self.event['policy']=name
        r=received['boundaries']['recovery'];c=received['boundaries']['completion']
        plan=received['policy_plans'][name]
        self.trigger=plan['trigger_steps'];self.reasons=plan['reasons']
        self.recovery_verified_through=plan['recovery_verified_through_steps']
        self.event.update(policy=name,recovery_expiry_steps=r['first_invalid_steps'],
            recovery_verified_through_steps=self.recovery_verified_through,
            completion_expiry_steps=c['first_invalid_steps'],early_trigger_steps=self.trigger,
            early_trigger_reasons=self.reasons,prediction_kind='command_completion_recovery')
        self.missed_announced=False

    def approve(self):
        super().approve()
        self.event['prediction_available_by_trigger']=(None if self.trigger is None else
            self._forecast['computation_s']<=self.trigger/HZ)
        self.event['prediction_available_by_recovery_prefix']=(None if self.recovery_verified_through is None else
            self._forecast['computation_s']<=self.recovery_verified_through/HZ)

    def _recovery_verified(self,age):
        return self.recovery_verified_through is not None and age<=self.recovery_verified_through

    def _recover(self,step,reason):
        verified=self._recovery_verified(step-self.approval)
        self.event.update(predicted_recovery_verified_at_start=verified,best_effort_recovery=not verified)
        self._start('sideways',step,reason)

    def dispatch(self,step,snapshot,platform):
        self.event['dispatch_observed_step']=step
        self.event['prediction_ready_at_dispatch']=self.ready_announced
        self.log('command_dispatch',step,canceled=self.phase=='recovery')
        if self.phase!='pending':self.log('dispatch_ignored',step);return
        age=step-self.approval;ready,expired=staleguard_status(self._forecast,age)
        self.event['prediction_ready']=ready
        completion_expired=(self.event['completion_expiry_steps'] is not None and age>=self.event['completion_expiry_steps'])
        if ready and expired:self._recover(step,'dispatch')
        elif (ready and self.variant=='staleguard_dual_horizon' and completion_expired
              and self._recovery_verified(age)):
            self._recover(step,'completion_at_dispatch')
        else:self._start('dive',step,'dispatch')

    def tick(self,step):
        if self.phase!='pending':return
        age=step-self.approval;ready,expired=staleguard_status(self._forecast,age)
        if not ready or self.trigger is None or age<self.trigger:return
        if self._recovery_verified(age) or expired:
            self.event['prediction_ready']=True
            self.log('horizon_trigger_reached',step,reasons=self.reasons)
            self._recover(step,'horizon_trigger')
        elif not self.missed_announced:
            self.missed_announced=True;self.event['recovery_prefix_missed_before_prediction_ready']=True
            self.log('recovery_prefix_missed',step)
