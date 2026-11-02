"""PPR rollout and causal scheduler tests; toy dynamics are not flight results."""
from types import SimpleNamespace
from unittest.mock import patch
import copy
import inspect
import unittest
import numpy as np

from geofence_turn.baselines import ppr
from geofence_turn.scenario import Config, Command, POS, VEL, HZ, clearance, safety_only_reference
from geofence_turn.baseline_benchmark.metrics import policy_metrics
from geofence_turn.support.tests import test_new_baselines as fixtures


class BrakingModel:
    """Simple inertial fixture, deliberately independent of reference position."""
    instances = []

    def __init__(self, *args):
        self.references = []; self.closed = False; self.starts = 0
        self.instances.append(self)

    def start(self, snapshot):
        self.starts += 1
        self.ctrl = SimpleNamespace(reference=snapshot.remaining_reference.copy(), memory=17)
        self.ctrl.select_action = lambda obs, info: self.ctrl.reference[info['current_step'], VEL]
        return self.ctrl, snapshot.state.copy(), {'current_step': 0}, self.ctrl.reference.copy()

    def step(self, action, state, info):
        state = state.copy()
        state[POS] += state[VEL]/HZ
        state[VEL] += .5*(action-state[VEL])
        self.ctrl.memory += 1
        return state, False, {'current_step': info['current_step']+1}

    def set_reference(self, ctrl, reference):
        self.references.append((reference.copy(), ctrl.memory))
        ctrl.reference = reference.copy()

    def close(self, checkpoints):
        self.closed = True


class PPRTests(unittest.TestCase):
    def snapshot(self, fence=2., horizon=60, speed=1.):
        ref = np.zeros((horizon+1,12)); ref[:,0] = 100.; ref[:,1] = speed; ref[:,4] = 1.2
        state = np.zeros(12); state[1] = speed; state[4] = 1.2
        return SimpleNamespace(state=state, pid_state='nonzero fixture memory',
            remaining_reference=ref, horizon_steps=horizon, config=Config(), fence_x=fence,
            prefix_min_clearance_m=float(clearance(state[0], fence)),
            command=Command(.4,speed or .8,(.2,1.,1.),(.8,1.2,1.4)))

    def predict(self, snapshot):
        with patch.object(ppr,'LightweightPredictiveModel',BrakingModel), \
             patch('geofence_turn.baselines.staleguard.predict_delays',side_effect=AssertionError('No TTL forecast')), \
             patch('geofence_turn.baselines.use_time_recheck.predict_immediate',side_effect=AssertionError('No turn check')):
            return ppr.predict_recovery(snapshot,(None,None))

    def test_rollout_restores_state_then_waits_interval_plus_latency_before_braking(self):
        snapshot = self.snapshot(); before = copy.deepcopy(snapshot)
        result = self.predict(snapshot); model = BrakingModel.instances[-1]
        self.assertEqual((ppr.INTERVAL_STEPS,ppr.COMPUTATION_BOUND_S,ppr.LATENCY_BOUND_STEPS),(10,.02,2))
        self.assertTrue(result['safe']); self.assertEqual(result['predicted_steps'],60)
        self.assertEqual(result['recovery_after_steps'],12)
        self.assertEqual(model.starts,1); self.assertTrue(model.closed)
        changed,memory = model.references[0]
        self.assertEqual(memory,17+12)  # No controller-memory reset at recovery.
        np.testing.assert_allclose(changed[12:,POS],np.tile([.24,0,1.2],(49,1)))
        np.testing.assert_array_equal(changed[12:,VEL],0.)
        # Ref X=100 is unsafe, but the *physical* braking flight is safe.
        self.assertLess(result['min_clearance_m'],snapshot.prefix_min_clearance_m)
        np.testing.assert_array_equal(snapshot.state,before.state)
        np.testing.assert_array_equal(snapshot.remaining_reference,before.remaining_reference)

    def test_predicts_unsafe_braking_overshoot_not_queued_turn(self):
        result = self.predict(self.snapshot(fence=.39))
        self.assertFalse(result['safe'])
        self.assertGreater(result['predicted_steps'],12)  # Violation after braking starts.
        self.assertLess(result['predicted_steps'],60)
        self.assertLess(result['min_clearance_m'],Config().d_min_m)
        self.assertEqual(list(inspect.signature(ppr.predict_recovery).parameters),['snapshot','platform'])

    def test_prefix_violation_and_horizon_are_preserved(self):
        snapshot = self.snapshot(); snapshot.prefix_min_clearance_m = .049
        result = self.predict(snapshot)
        self.assertFalse(result['safe']); self.assertEqual(result['predicted_steps'],0)
        result = self.predict(self.snapshot(horizon=8))
        self.assertTrue(result['safe']); self.assertEqual(result['predicted_steps'],8)
        self.assertIsNone(result['recovery_after_steps'])
        self.assertEqual(BrakingModel.instances[-1].references,[])
        snapshot = self.snapshot(speed=0.)
        snapshot.config = Config(d_min_m=float(clearance(0.,snapshot.fence_x)))
        self.assertTrue(self.predict(snapshot)['safe'])  # >= is inclusive.

    def flight(self, delay, duration=.001, safe=True, change_future=False):
        with patch.object(ppr,'predict_recovery',return_value=dict(safe=safe,min_clearance_m=.1)) as predicted:
            output = fixtures.NewBaselineTests().flight('ppr',delay,computation=duration,change_future=change_future)
        return output,predicted

    def test_d0_executes_without_check_and_monitoring_starts_at_approval(self):
        (s,result,_,_),predicted = self.flight(0.)
        self.assertEqual(predicted.call_count,0)
        self.assertEqual(result['event']['application_step'],round(s.approval_s*HZ))
        self.assertEqual(result['event']['prediction_check_count'],0)
        self.assertEqual(result['event']['prediction_computation_total_s'],0.)
        self.assertIsNone(result['event']['prediction_computation_max_s'])
        (s,result,_,_),predicted = self.flight(.30)
        self.assertEqual([c['observation_step'] for c in result['event']['ppr_checks']],
                         [round(s.approval_s*HZ),round(s.approval_s*HZ)+10])

    def test_latency_recovery_reference_and_permanent_cancellation(self):
        for duration,offset in ((.001,1),(.020,2),(.045,3)):
            (s,result,_,trace),predicted = self.flight(.30,duration,safe=False)
            event = result['event']; approval = round(s.approval_s*HZ)
            self.assertTrue(event['early_recovery'])
            self.assertEqual(event['fallback_step'],approval+offset)
            self.assertIsNone(event['application_step'])
            self.assertEqual(predicted.call_count,1)
            check = event['ppr_checks'][0]
            self.assertGreater(check['available_step']/HZ,s.approval_s+duration-1e-12)
            self.assertEqual(check['decision_step'],check['action_step'])
            self.assertEqual(check['timing_bound_exceeded'],duration>.020)
            expected = safety_only_reference(trace[:,13:25],approval+offset,trace[approval+offset,1:13])
            np.testing.assert_array_equal(trace[approval+offset:,13:25],expected[approval+offset:])
            self.assertIsNotNone(result['pre_dispatch_violation_s'])  # Earlier violations remain.

    def test_dispatch_wins_ties_and_late_predictions_never_cancel(self):
        for duration in (.020,.025):
            (s,result,_,_),_ = self.flight(.02,duration,safe=False)
            e = result['event']; self.assertIsNone(e['fallback_step'])
            self.assertEqual(e['application_step'],e['scheduled_dispatch_step'])
            self.assertTrue(e['ppr_checks'][0]['discarded_at_dispatch'])
            self.assertIsNone(e['ppr_checks'][0]['decision_step'])
        (_,result,_,_),_ = self.flight(.02,.001,safe=False)  # available tick == dispatch tick
        self.assertIsNone(result['event']['fallback_step'])
        self.assertEqual(result['event']['application_step'],result['event']['scheduled_dispatch_step'])

    def test_no_overlapping_checks_and_no_future_dispatch_input(self):
        (s,result,_,_),predicted = self.flight(.36,.205)
        checks = result['event']['ppr_checks']
        self.assertEqual(len(checks),1)  # Tick-10 opportunity skipped during computation.
        self.assertEqual(checks[0]['available_step'],round(s.approval_s*HZ)+11)
        first = self.flight(.10,safe=False)[1].call_args.args[0]
        second = self.flight(.30,safe=False,change_future=True)[1].call_args.args[0]
        np.testing.assert_array_equal(first.state,second.state)
        np.testing.assert_array_equal(first.remaining_reference,second.remaining_reference)
        for name in ('integral_pos_e','last_rpy','integral_rpy_e'):
            np.testing.assert_array_equal(getattr(first.pid_state,name),getattr(second.pid_state,name))
        self.assertFalse(hasattr(first,'delay')); self.assertFalse(hasattr(first,'dispatch_step'))

    def test_metrics_distinguish_check_time_flight_totals_and_decision_latency(self):
        rows = [self.flight(0.)[0][1],self.flight(.30,safe=False)[0][1],self.flight(.30)[0][1]]
        out = policy_metrics(rows,'ppr',HZ)
        self.assertEqual(out['predictive_checks'],3)
        self.assertEqual(out['prediction_computation_s']['samples'],3)
        self.assertEqual(out['prediction_computation_total_s']['samples'],3)
        self.assertEqual(out['prediction_computation_max_s']['samples'],2)
        self.assertAlmostEqual(out['prediction_computation_sum_s'],.003)
        self.assertEqual(out['prediction_to_recovery_s']['mean_s'],.02)
        self.assertEqual(out['prediction_to_decision_s']['samples'],3)
        self.assertIsNone(out['prediction_ready'])  # Not a command-validity forecast.
        self.assertEqual(out['recovery_attempts'],out['recovery_successes']+out['recovery_failures'])
        self.assertEqual(out['flights'],out['safe_completion']+out['safe_noncompletion']+out['unsafe'])


if __name__ == '__main__':
    unittest.main()
