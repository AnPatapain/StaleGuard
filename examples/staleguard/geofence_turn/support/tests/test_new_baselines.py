"""FBS arithmetic and scheduler fixtures; no real flights in this test module."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import json
import tempfile
import time
import unittest
import numpy as np
import fbs as inspection_fbs
from geofence_turn.baselines import always_recover, fbs
from geofence_turn import simulation
from geofence_turn.scenario import (Config, Command, Scenario, HZ, PHYSICS_HZ,
    BODY_RADIUS_M, POS, VEL, make_forward_trajectory, safety_only_reference)
from geofence_turn.storage import ROOT, load_development


class NewBaselineTests(unittest.TestCase):
    def test_feature_sign_includes_body_radius_and_required_clearance(self):
        s = SimpleNamespace(fence_x=2.)
        boundary = s.fence_x-BODY_RADIUS_M-Config().d_min_m
        for offset in (-.1,0.,.1):
            state = np.zeros(12)
            state[0] = boundary+offset
            self.assertAlmostEqual(fbs.safety_feature(state,s,Config()),offset)

    def test_self_contained_algorithm_and_initialization(self):
        for function in (fbs.update_volatility,fbs.freeze_prediction,fbs.dispatch_status):
            self.assertEqual(function.__module__,fbs.__name__)
        self.assertEqual((fbs.ALPHA,fbs.EPSILON,fbs.MAX_TTL_STEPS),(.1,1e-12,210))
        previous,volatility = None,0.
        for feature,expected in ((-1.,0.),(-.9,.01),(-.7,.029)):
            volatility = fbs.update_volatility(feature,previous,volatility)
            self.assertAlmostEqual(volatility,expected)
            previous = feature
        self.assertEqual(fbs.freeze_prediction(previous,volatility),
                         inspection_fbs.freeze_prediction(-previous,volatility))
        self.assertEqual(fbs.freeze_prediction(previous,volatility)['ttl_steps'],24)
        self.assertEqual(fbs.freeze_prediction(-1.,0.)['ttl_steps'],210)

    def test_ema_exactly_matches_inspection_descent(self):
        for history in ((-1.,-.9,-.7),(-1.,-1.,-1.),(0.,.1,-.2,0.),
                        (1e-13,-1e-13,0.)):
            for initial in (0.,.25):
                previous = None
                actual = expected = initial
                for feature in history:
                    actual = fbs.update_volatility(feature,previous,actual)
                    expected = inspection_fbs.update_volatility(feature,previous,expected)
                    self.assertEqual(actual,expected)
                    previous = feature

    def test_prediction_and_dispatch_exactly_match_inspection_descent(self):
        # Include zero volatility, epsilon floor, margin boundary, cap and flooring.
        for feature in (-1.,-.1,-1e-13,0.,.1):
            for volatility in (0.,1e-13,1e-12,.02,.029):
                with self.subTest(feature=feature,volatility=volatility):
                    actual = fbs.freeze_prediction(feature,volatility)
                    expected = inspection_fbs.freeze_prediction(-feature,volatility)
                    self.assertEqual(actual,expected)
                    ttl = actual['ttl_steps']
                    for waited in sorted({0,max(0,ttl-1),ttl,ttl+1,210,211}):
                        boundary = waited/HZ
                        for computation in (0.,boundary,np.nextafter(boundary,np.inf)):
                            self.assertEqual(
                                fbs.dispatch_status(actual,computation,waited),
                                inspection_fbs.dispatch_status(expected,computation,waited))
        self.assertEqual(fbs.freeze_prediction(-.5,0.,epsilon=.1),
                         inspection_fbs.freeze_prediction(.5,0.,epsilon=.1))
        for epsilon in (0.,-1e-12):
            for freeze,feature in ((fbs.freeze_prediction,-1.),
                                   (inspection_fbs.freeze_prediction,1.)):
                with self.assertRaisesRegex(ValueError,'epsilon must be positive'):
                    freeze(feature,0.,epsilon=epsilon)

    def test_margin_boundary_expiry_equality_and_readiness_equality(self):
        prediction = fbs.freeze_prediction(-.1,.02)
        self.assertEqual(prediction['ttl_steps'],5)
        self.assertFalse(fbs.dispatch_status(prediction,.1,5)['recover'])
        self.assertTrue(fbs.dispatch_status(prediction,.12,6)['recover'])
        self.assertFalse(fbs.dispatch_status(prediction,.121,6)['recover'])
        for feature in (0.,.1):
            prediction = fbs.freeze_prediction(feature,.01)
            self.assertTrue(fbs.dispatch_status(prediction,0.,0)['recover'])
            self.assertFalse(fbs.dispatch_status(prediction,.001,0)['recover'])

    def flight(self, policy, delay, computation=.001, change_future=False):
        """Prescribed observations test scheduling; they are not physics results."""
        config = Config()
        command = Command(.4,.6,(.2,.8,1.15),(.8,1.,1.35))
        s = Scenario(0,.6,1.25,0.,0.,.48+BODY_RADIUS_M+.05+.01,.4,command,.6)
        approval = round(s.approval_s*HZ)
        total = approval+round(s.deadline_s*HZ)
        states = np.zeros((total+1,12))
        states[:,0] = np.arange(total+1)*.016
        states[:,1] = .8
        states[:,4] = s.height
        if change_future:
            states[approval+1:,0] += 5
        env = SimpleNamespace(index=0,DRONE_ID=0,PYB_CLIENT=0,close=lambda:None)
        env.reset = lambda **_: (states[0].copy(),{})
        def step(action):
            env.index += 1
            return states[env.index].copy(),0.,False,{}
        env.step = step
        ctrl = SimpleNamespace(integral_pos_e=np.zeros(3),last_rpy=np.zeros(3),
            integral_rpy_e=np.zeros(3),reset_before_run=lambda **_:None,
            select_action=lambda *_:None,close=lambda:None)
        def monitor(environment,observe):
            last = -1
            def sample():
                nonlocal last
                end = environment.index*(PHYSICS_HZ//HZ)
                for tick in range(last+1,end+1):
                    index = tick/(PHYSICS_HZ//HZ)
                    lo = int(index);hi = min(lo+1,total)
                    position = ((1-(index-lo))*states[lo]+(index-lo)*states[hi])[POS]
                    observe(position,tick)
                last = end
            return sample
        timings = iter([0.]+[value for _ in range(approval+1) for value in (0.,computation)]+[100.])
        clock = SimpleNamespace(perf_counter=lambda:next(timings),monotonic=time.monotonic)
        with patch.object(simulation,'env_factory',return_value=lambda:env), \
             patch.object(simulation,'make',return_value=ctrl), \
             patch.object(simulation,'check_memory',return_value=(100,3000)), \
             patch.object(simulation,'monitor_physics_steps',side_effect=monitor), \
             patch.object(simulation.p,'getBaseVelocity',side_effect=lambda *a,**k:(states[env.index][VEL],np.zeros(3))), \
             patch.object(simulation,'time',clock):
            result,snapshot,trace = simulation.execute(s,delay,policy=policy,save_trace=True)
        return s,result,snapshot,trace

    def test_fbs_observes_only_through_approval_then_recovers_at_dispatch(self):
        s,result,_,trace = self.flight('fbs',.02)
        event = result['event'];approval = round(s.approval_s*HZ)
        self.assertEqual(event['observation_steps'],list(range(approval+1)))
        np.testing.assert_array_equal(event['observation_state'],trace[approval,1:13])
        self.assertEqual(event['prediction']['ttl_steps'],0)
        self.assertTrue(event['prediction_ready'])
        self.assertEqual(event['fallback_step'],approval+1)
        self.assertIsNone(event['application_step'])
        expected = safety_only_reference(make_forward_trajectory(len(trace),s),approval+1,trace[approval+1,1:13])
        np.testing.assert_array_equal(trace[approval+1:,13:25],expected[approval+1:])
        other = self.flight('fbs',.02,change_future=True)[1]['event']
        self.assertEqual(other['features_m'],event['features_m'])
        self.assertEqual(other['prediction'],event['prediction'])

    def test_unavailable_fbs_fails_open_without_late_cancellation(self):
        for delay,computation in ((0.,.001),(.02,.1)):
            s,result,_,trace = self.flight('fbs',delay,computation)
            event = result['event']
            self.assertFalse(event['prediction_ready'])
            self.assertEqual(event['expired'],delay > 0)  # TTL=0 still permits age=0.
            self.assertEqual(event['reason'],'unavailable')
            self.assertIsNone(event['fallback_step'])
            self.assertEqual(event['application_step'],round((s.approval_s+delay)*HZ))
            self.assertEqual(len(trace),round((s.approval_s+s.deadline_s)*HZ)+1)

    def test_always_recovers_at_dispatch_without_prediction(self):
        for delay in (0.,.02,.1):
            with patch.object(fbs,'safety_feature',side_effect=AssertionError('Unexpected FBS observation')), \
                 patch('geofence_turn.baselines.staleguard.predict_delays',side_effect=AssertionError('Unexpected prediction')):
                s,result,_,trace = self.flight('always_recover',delay)
            event = result['event']
            self.assertTrue(always_recover.should_recover())
            self.assertEqual(event['fallback_step'],round((s.approval_s+delay)*HZ))
            self.assertIsNone(event['application_step'])
            self.assertNotIn('computation_s',event)
            self.assertNotIn('features_m',event)
            if delay==.1:
                self.assertIsNotNone(result['pre_dispatch_violation_s'])
                self.assertFalse(result['safe'])

    def test_benchmark_selects_new_policies_without_model_rollouts(self):
        from geofence_turn.baseline_benchmark.run import run_comparison
        from geofence_turn.baseline_benchmark.metrics import DEFAULT_POLICIES,POLICIES,summarize
        self.assertEqual(DEFAULT_POLICIES,('no_recovery','staleguard','use_time_recheck','fixed_ttl'))
        self.assertEqual(len(set(POLICIES)),6)
        scenario,snapshot,_ = load_development(ROOT/'results/development/calibration')[0]
        def recorded(s,delay,policy,**kwargs):
            self.assertIsNone(kwargs['platform'])
            dispatch = round(s.approval_s*HZ)
            recovery = policy=='always_recover'
            result = dict(seed=s.seed,delay_s=delay,safe=True,completed=False,safe_completion=False,
                event=dict(policy=policy,scheduled_dispatch_step=dispatch,
                application_step=None if recovery else dispatch,fallback_step=dispatch if recovery else None))
            return result,snapshot,np.empty((0,))
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'run'
            args = SimpleNamespace(development=str(ROOT/'results/development/calibration'),seeds=[0],
                delays=(0.,),policies=('always_recover','fbs'),resume=False,diagnostics=False)
            with patch('model.scg.load_scg_platform_params',side_effect=AssertionError('Unexpected model load')), \
                 patch('geofence_turn.baselines.staleguard.predict_delays',side_effect=AssertionError('Unexpected prediction')), \
                 patch('geofence_turn.baselines.use_time_recheck.predict_immediate',side_effect=AssertionError('Unexpected check')), \
                 patch('geofence_turn.baseline_benchmark.run.execute',side_effect=recorded), \
                 patch('geofence_turn.baseline_benchmark.run.check_memory',return_value=(100,3000)):
                run_comparison(out,args,simulation.start_work(10,60),Config())
            self.assertEqual({p.name for p in out.iterdir()},{'run.json','metrics.json','executions.jsonl'})
            self.assertEqual(json.loads((out/'run.json').read_text())['predictions'],{})
            metrics = summarize(out/'executions.jsonl')
            self.assertEqual(metrics['recorded_flights'],2)
            self.assertEqual(metrics['by_policy']['always_recover']['prediction_computation_s']['samples'],0)


if __name__ == '__main__':
    unittest.main()
