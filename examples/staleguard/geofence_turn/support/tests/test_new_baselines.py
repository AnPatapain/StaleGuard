"""FBS arithmetic and scheduler fixtures; no real flights in this test module."""
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

    def flight(self, policy, delay, computation=.001, change_future=False, save_trace=True):
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
            result,snapshot,trace = simulation.execute(s,delay,policy=policy,save_trace=save_trace)
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

    def test_optional_fbs_diagnostics_do_not_change_decision_or_timing(self):
        for delay,computation in ((0.,.001),(.02,.001),(.02,.1)):
            detailed = self.flight('fbs',delay,computation)[1]['event']
            compact = self.flight('fbs',delay,computation,save_trace=False)[1]['event']
            for name in ('prediction','computation_s','prediction_ready','reason',
                         'expired','scheduled_dispatch_step','application_step','fallback_step'):
                self.assertEqual(compact[name],detailed[name])
            for name in ('observation_steps','features_m','observation_update_s','observation_state'):
                self.assertNotIn(name,compact)

    def test_fbs_readiness_and_separate_measured_history_metrics(self):
        from geofence_turn.baseline_benchmark.metrics import policy_metrics
        rows = []
        for seed,delay,safe,completed,ready,recover,duration,history in (
                (0,0.,True,True,False,False,.001,None),
                (1,.02,True,False,True,True,.002,[.001,.002]),
                (2,.24,False,True,True,False,.003,[.004,.003])):
            dispatch = round((1+delay)*HZ)
            event = dict(policy='fbs',scheduled_dispatch_step=dispatch,
                application_step=None if recover else dispatch,
                fallback_step=dispatch if recover else None,
                computation_s=duration,prediction_ready=ready)
            if history is not None:
                event['observation_update_s'] = history
            rows.append(dict(seed=seed,delay_s=delay,safe=safe,completed=completed,
                             safe_completion=safe and completed,event=event))
        result = policy_metrics(rows,'fbs',HZ)
        self.assertEqual((result['safe_completion'],result['safe_noncompletion'],result['unsafe']),(1,1,1))
        self.assertEqual((result['prediction_ready'],result['prediction_unavailable'],result['unverified_executions']),(2,1,1))
        self.assertEqual(result['prediction_computation_s']['samples'],3)
        self.assertAlmostEqual(result['prediction_computation_s']['mean_s'],.002)
        cumulative = result['fbs_cumulative_observation_computation_s']
        self.assertEqual(cumulative['samples'],2)
        self.assertAlmostEqual(cumulative['mean_s'],.005)
        self.assertAlmostEqual(cumulative['p95_s'],.007)
        missing = policy_metrics(rows[:1],'fbs',HZ)['fbs_cumulative_observation_computation_s']
        self.assertEqual(missing,dict(samples=0,mean_s=None,median_s=None,p95_s=None))

    def test_cli_defaults_use_the_single_registry_and_all_23_delays(self):
        from geofence_turn.baseline_benchmark import run as benchmark
        from geofence_turn.baseline_benchmark.metrics import POLICIES
        self.assertEqual(POLICIES,('no_recovery','always_recover','fixed_ttl','fbs','use_time_recheck','staleguard'))
        self.assertIs(benchmark.POLICIES,POLICIES)
        self.assertEqual(benchmark.TTL_GRID,(0.,.02,.04,.06,.08,.10,.12,.14,.16,.18,.20,
                                           .30,.40,.50,.60,.70,.80,.90,1.))
        self.assertEqual(benchmark.DISPATCH_DELAYS,tuple(sorted(set(benchmark.TTL_GRID)|{.24,.44,.64,.84})))
        self.assertEqual(len(benchmark.DISPATCH_DELAYS),23)
        self.assertTrue(all(abs(delay*HZ-round(delay*HZ))<1e-9 for delay in benchmark.DISPATCH_DELAYS))
        with patch('sys.argv',['run.py']),patch.object(benchmark,'run') as run:
            benchmark.main()
        args = run.call_args.args[0]
        self.assertEqual(args.policies,POLICIES)
        self.assertEqual(args.delays,benchmark.DISPATCH_DELAYS)

    def test_off_grid_dispatch_keeps_frozen_expiry_and_fail_open_rules(self):
        from geofence_turn.baselines.staleguard import staleguard_status
        prediction = dict(computation_s=.05,expiry_steps=15)
        for delay,expected in ((0.,(False,False)),(.04,(False,False)),
                               (.06,(True,False)),(.24,(True,False)),
                               (.44,(True,True)),(.64,(True,True)),(.84,(True,True))):
            self.assertEqual(staleguard_status(prediction,round(delay*HZ)),expected)
        fbs_prediction = fbs.freeze_prediction(-.64,.02)
        self.assertEqual(fbs_prediction['ttl_steps'],32)
        self.assertFalse(fbs.dispatch_status(fbs_prediction,.001,32)['expired'])
        self.assertTrue(fbs.dispatch_status(fbs_prediction,.001,42)['recover'])

    def test_six_policies_share_inputs_forecast_once_and_pair_against_all_five(self):
        from unittest.mock import MagicMock
        import contextlib
        import io
        from geofence_turn.baseline_benchmark import run as benchmark, metrics
        cases = [case for case in load_development(ROOT/'results/development/calibration') if case[0].seed in (0,3)]
        snapshots = {s.seed:snapshot for s,snapshot,_ in cases}
        forecasts = {}
        warm = MagicMock()
        warm.start.return_value = (SimpleNamespace(select_action=lambda *_:None),None,None,None)
        def predict(snapshot,platform,delays):
            self.assertEqual(delays,benchmark.TTL_GRID)
            seed = next(seed for seed,saved in snapshots.items() if saved is snapshot)
            self.assertNotIn(seed,forecasts)
            forecasts[seed] = dict(computation_s=.001,expiry_steps=15,grid_s=list(delays))
            return forecasts[seed]
        def execute(scenario,delay,policy,**kwargs):
            self.assertIs(kwargs['received'],forecasts[scenario.seed])
            self.assertIs(scenario,next(s for s,_,_ in cases if s.seed==scenario.seed))
            dispatch = round((scenario.approval_s+delay)*HZ)
            index = metrics.POLICIES.index(policy)+round(delay*HZ)+scenario.seed
            safe,complete = index%5!=0,index%3==0
            recover = policy=='always_recover' or (policy!='no_recovery' and index%2==0)
            event = dict(policy=policy,scheduled_dispatch_step=dispatch,
                application_step=None if recover else dispatch,fallback_step=dispatch if recover else None)
            if policy=='staleguard':
                event.update(computation_s=.001,expiry_steps=15,prediction_ready=delay>0)
            return dict(seed=scenario.seed,delay_s=delay,safe=safe,completed=complete,
                        safe_completion=safe and complete,event=event),snapshots[scenario.seed],np.empty(0)
        with tempfile.TemporaryDirectory() as folder:
            args = SimpleNamespace(calibration=ROOT/'results/development/calibration',out=Path(folder),
                seeds=[0,3],delays=benchmark.DISPATCH_DELAYS,policies=metrics.POLICIES,max_flights=0)
            with patch.object(benchmark,'load_development',return_value=cases), \
                 patch.object(benchmark,'load_scg_platform_params',return_value=(None,None)), \
                 patch.object(benchmark,'LightweightPredictiveModel',return_value=warm), \
                 patch.object(benchmark,'predict_delays',side_effect=predict) as predicted, \
                 patch.object(benchmark,'execute',side_effect=execute) as flown, \
                 patch.object(metrics,'paired_metrics',wraps=metrics.paired_metrics) as paired, \
                 contextlib.redirect_stdout(io.StringIO()):
                benchmark.run(args)
            self.assertEqual(predicted.call_count,2)
            self.assertEqual(flown.call_count,2*23*6)
            self.assertEqual(paired.call_count,5*(23+1))
            warm.step.assert_called_once()
            rows = metrics.read_executions(Path(folder)/'executions.jsonl')
            self.assertEqual({metrics.flight_key(row) for row in rows},
                {(seed,delay,policy) for seed in (0,3) for delay in benchmark.DISPATCH_DELAYS for policy in metrics.POLICIES})
            report = json.loads((Path(folder)/'metrics.json').read_text())
            for group in [report,*report['by_delay'].values()]:
                self.assertEqual(set(group['paired_against_staleguard']),set(metrics.POLICIES)-{'staleguard'})
                for policy,outcome in group['by_policy'].items():
                    self.assertEqual(outcome['safe_completion']+outcome['safe_noncompletion']+outcome['unsafe'],outcome['flights'])
                    self.assertEqual(outcome['recovery_successes']+outcome['recovery_failures'],outcome['recovery_attempts'])
                    if policy!='staleguard':
                        comparison = group['paired_against_staleguard'][policy]
                        sg = group['by_policy']['staleguard']
                        self.assertEqual(comparison['net_completion_benefit'],sg['safe_completion']-outcome['safe_completion'])
                        self.assertEqual(comparison['net_unsafe_reduction'],outcome['unsafe']-sg['unsafe'])
                self.assertEqual(group['paired_comparison'],group['paired_against_staleguard']['use_time_recheck'])

    def test_benchmark_selects_new_policies_without_model_rollouts(self):
        from geofence_turn.baseline_benchmark.run import run as run_benchmark
        from geofence_turn.baseline_benchmark.metrics import POLICIES,summarize
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
            args = SimpleNamespace(calibration=ROOT/'results/development/calibration',out=out,seeds=[0],
                delays=(0.,),policies=('always_recover','fbs'),max_flights=0)
            with patch('geofence_turn.baseline_benchmark.run.load_scg_platform_params',side_effect=AssertionError('Unexpected model load')), \
                 patch('geofence_turn.baselines.staleguard.predict_delays',side_effect=AssertionError('Unexpected prediction')), \
                 patch('geofence_turn.baselines.use_time_recheck.predict_immediate',side_effect=AssertionError('Unexpected check')), \
                 patch('geofence_turn.baseline_benchmark.run.execute',side_effect=recorded):
                run_benchmark(args)
            self.assertEqual({p.name for p in out.iterdir()},{'run.json','metrics.json','executions.jsonl'})
            self.assertEqual(json.loads((out/'run.json').read_text())['predictions'],{})
            metrics = summarize(out/'executions.jsonl')
            self.assertEqual(metrics['recorded_flights'],2)
            self.assertEqual(metrics['by_policy']['always_recover']['prediction_computation_s']['samples'],0)


if __name__ == '__main__':
    unittest.main()
