"""Policy/forecast fixtures are synthetic; native flight scores live in journals."""
from dataclasses import replace
from copy import deepcopy
from types import SimpleNamespace
import inspect
import ast
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
import numpy as np
from underpass_inspection.baselines import fbs as freshness
from underpass_inspection.scenario import Scenario,BODY_RADIUS_M,POS,body_clearances
from underpass_inspection.storage import load_calibrated_scenarios,ROOT
from underpass_inspection.simulation import Snapshot
from underpass_inspection.baselines import staleguard,policies
from underpass_inspection.baselines.policies import PendingCommand,POLICIES,HORIZON_POLICIES


class KinematicModel:
    def __init__(self, *args):
        pass

    def start(self, snapshot):
        self.ctrl = SimpleNamespace(reference=snapshot.remaining_reference.copy())
        self.ctrl.select_action = lambda obs, info: None
        return self.ctrl, snapshot.state.copy(), {'current_step':0}, self.ctrl.reference.copy()

    def save_checkpoint(self, ctrl, obs, info):
        return obs.copy(), deepcopy(info)

    def restore_checkpoint(self, ctrl, checkpoint):
        return checkpoint[0].copy(), deepcopy(checkpoint[1])

    def set_reference(self, ctrl, reference):
        ctrl.reference = reference.copy()

    def step(self, action, obs, info):
        step = info['current_step']+1
        # Exact reference tracking is solely a unit-test fixture.
        return self.ctrl.reference[step].copy(), False, {'current_step':step}

    def close(self, checkpoints):
        pass


class PolicyTests(unittest.TestCase):
    def test_no_cross_mission_imports(self):
        for source in ROOT.rglob('*.py'):
            for node in ast.walk(ast.parse(source.read_text())):
                modules=([node.module] if isinstance(node,ast.ImportFrom) else
                         [alias.name for alias in node.names] if isinstance(node,ast.Import) else [])
                for module in modules:
                    self.assertFalse(module and module.split('.')[0] in
                                     ('geofence_turn','inspection_descent'),str(source))
        self.assertFalse(any(name.split('.')[0] in ('geofence_turn','inspection_descent')
                             for name in sys.modules))

    def agent(self,name='staleguard_early_recovery',runtime=.06,expiry=10):
        received=dict(computation_s=runtime,expiry_steps=expiry)
        agent=PendingCommand(name,Scenario(),received if name.startswith('staleguard') else None)
        agent.approve()
        return agent

    def advance(self,agent,age,dispatch=False,snapshot=None):
        step=agent.approval+age
        agent.announce_readiness(step)
        if dispatch:agent.dispatch(step,snapshot,None)
        agent.tick(step)
        return agent.take_action()

    def test_interfaces_exclude_unknown_future_schedule(self):
        for call in (PendingCommand.__init__,PendingCommand.approve,PendingCommand.tick,
                     PendingCommand.announce_readiness,staleguard.predict_delays):
            self.assertFalse({'dispatch_time','dispatch_step','delay_s','ground_truth','recovery_deadline'}&
                             set(inspect.signature(call).parameters))
        for name in POLICIES:
            if name in HORIZON_POLICIES:continue
            a=self.agent(name)
            self.assertNotIn('scheduled_dispatch_step',a.event)
            self.assertIsNone(a.event['dispatch_observed_step'])

    def test_always_recover_at_approval_and_cancel_later_dispatch(self):
        a=self.agent('always_recover')
        self.assertEqual(a.take_action(),'sideways')
        self.assertEqual(a.event['fallback_step'],a.approval)
        self.assertIsNone(self.advance(a,20,dispatch=True))
        self.assertIsNone(a.event['application_step'])
        self.assertEqual(sum(e['kind']=='recovery_started' for e in a.events),1)

    def test_no_recovery_dispatches_and_never_intervenes(self):
        a=self.agent('no_recovery')
        for t in range(20):self.assertIsNone(self.advance(a,t))
        self.assertEqual(self.advance(a,20,dispatch=True),'dive')
        for t in range(21,30):self.assertIsNone(self.advance(a,t))
        self.assertIsNone(a.event['fallback_step'])

    def test_dispatch_staleguard_does_not_recover_on_pending_expiry(self):
        a=self.agent('staleguard')
        for t in range(20):self.assertIsNone(self.advance(a,t))
        self.assertEqual(self.advance(a,20,dispatch=True),'sideways')

    def test_early_recovery_fires_while_pending_and_cancels_dispatch(self):
        a=self.agent()
        for t in range(10):self.assertIsNone(self.advance(a,t))
        self.assertEqual(self.advance(a,10),'sideways')
        self.assertIsNone(self.advance(a,20,dispatch=True))
        self.assertIsNone(a.event['application_step'])

    def test_same_approval_prediction_for_both_staleguard_variants(self):
        forecast=dict(computation_s=.06,expiry_steps=10)
        a=PendingCommand('staleguard',Scenario(),forecast)
        b=PendingCommand('staleguard_early_recovery',Scenario(),forecast)
        self.assertIs(a._forecast,b._forecast)

    def test_late_prediction_never_causes_retroactive_intervention(self):
        a=self.agent(runtime=.36,expiry=10)
        for t in range(18):self.assertIsNone(self.advance(a,t))
        self.assertEqual(self.advance(a,18),'sideways')
        self.assertEqual(a.event['fallback_step'],a.approval+18)

    def test_unready_forecast_fails_open_without_later_override(self):
        for name in ('staleguard','staleguard_early_recovery'):
            a=self.agent(name,runtime=.36,expiry=10)
            self.assertEqual(self.advance(a,15,dispatch=True),'dive')
            self.assertFalse(a.event['prediction_ready'])
            for t in range(16,30):self.assertIsNone(self.advance(a,t))
            self.assertIsNone(a.event['fallback_step'])
            self.assertEqual(a.event['prediction_ready_step'],a.approval+18)

    def test_fixed_timer_only_uses_elapsed_time_and_dispatch_wins_tie(self):
        a=self.agent('fixed_early_recovery')
        for t in range(10):self.assertIsNone(self.advance(a,t))
        self.assertEqual(self.advance(a,10),'sideways')
        b=self.agent('fixed_early_recovery')
        self.assertEqual(self.advance(b,9,dispatch=True),'dive')
        self.assertIsNone(self.advance(b,10))
        c=self.agent('fixed_early_recovery')
        self.assertEqual(self.advance(c,10,dispatch=True),'sideways')
        self.assertEqual(c.event['recovery_reason'],'dispatch')

    def test_recheck_occurs_only_at_dispatch_and_replays_computation_wait(self):
        a=self.agent('use_time_recheck')
        with patch.object(policies,'predict_immediate',return_value=dict(safe=False)) as check:
            for t in range(20):self.assertIsNone(self.advance(a,t))
            check.assert_not_called()
            with patch.object(policies.time,'perf_counter',side_effect=[10.,10.045]):
                self.assertIsNone(self.advance(a,20,dispatch=True,snapshot=object_snapshot()))
            self.assertEqual(check.call_count,1)
            self.assertIsNone(self.advance(a,21));self.assertIsNone(self.advance(a,22))
            self.assertEqual(self.advance(a,23),'sideways')
            self.assertEqual(a.event['decision_step'],a.approval+23)

    def test_fbs_ema_initialization_floor_cap_and_strict_expiry(self):
        self.assertEqual(freshness.ALPHA,.1)
        previous=None;volatility=0.
        for f in (-1.,-.9,-.7):
            volatility=freshness.update_volatility(f,previous,volatility);previous=f
        self.assertAlmostEqual(volatility,.029)
        prediction=freshness.freeze_prediction(previous,volatility)
        self.assertEqual(prediction['ttl_steps'],24)
        self.assertFalse(freshness.dispatch_status(prediction,0.,24)['recover'])
        self.assertTrue(freshness.dispatch_status(prediction,0.,25)['recover'])
        self.assertEqual(freshness.freeze_prediction(-1.,0.)['ttl_steps'],210)

    def test_fbs_history_stops_at_approval(self):
        s=Scenario();a=PendingCommand('fbs',s)
        state=np.zeros(12);state[4]=s.cruise_z
        for step in range(a.approval+1):
            state[0]=step*.01;a.observe(step,state)
        prediction=a.event['prediction'].copy();history=list(a.features)
        state[0]=100.;a.observe(a.approval+1,state)
        self.assertEqual(a.event['prediction'],prediction);self.assertEqual(a.features,history)
        self.assertEqual(len(a.features),a.approval+1)

    def test_dataset_ids_are_identifiers(self):
        self.assertEqual(Scenario(seed=101).seed,101)


def object_snapshot():
    from types import SimpleNamespace
    return SimpleNamespace(state=np.zeros(12))


class PredictionTests(unittest.TestCase):
    def test_segment_checker_exact_faces_interior_and_rounded_corners(self):
        lo=np.zeros(3);hi=np.ones(3)
        cases=[([-1,.5,.5],[2,.5,.5],-.5),
               ([-1,-1,.5],[2,-1,.5],1.),
               ([1.3,1.4,.5],[1.3,1.4,.5],.5),
               ([-.09,-.09,.5],[-.09,-.09,.5],np.sqrt(2*.09**2))]
        for a,b,distance in cases:
            self.assertAlmostEqual(staleguard.segment_box_clearance(np.array(a),np.array(b),lo,hi),distance-BODY_RADIUS_M)
        self.assertGreater(staleguard.segment_box_clearance(np.array(cases[-1][0]),np.array(cases[-1][1]),lo,hi),.05)

    def test_waiting_is_predicted_then_relative_command_uses_predicted_anchor(self):
        s,snapshot,_=load_calibrated_scenarios(ROOT/'results/development/calibration')[0]
        saved=snapshot.remaining_reference.copy();state=snapshot.state.copy()
        from underpass_inspection.scenario import apply_maneuver
        anchors=[]
        def edit(ref,step,obs,scenario,mode):
            anchors.append((step,obs[POS].copy()))
            return apply_maneuver(ref,step,obs,scenario,mode)
        with patch.object(staleguard,'LightweightPredictiveModel',KinematicModel), \
             patch.object(staleguard,'apply_maneuver',side_effect=edit):
            result=staleguard.predict_delays(snapshot,(None,None),(0.,.2,1.))
        self.assertTrue(result['results'][0]['safe'])
        self.assertFalse(result['results'][-1]['safe'])
        self.assertGreater(anchors[-1][1][0],anchors[0][1][0])
        np.testing.assert_array_equal(snapshot.remaining_reference,saved)
        np.testing.assert_array_equal(snapshot.state,state)

    def test_a_prefix_violation_cannot_be_erased(self):
        _,snapshot,_=load_calibrated_scenarios(ROOT/'results/development/calibration')[0]
        with patch.object(staleguard,'LightweightPredictiveModel',KinematicModel):
            result=staleguard.predict_delays(replace(snapshot,prefix_min_clearance_m=.01),(None,None),(0.,.2))
        self.assertEqual(result['expiry_steps'],0)
        self.assertTrue(all(not r['safe'] for r in result['results']))


class PhysicalWaitingTests(unittest.TestCase):
    def test_actual_waiting_violation_survives_dispatch_and_recheck(self):
        # Explicit out-of-grid stress test, never a benchmark score or fitted
        # policy parameter. Both executions use actual PID/PyBullet dynamics.
        from model.scg import load_scg_platform_params
        from underpass_inspection.simulation import execute
        s,snapshot,_=load_calibrated_scenarios(ROOT/'results/development/calibration')[0]
        platform=load_scg_platform_params()
        for name in ('no_recovery','use_time_recheck'):
            row,_=execute(s,policy=name,delay_s=2.,platform=platform,expected_snapshot=snapshot)
            self.assertFalse(row['safe'])
            self.assertFalse(row['safe_completion'])
            self.assertIsNotNone(row['pre_dispatch_violation_s'])
            self.assertTrue(row['violation_before_intervention'])
            self.assertEqual(row['physics_samples'],row['horizon_step']*20+1)
            if name=='use_time_recheck':
                self.assertFalse(row['event']['check']['safe'])
                self.assertIsNotNone(row['event']['fallback_step'])
                self.assertEqual(row['mode'],'sideways')


if __name__=='__main__':unittest.main()
