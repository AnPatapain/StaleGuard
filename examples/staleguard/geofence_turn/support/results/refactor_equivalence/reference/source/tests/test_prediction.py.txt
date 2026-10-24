"""Adapter tests with deterministic kinematics, never presented as physics scores."""
from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from geofence_turn.mission import Config, Command, Scenario, HZ, make_forward_trajectory
from geofence_turn.prediction import predict_delays


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


class PredictionTests(unittest.TestCase):
    def snapshot(self, fence=1.3, prefix=10):
        cmd = Command(.2, .6, (.7,1.3,1.15), (1.1,1.5,1.35))
        s = Scenario(0, .6, 1.25, 0., 0., fence, 6., cmd, 1.2)
        remaining = make_forward_trajectory(361, s)[60:]
        return SimpleNamespace(state=remaining[0].copy(), remaining_reference=remaining,
            command=cmd, horizon_steps=len(remaining)-1, config=Config(), fence_x=fence,
            prefix_min_clearance_m=prefix)

    def test_nominal_eventual_violation_does_not_invalidate_safe_turn(self):
        snapshot = self.snapshot()
        self.assertGreater(snapshot.remaining_reference[-1,0], snapshot.fence_x)
        with patch('geofence_turn.prediction.LightweightPredictiveModel', KinematicModel):
            result = predict_delays(snapshot, (None,None), (0., .2, 1.))
        self.assertTrue(result['results'][0]['safe'])
        self.assertTrue(result['results'][1]['safe'])
        self.assertFalse(result['results'][2]['safe'])
        self.assertEqual(result['expiry_steps'], HZ)

    def test_prefix_violation_is_irrevocable(self):
        with patch('geofence_turn.prediction.LightweightPredictiveModel', KinematicModel):
            result = predict_delays(self.snapshot(prefix=.01), (None,None), (0., .2))
        self.assertTrue(all(not r['safe'] for r in result['results']))
        self.assertEqual(result['expiry_steps'], 0)

    def test_legacy_connector_snapshot_cannot_be_reinterpreted(self):
        snapshot = self.snapshot()
        snapshot.command = SimpleNamespace(radius=.2,speed=.6,waypoint=(.8,1.4,1.25))
        with self.assertRaisesRegex(ValueError, 'Legacy connector snapshot'):
            predict_delays(snapshot,(None,None))

    def test_prediction_does_not_mutate_frozen_snapshot(self):
        snapshot = self.snapshot()
        before = snapshot.remaining_reference.copy()
        with patch('geofence_turn.prediction.LightweightPredictiveModel', KinematicModel):
            predict_delays(snapshot, (None,None), (0., .2, .4))
        np.testing.assert_array_equal(snapshot.remaining_reference, before)
        self.assertEqual(snapshot.command.checkpoint_lower, (.7,1.3,1.15))
        self.assertEqual(snapshot.command.checkpoint_upper, (1.1,1.5,1.35))


if __name__ == '__main__':
    unittest.main()
