"""FBS arithmetic and dispatch integration, without real PyBullet flights."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np
from inspection_descent.baselines import fbs
from inspection_descent import simulation as b


class FBSTests(unittest.TestCase):
    def test_margin_sign_matches_closed_unsafe_volume(self):
        geometry = SimpleNamespace(box_x_min=1, box_x_max=2, box_y_limit=1, unsafe_below_z=1)
        for x in (0, 1, 1.5, 2, 3):
            for y in (-2, -1, 0, 1, 2):
                for z in (0, 1, 2):
                    state = np.zeros(12)
                    state[[0, 2, 4]] = x, y, z
                    self.assertEqual(fbs.clearance(state, geometry) <= 0, b.unsafe(state, geometry))

    def test_ema_initialization_and_approval_update(self):
        previous, volatility = None, 0
        for feature, expected in ((1, 0), (.9, .01), (.7, .029)):
            volatility = fbs.update_volatility(feature, previous, volatility)
            self.assertAlmostEqual(volatility, expected)
            previous = feature
        prediction = fbs.freeze_prediction(previous, volatility)
        self.assertEqual(prediction["ttl_steps"], 24)  # metres / (metres/sample).
        self.assertAlmostEqual(prediction["delta_steps"], .7/.029)

    def test_numerical_floor_cap_and_invalid_margin(self):
        self.assertEqual(fbs.freeze_prediction(1, 0)["ttl_steps"], 210)
        self.assertEqual(fbs.freeze_prediction(.5, 0, epsilon=.1)["ttl_steps"], 5)
        for margin in (0, -.1):
            prediction = fbs.freeze_prediction(margin, .01)
            self.assertTrue(fbs.dispatch_status(prediction, 0, 0)["recover"])
        with self.assertRaises(ValueError):
            fbs.freeze_prediction(1, 0, epsilon=0)

    def test_expiry_is_strict_and_readiness_is_inclusive(self):
        prediction = fbs.freeze_prediction(.1, .02)
        self.assertFalse(fbs.dispatch_status(prediction, .1, 5)["recover"])
        self.assertTrue(fbs.dispatch_status(prediction, .12, 6)["recover"])
        self.assertFalse(fbs.dispatch_status(prediction, .121, 6)["recover"])
        self.assertEqual(fbs.dispatch_status(prediction, .001, 0)["reason"], "unavailable")

    def flight(self, duration, dispatch):
        scenario = SimpleNamespace(box_x_min=3, box_x_max=4, box_y_limit=1,
                                   unsafe_below_z=1, usual_height=.9, low_height=.5,
                                   speed=.1, low_seconds=.04)
        states = np.zeros((9, 12))
        states[:, 0] = [1, 2, 2.9, 3, 3.1, 3.2, 3.3, 3.4, 3.5]
        states[:, 4] = .9
        snapshot = SimpleNamespace(**vars(scenario), approval_time=.04, low_steps=2)
        env = SimpleNamespace(X_GOAL=np.zeros((10, 12)), CTRL_FREQ=50,
                              reset=lambda **kw: (states[0].copy(), {}), close=lambda: None)
        index = 0
        def advance(action):
            nonlocal index
            index += 1
            return states[index].copy(), 0, False, {}
        env.step = advance
        ctrl = SimpleNamespace(reset_before_run=lambda **kw: None,
                               select_action=lambda *args: None, close=lambda: None)
        clock = [value for i in range(3) for value in (i, i+duration)]
        trace, event = [], {}
        with patch.object(b, "make_scg_env_factory", return_value=lambda **kw: env), \
             patch.object(b, "make", return_value=ctrl), \
             patch.object(b, "monitor_physics_steps", return_value=lambda: None), \
             patch.object(b.time, "perf_counter", side_effect=clock):
            result = b.run_actual(scenario, snapshot, dispatch, dispatch, 8, False,
                                  trace=trace, fbs_log=event)
        return scenario, states, result, trace, event

    def test_actual_prefix_freezes_at_approval_then_falls_back_at_dispatch(self):
        scenario, states, result, trace, event = self.flight(.001, 3)
        self.assertEqual(event["observation_steps"], [0, 1, 2])
        np.testing.assert_array_equal(event["observation_state"], states[2])
        self.assertAlmostEqual(event["prediction"]["volatility_m_per_sample"], .18)
        self.assertEqual(event["prediction"]["ttl_steps"], 0)
        self.assertTrue(result[2])
        self.assertEqual(event["fallback_step"], 3)
        self.assertEqual(trace[4][2][4], scenario.usual_height)

    def test_unavailable_executes_on_schedule_and_never_cancels_later(self):
        scenario, states, result, trace, event = self.flight(.1, 3)
        self.assertEqual(event["reason"], "unavailable")
        self.assertFalse(result[2])
        self.assertEqual(event["application_step"], 3)
        self.assertEqual(len(trace), 9)
        self.assertEqual([row[2][4] for row in trace[1:]],
                         [.9, .9, .9, .5, .5, .9, .9, .9])

    def test_zero_age_fails_open_despite_expired_zero_step_ttl(self):
        scenario, states, result, trace, event = self.flight(.001, 2)
        self.assertEqual(event["prediction"]["ttl_steps"], 0)
        self.assertFalse(event["ready"])
        self.assertFalse(result[2])
        self.assertEqual(event["application_step"], 2)
        self.assertEqual(trace[3][2][4], scenario.low_height)


if __name__ == "__main__":
    unittest.main()
