"""Fixed expiration boundaries and native dispatch integration, without PyBullet."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from staleguard.inspection_descent import simulation as b


class FixedTTLTests(unittest.TestCase):
    def test_inclusive_lifetime_on_control_grid(self):
        for ttl_steps in range(0, 51, 5):
            self.assertFalse(b.fixed_ttl_expired_at_dispatch(ttl_steps, ttl_steps))
            self.assertTrue(b.fixed_ttl_expired_at_dispatch(ttl_steps, ttl_steps+1))
            if ttl_steps:
                self.assertFalse(b.fixed_ttl_expired_at_dispatch(ttl_steps, ttl_steps-1))

    def flight(self, ttl_steps, waited_steps):
        scenario = SimpleNamespace(box_x_min=1, box_x_max=2, box_y_limit=.35,
                                   unsafe_below_z=1, usual_height=1.2, low_height=.5, speed=.1)
        snapshot = SimpleNamespace(**vars(scenario), approval_time=.04, low_steps=2)
        states = np.zeros((13, 12))
        states[:, 0] = np.arange(13)*.01
        states[:, 4] = 1.2
        env = SimpleNamespace(X_GOAL=np.zeros((15, 12)), CTRL_FREQ=50,
                              reset=lambda **kw: (states[0].copy(), {}), close=lambda: None)
        index = 0
        def advance(action):
            nonlocal index
            index += 1
            return states[index].copy(), 0, False, {}
        env.step = advance
        ctrl = SimpleNamespace(reset_before_run=lambda **kw: None,
                               select_action=lambda *args: None, close=lambda: None)
        trace, event, checks = [], {}, []
        dispatch = 2+waited_steps
        expiry = b.fixed_ttl_expired_at_dispatch
        def checked(ttl, waited):
            checks.append((trace[-1][0], ttl, waited))
            return expiry(ttl, waited)
        with patch.object(b, "make_scg_env_factory", return_value=lambda **kw: env), \
             patch.object(b, "make", return_value=ctrl), \
             patch.object(b, "monitor_physics_steps", return_value=lambda: None), \
             patch.object(b, "fixed_ttl_expired_at_dispatch", side_effect=checked), \
             patch("staleguard.inspection_descent.baselines.staleguard.predict", side_effect=AssertionError("No predictive model")), \
             patch.object(b, "predict_immediate", side_effect=AssertionError("No recheck")), \
             patch.object(b.fbs, "clearance", side_effect=AssertionError("No state feature")):
            outcome = b.run_actual(scenario, snapshot, dispatch, dispatch, 12, False,
                                   trace=trace, fixed_ttl_steps=ttl_steps, fixed_ttl_log=event)
        self.assertEqual(checks, [(dispatch, ttl_steps, waited_steps)])
        self.assertEqual(event["approval_step"], 2)
        self.assertEqual(event["expiry_step"], 2+ttl_steps)
        self.assertEqual(len(trace), 13)  # The fixed deadline never moves.
        for row in trace[1:dispatch+1]:
            self.assertEqual(row[2][4], 1.2)  # No early cancellation or descent.
        return outcome, trace, event

    def test_before_at_and_after_expiration(self):
        for waited in (4, 5, 6):  # 80, 100, 120 ms after approval, TTL=100 ms.
            outcome, trace, event = self.flight(5, waited)
            expired = waited == 6
            dispatch = 2+waited
            self.assertEqual(outcome[2], expired)
            self.assertEqual(event["expired"], expired)
            self.assertEqual(event["fallback_step"], dispatch if expired else None)
            self.assertEqual(event["application_step"], None if expired else dispatch)
            heights = [row[2][4] for row in trace[dispatch+1:]]
            self.assertEqual(heights[:2], [1.2, 1.2] if expired else [.5, .5])
            self.assertTrue(all(z == 1.2 for z in heights[2:]))

    def test_zero_ttl_allows_immediate_dispatch_then_expires(self):
        self.assertFalse(self.flight(0, 0)[0][2])
        self.assertTrue(self.flight(0, 1)[0][2])

    def test_cannot_combine_with_other_policy_decisions(self):
        for kwargs in ({"recover": True},
                       {"ttl_prediction": ({}, 0)}, {"fbs_log": {}},
                       {"use_time_check": lambda state: "SAFE"}):
            params = dict(s=None, snapshot=None, dispatch_step=0, recover_step=0,
                          deadline_step=1, recover=False, fixed_ttl_steps=0)
            params.update(kwargs)
            with self.assertRaises(ValueError):
                b.run_actual(**params)


if __name__ == "__main__":
    unittest.main()
