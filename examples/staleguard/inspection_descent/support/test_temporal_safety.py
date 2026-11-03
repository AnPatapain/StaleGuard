"""Geometry boundaries, inter-sample checks, and physics observation timing."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from inspection_descent.scenario import segment_intersects_box
from inspection_descent.scenario import unsafe, unsafe_segment
from physics_monitor import monitor_physics_steps
from inspection_descent.baselines.staleguard import build_nominal_checkpoints, rollout_from_checkpoint


class TemporalSafetyTests(unittest.TestCase):
    def test_stationary_segments_match_existing_unsafe_predicate(self):
        region = SimpleNamespace(box_x_min=1, box_x_max=2, box_y_limit=1, unsafe_below_z=1)
        for x in (0, 1, 1.5, 2, 3):
            for y in (-2, -1, 0, 1, 2):
                for z in (0, 1, 2):
                    state = np.zeros(12)
                    state[[0, 2, 4]] = x, y, z
                    self.assertEqual(unsafe_segment(state, state, region), unsafe(state, region))

    def test_closed_box_segments(self):
        lower, upper = (1, -1, -math.inf), (2, 1, 1)
        cases = [
            ((0, 0, 0), (3, 0, 0), True),       # Both endpoints outside.
            ((3, 0, 0), (0, 0, 0), True),       # Reverse direction.
            ((0, 2, 0), (3, 2, 0), False),      # Parallel to an outside face.
            ((0, 1, 0), (3, 1, 0), True),       # On a closed side boundary.
            ((0, 0, 2), (3, 0, 2), False),      # Above the unsafe region.
            ((0, 0, 0), (2, 0, 2), True),       # Touches one corner at (1,0,1).
            ((1, 0, 1), (1, 0, 1), True),       # Stationary boundary point.
            ((0, 0, 0), (0, 0, 0), False),      # Stationary outside point.
            ((1.5, 0, -100), (1.5, 0, 0), True),
            ((0, math.nextafter(1, math.inf), 0),
             (3, math.nextafter(1, math.inf), 0), False),  # No margin.
        ]
        for start, end, expected in cases:
            with self.subTest(start=start, end=end):
                self.assertEqual(segment_intersects_box(start, end, lower, upper), expected)

    def test_crossing_is_checked_in_nominal_and_command_rollouts(self):
        start = [0.0] * 12
        end = [0.0] * 12
        start[0], start[4] = 0, 0
        end[0], end[4] = 2, 2
        snapshot = SimpleNamespace(box_x_min=1, box_x_max=2, box_y_limit=1,
                                   unsafe_below_z=1, horizon_steps=1,
                                   low_steps=1, low_height=0)
        reference = np.zeros((1, 12))
        ctrl = SimpleNamespace(select_action=lambda obs, info: None)
        model = SimpleNamespace(
            restore_checkpoint=lambda ctrl, checkpoint: (start, {}),
            set_reference=lambda ctrl, reference: None,
            save_checkpoint=lambda ctrl, obs, info: None,
            step=lambda action, obs, info: (end, False, {}),
        )
        self.assertEqual(rollout_from_checkpoint(model, ctrl, None, reference, snapshot, 0), "UNSAFE")
        nominal_safe, _ = build_nominal_checkpoints(model, ctrl, start, {}, snapshot, [0.0])
        self.assertFalse(nominal_safe)

    def test_monitor_observes_endpoints_once_and_preserves_steps(self):
        position = [0.0, 0.0, 0.0]
        actions, samples = [], []
        def physics(action):
            actions.append(action)
            position[0] += 1
        env = SimpleNamespace(DRONE_ID=1, PYB_CLIENT=2, _physics=physics)
        with patch("physics_monitor.p.getBasePositionAndOrientation",
                   side_effect=lambda *args, **kwargs: (tuple(position), None)):
            sample = monitor_physics_steps(env, lambda pos, tick: samples.append((tick, pos)))
            sample()
            sample()
            env._physics(11)
            sample()
            env._physics(22)
            sample()
            sample()
        self.assertEqual(actions, [11, 22])
        self.assertEqual(samples, [(0, (0, 0, 0)), (1, (1, 0, 0)), (2, (2, 0, 0))])


if __name__ == "__main__":
    unittest.main()
