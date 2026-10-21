"""Immediate-rollout and control-boundary tests; no PyBullet flights."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from use_time_recheck import predict_immediate, first_step_after_check


class UseTimeRecheckTests(unittest.TestCase):
    def test_first_tick_strictly_after_completion(self):
        for duration, expected in ((0, 11), (.001, 11), (.019, 11), (.02, 12), (.021, 12), (.04, 13)):
            self.assertEqual(first_step_after_check(10, duration), expected)

    def model(self, start, end):
        ctrl = SimpleNamespace(select_action=lambda obs, info: None)
        model = SimpleNamespace(
            start=Mock(return_value=(ctrl, start, {}, np.zeros((3, 12)))),
            save_checkpoint=Mock(return_value="checkpoint"),
            restore_checkpoint=Mock(return_value=(start.copy(), {})),
            set_reference=Mock(), step=Mock(return_value=(end, False, {})), close=Mock(),
        )
        return model

    def test_one_immediate_rollout_uses_swept_checker_and_original_duration(self):
        snapshot = SimpleNamespace(box_x_min=1, box_x_max=2, box_y_limit=1,
                                   unsafe_below_z=1, horizon_steps=3, low_steps=2, low_height=.5)
        start, end = np.zeros(12), np.zeros(12)
        end[0], end[4] = 2, 2  # Safe endpoints, but segment touches unsafe corner.
        model = self.model(start, end)
        self.assertEqual(predict_immediate(snapshot, model), "UNSAFE")
        model.start.assert_called_once_with(snapshot)
        model.save_checkpoint.assert_called_once()
        model.restore_checkpoint.assert_called_once()
        model.step.assert_called_once()
        reference = model.set_reference.call_args.args[1]
        np.testing.assert_array_equal(reference[:, 4], [.5, .5, 0])
        model.close.assert_called_once()

    def test_initial_unsafe_state_rejected_without_rollout(self):
        snapshot = SimpleNamespace(box_x_min=1, box_x_max=2, box_y_limit=1, unsafe_below_z=1)
        state = np.zeros(12); state[0] = 1
        model = self.model(state, state)
        self.assertEqual(predict_immediate(snapshot, model), "UNSAFE")
        model.step.assert_not_called()
        model.close.assert_called_once_with({})

    def test_safe_command_rolls_to_existing_horizon_once(self):
        snapshot = SimpleNamespace(box_x_min=1, box_x_max=2, box_y_limit=1,
                                   unsafe_below_z=1, horizon_steps=3, low_steps=2, low_height=.5)
        start, end = np.zeros(12), np.zeros(12)
        end[0], end[4] = .5, 2
        model = self.model(start, end)
        self.assertEqual(predict_immediate(snapshot, model), "SAFE")
        self.assertEqual(model.step.call_count, 3)
        model.start.assert_called_once()


if __name__ == "__main__":
    unittest.main()
