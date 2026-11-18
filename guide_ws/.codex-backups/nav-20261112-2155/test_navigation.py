"""Pure navigation regressions: fake chassis only, never sends robot commands."""
import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


def load_navigation():
    spec = importlib.util.spec_from_file_location(
        "tested_navigation", Path(__file__).parents[1] / "guide_voice/navigation.py")
    module = importlib.util.module_from_spec(spec)
    stub = types.ModuleType("robot_controller.chassis")
    stub.KnewbotChassisNav = Mock()
    with patch.dict(sys.modules, {"robot_controller": types.ModuleType("robot_controller"),
                                  "robot_controller.chassis": stub}):
        spec.loader.exec_module(module)
    return module


nav = load_navigation()


class NavigationTest(unittest.TestCase):
    def setUp(self):
        self.n = nav.Navigator("unused", 0, 1, Mock())
        self.n._chassis = Mock()
        self.n._pois = {"A": (0., 0., 0.), "B": (10., 0., 0.),
                        "C": (20., 0., 0.), "D": (20., 5., 0.)}
        self.n.set_route(["A", "B", "C"], {"D": "C"})

    def test_forward_resume_keeps_unreached_nearest_point(self):
        self.assertEqual(self.n._route_names((7., 0.), "C"), ["B", "C"])

    def test_reverse_resume_keeps_unreached_nearest_point(self):
        self.assertEqual(self.n._route_names((13., 0.), "A"), ["B", "A"])

    def test_branch_keeps_attachment(self):
        self.assertEqual(self.n._route_names((7., 0.), "D"), ["B", "C", "D"])

    def test_branch_exit_goes_via_attachment(self):
        self.assertEqual(self.n._route_names((20., 5.), "A"), ["C", "B", "A"])

    def test_failed_done_is_failure(self):
        self.assertEqual(nav.classify({"status": "failed", "done": True}), "failure")

    def test_done_alone_is_not_arrival(self):
        self.assertEqual(nav.classify({"done": True}), "running")

    def test_request_success_is_not_action_success(self):
        self.assertEqual(nav.classify({"success": True, "status": "running"}), "running")

    def test_dispatch_uses_computed_yaw(self):
        self.n._chassis.move_to.return_value = {"action_id": "id"}
        self.assertEqual(self.n._dispatch(nav.Leg("A", 0, 0, math.pi)), "id")
        self.n._chassis.move_to.assert_called_once_with(0, 0, yaw=math.pi, flags=0)
        self.n._chassis.move_to_poi.assert_not_called()

    def test_missing_action_id_is_rejected(self):
        self.n._chassis.move_to.return_value = {"action_id": None}
        self.assertIsNone(self.n._dispatch(nav.Leg("A", 0, 0, 0)))

    def test_error_response_with_id_is_rejected(self):
        self.n._chassis.move_to.return_value = {"success": False, "action_id": "id"}
        self.assertIsNone(self.n._dispatch(nav.Leg("A", 0, 0, 0)))

    def prepare_retry(self):
        self.n._check_localization = Mock(return_value=True)
        self.n._check_health = Mock(return_value=True)
        self.n._read_pose = Mock(return_value={"x": 0, "y": 0})
        self.n._dispatch = Mock(return_value="id")

    def test_failed_long_leg_is_split_and_reaches_original_target(self):
        self.prepare_retry()
        self.n._run_action = Mock(side_effect=["failure", "success", "success", "success"])
        with patch.object(nav.time, "sleep"):
            result = self.n._navigate_leg(nav.Leg("B", 5, 0, 0), nav.time.monotonic()+100, None)
        self.assertEqual(result, "success")
        legs = [call.args[0] for call in self.n._dispatch.call_args_list]
        self.assertEqual(len(legs), 4)
        positions = [0] + [leg.x for leg in legs[1:]]
        self.assertTrue(all(b-a <= 2.00001 for a,b in zip(positions, positions[1:])))
        self.assertEqual(legs[-1].x, 5)

    def test_blocked_recovery_does_not_continue_to_later_steps(self):
        self.prepare_retry()
        self.n._run_action = Mock(side_effect=["failure", "blocked"])
        with patch.object(nav.time, "sleep"):
            self.assertEqual(self.n._navigate_leg(nav.Leg("B", 5, 0, 0),
                             nav.time.monotonic()+100, None), "blocked")
        self.assertEqual(self.n._dispatch.call_count, 2)

    def test_expired_budget_never_dispatches(self):
        self.prepare_retry()
        self.assertEqual(self.n._navigate_leg(nav.Leg("B", 5, 0, 0),
                         nav.time.monotonic()-1, None), "timeout")
        self.n._dispatch.assert_not_called()

    def test_localization_failure_stops_retry(self):
        self.prepare_retry()
        self.n._check_localization.return_value = False
        self.n._run_action = Mock(return_value="failure")
        with patch.object(nav.time, "sleep"):
            self.assertEqual(self.n._navigate_leg(nav.Leg("B", 5, 0, 0),
                             nav.time.monotonic()+100, None), "localization")
        self.assertEqual(self.n._dispatch.call_count, 1)

    def test_action_success_far_from_target_is_rejected(self):
        self.n._chassis.get_action_status.return_value = {"status": "succeeded"}
        self.n._read_pose = Mock(return_value={"x": 0, "y": 0})
        self.n._cancel_and_settle = Mock()
        result = self.n._run_action("id", "B", nav.time.monotonic()+100, None,
                                    nav.Leg("B", 10, 0, 0))
        self.assertEqual(result, "failure")

    def test_action_success_at_target_is_accepted(self):
        self.n._chassis.get_action_status.return_value = {"status": "succeeded"}
        self.n._read_pose = Mock(return_value={"x": 9.9, "y": 0})
        self.assertEqual(self.n._run_action("id", "B", nav.time.monotonic()+100,
                         None, nav.Leg("B", 10, 0, 0)), "success")


if __name__ == "__main__":
    unittest.main()
