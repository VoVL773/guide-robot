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

    def test_low_confidence_waits_then_resumes_only_after_health_clears(self):
        low = types.SimpleNamespace(id=0x03020003, severity="B类故障(ERROR)")
        self.n._chassis.get_robot_health.side_effect = [
            types.SimpleNamespace(baseError=[low]), types.SimpleNamespace(baseError=[])]
        self.n._check_localization = Mock(return_value=True)
        self.n._dispatch = Mock(return_value="id")
        self.n._run_action = Mock(return_value="success")
        self.assertEqual(self.n._navigate_leg(nav.Leg("B", 10, 0, 0),
                         nav.time.monotonic()+100, None), "success")
        self.n._check_localization.assert_called_once()
        self.assertEqual(self.n._chassis.get_robot_health.call_count, 2)

    def test_low_confidence_never_dispatches_when_recovery_fails(self):
        low = types.SimpleNamespace(id=0x03020003, severity="B类故障(ERROR)")
        self.n._chassis.get_robot_health.return_value = types.SimpleNamespace(baseError=[low])
        self.n._check_localization = Mock(return_value=False)
        self.n._dispatch = Mock()
        self.assertEqual(self.n._navigate_leg(nav.Leg("B", 10, 0, 0),
                         nav.time.monotonic()+100, None), "localization")
        self.n._dispatch.assert_not_called()

    def test_low_confidence_dtc_must_clear_before_dispatch(self):
        low = types.SimpleNamespace(id=0x03020003, severity="B类故障(ERROR)")
        self.n._chassis.get_robot_health.return_value = types.SimpleNamespace(baseError=[low])
        self.n._check_localization = Mock(return_value=True)
        self.n._dispatch = Mock()
        self.assertEqual(self.n._navigate_leg(nav.Leg("B", 10, 0, 0),
                         nav.time.monotonic()+100, None), "health")
        self.n._dispatch.assert_not_called()

    def test_maintenance_still_blocks_even_with_low_confidence(self):
        entries = [types.SimpleNamespace(id=code, severity="B类故障(ERROR)")
                   for code in [0x03020003, 0x01010117]]
        self.n._chassis.get_robot_health.return_value = types.SimpleNamespace(baseError=entries)
        self.n._check_localization = Mock()
        self.n._dispatch = Mock()
        self.assertEqual(self.n._navigate_leg(nav.Leg("B", 10, 0, 0),
                         nav.time.monotonic()+100, None), "health")
        self.n._check_localization.assert_not_called()
        self.n._dispatch.assert_not_called()

    def test_go_entry_allows_localization_recovery_before_health_recheck(self):
        low = types.SimpleNamespace(id=0x03020003, severity="B类故障(ERROR)")
        self.n._chassis.get_robot_health.side_effect = [
            types.SimpleNamespace(baseError=[low]), types.SimpleNamespace(baseError=[])]
        self.n._load_pois = Mock(return_value=self.n._pois)
        self.n._check_localization = Mock(return_value=True)
        self.n.plan_route = Mock(return_value=[nav.Leg("B", 10, 0, 0)])
        self.n._navigate_leg = Mock(return_value="success")
        self.n._cancel_action = Mock(return_value=True)
        self.assertEqual(self.n.go("B", 100), "success")
        self.n._check_localization.assert_called_once()
        self.n._navigate_leg.assert_called_once()
        self.assertEqual(self.n._chassis.get_robot_health.call_count, 2)

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
        self.n._read_pose = Mock(return_value={"x": 0, "y": 0, "yaw": 0, "confidence": .95})
        self.n._dispatch = Mock(return_value="id")

    def test_retry_waits_for_localization_instead_of_splitting(self):
        self.prepare_retry()
        self.n._run_action = Mock(side_effect=["failure", "success"])
        with patch.object(nav.time, "sleep"):
            result = self.n._navigate_leg(nav.Leg("B", 14, 0, 0), nav.time.monotonic()+100, None)
        self.assertEqual(result, "success")
        self.assertEqual(self.n._dispatch.call_count, 2)
        self.assertTrue(all(call.args[0].x == 14 for call in self.n._dispatch.call_args_list))
        self.assertEqual(self.n._check_localization.call_count, 2)

    def test_blocked_does_not_redispatch(self):
        self.prepare_retry()
        self.n._run_action = Mock(return_value="blocked")
        with patch.object(nav.time, "sleep"):
            self.assertEqual(self.n._navigate_leg(nav.Leg("B", 5, 0, 0),
                             nav.time.monotonic()+100, None), "blocked")
        self.assertEqual(self.n._dispatch.call_count, 1)

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
        self.n._dispatch.assert_not_called()

    def test_action_success_far_from_target_is_rejected(self):
        self.n._wait_slam_ok = Mock(return_value=True)
        self.n._chassis.get_action_status.return_value = {"status": "succeeded"}
        self.n._read_pose = Mock(return_value={"x": 0, "y": 0, "yaw": 0, "confidence": .95})
        self.n._cancel_and_settle = Mock()
        result = self.n._run_action("id", "B", nav.time.monotonic()+100, None,
                                    nav.Leg("B", 10, 0, 0))
        self.assertEqual(result, "failure")

    def test_action_success_at_target_is_accepted(self):
        self.n._wait_slam_ok = Mock(return_value=True)
        self.n._chassis.get_action_status.return_value = {"status": "succeeded"}
        self.n._read_pose = Mock(return_value={"x": 9.9, "y": 0, "yaw": 0, "confidence": .95})
        self.assertEqual(self.n._run_action("id", "B", nav.time.monotonic()+100,
                         None, nav.Leg("B", 10, 0, 0)), "success")

    def fake_clock(self):
        ticks = [0.0]
        def sleep(seconds):
            ticks[0] += seconds
        return ticks, types.SimpleNamespace(monotonic=lambda: ticks[0], sleep=sleep)

    def test_continuous_three_seconds_required(self):
        ticks, clock = self.fake_clock()
        self.n._slam_status = Mock(return_value=0)
        self.n._read_pose = Mock(return_value={"x":1,"y":0,"yaw":0,"confidence":.95})
        with patch.object(nav, "time", clock):
            self.assertTrue(self.n._wait_slam_ok(4))
        self.assertGreaterEqual(ticks[0], 3)

    def test_stable_localization_is_reused_between_legs(self):
        ticks, clock = self.fake_clock()
        self.n._slam_status = Mock(return_value=0)
        self.n._read_pose = Mock(return_value={"x":1,"y":0,"yaw":0,"confidence":.95})
        with patch.object(nav, "time", clock):
            self.assertTrue(self.n._wait_slam_ok(4))
            waited = ticks[0]
            self.assertTrue(self.n._wait_slam_ok(4))
        self.assertLess(ticks[0] - waited, 0.5)

    def test_configured_speed_is_sent_to_chassis_once(self):
        self.n._max_speed = 0.6
        self.n._apply_speed()
        self.n._apply_speed()
        self.n._chassis.set_speed_params.assert_called_once_with(
            max_moving_speed=0.6)

    def test_intermittent_recovery_never_passes(self):
        ticks, clock = self.fake_clock()
        self.n._slam_status = lambda: 0 if ticks[0] % 2 < 1.5 else 3
        self.n._read_pose = Mock(return_value={"x":1,"y":0,"yaw":0,"confidence":.95})
        with patch.object(nav, "time", clock):
            self.assertFalse(self.n._wait_slam_ok(8))
        self.assertLessEqual(ticks[0], 8)

    def test_zero_confidence_is_not_ready_even_with_status_zero(self):
        ticks, clock = self.fake_clock()
        self.n._slam_status = Mock(return_value=0)
        self.n._read_pose = Mock(return_value={"x":1,"y":0,"yaw":0,"confidence":0})
        with patch.object(nav, "time", clock):
            self.assertFalse(self.n._wait_slam_ok(4))

    def test_recovery_preserves_full_recent_pose_instead_of_nearby_poi(self):
        self.n._slam_status_stable = Mock(return_value=3)
        self.n._read_pose = Mock(return_value={"x":7,"y":.3,"yaw":3,"confidence":0})
        self.n._last_good_pose = (nav.time.monotonic(), {"x":7,"y":.3,"yaw":3,"confidence":.95})
        self.n._trigger_recover = Mock(return_value=True)
        self.n._wait_slam_ok = Mock(return_value=True)
        self.assertTrue(self.n.recover_here())
        self.n._trigger_recover.assert_called_once_with(
            7, .3, 3, timeout_s=nav._RECOVER_WAIT_S)
        self.assertFalse(self.n.recover_here())
        self.assertEqual(self.n._trigger_recover.call_count, 1)

    def test_recovery_rejects_zero_confidence_without_recent_pose(self):
        self.n._slam_status_stable = Mock(return_value=3)
        self.n._read_pose = Mock(return_value={"x":7,"y":.3,"yaw":3,"confidence":0})
        self.n._trigger_recover = Mock()
        self.assertFalse(self.n.recover_here())
        self.n._trigger_recover.assert_not_called()

    def test_idle_localization_restart_failure_is_actionable(self):
        self.n._chassis.get_localization_status.return_value = {
            "slam_status": 4, "slam_type": 0}
        self.n._chassis.get_current_map.return_value = {"map_name": "daolankuozhan"}
        self.n._chassis.get_pose.return_value = {"confidence": 0.0}
        self.n.restart_localization = Mock(return_value=False)
        ok, message = self.n.check_ready()
        self.assertFalse(ok)
        self.assertIn("定位服务没有启动", message)
        self.n._chassis.set_aeb_control.assert_not_called()

    def test_idle_localization_is_restarted_before_navigation(self):
        self.n._chassis.get_localization_status.return_value = {
            "slam_status": 4, "slam_type": 0}
        self.n._chassis.get_current_map.return_value = {"map_name": "daolankuozhan"}
        self.n._chassis.get_pose.return_value = {"confidence": 0.0}
        self.n._chassis.get_robot_health.return_value = types.SimpleNamespace(
            baseError=[], hasFatal=False, hasError=False)
        self.n.restart_localization = Mock(return_value=True)
        ok, message = self.n.check_ready()
        self.assertTrue(ok)
        self.assertEqual(message, "")
        self.n.restart_localization.assert_called_once()
        self.n._chassis.set_aeb_control.assert_called_once_with("On")

    def test_idle_localization_restarts_before_passive_wait(self):
        self.n._chassis.get_localization_status.return_value = {"slam_status": 4}
        self.n._chassis.get_current_map.return_value = {"map_name": "daolankuozhan"}
        self.n._chassis.get_pose.return_value = {"confidence": 0.0}
        self.n.restart_localization = Mock(return_value=True)
        self.n._wait_slam_ok = Mock(return_value=True)
        self.assertTrue(self.n._check_localization())
        self.n.restart_localization.assert_called_once()

    def test_recovery_pose_snaps_to_nearby_poi(self):
        self.n._read_pose = Mock(return_value={
            "x": 0.8, "y": 0.6, "yaw": 1.0, "confidence": 0.0})
        self.assertEqual(self.n._recovery_pose(), (0.0, 0.0, 0.0))

    def test_recovery_pose_refuses_untrusted_far_pose(self):
        self.n._read_pose = Mock(return_value={
            "x": 5.0, "y": 3.0, "yaw": 0.0, "confidence": 0.0})
        self.assertIsNone(self.n._recovery_pose())

    def test_finished_action_cannot_pass_with_unstable_localization(self):
        self.n._wait_slam_ok = Mock(return_value=False)
        self.n._chassis.get_action_status.return_value = {"status":"succeeded"}
        self.assertEqual(self.n._run_action("id", "B", nav.time.monotonic()+100,
                         None, nav.Leg("B",10,0,0)), "localization")
        self.n._chassis.cancel_action.assert_not_called()

    def test_maintenance_is_reported_as_health_not_obstacle(self):
        entry = types.SimpleNamespace(severity="A类故障(FATAL)",description="维护模式已启用")
        self.n._chassis.get_robot_health.return_value = types.SimpleNamespace(baseError=[entry])
        self.prepare_retry()
        del self.n._check_health
        self.assertEqual(self.n._navigate_leg(nav.Leg("B",10,0,0),
                         nav.time.monotonic()+100, None), "health")
        self.assertIn("维护模式", self.n.health_message)
        self.n._dispatch.assert_not_called()

    def test_terminal_failure_does_not_send_duplicate_cancel(self):
        self.n._chassis.get_action_status.return_value = {"status":"failed"}
        self.n._slam_status = Mock(return_value=3)
        self.n._read_pose = Mock(return_value={"confidence":0})
        self.assertEqual(self.n._run_action("id","B",nav.time.monotonic()+100,None), "failure")
        self.n._chassis.cancel_action.assert_not_called()


    def test_persistent_localization_loss_cancels_active_task(self):
        ticks, clock = self.fake_clock()
        self.n._chassis.get_action_status.return_value = {"status":"running"}
        self.n._slam_status = Mock(return_value=3)
        self.n._read_pose = Mock(return_value={"x":1,"y":0,"yaw":0,"confidence":0})
        self.n._cancel_and_settle = Mock()
        with patch.object(nav, "time", clock):
            result = self.n._run_action("id", "B", 100, None, nav.Leg("B",10,0,0))
        self.assertEqual(result,"failure")
        self.n._cancel_and_settle.assert_called_once_with("id")
        self.assertGreaterEqual(ticks[0], nav._MOTION_BAD_CANCEL_S)
        self.assertLess(ticks[0], nav._MOTION_BAD_CANCEL_S + 5)

    def test_localization_flicker_with_progress_does_not_cancel(self):
        ticks, clock = self.fake_clock()
        reads = {"n": 0}

        def fake_pose():
            i = reads["n"]
            reads["n"] += 1
            if i < 5:
                return {"x": 20.0 - i, "y": 0, "yaw": 0, "confidence": 0.0}
            return {"x": 9.9, "y": 0, "yaw": 0, "confidence": 0.95}

        self.n._read_pose = Mock(side_effect=fake_pose)
        self.n._slam_status = Mock(return_value=3)
        self.n._chassis.get_action_status.side_effect = (
            [{"status": "running"}] * 60 + [{"status": "succeeded"}])
        self.n._cancel_and_settle = Mock()
        self.n._wait_slam_ok = Mock(return_value=True)
        with patch.object(nav, "time", clock):
            result = self.n._run_action(
                "id", "B", 100, None, nav.Leg("B", 10, 0, 0))
        self.assertEqual(result, "success")
        self.n._cancel_and_settle.assert_not_called()


    def test_dispatch_timeout_is_uncertain_and_not_blindly_retried(self):
        self.n._check_health = Mock(return_value=True)
        self.n._check_localization = Mock(return_value=True)
        self.n._chassis.move_to.side_effect = TimeoutError("timeout after PUT")
        self.assertEqual(self.n._navigate_leg(nav.Leg("B",10,0,0),
                         nav.time.monotonic()+100, None), "network")
        self.assertTrue(self.n._pending_cancel)
        self.n._chassis.move_to.assert_called_once()

    def test_two_failed_status_queries_leave_running_without_waiting_full_timeout(self):
        ticks, clock = self.fake_clock()
        self.n._chassis.get_action_status.side_effect = TimeoutError("offline")
        with patch.object(nav, "time", clock):
            self.assertEqual(self.n._run_action("id","B",100,None), "network")
        self.assertEqual(self.n._chassis.get_action_status.call_count, 2)
        self.assertTrue(self.n._pending_cancel)
        self.assertEqual(self.n._active_action_id,"id")
        self.assertLess(ticks[0], 1)

    def test_failed_cancel_retains_pending_task_and_blocks_new_go(self):
        self.n._pending_cancel = True
        self.n._active_action_id = "old"
        self.n._chassis.cancel_action.side_effect = ConnectionError("offline")
        self.assertEqual(self.n.go("B",100), "network")
        self.assertTrue(self.n._pending_cancel)
        self.n._chassis.move_to.assert_not_called()

    def test_reconnected_cancel_requires_terminal_confirmation(self):
        self.n._pending_cancel = True
        self.n._active_action_id = "old"
        self.n._chassis.cancel_action.return_value = {"success":True}
        self.n._chassis.get_action_status.return_value = {"status":"running"}
        self.assertFalse(self.n._cancel_action())
        self.assertTrue(self.n._pending_cancel)
        self.n._chassis.get_action_status.return_value = {"status":"cancelled"}
        self.assertTrue(self.n._cancel_action())
        self.assertFalse(self.n._pending_cancel)
        self.assertIsNone(self.n._active_action_id)

    def test_cancel_without_ack_does_not_clear_uncertainty(self):
        self.n._pending_cancel = True
        self.n._chassis.cancel_action.return_value = {}
        self.assertFalse(self.n._cancel_action())
        self.assertTrue(self.n._pending_cancel)

    def test_offline_localization_exits_after_two_failed_requests(self):
        ticks, clock = self.fake_clock()
        self.n._chassis.get_localization_status.side_effect = TimeoutError("offline")
        with patch.object(nav, "time", clock):
            self.assertFalse(self.n._wait_slam_ok(20))
        self.assertEqual(self.n._chassis.get_localization_status.call_count,2)
        self.assertTrue(self.n._network_failed)
        self.assertLess(ticks[0],1)

    def test_completed_task_cleanup_does_not_cancel_other_tasks(self):
        self.assertTrue(self.n._cancel_action())
        self.n._chassis.cancel_action.assert_not_called()


    def test_unconfirmed_cancel_keeps_aeb_enabled(self):
        self.n._check_health = Mock(return_value=True)
        self.n._check_localization = Mock(return_value=True)
        self.n.plan_route = Mock(return_value=[nav.Leg("B",10,0,0)])
        self.n._navigate_leg = Mock(return_value="network")
        self.n._cancel_action = Mock(return_value=False)
        self.n._set_aeb = Mock()
        self.assertEqual(self.n.go("B",100), "network")
        self.n._set_aeb.assert_called_once_with(True)

    def test_track_mode_uses_track_api(self):
        self.n._track_id = 1001
        self.n._load_pois = Mock(return_value={"厕所": (42.55, -0.32, 0.0)})
        self.n._chassis.move_to_locations_poi.return_value = {
            "action_id": "aid"}
        self.n._run_action = Mock(return_value="success")
        self.assertEqual(
            self.n._navigate_by_track("厕所", 100, None), "success")
        self.n._chassis.move_to_locations_poi.assert_called_once_with(
            track_id=1001, poi_name="厕所", uptrack_enabled=True)
        self.n._chassis.move_to.assert_not_called()



if __name__ == "__main__":
    unittest.main()
