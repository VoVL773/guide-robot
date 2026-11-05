"""导览编排节点：整条参观流程的唯一指挥者。

流程：
    起点（点对点） -> 开场白 -> 每个讲解点（沿轨道导航 -> 头部朝向 -> 讲解
    -> 停留） -> 结束语 -> 沿返程轨道回到起点

设计约束：
- 只有本节点调用 Action/服务，导航、头部、语音三个节点都不互调用；
- 点位与顺序全部来自 YAML，代码里没有任何 POI_xx 常量；
- 导航点/讲解词缺失时，启动即给出明确错误并保持 idle，不会崩溃；
- 通过 /guide/command 服务支持 start / pause / resume / stop / goto。
"""

from __future__ import annotations

import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from guide_core import (
    PointsConfig,
    ScriptConfig,
    load_points,
    load_script,
    missing_scripts,
)

from guide_interfaces.action import NavigateTo, Speak
from guide_interfaces.msg import GuideStatus
from guide_interfaces.srv import GuideCommand, SetHeadPose


def _share_path(*parts: str) -> str:
    return os.path.join(get_package_share_directory("guide_orchestrator"), *parts)


class GuideOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("guide_orchestrator")

        self.declare_parameter("points_config", _share_path("config", "guide_points.yaml"))
        self.declare_parameter("script_config", _share_path("config", "guide_script.yaml"))
        self.declare_parameter("auto_start", True)
        self.declare_parameter("enable_head", False)

        self._stop = threading.Event()
        self._paused = threading.Event()
        self._active_goal = None
        self._worker = None
        self._state = "idle"
        self.enable_head = bool(self.get_parameter("enable_head").value)

        self.cfg: PointsConfig | None = None
        self.script: ScriptConfig | None = None
        self._problems: list[str] = []

        try:
            self.cfg = load_points(str(self.get_parameter("points_config").value))
            self.get_logger().info(f"点位配置：{self.cfg.source}")
        except Exception as exc:  # noqa: BLE001
            self._problems.append(f"点位配置加载失败：{exc}")
            self.get_logger().error(self._problems[-1])

        try:
            self.script = load_script(str(self.get_parameter("script_config").value))
            self.get_logger().info(f"讲解配置：{self.script.source}")
        except Exception as exc:  # noqa: BLE001
            self._problems.append(f"讲解配置加载失败：{exc}")
            self.get_logger().error(self._problems[-1])

        if self.cfg is not None and self.script is not None:
            if not self.cfg.marked:
                self._problems.append(
                    "guide_points.yaml 还未标记点位：请先在底盘地图上标记 POI，"
                    "再把 start_poi 与 poi_order 填好")
            missing = missing_scripts(self.cfg, self.script)
            if missing:
                self.get_logger().warn(
                    f"这些点位还没有讲解词，将只导航不播报：{missing}")

        self.group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(self, NavigateTo, "/guide/navigate_to")
        self._speak_client = ActionClient(self, Speak, "/guide/speak")
        self._head_client = self.create_client(SetHeadPose, "/guide/set_head_pose")
        self.status_pub = self.create_publisher(GuideStatus, "/guide/status", 10)

        self.create_service(
            GuideCommand, "/guide/command", self._on_command,
            callback_group=self.group)

        for problem in self._problems:
            self.get_logger().error(f"导览不可启动：{problem}")
        if self._problems:
            self.get_logger().error(
                "节点保持 idle；修好 YAML 后重启，或用 /guide/command 的 start 重试")

        self._publish("idle", "idle", "", 0, "；".join(self._problems) or "等待 start")

        if bool(self.get_parameter("auto_start").value) and not self._problems:
            self.get_logger().info("auto_start=true，自动开始导览")
            self._start_worker()

    # ------------------------------------------------------------------ 命令
    def _on_command(self, request, response):
        command = request.command.strip().lower()
        if command == "start":
            if self._problems or self.cfg is None:
                response.accepted = False
                response.message = "；".join(self._problems) or "配置未就绪"
                return response
            if self._worker is not None and self._worker.is_alive():
                response.accepted = False
                response.message = "导览正在运行，先 stop 或等待结束"
                return response
            self._start_worker(request.poi_name.strip() or None)
            response.accepted = True
            response.message = "已开始导览"
        elif command == "pause":
            self._paused.set()
            self._publish("paused", "paused", "", 0, "已在步骤边界暂停")
            response.accepted = True
            response.message = "已暂停（当前动作完成后生效）"
        elif command == "resume":
            self._paused.clear()
            self._publish("running", "resumed", "", 0, "已恢复")
            response.accepted = True
            response.message = "已恢复"
        elif command == "stop":
            self._stop_tour()
            response.accepted = True
            response.message = "已请求停止"
        elif command == "goto":
            poi = request.poi_name.strip()
            if not poi or self.cfg is None or poi not in self.cfg.poi_order:
                response.accepted = False
                response.message = f"goto 需要 poi_order 中的点位，收到 {poi!r}"
                return response
            if self._worker is not None and self._worker.is_alive():
                response.accepted = False
                response.message = "导览运行中不能跳点，先 stop 再 start 并带上目标点位"
                return response
            self._start_worker(poi)
            response.accepted = True
            response.message = f"已从 {poi} 开始导览"
        else:
            response.accepted = False
            response.message = f"未知命令 {request.command!r}"
        return response

    def _start_worker(self, first_poi: str | None = None) -> None:
        self._stop.clear()
        self._paused.clear()
        self._worker = threading.Thread(
            target=self._run_tour, args=(first_poi,), daemon=True)
        self._worker.start()

    def _stop_tour(self) -> None:
        self._stop.set()
        handle = self._active_goal
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"取消当前目标失败：{exc}")

    # ------------------------------------------------------------------ 主流程
    def _run_tour(self, first_poi: str | None) -> None:
        cfg = self.cfg
        script = self.script
        assert cfg is not None and script is not None

        try:
            order = list(cfg.poi_order)
            index = 0
            if first_poi is not None:
                index = order.index(first_poi)

            self._state = "running"
            # 起点只用点对点；goto 时直接去目标点位，避免先回起点绕路
            if first_poi is None:
                if not self._navigate(cfg.start_poi, False, "",
                                      cfg.timeouts.to_start_s, "前往起点",
                                      index=0, total=len(order)):
                    return
                if script.intro and not self._speak(script.intro, "开场白"):
                    return

            while index < len(order):
                if not self._wait_ready():
                    return
                poi = order[index]
                # transit_pois 里的点不在去程轨道上，只能点对点过去
                use_track = poi not in cfg.transit_pois
                track = cfg.tracks.outbound if use_track else ""
                if not self._navigate(poi, use_track, track,
                                      cfg.timeouts.per_poi_s, "前往讲解点",
                                      index=index + 1, total=len(order)):
                    return

                point_script = script.point(poi)
                if not self._wait_ready():
                    return
                if self.enable_head:
                    self._set_head(point_script.head_yaw, point_script.head_pitch, poi)

                if not self._wait_ready():
                    return
                if point_script.text:
                    if not self._speak(point_script.text, poi):
                        return
                else:
                    self.get_logger().warn(f"{poi} 没有讲解词，只导航不播报")

                if not self._dwell(cfg.dwell_for(poi), poi):
                    return
                index += 1

            if script.outro and not self._speak(script.outro, "结束语"):
                return

            # 只有配置了返程轨道才返程；否则停在最后一个点位（单轨地图走一圈）
            if cfg.tracks.return_track:
                if not self._wait_ready():
                    return
                if not self._navigate(cfg.start_poi, True, cfg.tracks.return_track,
                                      cfg.timeouts.to_start_s, "返回起点",
                                      index=len(order), total=len(order)):
                    return
            else:
                self.get_logger().info("未配置返程轨道，停在最后一个点位")

            self._state = "finished"
            self._publish("finished", "idle", "", len(order), "导览结束")
            self.get_logger().info("导览结束")
        except Exception as exc:  # noqa: BLE001 - 任何异常都要落到状态，不能静默死线程
            self._state = "error"
            self._publish("error", "idle", "", 0, f"导览异常：{exc}")
            self.get_logger().error(f"导览异常：{exc}")
        finally:
            self._active_goal = None

    def _navigate(self, poi: str, use_track: bool, track_id: str,
                  timeout: float, stage: str, index: int, total: int) -> bool:
        """调用导航 Action；返回是否成功到达。"""
        if not self._wait_ready():
            return False
        self._publish("running", stage, poi, index, f"导航前往 {poi}")
        self.get_logger().info(f"{stage}：{poi}（轨道={track_id or '点对点'}）")

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self._fail(f"导航 Action 服务未就绪，无法前往 {poi}")
            return False

        goal = NavigateTo.Goal()
        goal.poi_name = poi
        goal.use_track = bool(use_track)
        goal.track_id = track_id

        try:
            send_future = self._nav_client.send_goal_async(goal)
            goal_handle = self._await_future(send_future, 15.0)
            if goal_handle is None:
                return self._aborted("发送导航目标超时")
            if not goal_handle.accepted:
                self._fail(f"导航目标被拒绝：{poi}")
                return False
            self._active_goal = goal_handle
            result_future = goal_handle.get_result_async()
            wrapped = self._await_future(result_future, timeout + 30.0)
        finally:
            self._active_goal = None

        if wrapped is None:
            if self._stop.is_set():
                self._finish_stopped()
                self.get_logger().info(f"已停止：{poi} 导航中断")
            else:
                self._fail(f"等待导航结果超时：{poi}")
            return False
        result = wrapped.result
        if not result.success:
            self._fail(f"导航到 {poi} 失败（{result.state}）：{result.message}")
            return False
        self.get_logger().info(f"已到达 {poi}：{result.message}")
        return True

    def _speak(self, text: str, stage: str) -> bool:
        if not self._wait_ready():
            return False
        self._publish("running", "speaking", stage, 0, f"讲解：{stage}")
        self.get_logger().info(f"播报（{stage}）：{text}")

        if not self._speak_client.wait_for_server(timeout_sec=10.0):
            self._fail("语音 Action 服务未就绪")
            return False

        goal = Speak.Goal()
        goal.text = text
        goal.interrupt = False

        try:
            send_future = self._speak_client.send_goal_async(goal)
            goal_handle = self._await_future(send_future, 15.0)
            if goal_handle is None:
                return self._aborted("发送播报目标超时")
            if not goal_handle.accepted:
                self._fail("播报目标被拒绝（可能有另一条播报在执行）")
                return False
            self._active_goal = goal_handle
            result_future = goal_handle.get_result_async()
            wrapped = self._await_future(result_future, self._speak_timeout(text))
        finally:
            self._active_goal = None

        if wrapped is None:
            if self._stop.is_set():
                self._finish_stopped()
                self.get_logger().info("已停止：播报中断")
            else:
                self._fail("等待播报结果超时")
            return False
        result = wrapped.result
        if not result.success:
            self._fail(f"播报失败（{result.state}）：{result.message}")
            return False
        self.get_logger().info(f"播报完成：{result.message}")
        return True

    @staticmethod
    def _speak_timeout(text: str) -> float:
        # 中文播报约 0.25s/字，留足环境噪声导致的重播余量
        return max(30.0, len(text) * 0.35 + 20.0)

    def _set_head(self, yaw: float, pitch: float, poi: str) -> None:
        if not self._head_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("头部服务未就绪，跳过朝向调整")
            return
        request = SetHeadPose.Request()
        request.yaw = float(yaw)
        request.pitch = float(pitch)
        request.duration_s = 0.0
        future = self._head_client.call_async(request)
        response = self._await_future(future, 10.0)
        if response is None or not response.success:
            self.get_logger().warn(
                f"{poi} 头部朝向失败：{getattr(response, 'message', '超时')}")
        else:
            self.get_logger().info(f"{poi} 头部朝向完成：{response.message}")

    def _dwell(self, seconds: float, poi: str) -> bool:
        if seconds <= 0:
            return True
        self._publish("running", "dwelling", poi, 0, f"停留 {seconds:.0f}s")
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._stop.is_set():
                return False
            if self._paused.is_set():
                self._publish("paused", "dwelling", poi, 0, "停留中暂停")
                while self._paused.is_set() and not self._stop.is_set():
                    time.sleep(0.1)
                deadline = time.monotonic() + seconds
            time.sleep(0.1)
        return not self._stop.is_set()

    # ------------------------------------------------------------------ 工具
    def _wait_ready(self) -> bool:
        """处理暂停与停止；返回 False 表示应终止本次导览。"""
        if self._stop.is_set():
            self._finish_stopped()
            return False
        if self._paused.is_set():
            self._publish("paused", "paused", "", 0, "已暂停")
            while self._paused.is_set() and not self._stop.is_set():
                time.sleep(0.1)
            if self._stop.is_set():
                self._finish_stopped()
                return False
            self._publish("running", "resumed", "", 0, "已恢复")
        return True

    def _finish_stopped(self) -> None:
        if self._state != "stopping":
            self._state = "stopping"
            self._publish("stopping", "idle", "", 0, "导览已停止")

    def _aborted(self, message: str) -> bool:
        """调用被中断：用户 stop 属于正常结束，其余情况才记为错误。"""
        if self._stop.is_set():
            self._finish_stopped()
        else:
            self._fail(message)
        return False

    def _fail(self, message: str) -> None:
        self._state = "error"
        self._publish("error", "idle", "", 0, message)
        self.get_logger().error(message)

    def _await_future(self, future, timeout: float):
        """在 worker 线程里等一个 rclpy Future（executor 在主线程 spin）。"""
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._stop.is_set():
                return None
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"等待 Future 结果异常：{exc}")
            return None

    def _publish(self, state: str, stage: str, poi: str,
                 index: int, message: str) -> None:
        msg = GuideStatus()
        msg.state = state
        msg.stage = stage
        msg.poi_name = poi
        msg.index = int(index)
        msg.total = len(self.cfg.poi_order) if self.cfg is not None else 0
        msg.message = message
        self.status_pub.publish(msg)

    def shutdown(self) -> None:
        self._stop.set()
        self._stop_tour()
        self._paused.clear()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=10.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuideOrchestrator()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
