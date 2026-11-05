"""头部服务端：/guide/set_head_pose（SetHeadPose 服务）。

编排层在讲解前调用：回中（yaw=0, pitch=0），或按讲解配置抬头/转向。
只负责头部，不读地图、不做导航决策。
"""

from __future__ import annotations

import os
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from guide_core import load_yaml

from guide_interfaces.srv import SetHeadPose

from .head_adapter import HeadAdapter


def _load_config() -> dict:
    share = get_package_share_directory("guide_head")
    path = os.path.join(share, "config", "head.yaml")
    return load_yaml(path)


class HeadServer(Node):
    def __init__(self) -> None:
        super().__init__("guide_head")

        cfg = _load_config()
        head_cfg = cfg.get("head") or {}
        self.declare_parameter("dry_run", bool(cfg.get("dry_run", True)))
        self.declare_parameter("cmd_topic", str(head_cfg.get("cmd_topic", "/head/cmd")))
        self.declare_parameter("state_topic",
                               str(head_cfg.get("state_topic", "/robot_state")))
        self.declare_parameter("rate_hz", float(head_cfg.get("rate_hz", 100.0)))
        self.declare_parameter("default_duration_s",
                               float(head_cfg.get("default_duration_s", 1.5)))
        self.declare_parameter("speed", float(head_cfg.get("speed", 0.15)))
        self.declare_parameter("current", float(head_cfg.get("current", 2.0)))
        self.declare_parameter("center_on_start",
                               bool(head_cfg.get("center_on_start", True)))

        self.default_duration = float(self.get_parameter("default_duration_s").value)
        self.speed = float(self.get_parameter("speed").value)
        self.current = float(self.get_parameter("current").value)

        self.adapter = HeadAdapter(
            self,
            cmd_topic=str(self.get_parameter("cmd_topic").value),
            state_topic=str(self.get_parameter("state_topic").value),
            rate=float(self.get_parameter("rate_hz").value),
            dry_run=bool(self.get_parameter("dry_run").value),
        )
        self._lock = threading.Lock()

        self.group = ReentrantCallbackGroup()
        self.create_service(
            SetHeadPose, "/guide/set_head_pose", self._on_set_pose,
            callback_group=self.group)

        self.get_logger().info(
            f"头部服务端就绪 /guide/set_head_pose dry_run={self.adapter.dry_run}")

        if bool(self.get_parameter("center_on_start").value):
            # 回中要阻塞 3~4 秒，放到后台线程，避免推迟服务可用时间
            threading.Thread(target=self._center, daemon=True).start()

    def _center(self) -> None:
        if not self._lock.acquire(timeout=1.0):
            return
        try:
            self.adapter.wait_pose(3.0)
            self.adapter.move_to([0.0, 0.0, 0.0], duration=1.0, hold=0.2,
                                 speed=self.speed, current=self.current)
            self.get_logger().info("启动回中完成")
        finally:
            self._lock.release()

    def _on_set_pose(self, request, response):
        if not self._lock.acquire(blocking=False):
            response.success = False
            response.message = "头部正在执行上一个动作"
            return response

        target = [0.0, float(request.pitch), float(request.yaw)]
        duration = (float(request.duration_s)
                    if request.duration_s > 0 else self.default_duration)
        try:
            self.adapter.stop_requested = False
            self.adapter.move_to(
                target, duration=duration, hold=0.2,
                speed=self.speed, current=self.current)
            response.success = True
            response.message = (
                f"头部已到位 yaw={target[2]:+.3f} pitch={target[1]:+.3f}")
            self.get_logger().info(response.message)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"头部动作失败：{exc}"
            self.get_logger().error(response.message)
        finally:
            self._lock.release()
        return response

    def shutdown(self) -> None:
        self.adapter.stop_requested = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadServer()
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
