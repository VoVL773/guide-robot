"""learn_e2c 第一个节点：启动后让底盘向前前进约 2 秒。

用法（构建并 source 后）：
    ros2 launch learn_e2c move_forward.launch.py
    ros2 launch learn_e2c move_forward.launch.py duration_s:=2.0 speed_mps:=0.3

运动接口来自底盘 REST（192.168.11.10:9090）：
    move_by(direction=0, distance=速度×时间, move_mode=0, v=限速)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from learn_e2c.chassis_forward import (
    KnewbotChassisNav,
    blocking_faults,
    move_forward,
)


class MoveForwardNode(Node):
    def __init__(self) -> None:
        super().__init__("move_forward")

        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("request_timeout", 10.0)
        self.declare_parameter("duration_s", 2.0)
        self.declare_parameter("speed_mps", 0.3)
        self.declare_parameter("check_health", True)

        self._duration_s = float(self.get_parameter("duration_s").value)
        self._speed_mps = float(self.get_parameter("speed_mps").value)
        self._check_health = bool(self.get_parameter("check_health").value)

        self._chassis = KnewbotChassisNav(
            host=str(self.get_parameter("host").value),
            rest_port=int(self.get_parameter("rest_port").value),
            request_timeout=float(self.get_parameter("request_timeout").value),
        )

    def run(self) -> bool:
        """执行一次前进；返回是否成功。"""
        if self._check_health:
            faults = blocking_faults(self._chassis)
            if faults:
                self.get_logger().error(
                    "底盘存在阻断性故障，取消前进：" + "; ".join(faults))
                return False

        try:
            self._chassis.set_aeb_control("On")
            self.get_logger().info("AEB 已开启")
        except Exception as exc:  # noqa: BLE001 - AEB 设置失败不阻断本次练习
            self.get_logger().warn(f"设置 AEB 失败（继续执行）：{exc}")

        return move_forward(
            self._chassis, self._duration_s, self._speed_mps, self.get_logger()
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveForwardNode()
    try:
        ok = node.run()
    except Exception as exc:  # noqa: BLE001 - 给学习者一条明确错误而不是堆栈
        node.get_logger().error(f"前进异常：{exc}")
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
