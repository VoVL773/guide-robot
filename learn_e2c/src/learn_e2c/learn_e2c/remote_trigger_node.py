"""把遥控器上「未被占用」的按键绑定到前进动作。

调研结论（joystick_bridge 的 bridge_config.yaml + joystick.py）：
  axes[8..11] = a/b/c/d 按键（-1 松开 / +1 按下）
  axes[4] = e 拨杆，axes[7] = f 拨杆，axes[5] = g 拨杆，axes[6] = h 拨杆
  已用：a/b/c/d、f（下拨）、h（左/右拨）、g 左拨 + c 短按（急停）
  未用：g 右拨（本节点把它作为「启动就绪」开关）

本节点不修改厂商的 bridge_config.yaml，只订阅 /sbus_data 做边沿检测：
  1. g 右拨（axes[5] == -1.0）-> 进入就绪状态；
  2. 就绪状态下按 A（axes[8] == +1.0）-> 执行一次前进 duration_s 秒；
  3. g 回中位即取消就绪，下次需重新右拨。
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy

from learn_e2c.chassis_forward import (
    KnewbotChassisNav,
    blocking_faults,
    move_forward,
)

# 拨杆 g / 按键 A 在 Joy.axes 中的下标
_G_AXIS = 5
_G_RIGHT = -1.0
_A_AXIS = 8
_A_PRESSED = 1.0


class RemoteTriggerNode(Node):
    def __init__(self) -> None:
        super().__init__("remote_trigger")

        self.declare_parameter("joy_topic", "/sbus_data")
        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("request_timeout", 10.0)
        self.declare_parameter("duration_s", 2.0)
        self.declare_parameter("speed_mps", 0.3)
        self.declare_parameter("check_health", True)
        self.declare_parameter("switch_axis", _G_AXIS)
        self.declare_parameter("trigger_value", _G_RIGHT)
        self.declare_parameter("button_axis", _A_AXIS)
        self.declare_parameter("button_pressed_value", _A_PRESSED)

        joy_topic = str(self.get_parameter("joy_topic").value)
        self._axis = int(self.get_parameter("switch_axis").value)
        self._trigger_value = float(self.get_parameter("trigger_value").value)
        self._button_axis = int(self.get_parameter("button_axis").value)
        self._button_pressed_value = float(
            self.get_parameter("button_pressed_value").value)
        self._duration_s = float(self.get_parameter("duration_s").value)
        self._speed_mps = float(self.get_parameter("speed_mps").value)
        self._check_health = bool(self.get_parameter("check_health").value)

        self._chassis = KnewbotChassisNav(
            host=str(self.get_parameter("host").value),
            rest_port=int(self.get_parameter("rest_port").value),
            request_timeout=float(self.get_parameter("request_timeout").value),
        )

        # 边沿检测：None 表示还没收到过消息，第一次只同步状态、不触发
        self._last_axis_value = None
        self._last_button_value = None
        self._armed = False
        self._busy = False
        self._lock = threading.Lock()

        self.create_subscription(Joy, joy_topic, self._joy_callback, QoSProfile(depth=10))
        self.get_logger().info(
            f"监听 {joy_topic}：先 g 右拨（axes[{self._axis}] == {self._trigger_value}）就绪，"
            f"再按 A（axes[{self._button_axis}] == {self._button_pressed_value}）"
            f"前进 {self._duration_s:.2f}s；g 回中位取消就绪")

    def _joy_callback(self, msg: Joy) -> None:
        if self._axis >= len(msg.axes) or self._button_axis >= len(msg.axes):
            return
        switch_value = float(msg.axes[self._axis])
        button_value = float(msg.axes[self._button_axis])

        if self._last_axis_value is None:
            self._last_axis_value = switch_value
            self._last_button_value = button_value
            return

        switch_entered = (switch_value == self._trigger_value
                          and self._last_axis_value != self._trigger_value)
        switch_left = switch_value != self._trigger_value
        button_pressed = (button_value == self._button_pressed_value
                          and self._last_button_value != self._button_pressed_value)

        self._last_axis_value = switch_value
        self._last_button_value = button_value

        if switch_entered:
            self._armed = True
            self.get_logger().info("g 右拨：已就绪，按 A 启动前进")
        elif switch_left and self._armed:
            self._armed = False
            self.get_logger().info("g 回中位：取消就绪")

        if button_pressed:
            if self._armed:
                self._armed = False
                self._trigger_async()
            else:
                self.get_logger().debug("A 已按下，但未就绪（需先 g 右拨）")

    def _trigger_async(self) -> None:
        with self._lock:
            if self._busy:
                self.get_logger().warn("上一次前进还没结束，忽略本次按键")
                return
            self._busy = True

        def _run() -> None:
            try:
                if self._check_health:
                    faults = blocking_faults(self._chassis)
                    if faults:
                        self.get_logger().error(
                            "底盘存在阻断性故障，忽略按键：" + "; ".join(faults))
                        return
                move_forward(
                    self._chassis, self._duration_s, self._speed_mps,
                    self.get_logger(),
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"按键触发前进异常：{exc}")
            finally:
                with self._lock:
                    self._busy = False

        self.get_logger().info("遥控器 g 右拨 + 按 A：触发前进")
        threading.Thread(target=_run, name="remote_forward", daemon=True).start()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RemoteTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
