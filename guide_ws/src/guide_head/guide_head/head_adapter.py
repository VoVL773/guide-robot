"""头部关节适配层：把 /head/cmd 协议封装成「平滑移动到目标姿态」。

协议：/head/cmd  ros2_bridge_msgs/msg/HeadCtrl
  mode=0（位置模式）、label=0、reserved=165
  ctrl=[MotorCtrl(name=关节ID, pos=弧度, spd=速度, cur=电流)]

实测映射（2026-10-24 逐关节验证）：
  1 = roll 侧倾（正 = 向右歪，导览固定 0）
  2 = pitch 俯仰（正 = 低头，抬头必须发负值）
  3 = yaw 左右转（正 = 向左）
限位：±26° / ±25° / ±90°
"""

from __future__ import annotations

import math
import threading
import time

from ros2_bridge_msgs.msg import HeadCtrl, MotorCtrl, RobotState
from std_msgs.msg import Header

HEAD_JOINTS = [1, 2, 3]
LIMIT_DEG = {1: 26.0, 2: 25.0, 3: 90.0}
LIMIT_RAD = {k: math.radians(v) for k, v in LIMIT_DEG.items()}
RESERVED = 165


def clamp(joint: int, pos: float) -> float:
    limit = LIMIT_RAD[joint]
    return max(-limit, min(limit, pos))


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


class HeadAdapter:
    def __init__(self, node, cmd_topic="/head/cmd", state_topic="/robot_state",
                 rate=100.0, dry_run=False):
        self.node = node
        self.rate = float(rate)
        self.dry_run = bool(dry_run)
        self.pub = node.create_publisher(HeadCtrl, cmd_topic, 10)
        node.create_subscription(RobotState, state_topic, self._on_state, 10)
        self.pose = [0.0, 0.0, 0.0]
        # 最近一次下发的目标姿态，中断时用它保持位置
        self.last_cmd = [0.0, 0.0, 0.0]
        self._last_dry_log_at = 0.0
        self._pose_ready = threading.Event()
        # 置 True 后 move_to 立即中断
        self.stop_requested = False
        self._log = node.get_logger()

    @property
    def pose_ready(self) -> bool:
        return self._pose_ready.is_set()

    def _on_state(self, msg) -> None:
        try:
            state = {int(s.name): float(s.pos) for s in msg.head.status}
        except AttributeError:
            return
        if all(joint in state for joint in HEAD_JOINTS):
            self.pose = [state[joint] for joint in HEAD_JOINTS]
            self._pose_ready.set()

    def wait_pose(self, timeout: float = 3.0) -> bool:
        return self._pose_ready.wait(timeout)

    def build(self, positions, speed: float, current: float) -> HeadCtrl:
        msg = HeadCtrl()
        msg.header = Header()
        msg.header.frame_id = "head"
        msg.mode = 0
        msg.label = 0
        msg.reserved = RESERVED
        ctrl = []
        for joint, pos in zip(HEAD_JOINTS, positions):
            motor = MotorCtrl()
            motor.name = joint
            motor.pos = float(clamp(joint, pos))
            motor.spd = float(speed)
            motor.cur = float(current)
            ctrl.append(motor)
        msg.ctrl = ctrl
        return msg

    def send(self, positions, speed: float, current: float) -> None:
        positions = [clamp(j, p) for j, p in zip(HEAD_JOINTS, positions)]
        self.last_cmd = positions
        if self.dry_run:
            # 100Hz 连续下发，dry_run 下按 0.5s 节流打印，避免刷屏
            now = time.time()
            if now - self._last_dry_log_at >= 0.5:
                self._last_dry_log_at = now
                self._log.info(
                    f"[dry_run] /head/cmd -> {[round(p, 3) for p in positions]}")
            return
        self.pub.publish(self.build(positions, speed, current))

    def move_to(self, target, duration: float = 1.5, hold: float = 0.3,
                speed: float = 0.15, current: float = 2.0):
        """从当前姿态平滑过渡到目标，再保持 hold 秒；stop_requested 可随时中断。"""
        target = [clamp(j, v) for j, v in zip(HEAD_JOINTS, target)]
        start = list(self.pose) if self.pose_ready else [0.0] * 3
        period = 1.0 / self.rate

        t0 = time.time()
        while not self.stop_requested:
            t = (time.time() - t0) / duration if duration > 0 else 1.0
            s = smoothstep(t)
            self.send([a + (b - a) * s for a, b in zip(start, target)], speed, current)
            if t >= 1.0:
                break
            time.sleep(period)

        end = time.time() + hold
        while not self.stop_requested and time.time() < end:
            self.send(target, speed, current)
            time.sleep(period)
        return target
