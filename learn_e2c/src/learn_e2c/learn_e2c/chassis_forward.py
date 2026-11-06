"""底盘「前进一段时间」的公共实现，供 move_forward / remote_trigger 两个节点复用。

底盘接口是 REST 异步任务: move_by 只负责下发，动作是否完成要轮询
get_action_status。这里把「下发 -> 轮询 -> 超时取消」收敛成一个函数。
"""

from __future__ import annotations

import os
import sys
import time

_ROBOT_CONTROLLER_SRC = os.environ.get(
    "ROBOT_CONTROLLER_SRC", "/home/nvidia/robot_controller/src"
)
if _ROBOT_CONTROLLER_SRC not in sys.path:
    sys.path.insert(0, _ROBOT_CONTROLLER_SRC)

from robot_controller.chassis import KnewbotChassisNav  # noqa: E402,F401

_TERMINAL_SUCCESS = {
    "successed", "succeeded", "success", "finished", "done", "completed",
}
_TERMINAL_FAIL = {
    "failed", "failure", "error",
    "cancelled", "canceled", "stopped", "aborted", "rejected",
}

# 动作下发后最多等 duration + 这个余量（秒），超时视为失败并取消
_TIMEOUT_MARGIN_S = 5.0
# 轮询间隔（秒）
_POLL_INTERVAL_S = 0.1


def classify_action_status(status) -> str:
    """把底盘动作状态分类为 success / failure / running。"""
    if not isinstance(status, dict):
        return "running"
    if status.get("done") is True or status.get("is_finished") is True:
        return "success"
    for key in ("status", "state", "action_state"):
        value = status.get(key)
        if isinstance(value, str):
            text = value.lower()
            if text in _TERMINAL_SUCCESS:
                return "success"
            if text in _TERMINAL_FAIL:
                return "failure"
    return "running"


def _is_blocking_dtc(entry) -> bool:
    """A/B 类故障阻断运动；C 类（如 WiFi 未连接）只告警。"""
    severity = str(getattr(entry, "severity", "") or "").strip().upper()
    if not severity:
        return True
    return any(tag in severity for tag in ("FATAL", "ERROR", "A类", "B类"))


def blocking_faults(chassis: KnewbotChassisNav) -> list[str]:
    """当前会阻断运动的故障描述列表；读不到健康状态也算一条。"""
    try:
        health = chassis.get_robot_health()
    except Exception as exc:  # noqa: BLE001
        return [f"读取底盘健康状态失败：{exc}"]
    return [
        f"0x{int(getattr(e, 'id', 0)):08X} "
        f"{getattr(e, 'severity', '')} {getattr(e, 'description', '')}"
        for e in (health.baseError or []) if _is_blocking_dtc(e)
    ]


def move_forward(
    chassis: KnewbotChassisNav,
    duration_s: float,
    speed_mps: float,
    logger,
) -> bool:
    """让底盘沿当前车头方向前进 duration_s 秒。

    底盘接口按「距离」执行，所以用 距离 = 速度 × 时间 换算；
    实际速度受 v 限速与 AEB 影响，可能比理论值慢，这里只保证不超时卡死。
    """
    distance_mm = int(round(speed_mps * duration_s * 1000.0))
    distance_mm = max(50, min(5000, distance_mm))

    logger.info(
        f"前进 {duration_s:.2f}s：distance={distance_mm}mm，"
        f"限速 {speed_mps:.2f} m/s（约 {distance_mm / 1000.0:.2f}m）"
    )

    started_at = time.monotonic()
    response = chassis.move_by(
        direction=0.0, distance=distance_mm, move_mode=0, v=speed_mps
    )
    action_id = str(response.get("action_id")) if isinstance(response, dict) else "?"
    logger.info(f"已下发 move_by，action_id={action_id}")

    deadline = started_at + max(duration_s, 1.0) + _TIMEOUT_MARGIN_S
    while time.monotonic() < deadline:
        status = chassis.get_action_status(action_id)
        state = classify_action_status(status)
        if state != "running":
            elapsed = time.monotonic() - started_at
            if state == "success":
                logger.info(f"前进完成，用时 {elapsed:.2f}s")
            else:
                logger.error(f"前进失败，用时 {elapsed:.2f}s，status={status}")
            return state == "success"
        time.sleep(_POLL_INTERVAL_S)

    logger.error(f"前进超时（>{deadline - started_at:.1f}s），取消动作 {action_id}")
    try:
        chassis.cancel_action([action_id])
    except Exception as exc:  # noqa: BLE001
        logger.error(f"取消动作失败：{exc}")
    return False
