"""导览导航服务端：/guide/navigate_to（NavigateTo Action）。

只做一件事：把导览编排给出的「去哪个 POI、沿哪条轨道」落到底盘 REST 接口。
点位的坐标与顺序以底盘 POI 服务端（192.168.11.10:9090）为唯一真源，
本地 YAML 只维护参观顺序、轨道、限速和停留时间。

dry_run:=true（默认）时：
- 只做 YAML 校验，完全不连底盘、不下发运动、不写限速；
- 每个 goal 按模拟耗时返回成功，用于编排层联调。

dry_run:=false 时：
- 启动时校验底盘 POI/轨道/定位/健康，失败则拒绝所有 goal（不崩溃）；
- 到点后校验实际位姿与 POI 坐标的偏差，超过 arrival_tolerance_m 判本次失败；
- 节点退出时恢复启动前的限速。

单点导航语义与 fire 巡检一致：首个点/返程点点对点，讲解点沿轨道。
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from guide_core import load_points, missing_pois

from guide_interfaces.action import NavigateTo

_ROBOT_CONTROLLER_SRC = os.environ.get(
    "ROBOT_CONTROLLER_SRC", "/home/nvidia/robot_controller/src"
)
if _ROBOT_CONTROLLER_SRC not in sys.path:
    sys.path.insert(0, _ROBOT_CONTROLLER_SRC)

from robot_controller.chassis import KnewbotChassisNav  # noqa: E402

# 底盘动作终态（兼容不同固件的字段与措辞）
_TERMINAL_SUCCESS = {
    "successed", "succeeded", "success", "finished", "done", "completed",
}
_TERMINAL_FAIL = {
    "failed", "failure", "error",
    "cancelled", "canceled", "stopped", "aborted", "rejected",
}

# 只有这些定位状态才值得主动下发重定位；
# 6=重定位中、7=加载地图、8=等待传感器属于底盘正在恢复，重复下发只会互相打断。
_RELOCALIZE_TRIGGER_STATUS = frozenset({1, 2, 3, 5})


def _unwrap_list(rv, *keys):
    """容错：REST 返回可能是 {"<key>": [...]} / dict 包 list / 直接 list。"""
    if rv is None:
        return []
    if isinstance(rv, list):
        return rv
    if isinstance(rv, dict):
        for key in keys:
            value = rv.get(key)
            if isinstance(value, list):
                return value
    return []


def _classify_action_status(status) -> str:
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
    """DTC 是否必须阻止运动：A/B 类阻断，C 类（如 WiFi 未连接）只告警。"""
    severity = str(getattr(entry, "severity", "") or "").strip().upper()
    if not severity:
        return True
    return any(tag in severity for tag in ("FATAL", "ERROR", "A类", "B类"))


def _resolve_points_config(configured: str) -> str:
    """配置路径由 launch 统一传入；单独 ros2 run 时回退到编排包的安装目录。"""
    if configured:
        return configured
    try:
        share = get_package_share_directory("guide_orchestrator")
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "未指定 points_config，且找不到 guide_orchestrator 的安装目录；"
            "请通过 guide.launch.py 启动"
        ) from exc
    return os.path.join(share, "config", "guide_points.yaml")


class NavigateToServer(Node):
    def __init__(self) -> None:
        super().__init__("guide_navigate_to")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("request_timeout", 10)
        self.declare_parameter("points_config", "")
        self.declare_parameter("action_poll_interval_s", 0.2)
        self.declare_parameter("simulated_travel_s", 3.0)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.poll = float(self.get_parameter("action_poll_interval_s").value)
        self.simulated_travel_s = float(self.get_parameter("simulated_travel_s").value)

        self._shutdown = False
        self._exec_lock = threading.Lock()
        self._speed_backup = None
        self._pois: dict[str, dict] = {}
        self._map_name = ""
        self._tracks: dict[str, int] = {}
        self._problems: list[str] = []
        self.cfg = None

        self._chassis = KnewbotChassisNav(
            host=str(self.get_parameter("host").value),
            rest_port=int(self.get_parameter("rest_port").value),
            request_timeout=float(self.get_parameter("request_timeout").value),
        )

        try:
            path = _resolve_points_config(str(self.get_parameter("points_config").value))
            self.cfg = load_points(path)
            self.get_logger().info(f"点位配置：{path}")
        except Exception as exc:  # noqa: BLE001 - 配置错误要给出完整原因而不是堆栈退出
            self._problems.append(f"点位配置加载失败：{exc}")
            self.get_logger().error(self._problems[-1])

        if self.cfg is not None:
            self._preflight()

        for problem in self._problems:
            self.get_logger().error(f"导航不可用：{problem}")
        if self._problems:
            self.get_logger().error(
                "导航服务将以「拒绝所有 goal」的方式运行；修好配置后重启节点即可")

        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self,
            NavigateTo,
            "/guide/navigate_to",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self.group,
        )

        self.get_logger().info(
            f"导航服务端就绪 /guide/navigate_to dry_run={self.dry_run} "
            f"可用={not self._problems}")

    # ------------------------------------------------------------------ 前置校验
    def _preflight(self) -> None:
        """校验 YAML、底盘 POI/轨道、定位与健康；失败只记录，不抛异常退出。"""
        cfg = self.cfg
        if not cfg.marked:
            self._problems.append(
                "guide_points.yaml 还未标记点位：请先在底盘地图上标记 POI，"
                "再填写 start_poi 与 poi_order（点位名称必须与底盘一致）")
            return

        if cfg.tracks.outbound and not cfg.tracks.return_track:
            self.get_logger().warn("只配置了 outbound 轨道，导览结束后将不沿返程轨道返回")
        if not cfg.tracks.outbound:
            self._problems.append("tracks.outbound 为空：讲解点必须指定一条轨道")

        if self.dry_run:
            # dry_run 不要求底盘在线，只做只读尝试；失败仅告警。
            try:
                self._read_map_and_pois()
                self._read_tracks()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"[dry_run] 底盘不可达或读取失败，跳过现场校验：{exc}")
                return
            self._check_poi_names()
            self._check_track_names()
            self.get_logger().info(
                f"[dry_run] 只读校验通过：{len(self._pois)} 个 POI，"
                f"轨道 {sorted(self._tracks)}")
            return

        # 实机模式：任何一项不通过都拒绝导航
        self._read_map_and_pois()
        self._read_tracks()
        self._check_poi_names()
        self._check_track_names()
        self._check_health()

        if not self._ensure_localization():
            self._problems.append(
                "定位不可用（slam_status 非 0 且自动重定位失败），拒绝导航")

        try:
            backup = self._chassis.get_speed_params()
            self._speed_backup = {
                "max_moving_speed": backup.get("max_moving_speed"),
                "max_angular_speed": backup.get("max_angular_speed"),
            }
            self._chassis.set_speed_params(
                max_moving_speed=max(cfg.speed.outbound_mps, cfg.speed.return_mps))
            self._chassis.set_aeb_control("On")
            self.get_logger().info(
                f"已启用 AEB，限速 {max(cfg.speed.outbound_mps, cfg.speed.return_mps)} m/s")
        except Exception as exc:  # noqa: BLE001
            self._problems.append(f"设置 AEB/限速失败：{exc}")

        if not self._problems:
            self.get_logger().info(
                f"前置校验通过：起点 {cfg.start_poi}，讲解点 {list(cfg.poi_order)}")

    def _read_map_and_pois(self) -> None:
        rv = self._chassis.get_current_map()
        map_name = rv.get("map_name") if isinstance(rv, dict) else None
        self._map_name = str(map_name or "")
        rv = self._chassis.get_pois(map_name=map_name, point_name=None)
        self._pois = {}
        for poi in _unwrap_list(rv, "pois", "poi", "data"):
            if not isinstance(poi, dict):
                continue
            name = poi.get("name") or poi.get("point_name")
            if name:
                self._pois[str(name)] = poi

    def _read_tracks(self) -> None:
        # 轨道必须按当前地图名查询，否则底盘返回空列表
        rv = self._chassis.get_tracks(map_name=self._map_name or None)
        self._tracks = {}
        for track in _unwrap_list(rv, "tracks", "track", "data"):
            if not isinstance(track, dict):
                continue
            name = track.get("track_name") or track.get("track_id")
            track_id = track.get("track_id")
            if name is not None and track_id is not None:
                self._tracks[str(name)] = track_id

    def _check_poi_names(self) -> None:
        missing = missing_pois(self.cfg, set(self._pois))
        if missing:
            self._problems.append(
                f"底盘地图上不存在这些点位：{missing}；"
                f"当前地图 POI：{sorted(self._pois)}")

    def _check_track_names(self) -> None:
        wanted = [t for t in (self.cfg.tracks.outbound,
                              self.cfg.tracks.return_track) if t]
        missing = [name for name in wanted if name not in self._tracks]
        if missing:
            self._problems.append(
                f"底盘上不存在这些轨道：{missing}；当前轨道：{sorted(self._tracks)}")

    def _check_health(self) -> None:
        try:
            health = self._chassis.get_robot_health()
        except Exception as exc:  # noqa: BLE001
            self._problems.append(f"读取底盘健康状态失败：{exc}")
            return
        blocking = [e for e in (health.baseError or []) if _is_blocking_dtc(e)]
        if blocking:
            detail = "; ".join(
                f"0x{int(getattr(e, 'id', 0)):08X} "
                f"{getattr(e, 'severity', '')} {getattr(e, 'description', '')}"
                for e in blocking)
            self._problems.append(f"底盘存在阻断性故障：{detail}")

    # ------------------------------------------------------------------ 定位
    def _slam_status(self):
        rv = self._chassis.get_localization_status() or {}
        return rv.get("slam_status") if isinstance(rv, dict) else None

    def _ensure_localization(self) -> bool:
        """定位异常时按配置重定位；返回是否可用。"""
        status = self._slam_status()
        if status == 0:
            return True
        cfg = self.cfg
        if status not in _RELOCALIZE_TRIGGER_STATUS or not cfg.relocalize.enabled:
            self.get_logger().error(f"定位状态异常 slam_status={status}")
            return False

        pose = cfg.relocalize.pose
        if pose is None:
            target = self._pois.get(cfg.start_poi)
            if not isinstance(target, dict):
                self.get_logger().error(
                    f"无法取得起点 {cfg.start_poi} 的坐标，不能自动重定位")
                return False
            pose = (float(target.get("x", 0.0)),
                    float(target.get("y", 0.0)),
                    float(target.get("yaw", 0.0)))

        self.get_logger().warn(
            f"定位状态 {status}，按起点位姿重定位：{pose}")
        try:
            self._chassis.recover_localization(
                *pose, max_time_ms=cfg.relocalize.max_time_ms)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"下发重定位失败：{exc}")
            return False

        deadline = time.monotonic() + cfg.relocalize.timeout_s
        while time.monotonic() < deadline and not self._shutdown:
            status = self._slam_status()
            if status == 0:
                self.get_logger().info("重定位完成，定位状态正常")
                return True
            time.sleep(cfg.relocalize.poll_interval_s)
        self.get_logger().error(
            f"重定位超时（{cfg.relocalize.timeout_s:.0f}s），最后 slam_status={status}")
        return False

    # ------------------------------------------------------------------ Action
    def _on_goal(self, goal_request) -> GoalResponse:
        if self._problems or self.cfg is None:
            return GoalResponse.REJECT
        if not goal_request.poi_name.strip():
            self.get_logger().warn("拒绝空点位名的导航目标")
            return GoalResponse.REJECT
        if self._exec_lock.locked():
            self.get_logger().warn("已有导航 Goal 执行中，拒绝新 Goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> NavigateTo.Result:
        if not self._exec_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(False, "已有导航 Goal 执行中", "rejected", 0.0)
        try:
            return self._execute_locked(goal_handle)
        finally:
            self._exec_lock.release()

    def _execute_locked(self, goal_handle) -> NavigateTo.Result:
        req = goal_handle.request
        poi = req.poi_name.strip()
        if poi != self.cfg.start_poi and poi not in self.cfg.poi_order:
            message = f"点位 {poi} 不在 guide_points.yaml 的白名单里"
            self.get_logger().warn(message)
            goal_handle.abort()
            return self._result(False, message, "rejected", 0.0)

        distance = self._distance_to(poi)
        if not self.cfg.range_limit.contains(*self._poi_xy(poi)):
            message = (
                f"点位 {poi} 的坐标超出 range_limit {self.cfg.range_limit}，"
                "疑似点位坐标数量级写错")
            self.get_logger().error(message)
            goal_handle.abort()
            return self._result(False, message, "rejected", distance)

        if self.dry_run:
            return self._execute_dry(goal_handle, poi, distance)

        if not self._ensure_localization():
            goal_handle.abort()
            return self._result(False, "定位状态异常，拒绝导航", "rejected", distance)

        # 分段限速：返程轨道用 return_mps，其余（含点对点）用 outbound_mps
        is_return = str(req.track_id) == self.cfg.tracks.return_track
        try:
            self._chassis.set_speed_params(
                max_moving_speed=(self.cfg.speed.return_mps if is_return
                                  else self.cfg.speed.outbound_mps))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"设置分段限速失败（沿用启动值）：{exc}")

        try:
            action_id = self._dispatch(poi, bool(req.use_track), str(req.track_id))
        except Exception as exc:  # noqa: BLE001
            message = f"下发导航失败：{exc}"
            self.get_logger().error(message)
            goal_handle.abort()
            return self._result(False, message, "failed", distance)

        timeout = (self.cfg.timeouts.to_start_s
                   if poi == self.cfg.start_poi else self.cfg.timeouts.per_poi_s)
        feedback = NavigateTo.Feedback()
        feedback.phase = "dispatched"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        deadline = time.monotonic() + timeout
        while True:
            if self._shutdown or goal_handle.is_cancel_requested:
                self._cancel_action(action_id)
                goal_handle.canceled()
                return self._result(False, "导航被取消", "canceled", distance)

            try:
                status = self._chassis.get_action_status(action_id)
            except Exception as exc:  # noqa: BLE001 - 轮询抖动不终止
                self.get_logger().warn(f"轮询动作状态异常（继续）：{exc}")
                status = None

            verdict = _classify_action_status(status)
            if verdict == "success":
                return self._arrived(goal_handle, poi, distance)
            if verdict == "failure":
                message = f"底盘导航失败：{status!r}"
                self.get_logger().error(message)
                goal_handle.abort()
                return self._result(False, message, "failed", distance)

            remaining = self._distance_to(poi)
            if distance > 0.0:
                feedback.progress = max(0.0, min(1.0, 1.0 - remaining / distance))
            feedback.phase = "moving"
            goal_handle.publish_feedback(feedback)

            if time.monotonic() >= deadline:
                self._cancel_action(action_id)
                message = f"导航超时（{timeout:.0f}s），已下发取消"
                self.get_logger().error(message)
                goal_handle.abort()
                return self._result(False, message, "timeout", distance)

            time.sleep(self.poll)

    def _execute_dry(self, goal_handle, poi: str, distance: float) -> NavigateTo.Result:
        """按固定模拟耗时返回成功，不碰底盘。"""
        feedback = NavigateTo.Feedback()
        feedback.phase = "dispatched"
        goal_handle.publish_feedback(feedback)
        elapsed = 0.0
        while elapsed < self.simulated_travel_s:
            if goal_handle.is_cancel_requested or self._shutdown:
                goal_handle.canceled()
                return self._result(False, "导航被取消", "canceled", distance)
            time.sleep(self.poll)
            elapsed += self.poll
            feedback.progress = min(1.0, elapsed / self.simulated_travel_s)
            feedback.phase = "moving"
            goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"[dry_run] 已到达 {poi}（模拟）")
        goal_handle.succeed()
        return self._result(True, f"dry_run 已到达 {poi}", "done", distance)

    def _dispatch(self, poi: str, use_track: bool, track_name: str) -> int:
        if not use_track:
            self.get_logger().info(f"点对点导航 -> {poi}")
            response = self._chassis.move_to_poi(poi)
        else:
            if not track_name:
                raise RuntimeError("use_track=true 但目标里没有 track_id")
            if track_name not in self._tracks:
                raise RuntimeError(
                    f"未配置轨道 {track_name}，可用：{sorted(self._tracks)}")
            self.get_logger().info(f"沿轨道 {track_name} 导航 -> {poi}")
            response = self._chassis.move_to_locations_poi(
                self._tracks[track_name], poi, uptrack_enabled=True)
        action_id = response.get("action_id") if isinstance(response, dict) else None
        if action_id is None:
            raise RuntimeError(f"底盘未返回 action_id：{response!r}")
        self.get_logger().info(f"导航已下发 action_id={action_id}")
        return int(action_id)

    def _arrived(self, goal_handle, poi: str, distance: float) -> NavigateTo.Result:
        pose = self._read_pose()
        target = self._pois.get(poi)
        offset = None
        if isinstance(pose, dict) and isinstance(target, dict):
            try:
                offset = math.hypot(float(target["x"]) - float(pose["x"]),
                                    float(target["y"]) - float(pose["y"]))
            except (KeyError, TypeError, ValueError):
                offset = None

        if offset is not None and offset > self.cfg.arrival_tolerance_m:
            message = (f"已到达 {poi}，但位姿偏差 {offset:.2f}m 超过容差 "
                       f"{self.cfg.arrival_tolerance_m:.2f}m")
            self.get_logger().error(message)
            goal_handle.abort()
            return self._result(False, message, "failed", distance)

        detail = (f"已到达 {poi}" if offset is None
                  else f"已到达 {poi}（偏差 {offset:.2f}m）")
        self.get_logger().info(detail)
        goal_handle.succeed()
        return self._result(True, detail, "done", distance)

    # ------------------------------------------------------------------ 工具
    def _poi_xy(self, poi: str) -> tuple[float, float]:
        entry = self._pois.get(poi) or {}
        try:
            return float(entry.get("x", 0.0)), float(entry.get("y", 0.0))
        except (TypeError, ValueError):
            return 0.0, 0.0

    def _read_pose(self):
        try:
            return self._chassis.get_pose()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"读取位姿失败：{exc}")
            return None

    def _distance_to(self, poi: str) -> float:
        if self.dry_run:
            return 0.0
        pose = self._read_pose()
        if not isinstance(pose, dict) or poi not in self._pois:
            return 0.0
        x, y = self._poi_xy(poi)
        try:
            return math.hypot(x - float(pose["x"]), y - float(pose["y"]))
        except (KeyError, TypeError, ValueError):
            return 0.0

    def _cancel_action(self, action_id: int) -> None:
        try:
            self._chassis.cancel_action([action_id])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"取消底盘动作失败：{exc}")

    @staticmethod
    def _result(success, message, state, distance) -> NavigateTo.Result:
        result = NavigateTo.Result()
        result.success = bool(success)
        result.message = message
        result.state = state
        result.distance_m = float(distance)
        return result

    # ------------------------------------------------------------------ 退出
    def shutdown(self) -> None:
        self._shutdown = True
        if self.dry_run or self._speed_backup is None:
            return
        try:
            self._chassis.cancel_action()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"退出取消失败：{exc}")
        try:
            self._chassis.set_speed_params(
                max_moving_speed=self._speed_backup.get("max_moving_speed"),
                max_angular_speed=self._speed_backup.get("max_angular_speed"))
            self.get_logger().info("已恢复启动前的底盘限速")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"恢复底盘限速失败：{exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigateToServer()
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
