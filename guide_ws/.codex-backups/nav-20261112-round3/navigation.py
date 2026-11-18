"""底盘定点导航：按主线/支线逐点行驶，使用规划坐标及行进朝向。

每段调用原生 move_to(flags=0)，每次下发前要求定位连续稳定。
定位抖动优先等待自然恢复，每次 go 最多主动重定位一次，共用导航超时预算。
成功终态还必须通过 0.35m 到点位置校验。前向防撞随时可以取消任务。
中途恢复按主线段投影计算剩余必经点，不把前方最近点当成已经到达。
"""

from __future__ import annotations

import math
import os
import sys
import time
from collections import namedtuple

_ROBOT_CONTROLLER_SRC = os.environ.get(
    "ROBOT_CONTROLLER_SRC", "/home/nvidia/robot_controller/src"
)
if _ROBOT_CONTROLLER_SRC not in sys.path:
    sys.path.insert(0, _ROBOT_CONTROLLER_SRC)

from robot_controller.chassis import KnewbotChassisNav  # noqa: E402

# 一段直线：目标点名称、坐标、到达朝向（= 本段行进方向）
Leg = namedtuple("Leg", "name x y yaw")

_SUCCESS = {
    "successed", "succeeded", "success", "finished", "done", "completed",
}
_FAIL = {
    "failed", "failure", "error",
    "cancelled", "canceled", "stopped", "aborted", "rejected",
}
_POLL_S = 0.2
_NETWORK_FAILURE_LIMIT = 2
_REQUEST_TIMEOUT_CAP_S = 3.0


def _transport_error(exc) -> bool:
    # robot_controller SDK: 1002=request timeout, 3001=connection error.
    return (getattr(exc, "code", None) in (1002, 3001)
            or isinstance(exc, (TimeoutError, ConnectionError)))
# flags=0：SDK 定义为起步/到点时旋转；仍由底盘规划路径，不保证几何直线。
_POI_FLAGS = 0
# 到达一个点后让底盘停稳，再下发下一段
_LEG_SETTLE_S = 1.0
# 被判 failed 后，等多久重新下发 move_to
_REDISPATCH_SETTLE_S = 1.5
# 一段路线最多下发几次 move_to（底盘自己绕障，失败就重新发任务）
_LEG_ATTEMPTS = 3
# 导航中途停车后，在当前位置重定位最多等多久
_RECOVER_WAIT_S = 20.0
# 必须连续正常，不能把一次闪回正常视为恢复成功。
_LOCALIZATION_STABLE_S = 3.0
_LOCALIZATION_PASSIVE_WAIT_S = 8.0
_LAST_GOOD_POSE_MAX_AGE_S = 15.0
# 刚下发时先让底盘自己跑，过了这个时间才开始查前方障碍
_GUARD_GRACE_S = 1.5
# 连续多少次轮询都判到障碍才真正停车（防单帧误判）
_GUARD_CONFIRM = 3
# 底盘定位状态：0=正常；1=丢失 / 2=精度下降 / 3=异常 / 5=等待重定位
# 才允许主动下发重定位；6=重定位中 / 7=加载地图 / 8=等待传感器
# 是底盘自己在恢复，重复下发只会互相打断。
_RELOCALIZE_TRIGGER_STATUS = {1, 2, 3, 5}
_RELOCALIZE_BUSY_STATUS = {6, 7, 8}
# cancel 后等动作退出 running 的上限
_CANCEL_SETTLE_S = 2.0
# 短于这个长度的段不单独下发（已经站在点上）
_MIN_LEG_M = 0.15
_ARRIVAL_TOLERANCE_M = 0.35
# 防撞取消后，要求前方连续清空这么久才重新下发，避免抖动式反复取消
_GUARD_CLEAR_HOLD_S = 1.5
# 等前方清空的上限，超时返回 blocked 交给上层
_GUARD_CLEAR_WAIT_S = 10.0


def _never_started(status) -> bool:
    """动作 failed 时判断是否「从未起步」。

    底盘在停稳/换任务的过渡期会直接拒绝新任务，返回 failed 且距离与
    耗时都是 0；真正跑了一段才失败的动作这些字段非 0，不能重试。
    """
    if not isinstance(status, dict):
        return False
    seen = False
    for key in ("distance_time", "distance_remaining", "progress"):
        if key not in status:
            continue
        seen = True
        try:
            if float(status[key]) != 0.0:
                return False
        except (TypeError, ValueError):
            return False
    return seen


def classify(status) -> str:
    if not isinstance(status, dict):
        return "running"
    for key in ("status", "state", "action_state"):
        value = status.get(key)
        if isinstance(value, str):
            text = value.lower()
            if text in _SUCCESS:
                return "success"
            if text in _FAIL:
                return "failure"
    # done 只说明任务结束，不保证执行成功。
    if status.get("success") is False:
        return "failure"
    return "running"


def _is_blocking_dtc(entry) -> bool:
    """DTC 是否必须阻止运动。

    底盘健康接口的 hasFatal/hasError 是聚合标志：一条 C 类告警
    （如「WiFi 未连接」）也会把 hasFatal 置 True。真正决定能不能动的
    是每条故障的 severity：A/B 类阻断，C 类只告警；没有 severity
    字段的老固件保持保守策略，视为阻断。
    """
    severity = str(getattr(entry, "severity", "") or "").strip().upper()
    if not severity:
        return True
    return any(tag in severity for tag in ("FATAL", "ERROR", "A类", "B类"))


def _describe_dtc(entry) -> str:
    """单条 DTC 的日志文案：ID + 严重程度 + 描述 + 处理建议。"""
    try:
        dtc_id = f"0x{int(getattr(entry, 'id', 0)):08X}"
    except (TypeError, ValueError):
        dtc_id = "0x????????"
    severity = str(getattr(entry, "severity", "") or "").strip()
    desc = str(getattr(entry, "description", "") or "").strip()
    solution = str(getattr(entry, "solution", "") or "").strip()
    head = " ".join(x for x in (dtc_id, severity, desc) if x)
    return f"{head}（处理：{solution}）" if solution else head


class Navigator:
    def __init__(self, host: str, port: int, timeout: float, logger) -> None:
        self._log = logger
        self._chassis = KnewbotChassisNav(
            host=host, rest_port=port,
            request_timeout=min(float(timeout), _REQUEST_TIMEOUT_CAP_S))
        # 点位只在第一次用到时读，之后缓存（地图运行中不会变）
        self._pois = None
        self._recover_attempted = False
        self._network_failed = False
        self._transport_failures = 0
        self._pending_cancel = False
        self._active_action_id = None
        self._last_good_pose = None
        self._health_message = "底盘状态异常，暂时不能移动"
        # 定点导航主线点位顺序与支线挂接关系，由 places.yaml 注入
        self._main_line = []
        self._branches = {}
        # 上次因前向防撞取消的时间；用于重发前要求前方持续清空
        self._last_guard_cancel_at = None

    # ------------------------------------------------------------- 地图信息
    def _load_pois(self) -> dict:
        if self._pois is None:
            try:
                response = self._chassis.get_pois()
            except Exception as exc:  # noqa: BLE001 - 读不到下次再试
                self._log.warn(f"读取点位失败：{exc}")
                return {}
            rows = response.get("pois") if isinstance(response, dict) else response
            pois = {}
            for item in rows or []:
                if not isinstance(item, dict):
                    continue
                name = item.get("name") or item.get("point_name")
                if not name:
                    continue
                try:
                    pois[str(name)] = (
                        float(item.get("x", 0.0)),
                        float(item.get("y", 0.0)),
                        float(item.get("yaw", 0.0)),
                    )
                except (TypeError, ValueError):
                    continue
            if not pois:
                self._log.warn("点位列表为空，稍后重试")
                return {}
            self._pois = pois
        return self._pois

    def poi_names(self) -> list[str]:
        return list(self._load_pois())

    def _poi_pose(self, poi_name: str):
        """取点位位姿 (x, y, yaw)；地图上没有该点返回 None。"""
        return self._load_pois().get(poi_name)

    # ------------------------------------------------------------- 路线规划
    def set_route(self, main_line, branches) -> None:
        """注入主线点位顺序与支线挂接关系。"""
        self._main_line = [str(name) for name in (main_line or [])]
        self._branches = {
            str(name): str(attach)
            for name, attach in (branches or {}).items()
        }

    def plan_route(self, poi_name: str) -> list:
        """按主线顺序逐点规划路径，返回 [Leg(name, x, y, yaw), ...]。

        主线点位按物理顺序排列，用下标前缀的方式确定走法：当前位置落在
        主线的哪个点，目标在主线的哪个点，就按方向把中间每一个点都依次
        走一遍，不跳过。支线点位先从挂接点进入，并在挂接点原地转向后再
        进入支线。
        """
        target = self._poi_pose(poi_name)
        if target is None:
            return []
        xy = self._pose_xy(self._read_pose())
        names = self._route_names(xy, poi_name)
        if names is None:
            # 没配主线 / 点位不在路线上 / 读不到位姿：直接前往
            if xy is None:
                return [Leg(
                    poi_name, target[0], target[1], target[2])]
            yaw = math.atan2(target[1] - xy[1], target[0] - xy[0])
            return [Leg(poi_name, target[0], target[1], yaw)]

        points = []
        if xy is not None:
            points.append(("当前位置", xy))
        for name in names:
            pose = self._poi_pose(name)
            if pose is not None:
                points.append((name, (pose[0], pose[1])))
        if points[-1][0] != poi_name:
            points.append((poi_name, (target[0], target[1])))

        legs = []
        previous = points[0][1]
        for name, current in points[1:]:
            dx, dy = current[0] - previous[0], current[1] - previous[1]
            if math.hypot(dx, dy) < _MIN_LEG_M:
                continue
            legs.append(Leg(
                name, current[0], current[1], math.atan2(dy, dx)))
            previous = current
        return legs

    def _route_names(self, xy, poi_name: str):
        """按主线下标返回要经过的点位序列（不含当前位置）。

        返回 None 表示走不了这条路线（需要直接前往）；返回空列表表示
        按顺序走不需要再经过任何点（已经站在目标上）。
        """
        nodes = self._load_pois()
        chain = [name for name in self._main_line if name in nodes]
        if not chain or poi_name not in nodes or xy is None:
            return None
        index = {name: i for i, name in enumerate(chain)}
        branches = {
            name: attach for name, attach in self._branches.items()
            if name in nodes and attach in index
        }
        if poi_name in index:
            target_index, tail = index[poi_name], []
        elif poi_name in branches:
            target_index, tail = index[branches[poi_name]], [poi_name]
        else:
            return None

        # 当前位置在支线上时（离支线点比离挂接点更近），先从挂接点进入
        start_branch = None
        best_margin = 0.0
        for name, attach in branches.items():
            attach_distance = math.hypot(
                nodes[attach][0] - xy[0], nodes[attach][1] - xy[1])
            branch_distance = math.hypot(
                nodes[name][0] - xy[0], nodes[name][1] - xy[1])
            margin = attach_distance - branch_distance
            if margin > best_margin:
                start_branch, best_margin = name, margin
        if start_branch is not None:
            if start_branch == poi_name:
                return [poi_name]
            start_attach = branches[start_branch]
            return (
                [start_attach]
                + self._walk(chain, index[start_attach], target_index)
                + tail)

        # 将当前位置投影到主线段。最近点可能在前方，不能把它当成
        # 已经过的起点；否则恢复导航时会跳过拐角或必经点。
        if len(chain) == 1:
            return [chain[0]] + tail
        candidates = []
        for i in range(len(chain) - 1):
            ax, ay = nodes[chain[i]][:2]
            bx, by = nodes[chain[i + 1]][:2]
            dx, dy = bx - ax, by - ay
            length2 = dx * dx + dy * dy
            if length2 == 0:
                continue
            t = max(0.0, min(1.0,
                ((xy[0] - ax) * dx + (xy[1] - ay) * dy) / length2))
            candidates.append((math.hypot(
                xy[0] - ax - t * dx, xy[1] - ay - t * dy), i + t))
        if not candidates:
            return None
        position = min(candidates)[1]
        if target_index >= position:
            first = math.ceil(position)
            names = chain[first:target_index + 1]
        else:
            first = math.floor(position)
            names = [chain[i] for i in range(first, target_index - 1, -1)]
        return names + tail

    @staticmethod
    def _walk(chain, start: int, target: int) -> list:
        """按主线顺序从 start 下标走到 target 下标，返回途经点（不含起点）。"""
        if start == target:
            return []
        step = 1 if target > start else -1
        return [chain[i] for i in range(start + step, target + step, step)]

    # ------------------------------------------------------------- 位姿/定位
    def _read_pose(self):
        try:
            pose = self._chassis.get_pose() or {}
            self._transport_failures = 0
            if self._pose_confident(pose):
                self._last_good_pose = (time.monotonic(), dict(pose))
            return pose
        except Exception as exc:  # noqa: BLE001 - 单次读失败按不可用处理
            self._record_transport_failure(exc)
            self._log.warn(f"读取当前位姿失败：{exc}")
            return None

    @staticmethod
    def _pose_xy(pose):
        """从位姿字典取 (x, y)；缺失或非法返回 None。"""
        if not isinstance(pose, dict):
            return None
        try:
            x = float(pose["x"])
            y = float(pose["y"])
        except (KeyError, TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return x, y

    @classmethod
    def _pose_confident(cls, pose) -> bool:
        if cls._pose_xy(pose) is None:
            return False
        try:
            return (math.isfinite(float(pose["yaw"]))
                    and math.isfinite(float(pose["confidence"]))
                    and float(pose["confidence"]) > 0.0)
        except (KeyError, TypeError, ValueError):
            return False

    def _record_transport_failure(self, exc):
        if _transport_error(exc):
            self._transport_failures += 1
            if self._transport_failures >= _NETWORK_FAILURE_LIMIT:
                self._network_failed = True

    def _slam_status(self) -> int:
        """定位状态：0=正常，1=丢失，2=降级，5=等待重定位，6=重定位中。"""
        try:
            status = self._chassis.get_localization_status()
            self._transport_failures = 0
            return int(status.get("slam_status", -1))
        except Exception as exc:  # noqa: BLE001 - 查询抖动当成未知
            self._record_transport_failure(exc)
            self._log.warn(f"读取定位状态失败：{exc}")
            return -1

    def _slam_status_stable(self, retries: int = 3) -> int:
        """读定位状态；单次查询失败（-1）时短暂重读，避免把抖动当成未知状态。"""
        slam = self._slam_status()
        for _ in range(retries - 1):
            if slam != -1:
                break
            time.sleep(0.3)
            slam = self._slam_status()
        return slam

    def _trigger_recover(self, x: float, y: float, yaw: float,
                         timeout_s: float) -> bool:
        """触发重定位；接口 500 但任务已受理时也继续等定位状态。"""
        try:
            self._chassis.recover_localization(
                x=x, y=y, yaw=yaw, max_time_ms=int(timeout_s * 1000))
            return True
        except Exception as exc:  # noqa: BLE001
            # 底盘偶尔返回 500 Internal error，但响应里 accepted=true，
            # 实际重定位任务已经提交，这时不该直接当失败。
            if '"accepted":true' in str(exc).replace(" ", ""):
                self._log.warn(
                    f"重定位接口报错但任务已受理，继续等定位：{exc}")
                return True
            self._log.error(f"触发重定位失败：{exc}")
            return False

    def _wait_slam_ok(self, timeout_s: float) -> bool:
        """状态正常且位姿可信连续保持 3 秒；异常或查询失败重新计时。"""
        deadline = time.monotonic() + max(0.0, timeout_s)
        stable_since = None
        while time.monotonic() < deadline:
            slam = self._slam_status()
            pose = self._read_pose() if slam == 0 else None
            if self._network_failed:
                return False
            now = time.monotonic()
            if now >= deadline:
                break
            if slam == 0 and self._pose_confident(pose):
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= _LOCALIZATION_STABLE_S:
                    return True
            else:
                stable_since = None
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
        return False

    def recover_here(self, timeout_s: float = _RECOVER_WAIT_S) -> bool:
        """最多主动恢复一次，仅使用最近可信的完整位姿，不吸附 POI。"""
        if timeout_s <= _LOCALIZATION_STABLE_S:
            return False
        slam = self._slam_status_stable()
        if slam == 0 or slam in _RELOCALIZE_BUSY_STATUS:
            return self._wait_slam_ok(timeout_s)
        if slam not in _RELOCALIZE_TRIGGER_STATUS or self._recover_attempted:
            return False
        self._read_pose()
        trusted = self._last_good_pose
        if trusted is None or time.monotonic() - trusted[0] > _LAST_GOOD_POSE_MAX_AGE_S:
            self._log.error("没有近期可信位姿，拒绝用零置信度坐标或附近 POI 重定位")
            return False
        self._recover_attempted = True
        pose = trusted[1]
        self._log.warn(
            f"仅尝试一次原地重定位：x={pose['x']:.2f} y={pose['y']:.2f} "
            f"yaw={pose['yaw']:.2f}；保留实际朝向，不吸附附近点位")
        deadline = time.monotonic() + timeout_s
        if not self._trigger_recover(pose['x'], pose['y'], pose['yaw'], timeout_s):
            return False
        if self._wait_slam_ok(max(0.0, deadline - time.monotonic())):
            self._log.info("重定位后已连续稳定 3 秒")
            return True
        self._log.error("重定位后仍不稳定，停止本次导航，不重复重定位")
        return False

    def relocalize(self, poi_name: str, timeout_s: float = 20.0) -> bool:
        """启动时先等定位稳定；待命点名称不作为机器人已在该处的依据。"""
        if self._wait_slam_ok(min(_LOCALIZATION_PASSIVE_WAIT_S, timeout_s)):
            return True
        return self.recover_here(max(0.0, timeout_s - _LOCALIZATION_PASSIVE_WAIT_S))

    def check_ready(self) -> tuple[bool, str]:
        """导航前检查：定位可恢复 + 打开 AEB。

        可恢复状态（1/2/3/5）和底盘正在恢复（6/7/8）不在这里拦，
        交给 go() 按状态决定等待还是重定位；只有读不到状态或状态未知才拒绝。
        """
        if self._pending_cancel and not self._cancel_action():
            return False, "底盘通信未恢复，之前的任务还不能确认已取消，请先检查连接"
        self._network_failed = False
        self._transport_failures = 0
        try:
            status = self._chassis.get_localization_status()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"读取定位状态失败：{exc}")
            return False, ("我与底盘通信中断了，先不下发新任务，请检查底盘连接"
                           if _transport_error(exc) else "我这边读不到定位状态，先不移动了")
        slam = -1
        try:
            slam = int(status.get("slam_status", -1))
        except Exception:  # noqa: BLE001
            pass
        if slam != 0:
            if (slam in _RELOCALIZE_TRIGGER_STATUS
                    or slam in _RELOCALIZE_BUSY_STATUS):
                self._log.warn(
                    f"定位未就绪（slam_status={slam}），交给导航按状态恢复")
            else:
                self._log.error(
                    f"定位状态不可用（slam_status={slam}），{status}")
                return False, "我现在还没定位好，稍等一下再带你过去"
        if not self._check_health():
            return False, self._health_message
        self._set_aeb(True)
        return True, ""

    # ------------------------------------------------------------- 导航主流程
    def go(self, poi_name: str, timeout_s: float, guard=None) -> str:
        """定点导航到点位，返回 success / failure / blocked / timeout / localization / health / network。

        guard: 可选的前向防撞对象，判到障碍就取消本次导航并返回 blocked。
        整条路线按 places.yaml 的主线和支线拆成多段，逐段下发 move_to；
        中途被打断后由上层再次调用，按当前位姿重新规划剩余路段。
        """
        deadline = time.monotonic() + timeout_s
        self._recover_attempted = False
        self._network_failed = False
        self._transport_failures = 0
        if self._pending_cancel and not self._cancel_action():
            return "network"
        pois = self._load_pois()
        if not pois:
            # 点位读取偶发失败：短暂重试一次，别把一次抖动当成地图没有点位
            time.sleep(0.5)
            pois = self._load_pois()
        if poi_name not in pois:
            self._log.error(f"地图上没有点位 {poi_name}，无法导航")
            return "failure"

        # 出发前就有人 / 上次被防撞取消：必须等前方持续清空再下发。
        # 否则会先朝人走一段，或者障碍刚好闪开一帧就重发，
        # 变成反复取消/重发。
        if guard is not None:
            must_wait = (
                self._last_guard_cancel_at is not None or guard.blocked())
            if must_wait:
                if not self._wait_guard_clear(guard):
                    # 记下来，下次进 go 先要求连续清空，避免刚闪开就冲
                    self._last_guard_cancel_at = time.monotonic()
                    self._log.warn("前方持续有障碍，暂不下发导航")
                    return "blocked"
                self._last_guard_cancel_at = None

        if not self._check_health():
            return "health"
        if not self._check_localization(deadline):
            return "network" if self._network_failed else "localization"
        self._set_aeb(True)
        try:
            legs = self.plan_route(poi_name)
            if not legs:
                self._log.info(f"当前位置已在 {poi_name} 附近，无需移动")
                return "success"
            self._log.info(
                f"定点导航到 {poi_name}：{len(legs)} 个导航段，"
                + " -> ".join(
                    f"{leg.name}({leg.x:.2f},{leg.y:.2f})" for leg in legs))
            for index, leg in enumerate(legs):
                result = self._navigate_leg(leg, deadline, guard)
                if result != "success":
                    return result
                if index + 1 < len(legs):
                    # 让底盘停稳再下发下一段，避免停稳过渡期被瞬拒
                    time.sleep(_LEG_SETTLE_S)
            return "success"
        finally:
            # 取消没有获确认时保持 AEB，不能假设底盘已经停下。
            if self._cancel_action():
                self._set_aeb(False)
            else:
                self._log.error("任务取消尚未确认，保留 AEB 并禁止新的导航任务")

    def _navigate_leg(self, leg, deadline: float, guard) -> str:
        """逐段定位门控；定位未稳定时不靠拆短段或反复重发碰运气。"""
        for attempt in range(_LEG_ATTEMPTS):
            if time.monotonic() >= deadline:
                return "timeout"
            if attempt:
                self._log.warn(f"{leg.name} 重试前重新检查定位（{attempt+1}/{_LEG_ATTEMPTS}）")
                time.sleep(_REDISPATCH_SETTLE_S)
            if not self._check_health():
                return "health"
            if not self._check_localization(deadline):
                return "network" if self._network_failed else "localization"
            if time.monotonic() >= deadline:
                return "timeout"
            action_id = self._dispatch(leg)
            if action_id is None:
                if self._pending_cancel:
                    return "network"
                continue
            outcome = self._run_action(action_id, leg.name, deadline, guard, leg)
            if outcome in ("success", "blocked", "timeout", "localization", "network"):
                return outcome
        self._log.error(f"{leg.name} 连续 {_LEG_ATTEMPTS} 次导航任务失败")
        return "failure"

    def _check_localization(self, deadline=None) -> bool:
        """每次下发前连续稳定门控；先自然恢复，最多一次主动恢复。"""
        if deadline is None:
            deadline = time.monotonic() + _RECOVER_WAIT_S
        budget = min(_LOCALIZATION_PASSIVE_WAIT_S,
                     max(0.0, deadline - time.monotonic()))
        if self._wait_slam_ok(budget):
            return True
        if self._network_failed:
            return False
        slam = self._slam_status()
        self._log.warn(
            f"定位未连续稳定（slam_status={slam}），暂停下发运动任务")
        remaining = min(_RECOVER_WAIT_S, max(0.0, deadline - time.monotonic()))
        if remaining < _LOCALIZATION_STABLE_S:
            return False
        # 3=低置信度异常也可能自行恢复；恢复任务只能有一次。
        if (slam in _RELOCALIZE_TRIGGER_STATUS and not self._recover_attempted
                and self._last_good_pose is not None):
            return self.recover_here(remaining)
        return self._wait_slam_ok(remaining)

    @property
    def health_message(self) -> str:
        return self._health_message

    def _check_health(self) -> bool:
        """底盘健康检查：只有 A/B 类故障阻止运动，C 类告警仅记录。"""
        try:
            health = self._chassis.get_robot_health()
        except Exception as exc:  # noqa: BLE001 - 读不到健康状态不阻断导航
            self._record_transport_failure(exc)
            self._health_message = "读不到底盘健康状态，请检查底盘连接后再叫我"
            self._log.error(f"读取底盘健康状态失败，暂停运动：{exc}")
            return False
        entries = list(getattr(health, "baseError", []) or [])
        blocking = [entry for entry in entries if _is_blocking_dtc(entry)]
        warnings = [entry for entry in entries if not _is_blocking_dtc(entry)]
        if warnings:
            self._log.warn(
                "底盘告警（不影响运动，继续）："
                + "；".join(_describe_dtc(entry) for entry in warnings))
        if blocking:
            self._health_message = (
                "维护模式还开着，请先弹起维护按钮，退出手动模式后再叫我"
                if any("维护模式" in str(getattr(e, "description", "")) for e in blocking)
                else "底盘有故障，暂时不能移动，请先检查底盘告警")
            self._log.error(
                "底盘健康检查未通过："
                + "；".join(_describe_dtc(entry) for entry in blocking))
            return False
        if not entries and (
            getattr(health, "hasFatal", False)
            or getattr(health, "hasError", False)
        ):
            # 标志为真却没有任何 DTC 明细：无法判断严重程度，保守拒绝
            self._health_message = "底盘有故障，但读不到故障详情，请先检查底盘状态"
            self._log.error(
                "底盘健康检查未通过：未返回 DTC 详情"
                f"（hasFatal={getattr(health, 'hasFatal', None)}, "
                f"hasError={getattr(health, 'hasError', None)}）")
            return False
        return True

    def _run_action(self, action_id: str, label: str,
                    deadline: float, guard, leg=None) -> str:
        """轮询底盘动作到终态。

        返回 success / failure / timeout / blocked。
        failure 表示底盘导航任务失败，由调用方重新下发 move_to；
        blocked 表示前向防撞取消。
        """
        started = time.monotonic()
        blocked_hits = 0
        next_guard_log = 0.0
        last_log = 0.0
        localization_bad_since = None
        query_failures = 0
        while True:
            now = time.monotonic()
            if now >= deadline:
                self._log.error(
                    f"导航到 {label} 超时，取消任务 {action_id}")
                self._cancel_and_settle(action_id)
                return "timeout"

            if guard is not None and now - started >= _GUARD_GRACE_S:
                current = guard.distance()
                elapsed = now - started
                if current is not None and elapsed >= next_guard_log:
                    self._log.info(
                        f"防撞监测：正前方 {current:.2f}m，"
                        f"无效像素 {guard.zero_ratio():.0%}（阈值 "
                        f"{guard.stop_distance():.2f}m）")
                    next_guard_log = elapsed + 2.0
                if guard.blocked():
                    blocked_hits += 1
                else:
                    blocked_hits = 0
                if blocked_hits >= _GUARD_CONFIRM:
                    distance = "未知" if current is None else f"{current:.2f}m"
                    self._log.warn(
                        f"前方 {distance} 有障碍，取消导航任务 {action_id}")
                    self._cancel_and_settle(action_id)
                    self._last_guard_cancel_at = time.monotonic()
                    return "blocked"

            try:
                status = self._chassis.get_action_status(action_id)
                query_failures = 0
            except Exception as exc:  # noqa: BLE001 - 查询抖动继续等
                query_failures += 1
                self._log.warn(f"查询导航状态失败（{query_failures}/{_NETWORK_FAILURE_LIMIT}）：{exc}")
                if query_failures >= _NETWORK_FAILURE_LIMIT:
                    self._network_failed = True
                    self._pending_cancel = True
                    self._active_action_id = action_id
                    self._log.error("连续查询失败，退出等待；任务状态未知，恢复通信后须先确认取消")
                    return "network"
                time.sleep(_POLL_S)
                continue
            state = classify(status)
            if state != "running":
                self._active_action_id = None
            if state == "success":
                # 低置信度期间 REST 位姿会冻结，先等可信新位姿再做位置校验。
                if not self._wait_slam_ok(min(
                        _RECOVER_WAIT_S, max(0.0, deadline - time.monotonic()))):
                    self._log.error(f"{label} 动作结束，但定位未连续稳定，不能确认到达")
                    return "network" if self._network_failed else "localization"
                if leg is not None:
                    pose = self._read_pose()
                    xy = self._pose_xy(pose)
                    if not self._pose_confident(pose):
                        return "localization"
                    if xy is None or math.hypot(
                            leg.x - xy[0], leg.y - xy[1]) > _ARRIVAL_TOLERANCE_M:
                        self._log.error(
                            f"{label} 动作报成功但未通过到点位置校验：{xy}")
                        return "failure"
                self._log.info(
                    f"到达 {label}，用时 {now - started:.1f}s")
                return "success"
            if state == "failure":
                elapsed = time.monotonic() - started
                note = "（从未起步）" if _never_started(status) else ""
                self._log.error(
                    f"导航到 {label} 失败，用时 {elapsed:.1f}s{note}：{status}")
                self._log.warn(f"失败时 slam_status={self._slam_status()}，位姿={self._read_pose()}")
                return "failure"

            if now - last_log >= 2.0:
                last_log = now
                pose = self._read_pose()
                slam = self._slam_status()
                if slam != 0 or not self._pose_confident(pose):
                    if localization_bad_since is None:
                        localization_bad_since = now
                    elif now - localization_bad_since >= 2.0:
                        self._log.warn(f"{label} 行驶中定位持续异常，取消本任务并等待稳定")
                        self._cancel_and_settle(action_id)
                        return "failure"
                else:
                    localization_bad_since = None
                xy = self._pose_xy(pose)
                detail = f"，slam_status={slam}，置信度={pose.get('confidence') if isinstance(pose, dict) else None}"
                if xy is not None and leg is not None:
                    detail += (f"，位置({xy[0]:.2f},{xy[1]:.2f})，"
                              f"距目标 {math.hypot(leg.x-xy[0], leg.y-xy[1]):.2f}m")
                self._log.info(
                    f"{label} 运动进行中，剩余超时 "
                    f"{max(0.0, deadline - now):.0f}s{detail}")
            time.sleep(_POLL_S)

    # ------------------------------------------------------------- 运动下发
    def _dispatch(self, leg):
        """按规划坐标和朝向下发 move_to，返回 action_id；失败返回 None。"""
        try:
            # 显式使用规划坐标和行进朝向；POI 保存的 yaw 往往统一为 0，
            # 不能代表返程或支线的行进方向。
            response = self._chassis.move_to(
                leg.x, leg.y, yaw=leg.yaw, flags=_POI_FLAGS)
        except Exception as exc:  # noqa: BLE001
            # PUT 超时不等于底盘未接单，不能直接重新发送另一个任务。
            self._network_failed = True
            self._pending_cancel = True
            self._active_action_id = None
            self._log.error(f"下发 move_to({leg.name}) 结果不确定，禁止重发：{exc}")
            return None
        action_id = (
            str(response.get("action_id") or "")
            if isinstance(response, dict) else ""
        )
        if not action_id or response.get("success") is False:
            if not isinstance(response, dict) or response.get("success") is not False:
                self._pending_cancel = True
                self._network_failed = True
            self._log.error(
                f"导航到 {leg.name} 未返回 action_id：{response!r}")
            return None
        self._active_action_id = action_id
        self._log.info(
            f"已下发 move_to({leg.name}, x={leg.x:.2f}, y={leg.y:.2f}, "
            f"yaw={leg.yaw:.2f})，action_id={action_id}")
        return action_id

    def _wait_guard_clear(self, guard) -> bool:
        """要求前方连续清空 _GUARD_CLEAR_HOLD_S 才放行，避免抖动式反复取消。"""
        deadline = time.monotonic() + _GUARD_CLEAR_WAIT_S
        clear_since = None
        while time.monotonic() < deadline:
            if guard.blocked():
                clear_since = None
            else:
                if clear_since is None:
                    clear_since = time.monotonic()
                elif time.monotonic() - clear_since >= _GUARD_CLEAR_HOLD_S:
                    return True
            time.sleep(_POLL_S)
        return False

    # --------------------------------------------------------------- 收尾
    def _set_aeb(self, on: bool) -> None:
        try:
            self._chassis.set_aeb_control("On" if on else "Off")
        except Exception as exc:  # noqa: BLE001 - AEB 失败不阻塞主流程
            state = "开启" if on else "关闭"
            self._log.warn(f"AEB {state}失败（继续）：{exc}")

    def _cancel_and_settle(self, action_id: str) -> None:
        """取消动作并等它退出 running，再补一次取消。

        动作没退干净就下发新任务，底盘会在停稳过渡期以
        distance_time=0 / failed 直接拒绝，并容易带出定位降级。
        """
        self._cancel(action_id)
        deadline = time.monotonic() + _CANCEL_SETTLE_S
        while time.monotonic() < deadline:
            try:
                status = self._chassis.get_action_status(action_id)
            except Exception as exc:  # noqa: BLE001 - 查询失败就不再等
                self._log.warn(f"查询取消状态失败，停止等待：{exc}")
                break
            if classify(status) != "running":
                break
            time.sleep(_POLL_S)
        self._cancel(action_id)

    def _cancel(self, action_id=None) -> bool:
        try:
            response = self._chassis.cancel_action(
                [action_id] if action_id else None)
            if not isinstance(response, dict) or response.get("success") is not True:
                self._pending_cancel = True
                self._log.error(f"底盘未确认取消：{response}")
                return False
            return True
        except Exception as exc:  # noqa: BLE001
            self._pending_cancel = True
            self._log.error(f"取消导航未获确认（不能保证底盘已停）：{exc}")
            return False

    def _cancel_action(self) -> bool:
        """只清理本节点的活动任务；受理结果未知时须取消全部并取得确认。"""
        if self._active_action_id is None and not self._pending_cancel:
            return True
        action_id = self._active_action_id
        if not self._cancel(action_id):
            return False
        if action_id is not None:
            try:
                state = classify(self._chassis.get_action_status(action_id))
            except Exception as exc:  # noqa: BLE001
                self._pending_cancel = True
                self._log.error(f"取消后仍读不到任务终态：{exc}")
                return False
            if state == "running":
                self._pending_cancel = True
                return False
        self._pending_cancel = False
        self._active_action_id = None
        return True
