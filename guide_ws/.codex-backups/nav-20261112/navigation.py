"""底盘导航封装：定点导航、定位检查、分段按点位行驶并等到动作终态。

行驶部分直接复用 learn_e2c 的做法：每段用底盘原生的
move_to_poi(点位名, flags=0) 下发，只轮询 get_action_status 等终态，
超时就 cancel。flags=0 表示「先对着行进方向直行，到点后再转到点位
朝向」，所以每段天然走直线，中转点会先原地调整朝向再进入下一段。

地图已改为定点导航：places.yaml 按物理顺序配置主线点位和支线挂接点，
导航时按主线下标逐点经过每一个中间点，不依赖虚拟轨道。

整体生命周期：

  定位门控 -> 健康检查 -> 打开 AEB -> 规划分段路线
  -> 每段：move_to_poi -> 轮询（超时 / 失败重新下发 / 前向防撞）
  -> 统一收尾：取消动作 + 关闭 AEB

防撞时返回 blocked，由上层等障碍走开后再调用 go() 重发。
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
# flags=0：先对着行进方向直行，到点后再转到点位朝向（learn_e2c 同款）
_POI_FLAGS = 0
# 到达一个点后让底盘停稳，再下发下一段
_LEG_SETTLE_S = 1.0
# 被判 failed 后，等多久重新下发 move_to_poi
_REDISPATCH_SETTLE_S = 1.5
# 一段路线最多下发几次 move_to_poi（底盘自己绕障，失败就重新发任务）
_LEG_ATTEMPTS = 3
# 导航中途停车后，在当前位置重定位最多等多久
_RECOVER_WAIT_S = 20.0
# 重定位初值吸附到最近固定点的最大距离
_RECOVER_SNAP_MAX_M = 3.0
# 重定位刚完成时底盘还没稳住，先等一会再下发导航
_RECOVER_SETTLE_S = 1.5
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
    if status.get("done") is True or status.get("is_finished") is True:
        return "success"
    for key in ("status", "state", "action_state"):
        value = status.get(key)
        if isinstance(value, str):
            text = value.lower()
            if text in _SUCCESS:
                return "success"
            if text in _FAIL:
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
            host=host, rest_port=port, request_timeout=timeout)
        # 点位只在第一次用到时读，之后缓存（地图运行中不会变）
        self._pois = None
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

        # 主线：用最近的线上点作为起点下标，按方向逐点走到目标下标
        start_index = min(
            range(len(chain)),
            key=lambda i: math.hypot(
                nodes[chain[i]][0] - xy[0],
                nodes[chain[i]][1] - xy[1]))
        return self._walk(chain, start_index, target_index) + tail

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
            return self._chassis.get_pose() or {}
        except Exception as exc:  # noqa: BLE001 - 单次读失败按不可用处理
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

    def _nearest_fixed_point(self, x: float, y: float):
        """离 (x, y) 最近的固定点位，返回 (px, py, pyaw, name) 或 None。"""
        best = None
        for name, pose in self._load_pois().items():
            distance = math.hypot(pose[0] - x, pose[1] - y)
            if best is None or distance < best[0]:
                best = (distance, pose[0], pose[1], pose[2], name)
        if best is None:
            return None
        return best[1], best[2], best[3], best[4]

    def _slam_status(self) -> int:
        """定位状态：0=正常，1=丢失，2=降级，5=等待重定位，6=重定位中。"""
        try:
            status = self._chassis.get_localization_status()
            return int(status.get("slam_status", -1))
        except Exception as exc:  # noqa: BLE001 - 查询抖动当成未知
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
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._slam_status() == 0:
                return True
            time.sleep(0.5)
        return False

    def recover_here(self, timeout_s: float = _RECOVER_WAIT_S) -> bool:
        """以当前位姿为初值恢复定位，等 slam_status 回到 0。

        只有 1/2/3/5 才主动下发重定位；6/7/8 是底盘自己在恢复，
        这时只等状态回到 0，重复下发重定位会把恢复流程打断。
        """
        slam = self._slam_status_stable()
        if slam == 0:
            return True
        if slam in _RELOCALIZE_BUSY_STATUS:
            self._log.warn(
                f"底盘正在恢复定位（slam_status={slam}），"
                "等它自己完成，不重复下发重定位")
            return self._wait_slam_ok(timeout_s)
        if slam not in _RELOCALIZE_TRIGGER_STATUS:
            self._log.warn(
                f"定位状态 slam_status={slam} 不在自动重定位范围，先不动")
            return False
        pose = self._read_pose()
        if not isinstance(pose, dict) or pose.get("x") is None:
            self._log.error("读不到当前位姿，不能给重定位一个可信初值，放弃")
            return False
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))
        self._log.warn(
            f"当前位姿：x={x:.2f} y={y:.2f} yaw={yaw:.2f}"
            f"（置信度 {pose.get('confidence', '未知')}）")
        # 定位降级时位姿本身不可信，若机器人停在某个固定点附近，
        # 用该点的精确坐标作为重定位初值，避免拿漂移位姿越定越偏。
        snap = self._nearest_fixed_point(x, y)
        if snap is not None:
            offset = math.hypot(snap[0] - x, snap[1] - y)
            if offset <= _RECOVER_SNAP_MAX_M:
                self._log.warn(
                    f"重定位初值吸附到固定点 {snap[3]}："
                    f"x={snap[0]:.2f} y={snap[1]:.2f}（偏离当前 {offset:.2f}m）")
                x, y, yaw = snap[0], snap[1], snap[2]
        if not self._trigger_recover(x, y, yaw, timeout_s):
            return False
        if self._wait_slam_ok(timeout_s):
            time.sleep(_RECOVER_SETTLE_S)
            self._log.info("重定位完成，定位恢复正常")
            return True
        self._log.warn(f"重定位超时（>{timeout_s:.0f}s），定位仍未恢复")
        return False

    def relocalize(self, poi_name: str, timeout_s: float = 20.0) -> bool:
        """以点位坐标作为初始位姿触发重定位，等 slam_status 变成 0。"""
        slam = self._slam_status_stable()
        if slam == 0:
            self._log.info("定位已正常，跳过启动自动重定位")
            return True
        if slam in _RELOCALIZE_BUSY_STATUS:
            self._log.warn(
                f"底盘正在恢复定位（slam_status={slam}），"
                "等它自己完成，不重复下发重定位")
            return self._wait_slam_ok(timeout_s)
        if slam not in _RELOCALIZE_TRIGGER_STATUS:
            self._log.warn(
                f"定位状态 slam_status={slam} 不在自动重定位范围，先不动")
            return False
        pose = self._poi_pose(poi_name)
        if pose is None:
            self._log.warn(f"地图上找不到点位 {poi_name}，跳过自动重定位")
            return False
        x, y, yaw = pose
        self._log.info(
            f"在 {poi_name} 自动重定位：x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
        if not self._trigger_recover(x, y, yaw, timeout_s):
            return False
        if self._wait_slam_ok(timeout_s):
            self._log.info("自动重定位完成，定位正常")
            return True
        self._log.warn(f"自动重定位超时（>{timeout_s:.0f}s），定位仍未就绪")
        return False

    def check_ready(self) -> tuple[bool, str]:
        """导航前检查：定位可恢复 + 打开 AEB。

        可恢复状态（1/2/3/5）和底盘正在恢复（6/7/8）不在这里拦，
        交给 go() 按状态决定等待还是重定位；只有读不到状态或状态未知才拒绝。
        """
        try:
            status = self._chassis.get_localization_status()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"读取定位状态失败：{exc}")
            return False, "我这边读不到定位状态，先不移动了"
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
        self._set_aeb(True)
        return True, ""

    # ------------------------------------------------------------- 导航主流程
    def go(self, poi_name: str, timeout_s: float, guard=None) -> str:
        """定点导航到点位，返回 success / failure / blocked / timeout / localization。

        guard: 可选的前向防撞对象，判到障碍就取消本次导航并返回 blocked。
        整条路线按 places.yaml 的 routes 图拆成多段直线，逐段下发 move_to；
        中途被打断后由上层再次调用，按当前位姿重新规划剩余路段。
        """
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

        if not self._check_localization():
            return "localization"
        if not self._check_health():
            return "failure"

        deadline = time.monotonic() + timeout_s
        self._set_aeb(True)
        try:
            legs = self.plan_route(poi_name)
            if not legs:
                self._log.info(f"当前位置已在 {poi_name} 附近，无需移动")
                return "success"
            self._log.info(
                f"定点导航到 {poi_name}：{len(legs)} 段直线，"
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
            # 无论成功 / 失败 / 取消，都先取消动作再关 AEB
            self._cancel_action()
            self._set_aeb(False)

    def _navigate_leg(self, leg, deadline: float, guard) -> str:
        """下发一段 move_to_poi 并等终态；失败就重新下发同一个导航任务。

        避障交给底盘：只发 move_to_poi 导航任务，底盘自己绕/停；
        被防撞或超时打断才返回，任务失败则等一小会重新下发。
        """
        for attempt in range(_LEG_ATTEMPTS):
            if attempt:
                self._log.warn(
                    f"{leg.name} 导航任务失败，{_REDISPATCH_SETTLE_S:.1f}s 后"
                    f"重新下发 move_to_poi（{attempt + 1}/{_LEG_ATTEMPTS}）")
                time.sleep(_REDISPATCH_SETTLE_S)
            action_id = self._dispatch(leg)
            if action_id is None:
                continue
            outcome = self._run_action(action_id, leg.name, deadline, guard)
            if outcome == "success":
                return "success"
            if outcome in ("blocked", "timeout"):
                return outcome
            self._log.warn(
                f"{leg.name} 第 {attempt + 1} 次 move_to_poi 未完成"
                f"（{outcome}），按底盘导航任务重新下发")
        self._log.error(
            f"{leg.name} 连续 {_LEG_ATTEMPTS} 次 move_to_poi 均失败")
        return "failure"

    def _check_localization(self) -> bool:
        """定位门控：0 放行；1/2/3/5 主动重定位；6/7/8 只等不插手。"""
        slam = self._slam_status_stable()
        if slam == 0:
            return True
        if slam in _RELOCALIZE_BUSY_STATUS:
            self._log.warn(
                f"底盘正在恢复定位（slam_status={slam}），等它自己完成")
            return self._wait_slam_ok(_RECOVER_WAIT_S)
        if slam in _RELOCALIZE_TRIGGER_STATUS:
            self._log.warn(f"定位异常（slam_status={slam}），先重定位")
            return self.recover_here()
        self._log.error(
            f"定位状态不可用（slam_status={slam}），本次导航拒绝")
        return False

    def _check_health(self) -> bool:
        """底盘健康检查：只有 A/B 类故障阻止运动，C 类告警仅记录。"""
        try:
            health = self._chassis.get_robot_health()
        except Exception as exc:  # noqa: BLE001 - 读不到健康状态不阻断导航
            self._log.warn(f"读取底盘健康状态失败（继续）：{exc}")
            return True
        entries = list(getattr(health, "baseError", []) or [])
        blocking = [entry for entry in entries if _is_blocking_dtc(entry)]
        warnings = [entry for entry in entries if not _is_blocking_dtc(entry)]
        if warnings:
            self._log.warn(
                "底盘告警（不影响运动，继续）："
                + "；".join(_describe_dtc(entry) for entry in warnings))
        if blocking:
            self._log.error(
                "底盘健康检查未通过："
                + "；".join(_describe_dtc(entry) for entry in blocking))
            return False
        if not entries and (
            getattr(health, "hasFatal", False)
            or getattr(health, "hasError", False)
        ):
            # 标志为真却没有任何 DTC 明细：无法判断严重程度，保守拒绝
            self._log.error(
                "底盘健康检查未通过：未返回 DTC 详情"
                f"（hasFatal={getattr(health, 'hasFatal', None)}, "
                f"hasError={getattr(health, 'hasError', None)}）")
            return False
        return True

    def _run_action(self, action_id: str, label: str,
                    deadline: float, guard) -> str:
        """轮询底盘动作到终态。

        返回 success / failure / timeout / blocked。
        failure 表示底盘导航任务失败，由调用方重新下发 move_to_poi；
        blocked 表示前向防撞取消。
        """
        started = time.monotonic()
        blocked_hits = 0
        next_guard_log = 0.0
        last_log = 0.0
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
            except Exception as exc:  # noqa: BLE001 - 查询抖动继续等
                self._log.warn(f"查询导航状态失败（继续等待）：{exc}")
                time.sleep(_POLL_S)
                continue
            state = classify(status)
            if state == "success":
                self._log.info(
                    f"到达 {label}，用时 {now - started:.1f}s")
                return "success"
            if state == "failure":
                elapsed = time.monotonic() - started
                note = "（从未起步）" if _never_started(status) else ""
                self._log.error(
                    f"导航到 {label} 失败，用时 {elapsed:.1f}s{note}：{status}")
                self._cancel_and_settle(action_id)
                return "failure"

            if now - last_log >= 2.0:
                last_log = now
                self._log.info(
                    f"{label} 运动进行中，剩余超时 "
                    f"{max(0.0, deadline - now):.0f}s")
            time.sleep(_POLL_S)

    # ------------------------------------------------------------- 运动下发
    def _dispatch(self, leg):
        """按段下发 move_to_poi(点位名)，返回 action_id；失败返回 None。"""
        try:
            response = self._chassis.move_to_poi(
                leg.name, flags=_POI_FLAGS)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"下发 move_to_poi({leg.name}) 失败：{exc}")
            return None
        action_id = (
            str(response.get("action_id"))
            if isinstance(response, dict) else ""
        )
        if not action_id:
            self._log.error(
                f"导航到 {leg.name} 未返回 action_id：{response!r}")
            return None
        self._log.info(
            f"已下发 move_to_poi({leg.name})，action_id={action_id}")
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

    def _cancel(self, action_id=None) -> None:
        try:
            self._chassis.cancel_action(
                [action_id] if action_id else None)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"取消导航失败：{exc}")

    def _cancel_action(self) -> None:
        """兜底收尾：取消所有仍在执行的底盘动作。"""
        self._cancel(None)
