"""导览任务的 YAML 配置加载与校验。

配置文件是唯一真源：点位名称与参观顺序全部来自 YAML，代码里不出现任何
POI_xx 常量。用户在地图上标记点位后，只需要改 YAML，不需要改代码。

校验失败一律抛出 ValueError（reusable_model 的 ConfigError 也是 ValueError 子类），
消息里带上文件路径与出错的键，方便现场直接定位。
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any

# RobotForge 是 src 布局的普通 Python 包，未被 colcon 安装，
# 与 robot_controller 一样通过 sys.path 注入。
_ROBOTFORGE_SRC = os.environ.get(
    "ROBOTFORGE_SRC", "/home/nvidia/greeting-main/packages/reusable_model/src"
)
if _ROBOTFORGE_SRC and os.path.isdir(_ROBOTFORGE_SRC) and _ROBOTFORGE_SRC not in sys.path:
    sys.path.insert(0, _ROBOTFORGE_SRC)

try:
    from reusable_model.io.yaml_config import load_yaml as _robotforge_load_yaml
except Exception as exc:  # noqa: BLE001 - 缺依赖时必须给出可操作的提示
    raise RuntimeError(
        "无法导入 RobotForge reusable_model.io.yaml_config。"
        f"请确认 {_ROBOTFORGE_SRC} 存在，或用环境变量 ROBOTFORGE_SRC 指向"
        "/home/nvidia/greeting-main/packages/reusable_model/src"
    ) from exc


def load_yaml(path: str) -> dict[str, Any]:
    """读取 YAML 映射；文件缺失/语法错误由 RobotForge 给出带路径的错误。"""
    return _robotforge_load_yaml(path)


# --------------------------------------------------------------------------- 点位


@dataclass(frozen=True)
class Tracks:
    outbound: str = ""
    return_track: str = ""


@dataclass(frozen=True)
class Timeouts:
    to_start_s: float = 180.0
    per_poi_s: float = 180.0


@dataclass(frozen=True)
class Speed:
    outbound_mps: float = 0.3
    return_mps: float = 0.3


@dataclass(frozen=True)
class Relocalize:
    enabled: bool = True
    timeout_s: float = 60.0
    poll_interval_s: float = 0.5
    max_time_ms: int | None = None
    # (x, y, yaw)，为空时用 start_poi 的坐标
    pose: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class RangeLimit:
    """应用层软限位：拦截数量级写错的坐标，不是精确现场边界。"""

    x: tuple[float, float] = (-10.0, 45.0)
    y: tuple[float, float] = (-20.0, 20.0)

    def contains(self, x: float, y: float) -> bool:
        return self.x[0] <= x <= self.x[1] and self.y[0] <= y <= self.y[1]


@dataclass(frozen=True)
class PointsConfig:
    start_poi: str = ""
    poi_order: tuple[str, ...] = ()
    tracks: Tracks = field(default_factory=Tracks)
    transit_pois: tuple[str, ...] = ()
    timeouts: Timeouts = field(default_factory=Timeouts)
    dwell_s: float = 15.0
    dwell_overrides: dict[str, float] = field(default_factory=dict)
    speed: Speed = field(default_factory=Speed)
    arrival_tolerance_m: float = 0.5
    yaw_overrides: dict[str, float] = field(default_factory=dict)
    relocalize: Relocalize = field(default_factory=Relocalize)
    range_limit: RangeLimit = field(default_factory=RangeLimit)
    source: str = ""

    @property
    def marked(self) -> bool:
        """点位是否已经标记完成（起点 + 至少一个讲解点）。"""
        return bool(self.start_poi) and bool(self.poi_order)

    def dwell_for(self, poi: str) -> float:
        return float(self.dwell_overrides.get(poi, self.dwell_s))

    def yaw_for(self, poi: str, default: float = 0.0) -> float:
        return float(self.yaw_overrides.get(poi, default))


_POINTS_KEYS = {
    "poi_source", "start_poi", "poi_order", "tracks", "transit_pois",
    "timeouts", "dwell_s", "dwell_overrides", "speed", "arrival_tolerance_m",
    "yaw_overrides", "relocalize", "range_limit",
}


def _error(path: str, key: str, message: str) -> ValueError:
    return ValueError(f"{path}: {key} {message}")


def _num(path: str, key: str, value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        raise _error(path, key, f"必须是数值，实际为 {value!r}")
    if not math.isfinite(out):
        raise _error(path, key, f"必须是有限数值，实际为 {value!r}")
    return out


def _mapping(path: str, key: str, value: Any) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise _error(path, key, f"必须是映射，实际为 {value!r}")
    return value


def _unknown(path: str, key: str, raw: dict, allowed: set[str]) -> None:
    extra = set(raw) - allowed
    if extra:
        raise _error(path, key, f"存在未知配置项 {sorted(extra)}")


def _range(path: str, key: str, value: Any) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise _error(path, key, f"必须是 [下限, 上限]，实际为 {value!r}")
    lo = _num(path, key, value[0])
    hi = _num(path, key, value[1])
    if lo >= hi:
        raise _error(path, key, f"下限必须小于上限，实际为 {value!r}")
    return (lo, hi)


def load_points(path: str) -> PointsConfig:
    """加载并校验 guide_points.yaml。"""
    raw = load_yaml(path)
    _unknown(path, "<root>", raw, _POINTS_KEYS)

    source = str(raw.get("poi_source", "chassis")).strip() or "chassis"
    if source != "chassis":
        raise _error(path, "poi_source", f"目前只支持 chassis，实际为 {source!r}")

    start_poi = str(raw.get("start_poi") or "").strip()

    order_raw = raw.get("poi_order") or []
    if not isinstance(order_raw, (list, tuple)):
        raise _error(path, "poi_order", f"必须是列表，实际为 {order_raw!r}")
    poi_order = tuple(str(v).strip() for v in order_raw)
    if any(not name for name in poi_order):
        raise _error(path, "poi_order", "存在空点位名")
    if len(set(poi_order)) != len(poi_order):
        raise _error(path, "poi_order", "存在重复点位")
    if start_poi and start_poi in poi_order:
        raise _error(path, "poi_order", f"起点 {start_poi} 不应出现在讲解顺序里")
    if poi_order and not start_poi:
        raise _error(path, "start_poi", "讲解顺序已填写，但起点为空")

    tracks_raw = _mapping(path, "tracks", raw.get("tracks"))
    _unknown(path, "tracks", tracks_raw, {"outbound", "return"})
    tracks = Tracks(
        outbound=str(tracks_raw.get("outbound") or "").strip(),
        return_track=str(tracks_raw.get("return") or "").strip(),
    )

    transit_raw = raw.get("transit_pois") or []
    if not isinstance(transit_raw, (list, tuple)):
        raise _error(path, "transit_pois", f"必须是列表，实际为 {transit_raw!r}")
    transit = tuple(str(v).strip() for v in transit_raw)
    extra = set(transit) - set(poi_order)
    if extra:
        raise _error(path, "transit_pois", f"不在 poi_order 中: {sorted(extra)}")

    timeouts_raw = _mapping(path, "timeouts", raw.get("timeouts"))
    _unknown(path, "timeouts", timeouts_raw, {"to_start_s", "per_poi_s"})
    timeouts = Timeouts(
        to_start_s=_num(path, "timeouts.to_start_s",
                        timeouts_raw.get("to_start_s", 180.0)),
        per_poi_s=_num(path, "timeouts.per_poi_s",
                       timeouts_raw.get("per_poi_s", 180.0)),
    )
    if timeouts.to_start_s <= 0 or timeouts.per_poi_s <= 0:
        raise _error(path, "timeouts", "超时必须为正数")

    dwell_s = _num(path, "dwell_s", raw.get("dwell_s", 15.0))
    if dwell_s < 0:
        raise _error(path, "dwell_s", "不能为负数")

    dwell_raw = _mapping(path, "dwell_overrides", raw.get("dwell_overrides"))
    dwell_overrides = {
        str(k): _num(path, f"dwell_overrides.{k}", v) for k, v in dwell_raw.items()
    }
    extra = set(dwell_overrides) - set(poi_order)
    if extra:
        raise _error(path, "dwell_overrides", f"不在 poi_order 中: {sorted(extra)}")
    if any(v < 0 for v in dwell_overrides.values()):
        raise _error(path, "dwell_overrides", "不能为负数")

    speed_raw = _mapping(path, "speed", raw.get("speed"))
    _unknown(path, "speed", speed_raw, {"outbound_mps", "return_mps"})
    speed = Speed(
        outbound_mps=_num(path, "speed.outbound_mps",
                          speed_raw.get("outbound_mps", 0.3)),
        return_mps=_num(path, "speed.return_mps",
                        speed_raw.get("return_mps", 0.3)),
    )
    for name, value in (("speed.outbound_mps", speed.outbound_mps),
                        ("speed.return_mps", speed.return_mps)):
        if not 0.05 <= value <= 1.0:
            raise _error(path, name, f"应在 [0.05, 1.0] m/s，实际为 {value}")

    tolerance = _num(path, "arrival_tolerance_m",
                     raw.get("arrival_tolerance_m", 0.5))
    if not 0.05 <= tolerance <= 2.0:
        raise _error(path, "arrival_tolerance_m", f"应在 [0.05, 2.0]，实际为 {tolerance}")

    yaw_raw = _mapping(path, "yaw_overrides", raw.get("yaw_overrides"))
    yaw_overrides = {
        str(k): _num(path, f"yaw_overrides.{k}", v) for k, v in yaw_raw.items()
    }
    extra = set(yaw_overrides) - set(poi_order) - ({start_poi} if start_poi else set())
    if extra:
        raise _error(path, "yaw_overrides", f"未知点位: {sorted(extra)}")
    for name, value in yaw_overrides.items():
        if not -math.pi <= value <= math.pi:
            raise _error(path, f"yaw_overrides.{name}", f"应在 [-pi, pi]，实际为 {value}")

    relocalize_raw = _mapping(path, "relocalize", raw.get("relocalize"))
    _unknown(path, "relocalize", relocalize_raw,
             {"enabled", "timeout_s", "poll_interval_s", "max_time_ms", "pose"})
    pose_raw = relocalize_raw.get("pose")
    pose = None
    if pose_raw is not None:
        pose_map = _mapping(path, "relocalize.pose", pose_raw)
        _unknown(path, "relocalize.pose", pose_map, {"x", "y", "yaw"})
        pose = (
            _num(path, "relocalize.pose.x", pose_map.get("x", 0.0)),
            _num(path, "relocalize.pose.y", pose_map.get("y", 0.0)),
            _num(path, "relocalize.pose.yaw", pose_map.get("yaw", 0.0)),
        )
    max_time_ms = relocalize_raw.get("max_time_ms")
    relocalize = Relocalize(
        enabled=bool(relocalize_raw.get("enabled", True)),
        timeout_s=_num(path, "relocalize.timeout_s",
                       relocalize_raw.get("timeout_s", 60.0)),
        poll_interval_s=_num(path, "relocalize.poll_interval_s",
                             relocalize_raw.get("poll_interval_s", 0.5)),
        max_time_ms=None if max_time_ms is None else int(max_time_ms),
        pose=pose,
    )
    if relocalize.timeout_s <= 0 or relocalize.poll_interval_s <= 0:
        raise _error(path, "relocalize", "超时与轮询间隔必须为正数")

    range_raw = _mapping(path, "range_limit", raw.get("range_limit"))
    _unknown(path, "range_limit", range_raw, {"x", "y"})
    range_limit = RangeLimit(
        x=_range(path, "range_limit.x", range_raw.get("x", [-10.0, 45.0])),
        y=_range(path, "range_limit.y", range_raw.get("y", [-20.0, 20.0])),
    )

    return PointsConfig(
        start_poi=start_poi,
        poi_order=poi_order,
        tracks=tracks,
        transit_pois=transit,
        timeouts=timeouts,
        dwell_s=dwell_s,
        dwell_overrides=dwell_overrides,
        speed=speed,
        arrival_tolerance_m=tolerance,
        yaw_overrides=yaw_overrides,
        relocalize=relocalize,
        range_limit=range_limit,
        source=path,
    )


def missing_pois(config: PointsConfig, chassis_pois: set[str]) -> list[str]:
    """底盘地图上缺失的点位（含起点）。"""
    wanted = [p for p in (config.start_poi,) if p] + list(config.poi_order)
    return [p for p in wanted if p not in chassis_pois]


# --------------------------------------------------------------------------- 讲解词


@dataclass(frozen=True)
class PointScript:
    text: str = ""
    head_yaw: float = 0.0
    head_pitch: float = 0.0


@dataclass(frozen=True)
class ScriptConfig:
    intro: str = ""
    outro: str = ""
    default_head_yaw: float = 0.0
    default_head_pitch: float = 0.0
    points: dict[str, PointScript] = field(default_factory=dict)
    source: str = ""

    def point(self, poi: str) -> PointScript:
        found = self.points.get(poi)
        if found is not None:
            return found
        return PointScript(head_yaw=self.default_head_yaw,
                           head_pitch=self.default_head_pitch)


_SCRIPT_KEYS = {"intro", "outro", "head", "points"}
_POINT_KEYS = {"text", "head_yaw", "head_pitch"}


def load_script(path: str) -> ScriptConfig:
    """加载并校验 guide_script.yaml。"""
    raw = load_yaml(path)
    _unknown(path, "<root>", raw, _SCRIPT_KEYS)

    head_raw = _mapping(path, "head", raw.get("head"))
    _unknown(path, "head", head_raw, {"yaw", "pitch"})
    default_yaw = _num(path, "head.yaw", head_raw.get("yaw", 0.0))
    default_pitch = _num(path, "head.pitch", head_raw.get("pitch", 0.0))

    points_raw = _mapping(path, "points", raw.get("points"))
    points: dict[str, PointScript] = {}
    for name, value in points_raw.items():
        key = str(name).strip()
        if not key:
            raise _error(path, "points", "存在空点位名")
        entry = _mapping(path, f"points.{key}", value)
        _unknown(path, f"points.{key}", entry, _POINT_KEYS)
        points[key] = PointScript(
            text=str(entry.get("text") or "").strip(),
            head_yaw=_num(path, f"points.{key}.head_yaw",
                          entry.get("head_yaw", default_yaw)),
            head_pitch=_num(path, f"points.{key}.head_pitch",
                            entry.get("head_pitch", default_pitch)),
        )

    return ScriptConfig(
        intro=str(raw.get("intro") or "").strip(),
        outro=str(raw.get("outro") or "").strip(),
        default_head_yaw=default_yaw,
        default_head_pitch=default_pitch,
        points=points,
        source=path,
    )


def missing_scripts(config: PointsConfig, script: ScriptConfig) -> list[str]:
    """没有讲解词的点位。"""
    return [p for p in config.poi_order if not script.point(p).text]
