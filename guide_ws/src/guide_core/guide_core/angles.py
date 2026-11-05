"""角度工具（导航与编排共用）。"""

from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    """把任意弧度归一化到 [-pi, pi)。"""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    """a 相对 b 的最短有向角差，结果在 [-pi, pi)。"""
    return wrap_angle(float(a) - float(b))
