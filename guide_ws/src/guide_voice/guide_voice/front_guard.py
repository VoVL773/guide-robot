"""导航时的前向防撞：用头部深度相机检测正前方障碍。

底盘自带的 AEB 在自主导航里不生效（实测人站路上照样撞），
这里自己订阅深度图，取画面中央区域的最近距离，判到就取消导航。
"""

from __future__ import annotations

import threading
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# ROI 内无效像素（过近时输出 0）占比超过该值，视为前方有近处障碍
_ZERO_RATIO_BLOCK = 0.06


class FrontGuard:
    def __init__(
        self,
        node,
        topic: str = "/ob_camera_head/depth/image_raw",
        stop_distance: float = 0.9,
        roi_x: tuple[float, float] = (0.35, 0.65),
        roi_y: tuple[float, float] = (0.25, 0.60),
        stale_s: float = 1.5,
    ) -> None:
        self._log = node.get_logger()
        self._stop = float(stop_distance)
        self._roi_x = roi_x
        self._roi_y = roi_y
        self._stale_s = float(stale_s)
        self._lock = threading.Lock()
        self._distance: float | None = None
        self._zero_ratio = 0.0
        self._stamp = 0.0
        # 深度图发布端是 best-effort，用传感器 QoS 才能收到
        node.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        try:
            if msg.encoding == "16UC1":
                img = np.frombuffer(msg.data, dtype=np.uint16)
                img = img.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                img = np.frombuffer(msg.data, dtype=np.float32)
                img = img.reshape(msg.height, msg.step // 4)[:, :msg.width]
            else:
                return
        except Exception:  # noqa: BLE001 - 单帧解析异常忽略
            return

        h, w = img.shape[:2]
        roi = img[
            int(h * self._roi_y[0]):int(h * self._roi_y[1]),
            int(w * self._roi_x[0]):int(w * self._roi_x[1]),
        ]
        valid = roi[np.isfinite(roi) & (roi > 0.05)]
        if valid.size == 0:
            return
        # 用低分位数而不是最小值，避免个别噪点误触发
        zero_ratio = float(np.count_nonzero(roi <= 0.05)) / float(roi.size)
        with self._lock:
            self._distance = float(np.percentile(valid, 5))
            self._zero_ratio = zero_ratio
            self._stamp = time.monotonic()

    def distance(self) -> float | None:
        """正前方最近距离（米）；没有新数据时返回 None。"""
        with self._lock:
            if (self._distance is None
                    or time.monotonic() - self._stamp > self._stale_s):
                return None
            return self._distance

    def blocked(self) -> bool:
        distance = self.distance()
        if distance is None:
            return False
        if distance < self._stop:
            return True
        # 贴得太近时深度会输出大片 0（无效值），这时 p5 反而看到远处背景
        return self.zero_ratio() >= _ZERO_RATIO_BLOCK

    def zero_ratio(self) -> float:
        """ROI 内无效/过近像素占比（0~1）。"""
        with self._lock:
            return self._zero_ratio

    def stop_distance(self) -> float:
        """停车阈值（米）。"""
        return self._stop
