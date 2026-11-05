"""导览语音服务端：/guide/speak（Speak Action）-> 平台 TTS。

落到本体 TTS 服务 /intelligent_interaction/tts/play（interaction_msgs/srv/TtsService）：

    1. cmd='query' 查当前播报状态（playing 时 append 会自动排队）；
    2. cmd='append', type='text', audio_id=<audio_id> 排队播报；
    3. 轮询 cmd='query' 直到播报结束，期间发 Feedback；
    4. 目标被取消时用 cmd='stop', audio_id=<audio_id> 真正打断音频。

audio_id 必须是 audio_config.json 里开启 stop 权限的文本型实体（当前 audio_guide），
纯 stop 不带 audio_id 会被平台忽略，音频会播完。

执行回调里会阻塞轮询，必须 ReentrantCallbackGroup + MultiThreadedExecutor，
否则 cancel 和其它回调无法被处理。
"""

from __future__ import annotations

import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from interaction_msgs.srv import TtsService
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from guide_core import load_yaml

from guide_interfaces.action import Speak


def _load_config() -> dict:
    share = get_package_share_directory("guide_voice")
    path = os.path.join(share, "config", "voice.yaml")
    return load_yaml(path)


class SpeakActionServer(Node):
    def __init__(self) -> None:
        super().__init__("guide_speak")
        cfg = _load_config()
        timing = cfg.get("timing") or {}

        self.declare_parameter("dry_run", bool(cfg.get("dry_run", True)))
        self.declare_parameter("tts_service",
                               str(cfg.get("tts_service",
                                           "/intelligent_interaction/tts/play")))
        self.declare_parameter("audio_id", str(cfg.get("audio_id", "audio_guide")))
        self.declare_parameter("query_timeout_s",
                               float(timing.get("query_timeout_s", 120.0)))

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.tts = str(self.get_parameter("tts_service").value)
        self.audio_id = str(self.get_parameter("audio_id").value)
        self.timeout = float(self.get_parameter("query_timeout_s").value)

        self.poll = float(timing.get("query_poll_s", 0.2))
        self.playing_status = str(timing.get("playing_status", "playing"))
        self.settle = float(timing.get("settle_s", 0.4))
        self.s_per_char = float(timing.get("est_char_rate_s", 0.22))
        self.min_duration = float(timing.get("est_min_duration_s", 1.5))

        self.cli = self.create_client(TtsService, self.tts)
        self.group = ReentrantCallbackGroup()
        # 同一时间只允许一个语音 Goal，避免两句讲解叠在一起
        self._exec_lock = threading.Lock()
        self._shutting_down = False

        self.server = ActionServer(
            self,
            Speak,
            "/guide/speak",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self.group,
        )
        self.get_logger().info(
            f"语音服务端就绪 /guide/speak -> {self.tts} "
            f"audio_id={self.audio_id} dry_run={self.dry_run}")

    def _on_goal(self, goal_request) -> GoalResponse:
        if not goal_request.text.strip():
            self.get_logger().warn("拒绝空文本的播报目标")
            return GoalResponse.REJECT
        if self._exec_lock.locked():
            self.get_logger().warn("已有语音 Goal 执行中，拒绝新 Goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> Speak.Result:
        if not self._exec_lock.acquire(blocking=False):
            self.get_logger().warn("已有语音 Goal 执行中，拒绝并发 Goal")
            goal_handle.abort()
            return self._result(False, "已有语音 Goal 执行中", "rejected")
        try:
            return self._execute_locked(goal_handle)
        finally:
            self._exec_lock.release()

    def _execute_locked(self, goal_handle) -> Speak.Result:
        req = goal_handle.request
        text = req.text.strip()

        if self.dry_run:
            self.get_logger().info(f"[dry_run] 播报: {text}")
            goal_handle.succeed()
            return self._result(True, "dry_run 未调用 TTS", "done")

        if not self.cli.wait_for_service(timeout_sec=5.0):
            message = f"TTS 服务 {self.tts} 未就绪"
            self.get_logger().error(message)
            goal_handle.abort()
            return self._result(False, message, "rejected")

        if req.interrupt:
            self.get_logger().info("interrupt=true：先打断当前播报")
            self._stop_tts()

        est = max(self.min_duration, len(text) * self.s_per_char)
        hard = min(est * 2.0 + 5.0, self.timeout)
        est_deadline = min(est + 3.0, self.timeout)

        self._feedback(goal_handle, 0.0, "submitting")
        appended = self._call("append", text, self.audio_id, "text")
        if appended is None or not appended.success:
            message = "TTS append 未成功"
            self.get_logger().error(message)
            goal_handle.abort()
            return self._result(False, message, "tts_failed")

        t0 = time.monotonic()
        seen_playing = False
        idle_since = None
        while True:
            elapsed = time.monotonic() - t0
            if self._shutting_down or goal_handle.is_cancel_requested:
                self._stop_tts()
                goal_handle.canceled()
                message = f"播报被取消，已打断音频（已等 {elapsed:.1f}s）"
                self.get_logger().warn(message)
                return self._result(False, message, "rejected")

            response = self._call("query", "")
            playing = bool(response is not None and response.success
                           and response.status == self.playing_status)
            if playing:
                seen_playing = True
                idle_since = None
                phase = "playing"
            else:
                phase = "settling"
                if idle_since is None:
                    idle_since = elapsed

            done = (seen_playing and idle_since is not None
                    and elapsed - idle_since >= self.settle)
            if done:
                goal_handle.succeed()
                message = f"播报结束（{elapsed:.1f}s）"
                self.get_logger().info(message)
                return self._result(True, message, "done")
            if not seen_playing and elapsed >= est_deadline:
                message = (f"无法确认播报完成（从未观测到播放状态，"
                           f"已等待 {elapsed:.1f}s）")
                self.get_logger().warn(message)
                goal_handle.abort()
                return self._result(False, message, "timeout")
            if elapsed >= hard:
                message = f"播放查询超时（超过 {hard:.1f}s 上限）"
                self.get_logger().warn(message)
                self._stop_tts()
                goal_handle.abort()
                return self._result(False, message, "timeout")

            self._feedback(goal_handle, min(elapsed / est, 1.0), phase)
            time.sleep(self.poll)

    # ---------------- 工具 ----------------
    @staticmethod
    def _feedback(goal_handle, progress: float, phase: str) -> None:
        feedback = Speak.Feedback()
        feedback.progress = float(progress)
        feedback.phase = phase
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(success: bool, message: str, state: str) -> Speak.Result:
        result = Speak.Result()
        result.success = bool(success)
        result.message = message
        result.state = state
        return result

    def _call(self, cmd: str, text: str, audio_id: str = "", msg_type: str = ""):
        req = TtsService.Request()
        req.text = text
        req.type = msg_type or ("text" if text else "")
        req.cmd = cmd
        req.audio_id = audio_id
        req.extra = ""
        future = self.cli.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn(f"TTS {cmd} 调用超时")
                return None
            time.sleep(0.02)
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"TTS {cmd} 调用异常: {exc}")
            return None

    def _stop_tts(self) -> None:
        # stop 是 best-effort：调用异常不得中断播报主流程
        try:
            self._call("stop", "", self.audio_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TTS stop 异常（忽略）：{exc}")

    def shutdown(self) -> None:
        self._shutting_down = True
        if not self.dry_run:
            self._stop_tts()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakActionServer()
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
