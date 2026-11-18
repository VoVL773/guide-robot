"""guide_voice 对话节点：语音识别 -> 本地千问 -> TTS 播报，兼语音导航。

订阅：
  /audio_asr/iat          lyre_msgs/AsrIat   平台语音识别结果（需唤醒词）
  /guide_voice/chat       std_msgs/String    文本直连，便于没有麦克风时测试

播报：
  /intelligent_interaction/tts/play  interaction_msgs/TtsService（append）

导航：
  说「去 乐知院 / 3号点」导航到地图点位；到达后询问还有什么事，
  听到「没事了」就回待命点（config/places.yaml 的 standby）。

用法：
  ros2 launch guide_voice voice_chat.launch.py
  ros2 topic pub --once /guide_voice/chat std_msgs/msg/String "{data: '你好'}"
"""

from __future__ import annotations

import json
import os
import queue
import re
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from interaction_msgs.srv import TtsService
from lyre_msgs.msg import AsrIat
from std_msgs.msg import String

from guide_voice.mic_asr import MicAsr
from guide_voice.navigation import Navigator
from guide_voice.places import PlaceBook
from guide_voice.qwen import QwenClient

DEFAULT_SYSTEM_PROMPT = (
    "你是名字叫小郑的机器人，正在展厅里接待访客。"
    "你可以像正常人一样自由聊天：闲聊、讲笑话、回答问题都可以，不要只聊展厅相关的话题。"
    "用简短、自然的中文口语回答，一般一到三句话，不要使用列表、编号或markdown。"
    "不知道的事情就老实说不知道，不要编造。"
)


class VoiceChatNode(Node):
    def __init__(self) -> None:
        super().__init__("guide_voice_chat")

        self.declare_parameter("asr_topic", "/audio_asr/iat")
        self.declare_parameter("chat_topic", "/guide_voice/chat")
        self.declare_parameter("tts_service", "/intelligent_interaction/tts/play")
        self.declare_parameter("audio_id", "audio_guide")
        self.declare_parameter("qwen_base_url", "http://120.48.75.178:7006/v1")
        self.declare_parameter("qwen_model", "Qwen3.5-122B-A10B")
        self.declare_parameter("qwen_api_key", "EMPTY")
        self.declare_parameter("qwen_timeout", 30.0)
        self.declare_parameter("history_turns", 6)
        self.declare_parameter("system_prompt", DEFAULT_SYSTEM_PROMPT)
        self.declare_parameter("min_chars", 2)
        self.declare_parameter("dedupe_window", 2.0)
        self.declare_parameter("greet_text", "")
        self.declare_parameter("enable_mic", True)
        self.declare_parameter("mic_topic", "/lyre/audio_stream")
        self.declare_parameter("mic_control_service", "/lyre/audio_control")
        self.declare_parameter("mic_rate_in", 96000)
        self.declare_parameter("vad_threshold", 600.0)
        self.declare_parameter("vad_silence_s", 0.8)
        self.declare_parameter("asr_model", "small")
        self.declare_parameter("asr_compute_type", "int8")
        self.declare_parameter("asr_language", "zh")
        self.declare_parameter(
            "asr_hotwords",
            "乐知院、乐知教育、核心跑酷、核心力量跑酷、创想谷、厕所、卫生间、"
            "洗手间、跆拳道馆、带我去、我想去、回到")
        self.declare_parameter(
            "asr_initial_prompt",
            "以下是普通话的导览对话，常用地点：乐知院、核心力量跑酷、创想谷、"
            "厕所、跆拳道馆。"
            "可能的说法：我想去创想谷、带我去厕所、我想练跆拳道、"
            "回乐知院、去核心力量跑酷。")
        self.declare_parameter("asr_beam_size", 5)
        self.declare_parameter("enable_nav", True)
        self.declare_parameter("chassis_host", "192.168.11.10")
        self.declare_parameter("chassis_port", 9090)
        self.declare_parameter("chassis_timeout", 10.0)
        self.declare_parameter("places_file", "")
        self.declare_parameter("nav_timeout", 180.0)
        self.declare_parameter("track_id", 0)
        self.declare_parameter("max_moving_speed", 0.6)
        self.declare_parameter("auto_relocalize", True)
        self.declare_parameter("startup_goto_standby", True)
        self.declare_parameter("relocalize_timeout", 20.0)
        self.declare_parameter("front_guard_enabled", True)
        self.declare_parameter(
            "front_guard_topic", "/ob_camera_head/depth/image_raw")
        self.declare_parameter("front_guard_stop", 1.2)
        self.declare_parameter("front_guard_wait", 60.0)
        self.declare_parameter("front_guard_retries", 3)

        self._audio_id = str(self.get_parameter("audio_id").value)
        self._min_chars = int(self.get_parameter("min_chars").value)
        self._dedupe_window = float(self.get_parameter("dedupe_window").value)
        tts_service = str(self.get_parameter("tts_service").value)

        self._qwen = QwenClient(
            base_url=str(self.get_parameter("qwen_base_url").value),
            model=str(self.get_parameter("qwen_model").value),
            api_key=str(self.get_parameter("qwen_api_key").value),
            timeout=float(self.get_parameter("qwen_timeout").value),
            system_prompt=str(self.get_parameter("system_prompt").value),
            history_turns=int(self.get_parameter("history_turns").value),
        )

        self._queue: queue.Queue[str | None] = queue.Queue()
        self._closing = False
        self._last_key = ""
        self._last_at = 0.0
        self._awaiting = False
        self._navigating = False
        self._pending = None
        self._nav_timeout = float(self.get_parameter("nav_timeout").value)
        self._nav = None
        self._places = None
        self._guard = None
        if bool(self.get_parameter("enable_nav").value):
            self._setup_navigation()

        self._tts = self.create_client(TtsService, tts_service)
        self.create_subscription(
            AsrIat, str(self.get_parameter("asr_topic").value), self._on_asr, 10)
        self.create_subscription(
            String, str(self.get_parameter("chat_topic").value), self._on_chat, 10)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self._mic = None
        if bool(self.get_parameter("enable_mic").value):
            self._mic = MicAsr(
                self,
                self._on_mic_text,
                mic_topic=str(self.get_parameter("mic_topic").value),
                control_service=str(self.get_parameter("mic_control_service").value),
                rate_in=int(self.get_parameter("mic_rate_in").value),
                vad_threshold=float(self.get_parameter("vad_threshold").value),
                silence_s=float(self.get_parameter("vad_silence_s").value),
                model_name=str(self.get_parameter("asr_model").value),
                compute_type=str(self.get_parameter("asr_compute_type").value),
                language=str(self.get_parameter("asr_language").value),
                hotwords=str(self.get_parameter("asr_hotwords").value),
                initial_prompt=str(self.get_parameter("asr_initial_prompt").value),
                beam_size=int(self.get_parameter("asr_beam_size").value),
            )
            self._mic.start()

        greet = str(self.get_parameter("greet_text").value).strip()
        if greet:
            self._queue.put(greet)

        self.get_logger().info(
            f"对话已就绪：麦克风={'开' if self._mic else '关'}，"
            f"导航={'开' if self._nav else '关'}，"
            f"模型={self.get_parameter('qwen_model').value}")

        if bool(self.get_parameter("auto_relocalize").value):
            threading.Thread(target=self._startup_relocalize, daemon=True).start()

    def _startup_relocalize(self) -> None:
        """启动时以待命点坐标重定位，让底盘先确认自己在哪。"""
        if not self._nav or not self._places or self._places.standby is None:
            self.get_logger().warn("没有可用的待命点，跳过启动自动重定位")
            return
        state = self._nav.localization_state()
        self.get_logger().info(
            f"启动定位自检：{self._nav.localization_detail(state)}")
        if self._nav.localization_idle(state):
            self.get_logger().warn("定位服务处于空闲状态，尝试自动启动定位")
            if not self._nav.restart_localization():
                self.get_logger().error(
                    "定位服务自动启动失败，跳过启动重定位；"
                    "请先重启底盘定位，确认 slam_status 正常后再指挥机器人")
                return
            self.get_logger().info("定位服务已自动启动")
        try:
            ok = self._nav.relocalize(
                self._places.standby.poi,
                float(self.get_parameter("relocalize_timeout").value))
        except Exception as exc:  # noqa: BLE001 - 重定位失败不影响聊天
            self.get_logger().error(f"启动自动重定位异常：{exc}")
            return
        if not ok:
            self.get_logger().warn("自动重定位未成功，跳过启动回待命点")
            return
        if bool(self.get_parameter("startup_goto_standby").value):
            self.get_logger().info("启动后先回待命点")
            self._navigate(self._places.standby, standby=True)

    def _setup_navigation(self) -> None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = str(self.get_parameter("places_file").value).strip()
            if not path:
                path = os.path.join(
                    get_package_share_directory("guide_voice"), "config", "places.yaml")
            self._nav = Navigator(
                str(self.get_parameter("chassis_host").value),
                int(self.get_parameter("chassis_port").value),
                float(self.get_parameter("chassis_timeout").value),
                self.get_logger(),
                track_id=int(self.get_parameter("track_id").value),
                max_speed=float(self.get_parameter("max_moving_speed").value),
            )
            self._places = PlaceBook.load(
                path, self._nav.poi_names(), self.get_logger())
            self._nav.set_route(
                self._places.main_line, self._places.branches)
            self.get_logger().info(f"地点配置：{path}")
            self.get_logger().info(f"可去地点：{self._places.describe()}")
            if bool(self.get_parameter("front_guard_enabled").value):
                from guide_voice.front_guard import FrontGuard
                self._guard = FrontGuard(
                    self,
                    topic=str(self.get_parameter("front_guard_topic").value),
                    stop_distance=float(
                        self.get_parameter("front_guard_stop").value),
                )
                self.get_logger().info(
                    f"前向防撞已启用：{self.get_parameter('front_guard_stop').value}m")
        except Exception as exc:  # noqa: BLE001 - 导航不可用时只保留聊天
            self._nav = None
            self._places = None
            self.get_logger().error(f"导航初始化失败（只保留聊天）：{exc}")

    # ------------------------------------------------------------- 订阅回调
    def _on_asr(self, msg: AsrIat) -> None:
        text = (msg.text or "").strip()
        if not _meaningful(text, self._min_chars):
            return
        if self._navigating:
            self.get_logger().info(f"导航中忽略：{text}")
            return
        key = re.sub(r"\s+", "", text)
        now = time.monotonic()
        if key == self._last_key and now - self._last_at < self._dedupe_window:
            return
        self._last_key, self._last_at = key, now
        self.get_logger().info(f"听到：{text}")
        self._queue.put(text)

    def _on_chat(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not _meaningful(text, self._min_chars):
            return
        if self._navigating:
            self.get_logger().info(f"导航中忽略：{text}")
            return
        self.get_logger().info(f"文本输入：{text}")
        self._queue.put(text)

    def _on_mic_text(self, text: str) -> None:
        text = (text or "").strip()
        if not _meaningful(text, self._min_chars):
            return
        if self._navigating:
            self.get_logger().info(f"导航中忽略：{text}")
            return
        self.get_logger().info(f"听到：{text}")
        self._queue.put(text)

    # --------------------------------------------------------------- 工作线程
    def _worker_loop(self) -> None:
        while not self._closing:
            try:
                text = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if text is None:
                return
            try:
                self._handle(text)
            except Exception as exc:  # noqa: BLE001 - 单次失败不影响后续对话
                self.get_logger().error(f"处理失败：{exc}")

    def _handle(self, text: str) -> None:
        if self._nav is not None and not self._navigating:
            place = self._places.resolve(text)
            if place is not None:
                if _explicit_go(text):
                    self._pending = None
                    self._navigate(place, standby=False)
                    return
                # 「我想练跆拳道」这类只是表达意愿，先问要不要带路
                self._pending = place
                self._speak(f"可以去{place.spoken}，需要我带你去吗？")
                return

            # 上一句提议了地点，这一句看是同意还是拒绝
            if self._pending is not None:
                pending, self._pending = self._pending, None
                if _means_no(text):
                    self._speak("好的，想去的时候随时叫我")
                    return
                if _means_yes(text):
                    self._navigate(pending, standby=False)
                    return

            if self._awaiting and _wants_done(text):
                standby = self._places.standby
                if standby is None:
                    self._awaiting = False
                    self._speak("好的，那我就在这儿等你")
                    return
                self._navigate(standby, standby=True)
                return

            # 没直接说地点（比如「我想练跆拳道」），让模型推断对应地点
            suggestion = self._suggest_place(text)
            if suggestion is not None:
                place, reply = suggestion
                self._pending = place
                self._speak(reply)
                return

        try:
            reply = self._qwen.chat(text)
        except Exception as exc:  # noqa: BLE001 - 大模型不可用时给一句兜底
            self.get_logger().error(f"千问请求失败：{exc}")
            self._speak("我这边暂时联系不上大模型，请稍后再试")
            return
        self.get_logger().info(f"小郑：{reply}")
        self._speak(reply)

    def _suggest_place(self, text: str):
        """用大模型从「我想练跆拳道」这类话里推断地点，返回 (Place, 回复)。"""
        if not any(word in text for word in _PLACE_HINTS):
            return None
        prompt = (
            f"可去的地点：{self._places.describe()}。\n"
            f"访客说：{text}\n"
            "判断访客是不是想去某个地点，可能用活动间接表达（想练跆拳道 = 去跆拳道馆）。"
            "只能从上面地点里选，不要编造；没把握就留空。"
            '只输出 JSON，不要其他文字：'
            '{"selected": "地点名或空字符串", "reply": "一句口语回复，'
            '选到地点时告诉对方可以去哪里并问要不要带路"}'
        )
        try:
            raw = self._qwen.one_shot(prompt, system="你是意图解析器，只输出 JSON，不解释。")
        except Exception as exc:  # noqa: BLE001 - 解析失败就当普通聊天
            self.get_logger().warn(f"地点意图识别失败：{exc}")
            return None

        data = _extract_json(raw)
        if not isinstance(data, dict):
            return None
        selected = str(data.get("selected") or "").strip()
        if not selected:
            return None
        place = self._places.resolve(selected)
        if place is None:
            return None
        reply = str(data.get("reply") or "").strip()
        self.get_logger().info(f"推断地点：{selected} -> {place.poi}")
        return place, (reply or f"可以去{place.spoken}，需要我带你去吗？")

    def _navigate(self, place, standby: bool) -> None:
        self._navigating = True
        self._drain_queue()
        try:
            ok, message = self._nav.check_ready()
            if not ok:
                self._speak(message)
                return
            target = place.spoken
            if not standby:
                self._speak(f"好的，我这就带你去{target}")
            self.get_logger().info(
                f"开始定点导航：{place.poi}（{target}，"
                f"{'回待命点' if standby else '送访客'}）")
            result = self._nav.go(
                place.poi, self._nav_timeout, guard=self._guard)
            attempt = 1
            while result == "blocked":
                if attempt >= int(self.get_parameter("front_guard_retries").value):
                    break
                self._speak("这边暂时过不去，我先停一下")
                if not self._wait_front_clear():
                    break
                self._speak("路通了，我继续走")
                attempt += 1
                result = self._nav.go(
                    place.poi, self._nav_timeout, guard=self._guard)
            if result == "network":
                self._speak("我与底盘的通信异常，任务状态还不能确认，请先检查连接，不要重复下发导航")
                return
            if result == "health":
                self._speak(self._nav.health_message)
                return
            if result == "localization":
                self._speak("我现在定位持续不稳定，先停在这里，请检查地图定位后再叫我")
                return
            if result != "success":
                self._speak(f"抱歉，这次没能到{target}，底盘导航任务没有完成")
                return
            if standby:
                self._awaiting = False
                self._speak(f"好的，我已经回到{target}待命，有事随时叫我")
            else:
                self._awaiting = True
                self._speak(self._arrival_line(target))
        finally:
            self._navigating = False

    def _wait_front_clear(self) -> bool:
        """等正前方障碍消失；一直挡着就返回 False。"""
        if self._guard is None:
            return True
        deadline = time.monotonic() + float(
            self.get_parameter("front_guard_wait").value)
        while time.monotonic() < deadline:
            if not self._guard.blocked():
                distance = self._guard.distance()
                self.get_logger().info(
                    "前方已清空"
                    f"（{round(distance, 2) if distance else '无数据'}m），继续导航")
                return True
            time.sleep(0.3)
        self.get_logger().warn("前方障碍一直没走开，放弃本次导航")
        return False

    def _arrival_line(self, target: str) -> str:
        try:
            reply = self._qwen.one_shot(
                f"你刚把访客带到了{target}。"
                "用一句简短自然的口语告诉对方已经到了，并问现在还有什么事。不要列表。")
        except Exception as exc:  # noqa: BLE001 - 大模型不可用就用固定话术
            self.get_logger().warn(f"生成到达播报失败：{exc}")
            return f"我们到{target}了，现在还有什么事吗？"
        return reply or f"我们到{target}了，现在还有什么事吗？"

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    # ------------------------------------------------------------------ 播报
    def _speak(self, text: str) -> None:
        text = (text or "").strip()
        if not text:
            return
        if not self._tts.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("TTS 服务未就绪，跳过播报")
            return
        self.get_logger().info(f"播报：{text}")

        # 播报期间关麦克风，避免机器人听见自己的声音
        self._hold_mic(True)
        try:
            response = self._call_tts("append", text=text, msg_type="text")
            if response is None or not response.success:
                self.get_logger().warn(f"播报下发失败：{response}")
                return
            self._wait_playback(text)
        finally:
            self._hold_mic(False)

    def _call_tts(self, cmd: str, text: str = "", msg_type: str = ""):
        request = TtsService.Request()
        request.cmd = cmd
        request.type = msg_type
        request.text = text
        request.audio_id = self._audio_id
        request.extra = ""

        done = threading.Event()
        future = self._tts.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5.0):
            self.get_logger().warn(f"TTS {cmd} 调用超时")
            return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001 - 播报失败不打断对话
            self.get_logger().warn(f"TTS {cmd} 调用异常：{exc}")
            return None

    def _wait_playback(self, text: str) -> None:
        """轮询 TTS 状态，等这句话播完再放开麦克风。"""
        started = time.monotonic()
        est = max(1.5, len(text) * 0.22)
        limit = min(est * 2.0 + 5.0, 40.0)
        seen_playing = False
        idle_since = None
        while time.monotonic() - started < limit:
            response = self._call_tts("query")
            playing = bool(
                response is not None
                and response.success
                and response.status == "playing"
            )
            now = time.monotonic()
            if playing:
                seen_playing = True
                idle_since = None
            elif idle_since is None:
                idle_since = now
            elif seen_playing and now - idle_since >= 0.4:
                return
            elif not seen_playing and now - started >= est + 3.0:
                return
            time.sleep(0.2)

    def _hold_mic(self, muted: bool) -> None:
        if self._mic is not None:
            self._mic.hold(muted)

    def shutdown(self) -> None:
        self._closing = True
        self._queue.put(None)
        self._worker.join(timeout=2.0)
        if self._mic is not None:
            try:
                self._mic.stop()
            except Exception as exc:  # noqa: BLE001 - 退出时上下文可能已失效
                self.get_logger().warn(f"关闭麦克风失败（忽略）：{exc}")


def _meaningful(text: str, min_chars: int) -> bool:
    """过滤空串、纯标点、过短的误识别。"""
    return len(re.sub(r"\W", "", text or "")) >= max(1, min_chars)


_PLACE_HINTS = ("想", "要", "去", "打", "玩", "练", "参观", "逛", "看", "找")
_GO_WORDS = ("去", "到", "前往", "带我去", "领我去", "回", "走")
_YES_WORDS = ("需要", "好", "可以", "行", "带我去", "去吧", "麻烦", "要")
_NO_WORDS = ("不用", "不需要", "不了", "算了", "暂时不", "先不", "别")
_DONE_WORDS = ("没事", "没有了", "不用了", "没有别的", "没别的", "就这样", "结束", "回去")


def _plain(text: str) -> str:
    return re.sub(r"[\s，。！？、,.!?~～]", "", text or "")


def _means_yes(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _YES_WORDS)


def _explicit_go(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _GO_WORDS)


def _means_no(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _NO_WORDS)


def _wants_done(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _DONE_WORDS)


def _extract_json(text: str):
    match = re.search(r"\{.*\}", text or "", re.S)
    if not match:
        return None
    try:
        return json.loads(match.group(0))
    except json.JSONDecodeError:
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceChatNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 退出阶段再收到 Ctrl-C 时不再抛堆栈
        try:
            node.shutdown()
            executor.shutdown()
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
