# guide\_robot —— 展厅语音导览机器人「小郑」

基于 ROS 2 Jazzy 的展厅导览机器人：听懂访客的话，把访客带到指定场馆，到点播报，听到“没事了”自动回待命点。

点位名称、别名、参观顺序全部由 [`places.yaml`](guide_ws/src/guide_voice/config/places.yaml) 驱动，代码里没有任何 `POI_xx` 常量或坐标；换场地只需要改 YAML 和地图点位。

> 早期 README 中的多包方案（`guide_interfaces / guide_core / guide_nav / guide_head / guide_orchestrator`）已废弃。
> 当前只构建一个包：`guide_voice`。完整设计与运维说明见 [交付文档](docs/交付文档.md)。

## 功能

- **语音输入**：平台 ASR（`/audio_asr/iat`）、本地麦克风（VAD + faster-whisper）、文本话题三种通道
- **地点理解**：别名最长匹配 + 意愿词判断；规则未命中时由千问做隐式意图推断，结果必须落在白名单内
- **逐点导航**：按主线物理顺序去程/回程逐点经过，不跳点；支线先回挂接点
- **三层保护**：底盘 DTC 健康 → 定位门控 → 头部深度前向防撞
- **到点服务**：播报到达话术，询问是否还需要服务；说“没事了”返回待命点
- **启动自检**：定位检查、必要时重定位、自动回待命点
- **监测平台**：本机 Web 工作台，查看相机、任务、位姿、主机与 ROS 状态

## 运行环境

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 24.04 / aarch64 |
| ROS | ROS 2 Jazzy |
| 底盘 | Knewbot，REST `192.168.11.10:9090` |
| 当前地图 | `消防` |
| 依赖 | `interaction_msgs`、`lyre_msgs`、`robot_controller`、faster-whisper、本地千问服务 |

## 目录

```text
guide_robot/
├── guide_ws/src/guide_voice/          # 唯一交付的 ROS 2 包
│   ├── guide_voice/
│   │   ├── voice_chat_node.py         # 对话与导航编排（唯一指挥者）
│   │   ├── mic_asr.py                 # 麦克风采集、VAD、Whisper 识别
│   │   ├── qwen.py                    # 千问 OpenAI 兼容客户端
│   │   ├── places.py                  # 地点别名、主线/支线解析
│   │   ├── navigation.py              # 定点导航与可靠性逻辑
│   │   └── front_guard.py             # 深度图前向防撞
│   ├── config/places.yaml             # 地图相关唯一配置
│   ├── launch/voice_chat.launch.py
│   └── test/test_navigation.py        # 47 项纯逻辑测试
├── web_monitor/                       # 独立监测平台
│   ├── monitor.py                     # HTTP 服务 + 数据采集
│   ├── index.html / live.js           # 三个视图，原生 HTML/JS
│   ├── start_monitor.sh
│   └── test_monitor.py                # 13 项接口与安全测试
├── docs/
│   ├── 交付文档.md                     # 推荐阅读：架构、实现、运维、验收
│   └── 导览机器人交付文档.md            # 早期版本，保留备查
└── learn/                             # 学习实验包，不参与交付链路
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash

cd /home/nvidia/guide_robot/guide_ws
colcon build --packages-select guide_voice
source install/setup.bash
```

> 在 IDE 内置终端构建时若报 `GLIBCXX_3.4.32 not found`，是注入的库路径覆盖了系统库，先执行：
>
> ```bash
> export LD_LIBRARY_PATH=$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' \
>   | grep -v trae-cn-server | paste -sd:)
> ```

修改 `places.yaml` 后重启节点即可；只有改动 `setup.py` 的 `data_files` 才需要重新 build。

## 运行

```bash
ros2 launch guide_voice voice_chat.launch.py
```

常用参数：

```bash
ros2 run guide_voice voice_chat --ros-args \
  -p enable_mic:=true \
  -p enable_nav:=true \
  -p max_moving_speed:=0.6 \
  -p nav_timeout:=180.0 \
  -p startup_goto_standby:=true \
  -p front_guard_stop:=1.2
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `enable_mic` | `true` | 关闭后只接受平台 ASR 与文本 |
| `enable_nav` | `true` | 关闭后只保留聊天 |
| `max_moving_speed` | `0.6` | 底盘限速（m/s），`0` 表示不改 |
| `nav_timeout` | `180.0` | 整条路线预算（秒） |
| `front_guard_stop` | `1.2` | 前向停车距离（m） |
| `startup_goto_standby` | `true` | 启动后先回待命点 |
| `auto_relocalize` | `true` | 启动时自动重定位 |
| `qwen_base_url` | `http://120.48.75.178:7006/v1` | 千问服务地址 |
| `qwen_model` | `Qwen3.5-122B-A10B` | 模型名 |
| `places_file` | 包内 `config/places.yaml` | 自定义地点配置 |

## 文本调试

不经过麦克风，直接验证意图与导航：

```bash
# 明确导航
ros2 topic pub --once /guide_voice/chat std_msgs/msg/String "{data: '带我去跆拳道馆'}"

# 回待命点
ros2 topic pub --once /guide_voice/chat std_msgs/msg/String "{data: '回乐知院'}"

# 到点后结束服务
ros2 topic pub --once /guide_voice/chat std_msgs/msg/String "{data: '没事了'}"

# 检查音频流与深度图
ros2 topic hz /lyre/audio_stream
ros2 topic hz /ob_camera_head/depth/image_raw
```

## 配置点位（换场馆必做）

1. 在底盘 / App 里把车摆到讲解位置并建点，记录点位名称；
2. 编辑 [`places.yaml`](guide_ws/src/guide_voice/config/places.yaml)：

```yaml
standby: 乐知院

aliases:            # 语音别名 -> 地图点位名，可多个别名指向同一点
  乐知院: 乐知教育
  核心跑酷: 核心力量跑酷
  厕所: 厕所
  卫生间: 厕所
  跆拳道馆: 跆拳道馆

main_line:          # 必须按物理顺序排列
  - 乐知教育
  - 核心力量跑酷
  - 创想谷
  - 厕所
  - 跆拳道馆

branches: {}        # 支线：点位 -> 挂接的主线点位
transfer_points: [] # 只中转、不作为目的地
```

3. 重启节点，检查启动日志中的“可去地点 / 主线点”是否符合预期；
4. 先短距离单段试跑，再跑完整往返。

规则：

- 点位坐标**不在本工程维护**，以底盘 POI 服务端为唯一真源；
- 别名目标不存在于当前地图时会被静默忽略，不会崩溃；
- 主线里不存在的点位会被过滤；主线为空时直接前往目标；
- 换现场点位名后，同步更新 ASR 热词可提升识别率。

## 监测平台

独立于导览进程，不修改导览、导航、相机或底盘程序，也不参与控制决策。

```bash
cd /home/nvidia/guide_robot/web_monitor
bash start_monitor.sh --port 8765

# 浏览器访问
# http://192.168.21.19:8765/     （默认监听 0.0.0.0，仅限局域网）
```

三个视图：

| 视图 | 内容 |
|---|---|
| 总览 | 头部相机彩色/深度画面、当前任务与路线、运行示意图、启停节点、快捷导航、文本指令、事件流 |
| 设备状态 | CPU/温度/内存/运行时长、底盘电量、ROS 节点与话题、相机/麦克风/底盘/TTS 状态、趋势图 |
| 地图配置 | 全尺寸运行示意图、实时位姿与置信度、`places.yaml` 目的地清单 |

接口边界：

- 只读：`GET /api/status`、`/api/logs`、`/api/frame`
- 需确认（请求体含 `{"confirm": true}`）：
  - `POST /api/node/start` / `/api/node/stop`
  - `POST /api/navigate`（只接受 `places.yaml` 中的地点）
  - `POST /api/command`（≤200 字，发到 `/guide_voice/chat`）
- 页面不直接调用底盘 SDK，也不提供任意命令执行或文件读取

注意：

- 默认无登录鉴权、无 HTTPS，必须部署在受控局域网；
- 任务状态由 `guide_voice` 的导航日志解析，改动日志文案需同步检查正则；
- 示意图只表示沿线进度，不表示真实几何；
- 后台运行请自行重定向：`nohup bash start_monitor.sh --port 8765 >/tmp/guide-web.log 2>&1 &`

完整说明见 [交付文档 4.6 节](docs/交付文档.md)。

## 控制方式

当前版本没有单独的控制服务。外部控制统一走 `/guide_voice/chat`，语音则走平台 ASR 或本地麦克风通道：

- 语音直接说话；
- `ros2 topic pub` 发文本；
- 监测平台的快捷导航与文本指令；
- `Ctrl+C` 停止节点。

导航过程中新的语音会被丢弃，避免半路改目标。

## 可靠性设计

- **大模型只做语言理解**：不输出坐标、不输出动作；地点必须通过 `places.yaml` 白名单校验；
- **三道门控**：DTC 健康（A/B 阻断、C 告警）→ 定位连续稳定 3 秒 → 健康复查；
- **到点校验**：位姿置信度有效且偏差 ≤ 0.35 m；定位不稳时 1.0 m 内降级放行；
- **失败不盲目重发**：通信超时视为状态不确定，先取消并确认，才允许下一单；
- **取消未确认时保留 AEB**，不假设底盘已经停下；
- **前向防撞**：中央 ROI p5 距离 < 1.2 m 或无效像素 ≥ 6%，连续 3 次确认才停车；
- **播报互锁**：TTS 播放期间闭麦，防止机器人听见自己。

## 测试

```bash
# 导航与路线逻辑（47 项，使用假底盘，不会动车）
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash
cd guide_ws/src/guide_voice
python3 -m unittest discover -s test -v

# 监测平台（13 项）
cd /home/nvidia/guide_robot/web_monitor
python3 -m unittest test_monitor -v
```

## 常见问题

| 现象 | 排查 |
|---|---|
| 启动后不动 | 看启动日志的定位状态、待命点与“可去地点” |
| 提示“定位服务没有启动” | 底盘定位服务 / 地图异常；状态 4 会尝试重载地图 |
| 提示“维护模式还开着” | 弹起维护按钮，退出手动模式 |
| 一直提示前方有障碍 | 深度相机被遮挡或话题无数据；无数据时防撞不生效 |
| 听不到识别结果 | 检查 `/lyre/audio_control` 与 `/lyre/audio_stream` |
| 机器人识别到自己的播报 | TTS `audio_id` 需具备 stop 权限（当前 `audio_guide`） |
| 下发导航后无响应 | 检查 `192.168.11.10:9090`；不要重复下发 |
| 看板打不开 | 端口占用或进程退出；改为前台运行查看报错 |

更多排障命令与验收指标见 [交付文档第 7、8 章](docs/交付文档.md)。

## 相关文档

- [交付文档](docs/交付文档.md)：业务闭环、能力边界、语音/大模型/导航实现、接口、部署运维、测试验收
- [监测平台说明](web_monitor/README.md)
- [早期交付文档](docs/导览机器人交付文档.md)：多包 Action 方案，仅作历史参考
