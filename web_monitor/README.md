# 小郑 · 导览工作台

Web 工作台独立采集主机、ROS、相机与底盘状态，并提供需确认的语音节点启停、地点导航和文本指令入口。原机器人部署目录为 `/home/nvidia/guide_robot/web_monitor`；它通过 ROS 文本话题请求导览，不直接调用底盘运动 SDK。

返回 [项目首页](../README.md) · [部署与使用](../docs/部署与使用.md)

界面截图见 [Web 界面预览](../README.md#web-界面预览)，包含总览、设备状态、节点状态和地图配置四个视图。

## 启动

```bash
cd /home/nvidia/guide_robot/web_monitor
bash start_monitor.sh --port 8765
```

浏览器访问 `http://机器人IP:8765/`。依赖 Python、psutil、PyYAML、OpenCV、NumPy 和 ROS 2 Jazzy，以及运行设备话题所需的平台消息定义。

默认监听 `0.0.0.0`，没有登录鉴权或 HTTPS，只适合受控局域网。请求体中的 `confirm` 是操作确认标记，不是身份认证。仅本机访问时可使用 `bash start_monitor.sh --bind 127.0.0.1 --port 8765`。

## 迁移到其他机器

当前实现保留了现场部署配置，克隆后需核对：

- `start_monitor.sh`：平台 `/home/nvidia/xos/setup.bash` 和导览工作区安装路径。
- `monitor.py`：`PLACES_FILE`、`PLACES_FALLBACK`、底盘 REST 地址及地图相关路径。
- 工作台不会自动读取语音节点的 `places_file` 参数，需确保两者加载同一份点位配置。
- 节点启动参数 `VOICE_START_ARGS` 使用语音节点的其他默认值；通过网页启动前确认模型地址、SDK、地图和定位配置可用。

在已手动加载 ROS、平台及导览环境的终端中，也可以从本目录直接运行 `python3 monitor.py --bind 127.0.0.1 --port 8765`；后端的固定配置仍需适配。

## 页面

- 总览：摄像头画面、运行示意图、当前导航任务、启动节点、快捷指令下发、日志。
- 设备状态：CPU / 温度 / 内存 / 电量、ROS 图、相机、麦克风、底盘、TTS。
- 节点状态：列出 ROS 图中的全部节点，显示运行/已退出、命名空间、在线时长与最近在线时间，支持按名称过滤；节点集合每 5 秒刷新，退出节点保留 15 分钟。
- 地图配置：运行示意图 + 实时位姿；地点列表来自 `places.yaml`，只展示不修改。

## 运行示意图

- 页面用 SVG 画示意主线，站点按 `places.yaml` 的顺序排列，不要求与实际几何一致。
- 机器人位置取底盘 `GET /api/core/navigation/v1/pose` 的 `x/y`，投影到主线上得到沿线进度，绿色胶囊标记大致位置，超过 5 秒无数据会变暗。
- 站点坐标由 `GET /api/core/navigation/v1/poi` 读取，和语音节点使用同一套点位；改点位后无需改页面。
- 任务运行时浮层显示进度百分比与「前往 某站点」；待命时显示最近站点。示意图同时出现在总览摄像头下方和地图配置页。

## 控制边界

开放以下四个需要显式确认的接口，请求体必须带 `{"confirm": true}`：

- `POST /api/node/start`：以 `startup_goto_standby:=false` 启动 `guide_voice` 语音节点，启动后不会自行移动。
- `POST /api/node/stop`：只停止由本页启动的节点；外部启动的节点需在终端停止。
- `POST /api/navigate`：仅接受 `places.yaml` 中的地点，向 `/guide_voice/chat` 发送「去某地点」，由现有语音节点执行导航。
- `POST /api/command`：把输入框里的任意文本（≤200 字）原样发到 `/guide_voice/chat`，用于问时间、闲聊等非导航指令。

其余写方法（PUT / PATCH / DELETE）返回 405，未知路径返回 404，不提供任意文件读取或命令执行。页面不会直接调用底盘 SDK。

## 实际数据

- CPU、CPU 温度、内存、开机时长：读取主机，约 2 秒刷新。
- ROS 节点、话题与 TTS 服务是否可发现：约 5 秒刷新，不调用服务。
- 当前任务：解析 `guide_voice` 的导航日志（开始、路线、到达、失败），不额外发布状态话题。
- 电源：每 5 秒只读查询底盘 `GET http://192.168.11.10:9090/api/core/system/v1/battery/pack`，读取电量与明确的 `isCharging` 标志，不根据电流正负推断充电状态。
- 彩色／深度图：只订阅已存在的头部相机话题，网页约 3 帧/秒、宽度最多 768 像素，不录像、不落盘。
- 麦克风：只记录现有音频消息的接收时间，不保存或发送音频内容。

传感器、主机和 ROS 图分别显示新鲜度；断开时不继续显示为在线。

## 校验

```bash
python3 -m unittest test_monitor -v
```

测试不启动 ROS 或机器人服务，检查确认标记、地点白名单、节点未运行时拒绝导航、写方法拒绝、私有文件拒绝、旧图像不可访问与凭据遮盖。

2026-11-19 本次复跑：13 项全部通过；未启动真实 ROS 或底盘服务，未进行实车验证。
