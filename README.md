# guide_robot —— 机器人导览（介绍）任务

与消防巡检共用同一张底盘地图、同一套底盘 POI 服务端和轨道，但**点位不同**：
点位名称、参观顺序、讲解词全部由 YAML 驱动，代码里没有任何 `POI_xx` 常量。
用户在地图上标记好 POI 后，只改两个 YAML 即可运行。

## 目录

```
guide_ws/src/
├── guide_interfaces/      NavigateTo / Speak Action，SetHeadPose / GuideCommand 服务
├── guide_core/            YAML 加载与校验（RobotForge reusable_model.io.yaml_config）
├── guide_nav/             导航服务端（底盘 REST，AEB/限速/定位/到点容差）
├── guide_head/            头部服务端（/head/cmd，回中与朝向）
├── guide_voice/           语音服务端（Speak Action -> 平台 TTS）
└── guide_orchestrator/    编排（唯一指挥者）+ 配置 + 一键 launch
    ├── config/guide_points.yaml   点位顺序、轨道、限速、停留、容差
    └── config/guide_script.yaml   每个点位的讲解词与头部朝向
```

## 当前进度

- ✅ 第一步：起点 `POI_START` → 沿 `track_line_1000` 依次到 `POI_01`~`POI_05`，
  到点播报「到达点位N」，结束停在最后一个点位（地图只有这一条轨道，没有返程）
- ⏳ 下一步：头部回中/朝向（`guide_head`，launch 加 `enable_head:=true`）、正式讲解词

点位与轨道名称来自地图文件 `20261105_080739.bag`，已填入 `guide_points.yaml`。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash          # interaction_msgs / ros2_bridge_msgs
cd /home/nvidia/guide_robot/guide_ws
colcon build --symlink-install
source install/setup.bash
```

依赖说明：
- `robot_controller` 是普通 Python 包（未被 colcon 安装），导航节点通过
  `sys.path` 注入 `/home/nvidia/robot_controller/src`（可用 `ROBOT_CONTROLLER_SRC` 覆盖）。
- RobotForge 通过 `ROBOTFORGE_SRC` 注入，默认
  `/home/nvidia/greeting-main/packages/reusable_model/src`。

> 在 Trae/IDE 内置终端构建时，若 cmake 报 `GLIBCXX_3.4.32 not found`，是 IDE 注入的
> 库路径覆盖了系统库，构建前先清掉：
> `export LD_LIBRARY_PATH=$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v trae-cn-server | paste -sd:)`

## 标记点位（必做）

1. 在底盘/App 里把车摆到讲解位置，建点，记录 POI 名称；
2. 编辑 `src/guide_orchestrator/config/guide_points.yaml`：
   - `start_poi`：起点名称；
   - `poi_order`：按参观顺序排列的讲解点名称；
   - `tracks.outbound`：串联这些点的轨道名；`tracks.return`：返程轨道（可留空）；
3. 在 `src/guide_script.yaml` 里按同名点位填写 `text`（及可选的 `head_yaw` / `head_pitch`）；
4. 重新 `colcon build`（或直接改 `install/.../share/.../config` 下的文件后重启节点）。

启动时导航节点会做只读校验：点位/轨道不存在、`poi_order` 重复、`start_poi`
出现在顺序里等，都会打印明确原因并拒绝所有 goal，**不会崩溃**。

## 先空跑（默认 dry_run）

```bash
ros2 launch guide_orchestrator guide.launch.py                    # dry_run:=true
```

dry_run 行为：不连底盘、不下发运动、不写限速、不发头部指令、不调 TTS，
导航按固定耗时模拟到点，语音只打印文本。用于验证顺序、讲解词与状态机。

观察状态：

```bash
ros2 topic echo /guide/status
```

## 实机运行

```bash
ros2 launch guide_orchestrator guide.launch.py dry_run:=false
```

实机时导航节点会：
- 校验定位；定位异常且 `relocalize.enabled=true` 时按 `start_poi` 位姿自动重定位；
- 开启 AEB，按 `speed` 设置限速，退出时恢复原限速；
- 到点后校验实际位姿与 POI 坐标偏差（`arrival_tolerance_m`）。

## 运行中控制

```bash
# start / pause / resume / stop / goto
ros2 service call /guide/command guide_interfaces/srv/GuideCommand \
  "{command: 'pause'}"
ros2 service call /guide/command guide_interfaces/srv/GuideCommand \
  "{command: 'goto', poi_name: 'POI_06'}"

# 单独调试某一环
ros2 action send_goal /guide/navigate_to guide_interfaces/action/NavigateTo \
  "{poi_name: 'POI_05', use_track: false, track_id: ''}"
ros2 action send_goal /guide/speak guide_interfaces/action/Speak \
  "{text: '测试播报', interrupt: true}"
```

命令语义：
- `pause` / `resume` 在**步骤边界**生效（当前导航/播报会走完）；
- `stop` 立即取消当前目标并终止导览；
- `goto` 只能从 idle 状态启动，从指定点位开始并继续后续点位（不会先回起点）。

## 实机注意

- **TTS**：`audio_id` 必须是 `audio_config.json` 里开启 stop 权限的文本实体
  （当前 `audio_guide`）；否则 cancel 无法真正打断音频。
- **头部**：`pitch` 正值为低头，抬头填负值；限位 ±26°/±25°/±90°，
  节点内部会再次 clamp。关节 1（roll）固定为 0。
- **POI 坐标**：不在本工程维护；以底盘 POI 服务端为唯一真源，避免两份坐标漂移。
- **语音与头部**的服务端都各自做并发互斥，编排层是唯一调用方，不要绕过编排
  直接并发调用，否则会出现两句讲解叠播。

## 测试

```bash
cd src/guide_core
PYTHONPATH=. python3 -m pytest test -q
```

覆盖：空配置、正常配置、重复点位、起点混入顺序、未知配置项、讲解词默认值、
角度归一化。
