# 小郑 · 实时只读监测

机器人端独立目录：`/home/nvidia/guide_robot/web_monitor`。监测进程与现有导览、导航、相机及底盘进程相互独立；不改动这些程序、配置或启动项。

## 启动

```bash
cd /home/nvidia/guide_robot/web_monitor
bash start_monitor.sh --port 8765
```

浏览器访问 `http://192.168.21.19:8765/`。只需机器人上已有的 Python、psutil、OpenCV、NumPy 和 ROS 2 Jazzy，不安装第三方服务。启动脚本仅加载 ROS 环境并运行监测后端；不会运行导览 launch。

本次以后台监测进程运行；PID 文件 `/tmp/guide-web-readonly.pid`、诊断日志 `/tmp/guide-web-readonly.log`。当前未添加开机启动项，主机重启后可用上面的命令启动。

## 实际数据

- CPU、CPU 温度、内存、开机时长：读取主机，约 2 秒刷新。
- ROS 节点、话题与 TTS 服务是否可发现：约 5 秒刷新，不调用服务。
- 电源：优先每 5 秒只读查询底盘 `GET http://192.168.11.10:9090/api/core/system/v1/battery/pack`，分别读取电量与明确的 `isCharging` 标志，不根据电流正负推断充电状态。无有效底盘数据时再尝试订阅 `/power/battery/status` 和 `/power/board/status`。未安装电池、NaN 或电压为零的空数据不显示为有效电量。
- 彩色／深度图：只订阅已存在的头部相机话题，网页预览限至约 3 帧/秒、宽度最多 768 像素，不录像、不落盘。
- 前方距离：深度图中部 ROI 的有效深度第 5 百分位。用于观察，不是防撞控制或安全保证。
- 麦克风：只记录现有音频消息的接收时间，不保存或发送音频内容，不唤醒设备。
- 日志：监测连接事件和指定 ROS 模块的警告；内存最多保留 200 条，敏感字段遮盖。不是全量机器人日志。

传感器、主机和 ROS 图分别显示新鲜度；断开时不继续显示为在线。地点页只展示配置，不选择或发送导航目标。节点可发现不等于物理硬件健康，也不代表已探测模型接口。

## API 与边界

只提供 `GET /api/status`、`GET /api/logs`、`GET /api/frame?kind=color|depth` 及页面静态资源。所有写入 HTTP 方法返回 405；其他路径返回 404，不提供任意文件读取或命令执行。后端不创建机器人控制发布者、不调用服务或 Action。

## 校验

```bash
python3 -m unittest test_monitor -v
```

测试不启动 ROS 或机器人服务，检查写接口拒绝、私有文件拒绝、旧图像不可访问、凭据遮盖和数据序列化。
