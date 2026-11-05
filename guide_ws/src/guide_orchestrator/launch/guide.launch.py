"""一键启动导览：导航 + 头部 + 语音 + 编排。

用法：
    ros2 launch guide_orchestrator guide.launch.py
    ros2 launch guide_orchestrator guide.launch.py dry_run:=false
    ros2 launch guide_orchestrator guide.launch.py auto_start:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("guide_orchestrator")
    points_config = os.path.join(share, "config", "guide_points.yaml")
    script_config = os.path.join(share, "config", "guide_script.yaml")

    dry_run = LaunchConfiguration("dry_run")
    auto_start = LaunchConfiguration("auto_start")
    points = LaunchConfiguration("points_config")
    script = LaunchConfiguration("script_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="true=不连底盘/不下发头部指令/不调 TTS，仅联调流程"),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="启动后是否自动开始导览"),
        DeclareLaunchArgument(
            "points_config", default_value=points_config,
            description="点位配置 YAML（默认使用安装目录里的 guide_points.yaml）"),
        DeclareLaunchArgument(
            "script_config", default_value=script_config,
            description="讲解词配置 YAML（默认使用安装目录里的 guide_script.yaml）"),

        Node(
            package="guide_nav",
            executable="navigate_to_server",
            name="guide_navigate_to",
            output="screen",
            parameters=[{
                "dry_run": dry_run,
                "points_config": points,
            }],
        ),
        Node(
            package="guide_head",
            executable="head_server",
            name="guide_head",
            output="screen",
            parameters=[{"dry_run": dry_run}],
        ),
        Node(
            package="guide_voice",
            executable="speak_action_server",
            name="guide_speak",
            output="screen",
            parameters=[{"dry_run": dry_run}],
        ),
        Node(
            package="guide_orchestrator",
            executable="orchestrator",
            name="guide_orchestrator",
            output="screen",
            parameters=[{
                "points_config": points,
                "script_config": script,
                "auto_start": auto_start,
            }],
        ),
    ])
