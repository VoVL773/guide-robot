"""启动即前进：ros2 launch learn_e2c move_forward.launch.py

可用参数：
  duration_s    前进时长（秒，默认 2.0）
  speed_mps     限速（m/s，默认 0.3）
  check_health  有阻断性故障时是否拒绝运动（默认 true）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("duration_s", default_value="2.0"),
        DeclareLaunchArgument("speed_mps", default_value="0.3"),
        DeclareLaunchArgument("check_health", default_value="true"),
        Node(
            package="learn_e2c",
            executable="move_forward",
            name="move_forward",
            output="screen",
            parameters=[{
                "duration_s": ParameterValue(
                    LaunchConfiguration("duration_s"), value_type=float),
                "speed_mps": ParameterValue(
                    LaunchConfiguration("speed_mps"), value_type=float),
                "check_health": ParameterValue(
                    LaunchConfiguration("check_health"), value_type=bool),
            }],
        ),
    ])
