"""遥控器按键绑定：ros2 launch learn_e2c remote_trigger.launch.py

默认把未占用的 g 右拨（axes[5] == -1.0）绑定为「前进 2 秒」。
拨杆需先回到中位，再次右拨才会再次触发。
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
        DeclareLaunchArgument("switch_axis", default_value="5"),
        DeclareLaunchArgument("trigger_value", default_value="-1.0"),
        Node(
            package="learn_e2c",
            executable="remote_trigger",
            name="remote_trigger",
            output="screen",
            parameters=[{
                "duration_s": ParameterValue(
                    LaunchConfiguration("duration_s"), value_type=float),
                "speed_mps": ParameterValue(
                    LaunchConfiguration("speed_mps"), value_type=float),
                "switch_axis": ParameterValue(
                    LaunchConfiguration("switch_axis"), value_type=int),
                "trigger_value": ParameterValue(
                    LaunchConfiguration("trigger_value"), value_type=float),
            }],
        ),
    ])
