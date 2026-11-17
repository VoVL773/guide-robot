"""语音对话：ros2 launch guide_voice voice_chat.launch.py

可在命令行覆盖的参数（示例）：
  qwen_model:=Qwen3.5-122B-A10B
  greet_text:="你好，我是小郑"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("qwen_base_url", default_value="http://120.48.75.178:7006/v1"),
        DeclareLaunchArgument("qwen_model", default_value="Qwen3.5-122B-A10B"),
        DeclareLaunchArgument("qwen_api_key", default_value="EMPTY"),
        DeclareLaunchArgument("greet_text", default_value=""),
        Node(
            package="guide_voice",
            executable="voice_chat",
            name="guide_voice_chat",
            output="screen",
            parameters=[{
                "qwen_base_url": LaunchConfiguration("qwen_base_url"),
                "qwen_model": LaunchConfiguration("qwen_model"),
                "qwen_api_key": LaunchConfiguration("qwen_api_key"),
                "greet_text": LaunchConfiguration("greet_text"),
            }],
        ),
    ])
