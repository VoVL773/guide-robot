cmake_minimum_required(VERSION 3.8)
project(guide_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuideStatus.msg"
  "srv/SetHeadPose.srv"
  "srv/GuideCommand.srv"
  "action/NavigateTo.action"
  "action/Speak.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
