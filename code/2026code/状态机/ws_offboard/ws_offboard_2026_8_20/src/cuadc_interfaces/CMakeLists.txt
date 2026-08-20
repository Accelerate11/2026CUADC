cmake_minimum_required(VERSION 3.8)
project(cuadc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReleasePayload.action"
  "msg/MissionStatus.msg"
  "msg/PayloadStatus.msg"
  "msg/SafetyStatus.msg"
  "srv/PayloadCommand.srv"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
