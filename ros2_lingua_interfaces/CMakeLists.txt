cmake_minimum_required(VERSION 3.8)
project(ros2_lingua_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RegisterCapability.srv"
  "srv/GroundInstruction.srv"
  "srv/UpdateState.srv"
  "srv/LinguaService.srv"
  "msg/ExecutionStatus.msg"
  "action/LinguaAction.action"
)

ament_package()
