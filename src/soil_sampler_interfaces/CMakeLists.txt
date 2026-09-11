cmake_minimum_required(VERSION 3.8)

project(soil_sampler_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TakeSoilSample.action"
  "action/MoveRelative.action"
  "msg/ActuatorCommand.msg"
  "msg/ActuatorState.msg"
  "srv/SetSafeguards.srv"
)

ament_package()