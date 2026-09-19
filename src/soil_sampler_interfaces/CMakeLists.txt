cmake_minimum_required(VERSION 3.8)

project(soil_sampler_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TakeSoilSample.action"
  "action/RunExperiment.action"
  # "action/MoveRelative.action"
  "msg/ActuatorCommand.msg"
  "msg/ActuatorState.msg"
  "msg/Sen0658Data.msg"
  "msg/Teros12Data.msg"
  "msg/SoilSample.msg"
  "msg/ExperimentSample.msg"
  # "srv/SetSafeguards.srv"
)

ament_package()