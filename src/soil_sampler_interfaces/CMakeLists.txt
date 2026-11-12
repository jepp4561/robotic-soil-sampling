cmake_minimum_required(VERSION 3.8)

project(soil_sampler_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToRestDepth.action"
  "action/MeasureForceProfile.action"
  "action/MoveHorizontal.action"
  # "action/MoveRelative.action"
  "action/RunExperiment.action"
  "action/TakeSoilSample.action"
  
  "msg/ActuatorCommand.msg"
  "msg/ActuatorState.msg"
  "msg/ExperimentSample.msg"
  "msg/ForceProfileSample.msg"
  "msg/Sen0658Data.msg"
  "msg/SkippedLocation.msg"
  "msg/SoilSample.msg"
  "msg/Teros12Data.msg"

  # "srv/SetSafeguards.srv"
  "srv/GetRestDepth.srv"
  DEPENDENCIES std_msgs
)

ament_package()