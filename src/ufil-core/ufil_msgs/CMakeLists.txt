cmake_minimum_required(VERSION 3.8)
project(ufil_msgs VERSION 0.2.0)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Axle.msg
  msg/Classification.msg
  msg/Detection.msg
  msg/DetectionList.msg
  msg/DetectionStamped.msg
  msg/Dimension.msg
  msg/DimensionWithCovariance.msg
  msg/Features.msg
  msg/LaneletMapOsm.msg
  msg/Object.msg
  msg/ObjectList.msg
  msg/ObjectStamped.msg
  msg/SensorFOV.msg
  msg/State.msg
  msg/StateWithCovariance.msg
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
