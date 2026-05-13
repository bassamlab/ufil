cmake_minimum_required(VERSION 3.8)
project(ufil_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add message files
rosidl_generate_interfaces(ufil_msgs
  msg/Axle.msg
  msg/Dimension.msg
  msg/DimensionWithCovariance.msg
  msg/Features.msg
  msg/Classification.msg
  msg/Object.msg
  msg/ObjectList.msg
  msg/ObjectStamped.msg
  msg/Detection.msg
  msg/DetectionList.msg
  msg/DetectionStamped.msg
  msg/State.msg
  msg/StateWithCovariance.msg
  msg/SensorFOV.msg
  msg/LaneletMapOsm.msg
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
