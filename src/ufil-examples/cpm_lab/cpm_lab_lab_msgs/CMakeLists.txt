cmake_minimum_required(VERSION 3.8)
project(cpm_lab_lab_msgs VERSION 0.2.0)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The definitions mirror the interfaces of the CPM Lab and must not be changed.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PathPoint.msg
  msg/SystemTrigger.msg
  msg/VehicleCommand.msg
  msg/VehicleObservation.msg
  msg/VehicleState.msg
  msg/VehicleStateList.msg
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
