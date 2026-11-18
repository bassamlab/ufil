cmake_minimum_required(VERSION 3.8)
project(cpm_lab_map_msgs VERSION 0.2.0)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# The definitions mirror the interfaces of the CPM Lab and must not be changed.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LaneControlLight.msg
  msg/LaneControlLightSegment.msg
  msg/Lanelet2MapBin.msg
  srv/GetLanelet2Map.srv
  DEPENDENCIES
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
