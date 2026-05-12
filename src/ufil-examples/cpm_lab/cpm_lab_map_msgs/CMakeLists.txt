cmake_minimum_required(VERSION 3.8)
project(cpm_lab_map_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Lanelet2MapBin.msg
  msg/LaneControlLightSegment.msg
  msg/LaneControlLight.msg
  srv/GetLanelet2Map.srv
  DEPENDENCIES
  builtin_interfaces
  std_msgs
)
ament_package()
