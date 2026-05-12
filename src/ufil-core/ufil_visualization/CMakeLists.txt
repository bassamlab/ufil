cmake_minimum_required(VERSION 3.5)
project(ufil_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)

add_executable(object_list_marker_array_node
  src/object_list_marker_array_node.cpp
)
target_compile_features(object_list_marker_array_node PUBLIC cxx_std_20)
target_include_directories(object_list_marker_array_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(object_list_marker_array_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  ufil_msgs
)

add_executable(lanelet_map_marker_array_node
  src/lanelet_map_marker_array_node.cpp
)
target_compile_features(lanelet_map_marker_array_node PUBLIC cxx_std_20)
target_include_directories(lanelet_map_marker_array_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lanelet_map_marker_array_node
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_link_libraries(lanelet_map_marker_array_node
  Boost::boost
)

install(TARGETS
  object_list_marker_array_node
  lanelet_map_marker_array_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()