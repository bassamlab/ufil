cmake_minimum_required(VERSION 3.8)
project(ufil_ros VERSION 0.2.0)

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(ufil_msgs REQUIRED)
find_package(ufil_object_tracking REQUIRED)

add_library(ufil_ros
  src/ufil_ros.cpp
  src/json_ros.cpp
)
add_library(ufil_ros::ufil_ros ALIAS ufil_ros)
target_compile_features(ufil_ros PUBLIC cxx_std_20)
target_compile_options(ufil_ros PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -Wpedantic>)
# Only functions marked UFIL_ROS_PUBLIC are exported.
set_target_properties(ufil_ros PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_include_directories(ufil_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(ufil_ros PUBLIC
  rclcpp::rclcpp
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${ufil_msgs_TARGETS}
  ufil_object_tracking::ufil_object_tracking
)
# pcl_conversions is header-only, provides no CMake target and is only used in the
# implementation. Linking it via ament_target_dependencies would export all PCL and VTK
# libraries to every consumer.
target_include_directories(ufil_ros SYSTEM PRIVATE ${pcl_conversions_INCLUDE_DIRS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ufil_ros PRIVATE "UFIL_ROS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ufil_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_sensor_fov test_json_ros)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ufil_ros)
  endforeach()
endif()

# Export the target
ament_export_targets("export_ufil_ros" HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  ufil_msgs
  ufil_object_tracking
)

# Declare the ament package
ament_package()
