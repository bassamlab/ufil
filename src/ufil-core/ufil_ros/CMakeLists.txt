cmake_minimum_required(VERSION 3.8)
project(ufil_ros)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(ufil_msgs REQUIRED)
find_package(ufil_object_tracking REQUIRED)

add_library(ufil_ros
  src/ufil_ros.cpp
  src/json_ros.cpp
)
add_library(ufil_ros::ufil_ros ALIAS ufil_ros)
target_compile_features(ufil_ros PUBLIC c_std_99 cxx_std_20) # Require C99 and C++17
target_include_directories(ufil_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(ufil_ros
  "rclcpp"
  "ufil_msgs"
  "ufil_object_tracking"
  "tf2"
  "tf2_eigen"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "nav_msgs"
)

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

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensor_fov test/test_sensor_fov.cpp)
  target_link_libraries(test_sensor_fov ufil_ros)

endif()

# Export the target
ament_export_include_directories("include/ufil_ros")
ament_export_dependencies(tf2 tf2_geometry_msgs Boost tf2_ros tf2_eigen ufil_object_tracking nav_msgs)
ament_export_libraries(ufil_ros)
ament_export_targets("export_ufil_ros")

# Declare the ament package
ament_package()
