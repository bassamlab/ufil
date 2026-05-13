cmake_minimum_required(VERSION 3.8)
project(ufil_pcl_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ufil_object_tracking REQUIRED)
find_package(ufil_ros REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL COMPONENTS common geometry filters search sample_consensus kdtree octree features segmentation REQUIRED)

add_library(ufil_pcl_preprocessor
  src/ufil_pcl_preprocessor.cpp
  src/depth_buffer.cpp
  src/fov_estimator.cpp
)
target_compile_features(ufil_pcl_preprocessor PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(ufil_pcl_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ufil_pcl_preprocessor PRIVATE "ufil_pcl_preprocessor_BUILDING_LIBRARY")
target_link_libraries(ufil_pcl_preprocessor Eigen3::Eigen pcl_common pcl_filters Boost::boost)
ament_target_dependencies(ufil_pcl_preprocessor ufil_object_tracking ufil_msgs nav_msgs rclcpp Eigen3)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ufil_pcl_preprocessor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(ufil_pcl_preprocessor_node component/ufil_pcl_preprocessor_node.cpp)
target_include_directories(ufil_pcl_preprocessor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ufil_pcl_preprocessor_node
  ufil_pcl_preprocessor
  pcl_common
  pcl_geometry
  pcl_filters
  pcl_search
  pcl_sample_consensus
  pcl_kdtree pcl_octree
  pcl_features
  pcl_segmentation
  Eigen3::Eigen
)
ament_target_dependencies(ufil_pcl_preprocessor_node
  "rclcpp"
  "ament_index_cpp"
  "sensor_msgs"
  "ufil_msgs"
  "nav_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "tf2_eigen"
  "ufil_object_tracking"
  "ufil_ros"
)

install(TARGETS ufil_pcl_preprocessor_node
  DESTINATION lib/${PROJECT_NAME})

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
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ufil_pcl_preprocessor
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
