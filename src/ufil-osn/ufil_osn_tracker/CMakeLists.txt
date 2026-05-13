cmake_minimum_required(VERSION 3.8)
project(ufil_osn_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

# Ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_ros REQUIRED)
find_package(ufil_object_tracking REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL COMPONENTS
  common
  geometry
  filters
  search
  sample_consensus
  kdtree
  octree
  features
  segmentation
  REQUIRED
)

add_library(ufil_osn_tracker
  src/ufil_osn_tracker.cpp
  src/detection.cpp
  src/detector.cpp
  src/classifier.cpp
  src/orientation_checker.cpp
  src/existence_estimator.cpp
  src/existence_updater_bayesian.cpp
  src/existence_predictor_bayesian.cpp
  src/occlusion_predictor.cpp
  src/shape_fitter.cpp
  src/clustering.cpp
)

target_compile_features(ufil_osn_tracker PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(ufil_osn_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ufil_osn_tracker PRIVATE "UFIL_OSN_TRACKER_BUILDING_LIBRARY")
target_link_libraries(ufil_osn_tracker
  Eigen3::Eigen
  pcl_common
  pcl_search
  pcl_filters
  pcl_segmentation
  Boost::boost
  opencv_core
  opencv_highgui
  opencv_imgproc
)
ament_target_dependencies(ufil_osn_tracker ufil_object_tracking ufil_msgs nav_msgs rclcpp Eigen3)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ufil_osn_tracker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# add_executable(ufil_osn_tracker_node src/old/ufil_osn_tracker_node.cpp)
# target_include_directories(ufil_osn_tracker_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(ufil_osn_tracker_node ufil_osn_tracker)
# ament_target_dependencies(ufil_osn_tracker_node rclcpp ufil_msgs
#   visualization_msgs
#   sensor_msgs
#   pcl_conversions
#   nav_msgs
#   ufil_object_tracking
# )

# install(TARGETS ufil_osn_tracker_node
#   DESTINATION lib/${PROJECT_NAME})

add_executable(ufil_osn_tracker_node src/ufil_osn_tracker_node.cpp)
target_include_directories(ufil_osn_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ufil_osn_tracker_node ufil_osn_tracker)
ament_target_dependencies(ufil_osn_tracker_node
  "rclcpp"
  "ufil_msgs"
  "ufil_ros"
  "ufil_object_tracking"
  "visualization_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "nav_msgs"
  "ament_index_cpp"
)

install(TARGETS ufil_osn_tracker_node DESTINATION lib/${PROJECT_NAME})

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
  ufil_osn_tracker
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
