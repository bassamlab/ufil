cmake_minimum_required(VERSION 3.8)
project(ufil_pcl_preprocessor VERSION 0.2.0)

# Honour <Package>_ROOT variables as set by PCL's find modules.
cmake_policy(SET CMP0074 NEW)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

find_package(ament_cmake_ros REQUIRED)

# ROS 2 component
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Core library (no ROS 2 dependencies)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search)
find_package(ufil_object_tracking REQUIRED)

set(warning_options
  $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -Wpedantic>)

# --- Core library ------------------------------------------------------------------------

add_library(${PROJECT_NAME}
  src/clipped_area.cpp
  src/config.cpp
  src/depth_buffer.cpp
  src/fov_estimator.cpp
  src/ufil_pcl_preprocessor.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})
# Causes the visibility macros to use dllexport rather than dllimport.
target_compile_definitions(${PROJECT_NAME} PRIVATE UFIL_PCL_PREPROCESSOR_BUILDING_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pcl_common
  ufil_object_tracking::ufil_object_tracking
  PRIVATE
  pcl_filters
  pcl_kdtree
  pcl_search
)

# --- ROS 2 component ---------------------------------------------------------------------

add_library(${PROJECT_NAME}_component SHARED component/ufil_pcl_preprocessor_node.cpp)
target_compile_options(${PROJECT_NAME}_component PRIVATE ${warning_options})
# pcl_conversions is header-only and does not export a CMake target.
target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/component)
target_include_directories(${PROJECT_NAME}_component SYSTEM PUBLIC
  ${pcl_conversions_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ${PROJECT_NAME}
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${ufil_msgs_TARGETS}
  ufil_ros::ufil_ros
  ${visualization_msgs_TARGETS}
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_pcl_preprocessor::PclPreprocessorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# --- Install -----------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# --- Tests -------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(class_loader REQUIRED)

  # Core library tests, no ROS 2 required.
  foreach(test_name test_config test_clipped_area test_depth_buffer test_fov_estimator
      test_pcl_preprocessor)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME} pcl_common)
  endforeach()

  # ROS 2 adapter tests.
  foreach(test_name test_parameter_ros test_ros_conversions test_component)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/component)
    target_link_libraries(${test_name} ${PROJECT_NAME}_component)
  endforeach()
  target_link_libraries(test_component class_loader::class_loader)
  target_compile_definitions(test_component PRIVATE
    COMPONENT_LIBRARY="$<TARGET_FILE:${PROJECT_NAME}_component>"
    DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL ufil_object_tracking)

ament_package()
