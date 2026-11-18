cmake_minimum_required(VERSION 3.8)
project(ufil_examples_cpm_lab_adapter VERSION 0.2.0)

find_package(ament_cmake_ros REQUIRED)

# Core library (no ROS 2 dependencies)
find_package(Eigen3 REQUIRED NO_MODULE)
# GeographicLib ships a find module instead of a CMake package configuration.
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# ROS 2 components
find_package(builtin_interfaces REQUIRED)
find_package(cpm_lab_lab_msgs REQUIRED)
find_package(cpm_lab_map_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_object_tracking REQUIRED)
find_package(ufil_osc REQUIRED)
find_package(ufil_ros REQUIRED)

set(warning_options
  $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -Wpedantic>)

# --- Core library ------------------------------------------------------------------------

add_library(${PROJECT_NAME}
  src/artificial_gnss.cpp
  src/lane_control_light.cpp
  src/sensor_simulator.cpp
  src/vehicle_object.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})
# Causes the visibility macros to use dllexport rather than dllimport.
target_compile_definitions(${PROJECT_NAME} PRIVATE UFIL_EXAMPLES_CPM_LAB_ADAPTER_BUILDING_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
# GeographicLib is only used in the implementation.
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${GeographicLib_LIBRARIES})

# --- ROS 2 components --------------------------------------------------------------------

add_library(${PROJECT_NAME}_component SHARED
  component/cpm_lab_adapter_node.cpp
  component/cpm_lab_sensor_node.cpp
  component/osc_lane_control_light_node.cpp
)
target_compile_options(${PROJECT_NAME}_component PRIVATE ${warning_options})
target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/component)
target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ${PROJECT_NAME}
  ${builtin_interfaces_TARGETS}
  ${cpm_lab_lab_msgs_TARGETS}
  ${cpm_lab_map_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${ufil_msgs_TARGETS}
  ufil_object_tracking::ufil_object_tracking
  ufil_osc::ufil_osc
  ufil_ros::ufil_ros
)
# The executable names of the former stand-alone nodes are kept.
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_examples_cpm_lab_adapter::CpmLabAdapterNode"
  EXECUTABLE ufil_examples_cpm_lab_adapter_node
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_examples_cpm_lab_adapter::CpmLabSensorNode"
  EXECUTABLE ufil_examples_cpm_lab_adapter_sensor_node
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_examples_cpm_lab_adapter::OscLaneControlLightNode"
  EXECUTABLE ufil_examples_osc_message_converter_node
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
  foreach(test_name
      test_artificial_gnss test_lane_control_light test_sensor_simulator test_vehicle_object)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_options(${test_name} PRIVATE ${warning_options})
  endforeach()

  # ROS 2 adapter tests.
  foreach(test_name test_parameter_ros test_ros_conversions test_component)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/component)
    target_link_libraries(${test_name} ${PROJECT_NAME}_component)
    target_compile_options(${test_name} PRIVATE ${warning_options})
  endforeach()
  target_link_libraries(test_component class_loader::class_loader)
  target_compile_definitions(test_component PRIVATE
    COMPONENT_LIBRARY="$<TARGET_FILE:${PROJECT_NAME}_component>"
    ADAPTER_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/cpm_lab_adapter.yaml"
    SENSOR_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/cpm_lab_sensor.yaml"
    OSC_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/osc_lane_control_light.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
