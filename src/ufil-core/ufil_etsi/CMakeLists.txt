cmake_minimum_required(VERSION 3.8)
project(ufil_etsi VERSION 0.2.0)

find_package(ament_cmake_ros REQUIRED)

# Core library (no ROS 2 dependencies). The Debian package of GeographicLib only ships a find
# module, see https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173.
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# ROS 2 components
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_ros REQUIRED)

set(warning_options
  $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -Wpedantic>)

# --- Core library ------------------------------------------------------------------------

add_library(${PROJECT_NAME}
  src/cam_decoding.cpp
  src/cam_encoding.cpp
  src/its_time.cpp
  src/rsu_cam_generator.cpp
  src/types.cpp
  src/vehicle_cam_generator.cpp
  src/vehicle_cam_receiver.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})
# Causes the visibility macros to use dllexport rather than dllimport.
target_compile_definitions(${PROJECT_NAME} PRIVATE UFIL_ETSI_BUILDING_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${GeographicLib_LIBRARIES})

# --- ROS 2 components --------------------------------------------------------------------

add_library(${PROJECT_NAME}_component SHARED
  component/rsu_cam_generator_node.cpp
  component/vehicle_cam_generator_node.cpp
  component/vehicle_cam_receiver_node.cpp
)
target_compile_options(${PROJECT_NAME}_component PRIVATE ${warning_options})
target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/component)
target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ${PROJECT_NAME}
  ${builtin_interfaces_TARGETS}
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  ${etsi_its_cam_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  ${ufil_msgs_TARGETS}
  ufil_ros::ufil_ros
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_etsi::VehicleCamGeneratorNode"
  EXECUTABLE vehicle_cam_generator_node
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_etsi::VehicleCamReceiverNode"
  EXECUTABLE vehicle_cam_receiver_node
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_etsi::RsuCamGeneratorNode"
  EXECUTABLE rsu_cam_generator_node
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
      test_high_frequency_container_builder
      test_cam_encoding
      test_cam_decoding
      test_its_time
      test_types
      test_vehicle_cam_generator
      test_vehicle_cam_receiver
      test_rsu_cam_generator)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
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
    CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
