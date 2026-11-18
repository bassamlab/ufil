cmake_minimum_required(VERSION 3.8)
project(ssl_vehicle_tracking VERSION 0.2.0)

find_package(ament_cmake_ros REQUIRED)

# Core library (no ROS 2 dependencies)
find_package(yaml-cpp REQUIRED)

# ROS 2 component
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ssl_vehicle_tracking_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)

set(warning_options
  $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -Wpedantic>)

# --- Core library ------------------------------------------------------------------------

add_library(${PROJECT_NAME}
  src/vehicle_template.cpp
  src/vehicle_tracker.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})
# Causes the visibility macros to use dllexport rather than dllimport.
target_compile_definitions(${PROJECT_NAME} PRIVATE SSL_VEHICLE_TRACKING_BUILDING_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp::yaml-cpp)

# --- ROS 2 component ---------------------------------------------------------------------

add_library(${PROJECT_NAME}_component SHARED component/ssl_vehicle_tracking_node.cpp)
target_compile_options(${PROJECT_NAME}_component PRIVATE ${warning_options})
target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/component)
target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${ssl_vehicle_tracking_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${ufil_msgs_TARGETS}
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ssl_vehicle_tracking::SslVehicleTrackingNode"
  EXECUTABLE ssl_vehicle_tracking_node
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
install(DIRECTORY config launch templates DESTINATION share/${PROJECT_NAME})

# --- Tests -------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(class_loader REQUIRED)

  # Core library tests, no ROS 2 required.
  foreach(test_name test_vehicle_template test_vehicle_tracker)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_definitions(${test_name} PRIVATE
      TEMPLATE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/templates")
  endforeach()

  # ROS 2 adapter tests.
  foreach(test_name test_parameter_ros test_ros_conversions test_component)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/component)
    target_link_libraries(${test_name} ${PROJECT_NAME}_component)
    target_compile_definitions(${test_name} PRIVATE
      TEMPLATE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/templates")
  endforeach()
  target_link_libraries(test_component class_loader::class_loader)
  target_compile_definitions(test_component PRIVATE
    COMPONENT_LIBRARY="$<TARGET_FILE:${PROJECT_NAME}_component>"
    DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

ament_package()
