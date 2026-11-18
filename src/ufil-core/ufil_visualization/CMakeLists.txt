cmake_minimum_required(VERSION 3.8)
project(ufil_visualization VERSION 0.2.0)

find_package(ament_cmake_ros REQUIRED)

# Core library (no ROS 2 dependencies)
find_package(Boost REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

# ROS 2 components
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(warning_options
  $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -Wpedantic>)

# --- Core library ------------------------------------------------------------------------

add_library(${PROJECT_NAME}
  src/lanelet_map_loader.cpp
  src/lanelet_map_markers.cpp
  src/marker.cpp
  src/object_markers.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE ${warning_options})
# Causes the visibility macros to use dllexport rather than dllimport.
target_compile_definitions(${PROJECT_NAME} PRIVATE UFIL_VISUALIZATION_BUILDING_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    lanelet2_core::lanelet2_core
  PRIVATE
    Boost::headers
    lanelet2_io::lanelet2_io
    lanelet2_projection::lanelet2_projection
)

# --- ROS 2 components --------------------------------------------------------------------

add_library(${PROJECT_NAME}_component SHARED
  component/lanelet_map_marker_array_node.cpp
  component/object_list_marker_array_node.cpp
)
target_compile_options(${PROJECT_NAME}_component PRIVATE ${warning_options})
target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/component)
target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ${PROJECT_NAME}
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${ufil_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_visualization::LaneletMapMarkerArrayNode"
  EXECUTABLE lanelet_map_marker_array_node
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "ufil_visualization::ObjectListMarkerArrayNode"
  EXECUTABLE object_list_marker_array_node
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
  # src/earcut.hpp is third-party code (mapbox/earcut.hpp, ISC license) and keeps its own
  # license header and style.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE src/earcut.hpp)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(class_loader REQUIRED)

  set(test_map "${CMAKE_CURRENT_SOURCE_DIR}/test/data/test_map.osm")

  # Core library tests, no ROS 2 required.
  foreach(test_name test_object_markers test_lanelet_map_markers)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_options(${test_name} PRIVATE ${warning_options})
    target_compile_definitions(${test_name} PRIVATE TEST_MAP="${test_map}")
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
    LANELET_MAP_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/lanelet_map.yaml"
    OBJECT_LIST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/object_list.yaml"
    UFIL_SOURCES_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/ufil_sources.yaml"
    TEST_MAP="${test_map}")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lanelet2_core)

ament_package()
