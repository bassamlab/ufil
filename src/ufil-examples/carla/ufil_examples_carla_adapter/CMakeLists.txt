cmake_minimum_required(VERSION 3.8)
project(ufil_examples_carla_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(derived_object_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CarlaActorListStamped.msg
  DEPENDENCIES std_msgs carla_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Include Boost headers
include_directories(${Boost_INCLUDE_DIRS})

add_executable(ufil_examples_carla_adapter_node src/ufil_examples_carla_adapter_node.cpp)
target_include_directories(ufil_examples_carla_adapter_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(ufil_examples_carla_adapter_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
        ufil_examples_carla_adapter_node
        carla_msgs
        derived_object_msgs
        nav_msgs
        ufil_msgs
        rclcpp
        sensor_msgs
        tf2
        tf2_ros
        tf2_eigen
        GeographicLib
        tf2_geometry_msgs
        rosidl_default_runtime
)
target_link_libraries(ufil_examples_carla_adapter_node ${Boost_LIBRARIES})

rosidl_target_interfaces(ufil_examples_carla_adapter_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(ufil_examples_carla_adapter_message_converter
  src/message_converter.cpp)
target_include_directories(ufil_examples_carla_adapter_message_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ufil_examples_carla_adapter_message_converter PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(
  ufil_examples_carla_adapter_message_converter
  carla_msgs
  derived_object_msgs
  ufil_msgs
)

target_link_libraries(ufil_examples_carla_adapter_node
  ufil_examples_carla_adapter_message_converter)

install(TARGETS ufil_examples_carla_adapter_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS ufil_examples_carla_adapter_message_converter
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

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

ament_package()