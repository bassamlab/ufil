cmake_minimum_required(VERSION 3.8)
project(ufil_osc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OSCPACK REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_ros REQUIRED)

add_executable(osc_message_converter_node src/osc_message_converter_node.cpp)
target_include_directories(osc_message_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(osc_message_converter_node PUBLIC cxx_std_20)
target_link_libraries(osc_message_converter_node OscPack)

ament_target_dependencies(osc_message_converter_node
  nav_msgs
  rclcpp
  ufil_msgs
  ufil_ros)

install(TARGETS osc_message_converter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()