cmake_minimum_required(VERSION 3.8)
project(ufil_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_ros REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_executable(mqtt_message_converter_node src/mqtt_message_converter_node.cpp)
target_include_directories(mqtt_message_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mqtt_message_converter_node PUBLIC cxx_std_20)
target_link_libraries(mqtt_message_converter_node paho-mqtt3as paho-mqttpp3)

ament_target_dependencies(mqtt_message_converter_node
  nav_msgs
  rclcpp
  ufil_msgs
  ufil_ros)

install(TARGETS mqtt_message_converter_node
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