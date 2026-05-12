cmake_minimum_required(VERSION 3.8)
project(ufil_core VERSION 0.1.0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_visualization REQUIRED)
find_package(ufil_object_tracking REQUIRED)
find_package(ufil_etsi REQUIRED)
find_package(ufil_mqtt REQUIRED)
find_package(ufil_osc REQUIRED)
find_package(ufil_udp REQUIRED)
find_package(ufil_message_filter REQUIRED)

# supress warning about unused variable
set(supress "${CMAKE_EXPORT_COMPILE_COMMANDS}")

# create package
ament_package()
