cmake_minimum_required(VERSION 3.8)
project(ufil_examples VERSION 0.1.0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ufil_examples_pedestrian REQUIRED)
find_package(ufil_examples_carla REQUIRED)
find_package(ufil_examples_cpm_lab REQUIRED)

# supress warning about unused variable
set(supress "${CMAKE_EXPORT_COMPILE_COMMANDS}")

# create package
ament_package()
