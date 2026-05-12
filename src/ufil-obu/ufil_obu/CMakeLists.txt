cmake_minimum_required(VERSION 3.8)
project(ufil_obu)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ufil_obu_object_aggregation REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# supress warning about unused variable
set(supress "${CMAKE_EXPORT_COMPILE_COMMANDS}")

# create package
ament_package()
