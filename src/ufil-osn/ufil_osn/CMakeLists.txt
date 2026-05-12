cmake_minimum_required(VERSION 3.8)
project(ufil_osn VERSION 0.1.0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ufil_core REQUIRED)
find_package(ufil_osn_tracker REQUIRED)
find_package(ufil_pcl_preprocessor REQUIRED)

# supress warning about unused variable
set(supress "${CMAKE_EXPORT_COMPILE_COMMANDS}")

# install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# create package
ament_package()

