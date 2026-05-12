cmake_minimum_required(VERSION 3.8)
project(ufil_examples_cpm_lab)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ufil_core REQUIRED)
find_package(ufil_obu REQUIRED)
find_package(ufil_rsu REQUIRED)
find_package(ufil_examples_cpm_lab_adapter REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# supress warning about unused variable
set(supress "${CMAKE_EXPORT_COMPILE_COMMANDS}")

# create package
ament_package()
