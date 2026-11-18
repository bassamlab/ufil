cmake_minimum_required(VERSION 3.8)
project(ufil_examples_pedestrian VERSION 0.2.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
