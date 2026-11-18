cmake_minimum_required(VERSION 3.8)
project(ufil_core VERSION 0.2.0 LANGUAGES NONE)

# Meta package: it only pulls in the UFIL core packages as run dependencies (see package.xml).
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
