cmake_minimum_required(VERSION 3.8)
project(ufil_examples_pedestrian_simulator)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(ufil_examples_pedestrian_simulator
  src/ufil_examples_pedestrian_simulator.cpp
  src/agent.cpp
  src/scenario.cpp
)
add_library(ufil_examples_pedestrian_simulator::ufil_examples_pedestrian_simulator ALIAS ufil_examples_pedestrian_simulator)
target_compile_features(ufil_examples_pedestrian_simulator PUBLIC c_std_99 cxx_std_20) # Require C99 and C++17
target_include_directories(ufil_examples_pedestrian_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(ufil_examples_pedestrian_simulator yaml-cpp Eigen3::Eigen)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ufil_examples_pedestrian_simulator PRIVATE "UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ufil_examples_pedestrian_simulator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(ufil_examples_pedestrian_simulator_node
  src/ufil_examples_pedestrian_simulator_node.cpp
)
target_include_directories(ufil_examples_pedestrian_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(ufil_examples_pedestrian_simulator_node ufil_examples_pedestrian_simulator)
ament_target_dependencies(ufil_examples_pedestrian_simulator_node
  "rclcpp"
  "ufil_msgs"
  "ament_index_cpp"
  "yaml_cpp_vendor"
)

install(TARGETS ufil_examples_pedestrian_simulator_node
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  ufil_examples_pedestrian_simulator
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
