cmake_minimum_required(VERSION 3.8)
project(ufil_udp)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(udp_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(udp_sender_node src/udp_sender_node.cpp)
target_include_directories(udp_sender_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(udp_sender_node PUBLIC c_std_99 cxx_std_20) # Require C99 and C++17
ament_target_dependencies(
  udp_sender_node
  "rclcpp"
  "udp_msgs"
)
target_link_libraries(udp_sender_node Boost::system)

install(TARGETS udp_sender_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(udp_receive_node src/udp_receive_node.cpp)
target_include_directories(udp_receive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(udp_receive_node PUBLIC c_std_99 cxx_std_20) # Require C99 and C++17
ament_target_dependencies(
  udp_receive_node
  "rclcpp"
  "udp_msgs"
)
target_link_libraries(udp_receive_node Boost::system)

install(TARGETS udp_receive_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
