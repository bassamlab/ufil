cmake_minimum_required(VERSION 3.8)
project(ssl_instance_virtual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(kalman_cpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(fmt REQUIRED)

include_directories(libs/gcem)
link_directories(${Boost_LIBRARY_DIR})

add_library(ssl_virtual_component
        src/layer.cpp
        src/geom.cpp
        src/ssl_single_virtual.cpp
        src/simple_vehicle_model.cpp)

target_compile_features(ssl_virtual_component PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_link_libraries(ssl_virtual_component fmt::fmt)
target_include_directories(ssl_virtual_component PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ssl_virtual_component rclcpp ufil_msgs nav_msgs tf2_geometry_msgs Eigen3 kalman_cpp rclcpp_components)

install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS ssl_virtual_component
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_executable(ssl_virtual_component_node src/ssl_node.cpp)
target_include_directories(ssl_virtual_component_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(ssl_virtual_component_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_link_libraries(ssl_virtual_component_node ssl_virtual_component fmt::fmt)

ament_target_dependencies(ssl_virtual_component_node rclcpp ufil_msgs nav_msgs tf2_geometry_msgs Eigen3 kalman_cpp rclcpp_components)

install(TARGETS ssl_virtual_component_node
        DESTINATION lib/${PROJECT_NAME})

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
        include
)
ament_export_libraries(
        ssl_virtual_component
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
