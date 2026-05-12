cmake_minimum_required(VERSION 3.8)
project(ufil_etsi)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_library(ufil_etsi
        src/BasicContainerBuilder.cpp
        src/HighFrequencyContainerBuilder.cpp
        src/CamBuilder.cpp
        src/LowFrequencyContainerBuilder.cpp)
target_compile_features(ufil_etsi PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(ufil_etsi PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
        ufil_etsi
        etsi_its_cam_msgs
        etsi_its_msgs_utils
        ufil_msgs
        GeographicLib
        rclcpp
        tf2
        tf2_geometry_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ufil_etsi PRIVATE "ETSI_MESSAGE_CONVERTER_BUILDING_LIBRARY")

install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS ufil_etsi
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_executable(vehicle_cam_generator_node src/vehicle_cam_generator_node.cpp)
target_include_directories(vehicle_cam_generator_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(vehicle_cam_generator_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_link_libraries(vehicle_cam_generator_node ufil_etsi)

ament_target_dependencies(vehicle_cam_generator_node geometry_msgs etsi_its_cam_msgs etsi_its_msgs_utils rclcpp ufil_ros ufil_msgs)

install(TARGETS vehicle_cam_generator_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(rsu_cam_generator_node src/rsu_cam_generator_node.cpp)
target_include_directories(rsu_cam_generator_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(rsu_cam_generator_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_link_libraries(rsu_cam_generator_node ufil_etsi)

ament_target_dependencies(rsu_cam_generator_node etsi_its_cam_msgs etsi_its_msgs_utils ufil_ros rclcpp)

install(TARGETS rsu_cam_generator_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(vehicle_cam_receiver_node src/vehicle_cam_receiver_node.cpp)
target_include_directories(vehicle_cam_receiver_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(vehicle_cam_receiver_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_link_libraries(vehicle_cam_receiver_node ufil_etsi)

ament_target_dependencies(vehicle_cam_receiver_node Eigen3 etsi_its_cam_msgs etsi_its_msgs_utils GeographicLib ufil_msgs ufil_ros rclcpp)

install(TARGETS vehicle_cam_receiver_node
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

ament_export_include_directories(
        include
)
ament_export_libraries(
        ufil_etsi
)
ament_export_targets(
        export_${PROJECT_NAME}
)
ament_package()
