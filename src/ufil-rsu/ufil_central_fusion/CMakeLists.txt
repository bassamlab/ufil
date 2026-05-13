cmake_minimum_required(VERSION 3.8)
project(ufil_central_fusion)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ufil_ros REQUIRED)
find_package(ufil_object_tracking REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)

add_library(central_fusion
        src/classification_updater.cpp
        src/classification.cpp
        src/dimension.cpp
        src/existence_probability_updater.cpp
        src/existence_probability.cpp
        src/measurement_model.cpp
        src/measurement.cpp
        src/object_tracker.cpp
        src/sensor_data.cpp
        src/orientation_checker.cpp
)

target_compile_features(central_fusion PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(central_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(central_fusion PRIVATE "UFIL_CENTRAL_FUSION_BUILDING_LIBRARY")
target_link_libraries(central_fusion
  opencv_core
  opencv_highgui
  opencv_imgproc
)
ament_target_dependencies(central_fusion ufil_object_tracking)


add_executable(central_fusion_node component/central_fusion_node.cpp)

target_include_directories(central_fusion_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(central_fusion_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(central_fusion_node
        ament_index_cpp
        rclcpp
        ufil_msgs
        nav_msgs
        ufil_ros
        ufil_object_tracking
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
)

target_link_libraries(central_fusion_node central_fusion)

install(TARGETS central_fusion_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
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
