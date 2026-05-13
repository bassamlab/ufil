cmake_minimum_required(VERSION 3.8)
project(ufil_examples_cpm_lab_adapter)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Allow CMake to locate our script for the local third party library
set(CMAKE_MODULE_PATH " ${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/cmake;/usr/share/cmake/geographiclib")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(GeographicLib REQUIRED)
find_package(cpm_lab_lab_msgs REQUIRED)
find_package(cpm_lab_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ufil_msgs REQUIRED)
find_package(ufil_ros REQUIRED)
find_package(ufil_object_tracking REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OSCPACK REQUIRED)

add_library(ufil_examples_cpm_lab_adapter
        src/artificial_gnss.cpp
)
target_compile_features(ufil_examples_cpm_lab_adapter PUBLIC c_std_99 cxx_std_20) # Require C99 and C++20
target_include_directories(ufil_examples_cpm_lab_adapter PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ufil_examples_cpm_lab_adapter PUBLIC
        Eigen3
        cpm_lab_lab_msgs
        rclcpp
        tf2
        tf2_ros
        tf2_geometry_msgs
)
target_compile_definitions(ufil_examples_cpm_lab_adapter PRIVATE "LAB_ADAPTER_BUILDING_LIBRARY")
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS ufil_examples_cpm_lab_adapter
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_executable(ufil_examples_cpm_lab_adapter_node src/ufil_examples_cpm_lab_adapter_node.cpp)
target_include_directories(ufil_examples_cpm_lab_adapter_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(ufil_examples_cpm_lab_adapter_node GeographicLib)
target_compile_features(ufil_examples_cpm_lab_adapter_node PUBLIC c_std_99 cxx_std_20) # Require C99 and C++20
ament_target_dependencies(ufil_examples_cpm_lab_adapter_node
        rclcpp
        tf2
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        Eigen3
        cpm_lab_lab_msgs
        ufil_msgs
        ufil_ros
        ufil_object_tracking
)
install(TARGETS ufil_examples_cpm_lab_adapter_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(ufil_examples_cpm_lab_adapter_sensor_node src/ufil_examples_cpm_lab_adapter_sensor_node.cpp)
target_include_directories(ufil_examples_cpm_lab_adapter_sensor_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(ufil_examples_cpm_lab_adapter_sensor_node GeographicLib ufil_examples_cpm_lab_adapter)
target_compile_features(ufil_examples_cpm_lab_adapter_sensor_node PUBLIC c_std_99 cxx_std_20) # Require C99 and C++20
ament_target_dependencies(ufil_examples_cpm_lab_adapter_sensor_node
        rclcpp
        tf2
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        Eigen3
        cpm_lab_lab_msgs
        nav_msgs
        sensor_msgs
        ufil_msgs
        ufil_ros
        ufil_object_tracking
)
install(TARGETS ufil_examples_cpm_lab_adapter_sensor_node
        DESTINATION lib/${PROJECT_NAME})


add_executable(ufil_examples_osc_message_converter_node src/ufil_examples_osc_message_converter_node.cpp src/message_converter.cpp)
target_include_directories(ufil_examples_osc_message_converter_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(ufil_examples_osc_message_converter_node PUBLIC c_std_99 cxx_std_20) # Require C99 and C++20
target_link_libraries(ufil_examples_osc_message_converter_node OscPack)

ament_target_dependencies(ufil_examples_osc_message_converter_node
        "rclcpp"
        "cpm_lab_map_msgs"
)

install(TARGETS ufil_examples_osc_message_converter_node
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
        ufil_examples_cpm_lab_adapter
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()