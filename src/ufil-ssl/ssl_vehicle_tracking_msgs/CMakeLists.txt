# At least 3.10 so that IMPORTED targets can be used
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Specify the project the used language and the version
project(ssl_vehicle_tracking_msgs VERSION 0.1)

# Configure compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
        msg/Track.msg
        msg/TrackArray.msg
        msg/TrackStamped.msg
        msg/TrackState.msg
        msg/Wheel.msg
        msg/WheelArray.msg
        msg/WheelStamped.msg
        DEPENDENCIES builtin_interfaces std_msgs nav_msgs geometry_msgs
)

ament_package()

