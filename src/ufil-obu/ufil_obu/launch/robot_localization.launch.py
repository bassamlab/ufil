#  Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
# 
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
# 
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
# 
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#  THE SOFTWARE.

#!/usr/bin/env python
import os
from ament_index_python import get_package_share_directory

import launch
import launch.actions
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('params', default_value='', description='Parameter of eks.'),
        DeclareLaunchArgument('global_ouput_topic', default_value='odometry/global', description='Global output topic name'),
        DeclareLaunchArgument('local_ouput_topic', default_value='odometry/local', description='Local output topic name'),

        DeclareLaunchArgument('accel_input_topic', default_value='sensors/imu', description='Acceleration input topic name'),
        DeclareLaunchArgument('accel_output_topic', default_value='filtered/accel', description='Acceleration output topic name'),
        DeclareLaunchArgument('odom_input_topic', default_value='sensors/wheel', description='Wheel odometry input topic name'),
        DeclareLaunchArgument('odom_output_topic', default_value='filtered/wheel', description='Wheel odometry output topic name'),
        DeclareLaunchArgument('gps_input_topic', default_value='sensors/gps', description='Gps input topic name'),
        DeclareLaunchArgument('gps_output_topic', default_value='filtered/gps', description='Gps output topic name'),

        DeclareLaunchArgument('enable_stats', default_value='false', description='Enable packet forwarding statistics'),
        DeclareLaunchArgument('efk_frame_id_suffix', default_value='', description='Suffix for frame ids in robot lokalization ekf implementation.'),
        DeclareLaunchArgument('use_sim_time', description='Flag indicating that a recorded file is used.', choices=['false', 'true'], default_value='true'),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[LaunchConfiguration('params'), {
                'map_frame': 'map',
                'odom_frame': ['odom', LaunchConfiguration('efk_frame_id_suffix')],
                'base_link_frame': ['base_link', LaunchConfiguration('efk_frame_id_suffix')],
                'world_frame': ['odom', LaunchConfiguration('efk_frame_id_suffix')],
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                ('odometry/filtered', LaunchConfiguration('local_ouput_topic')),
                ('odometry/wheel',  LaunchConfiguration('wheel_input_topic')),
                ('imu',  LaunchConfiguration('accel_input_topic')),
            ],
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[LaunchConfiguration('params'), {
                'map_frame': 'map',
                'odom_frame': ['odom', LaunchConfiguration('efk_frame_id_suffix')],
                'base_link_frame': ['base_link', LaunchConfiguration('efk_frame_id_suffix')],
                'world_frame': 'map',
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                ('odometry/filtered', LaunchConfiguration('global_ouput_topic')),
                ('odometry/wheel',  LaunchConfiguration('wheel_input_topic')),
                ('imu',  LaunchConfiguration('accel_input_topic')),
                ('accel/filtered', LaunchConfiguration('accel_output_topic')),
            ],
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[LaunchConfiguration('params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('odometry/filtered', LaunchConfiguration('global_ouput_topic')),
                ('gps/fix', LaunchConfiguration('gps_input_topic')),
                ('gps/filtered', LaunchConfiguration('gps_output_topic')),

            ],
        ),
    ])
