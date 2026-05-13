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
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('base_namespace', description='Namespace of this node.', default_value="obu"),
        DeclareLaunchArgument('ekf_params', description='Config for robot lokalization.', default_value=""),
        DeclareLaunchArgument('efk_frame_id_suffix', description='Suffix for frame ids of ekf.', default_value=""),
        DeclareLaunchArgument('object_length', default_value='2.0', description='Length of object'),
        DeclareLaunchArgument('object_width', default_value='4.0', description='Width of object'),
        DeclareLaunchArgument('object_classification', default_value='0', description='Type of object in ufil_msgs/msg/Classifiction integer'),
        DeclareLaunchArgument('object_output_topic', default_value='cam', description='Output cam topic name'),
        DeclareLaunchArgument('imu_topic', default_value='sensors/imu', description='Acceleration topic name'),
        DeclareLaunchArgument('wheel_topic', default_value='sensors/wheel', description='Wheel odometry topic name'),
        DeclareLaunchArgument('gps_topic', default_value='sensors/gps', description='Gps topic name'),
        DeclareLaunchArgument('enable_stats', default_value='false', description='Enable packet forwarding statistics'),
        DeclareLaunchArgument('use_sim_time', description='Flag indicating that a recorded file is used.', choices=['false', 'true'], default_value='true'),
       
        GroupAction(actions=[
            PushRosNamespace([LaunchConfiguration('base_namespace')]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('ufil_obu'), 'launch'),'/robot_localization.launch.py']
                ),
                launch_arguments={
                    'accel_input_topic': [LaunchConfiguration('base_namespace'), '/sensors/imu'],
                    'wheel_input_topic': [LaunchConfiguration('base_namespace'), '/sensors/wheel'],
                    'gps_input_topic': [LaunchConfiguration('base_namespace'), '/sensors/gps'],
                    'accel_output_topic':  'filtered/accel',
                    'wheel_output_topic': 'filtered/wheel',
                    'gps_output_topic': 'filtered/gps',
                    'global_output_topic': 'odometry/global',
                    'local_output_topic': 'odometry/local',
                    'efk_frame_id_suffix': LaunchConfiguration('efk_frame_id_suffix'),
                    'enable_stats': LaunchConfiguration('enable_stats'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params': LaunchConfiguration('ekf_params')
                    }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('ufil_obu_object_aggregation'), 'launch'),'/default.launch.py']
                ),
                launch_arguments={
                    'output_topic': 'object',
                    'object_length': LaunchConfiguration('object_length'),
                    'object_width': LaunchConfiguration('object_width'),
                    'object_classification': LaunchConfiguration('object_classification'),
                    'accel_topic': 'filtered/accel',
                    'odom_topic': 'odometry/global',
                    'gps_topic': 'filtered/gps',
                    'enable_stats': LaunchConfiguration('enable_stats'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'output_topic': LaunchConfiguration('object_output_topic')
                }.items(),
            )
        ])
    ])