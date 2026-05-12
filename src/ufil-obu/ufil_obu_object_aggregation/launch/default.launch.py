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

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('object_length', default_value='2.0', description='Length of object'),
        DeclareLaunchArgument('object_width', default_value='4.0', description='Width of object'),
        DeclareLaunchArgument('object_classification', default_value='0', description='Type of object in ufil_msgs/msg/Classifiction integer'),
        DeclareLaunchArgument('accel_topic', default_value='accel', description='Acceleration topic name'),
        DeclareLaunchArgument('odom_topic', default_value='odometry', description='Wheel odometry topic name'),
        DeclareLaunchArgument('gps_topic', default_value='nav_sat_fix', description='Gps topic name'),
        DeclareLaunchArgument('output_topic', default_value='object', description='Object output topic name'),
        DeclareLaunchArgument('enable_stats', default_value='false', description='Enable packet forwarding statistics'),
        DeclareLaunchArgument('use_sim_time', description='Flag indicating that a recorded file is used.', choices=['false', 'true'], default_value='true'),

        launch_ros.actions.Node(
            package='ufil_obu_object_aggregation',
            executable='object_aggregation_node',
            name='object_aggregation_node',
            output='screen',
            parameters=[
                {'enable_stats': LaunchConfiguration('enable_stats')},
                {'object_length': LaunchConfiguration('object_length')},
                {'object_width': LaunchConfiguration('object_width')},
                {'object_classification': LaunchConfiguration('object_classification')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('accel', LaunchConfiguration('accel_topic')),
                ('odometry', LaunchConfiguration('odom_topic')),
                ('nav_sat_fix', LaunchConfiguration('gps_topic')),
                ('object', LaunchConfiguration('output_topic')),
            ]
        )
    ])
