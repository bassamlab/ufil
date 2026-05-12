#!/usr/bin/env python

# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    default_scenario_directory = os.path.join(get_package_share_directory('ufil_examples_pedestrian_simulator'),'config')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', description='Select scenario for simulation.', choices=['crossing.yaml', 'parallel.yaml', 'passing.yaml'], default_value='crossing.yaml'),
        DeclareLaunchArgument('scenario_directory', default_value=TextSubstitution(text=default_scenario_directory)),
        DeclareLaunchArgument('use_sim_time', description='Flag indicating that a recorded file is used.', choices=['false', 'true'], default_value='false'),
        Node(
            package='ufil_examples_pedestrian_simulator',
            namespace='ufil_examples_pedestrian',
            executable='ufil_examples_pedestrian_simulator_node',
            name='ufil_examples_pedestrian_simulator',
            parameters=[
                {'scenario_file': PathJoinSubstitution([LaunchConfiguration('scenario_directory'), LaunchConfiguration('scenario')])},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        )
    ])