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
import ast
from typing import List

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs) -> list[Node]:
    # Launch Arguments
    vehicle_ids: str = LaunchConfiguration('ids').perform(context=context)
    vehicle_ids_list: List[str] = [str(vehicle_id) for vehicle_id in ast.literal_eval(vehicle_ids)]
   
    urdf_file_name = 'urdf/utm_32n.urdf.xml'
    urdf = os.path.join(get_package_share_directory('ufil_examples_cpm_lab'), urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    nodes: list[Node] = []

    world_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_32n',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_desc}
        ]
    )
    nodes.append(world_node)


    for vehicle_id in vehicle_ids_list:
        obu_namespace: str = '/obu_'+vehicle_id
        efk_frame_id_suffix: str = '_'+vehicle_id
        obu_params_file_path: str = os.path.join(get_package_share_directory('ufil_examples_cpm_lab'), 'param', 'obu.yaml')

        adapter_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ufil_examples_cpm_lab_adapter'), 'launch'),
                '/sensors.launch.py']),
            launch_arguments={
                'id': vehicle_id,
                'namespace': [obu_namespace,'/sensors'],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_frame': 'map_utm'
                }.items(),
        )
        nodes.append(adapter_nodes)


        state_estimation_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ufil_obu'), 'launch'),
                '/state_estimation.launch.py']),
            launch_arguments={
                'efk_frame_id_suffix': efk_frame_id_suffix,
                'ekf_params': obu_params_file_path,
                'object_length': '4.524',
                'object_width': '1.838',
                'object_classification': '0',
                'object_output_topic': 'object',
                'base_namespace': obu_namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
        )
        nodes.append(state_estimation_nodes)

        obu_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ufil_obu'), 'launch'),
                '/default.launch.py']),
            launch_arguments={
                'input_topic': 'object',
                'base_namespace': obu_namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cam_time_shift_nanosec' : '500000000000'
                }.items(),
        )
        nodes.append(obu_nodes)

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', description='Flag indicating that a recorded file is used.', choices=['false', 'true'], default_value='true'),
        DeclareLaunchArgument(
            'ids',
            description='Ids of vehicles.',
            default_value="[1]"
        ),
        OpaqueFunction(function=launch_setup),
    ])
