# Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True'
    )

    host_launch_arg = launch.actions.DeclareLaunchArgument(
        name='host',
        default_value='localhost'
    )

    port_launch_arg = launch.actions.DeclareLaunchArgument(
        name='port',
        default_value='2000'
    )

    timeout_launch_arg = launch.actions.DeclareLaunchArgument(
        name='timeout',
        default_value='10'
    )

    passive_launch_arg = launch.actions.DeclareLaunchArgument(
        name='passive',
        default_value='False'
    )
    
    synchronous_mode_launch_arg = launch.actions.DeclareLaunchArgument(
        name='synchronous_mode',
        default_value='True'
    )

    synchronous_mode_wait_launch_arg = launch.actions.DeclareLaunchArgument(
        name='synchronous_mode_wait_for_vehicle_control_command',
        default_value='True'
    )

    fixed_delta_seconds_launch_arg = launch.actions.DeclareLaunchArgument(
        name='fixed_delta_seconds',
        default_value='0.05'
    )

    town_launch_arg = launch.actions.DeclareLaunchArgument(
        name='town',
        default_value='Town10HD_Opt'
    )

    objects_definition_file_launch_arg = launch.actions.DeclareLaunchArgument(
        name='objects_definition_file',
        default_value=os.path.join(
            '/objects.json'
        )
    )
    
    carla_spawn_objects_launch_include = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('carla_spawn_objects'),
                'carla_spawn_objects.launch.py'
            )
        ),
        launch_arguments={
            'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
        }.items()
    )
    
    ros_bridge_launch_include = launch.actions.GroupAction(
        actions=[
            launch_ros.actions.SetRemap(src="/tf", dst="/tf_carla"),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('carla_ros_bridge'),
                        'carla_ros_bridge.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                    'host': launch.substitutions.LaunchConfiguration('host'),
                    'port': launch.substitutions.LaunchConfiguration('port'),
                    'town': launch.substitutions.LaunchConfiguration('town'),
                    'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                    'passive': launch.substitutions.LaunchConfiguration('passive'),
                    'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode'),
                    'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                }.items()
            )
        ]
    )

    # Return full launch description

    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        host_launch_arg,
        port_launch_arg,
        timeout_launch_arg,
        passive_launch_arg,
        synchronous_mode_launch_arg,
        synchronous_mode_wait_launch_arg,
        fixed_delta_seconds_launch_arg,
        town_launch_arg,
        objects_definition_file_launch_arg,
        carla_spawn_objects_launch_include,
        ros_bridge_launch_include,
    ])


if __name__ == '__main__':
    generate_launch_description()