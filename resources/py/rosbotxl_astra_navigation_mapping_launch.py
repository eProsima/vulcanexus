# Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        get_package_share_directory('rosbot_xl_bringup'),
                        'launch',
                        'bringup.launch.py',
                    ])
                ),
                launch_arguments={
                    'mecanum': 'True',
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        get_package_share_directory('sllidar_ros2'),
                        'launch',
                        'sllidar_launch.py',
                    ])
                ),
                launch_arguments={
                    'serial_baudrate': '115200',
                    'serial_port': '/dev/ttyRPLIDAR',
                }.items(),
            ),

            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                get_package_share_directory('nav2_bringup'),
                                'launch',
                                'slam_launch.py',
                            ])
                        ),
                        launch_arguments={
                            'map': '/maps/map.yaml',
                            'use_sim_time': 'False',
                            'params_file': '/home/husarion/ros2_ws/config/nav2_params.yaml',
                        }.items(),
                    ),
                ]
            ),

            TimerAction(
                period=3.5,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                get_package_share_directory('nav2_bringup'),
                                'launch',
                                'navigation_launch.py',
                            ])
                        ),
                        launch_arguments={
                            'use_sim_time': 'False',
                            'params_file': '/home/husarion/ros2_ws/config/nav2_rpp_params.yaml',
                        }.items(),
                    ),
                ]
            ),

            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                get_package_share_directory('astra_camera'),
                                'launch',
                                'astra_mini.launch.py',
                            ])
                        ),
                    ),
                ]
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the nodes
    ld.add_action(bringup_cmd_group)

    return ld
