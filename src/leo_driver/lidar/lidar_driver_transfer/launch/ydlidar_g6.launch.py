# Copyright (c) 2022 NXROBO
#
# /* Author: litian.zhuang */
# /* email: litian.zhuang@nxrobo.com */
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
# 


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Get the launch directory
    ydlidar_ros2_driver_dir = get_package_share_directory('ydlidar_ros2_driver')



    start_lidar_rviz = LaunchConfiguration('start_lidar_rviz')   

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
        
    declare_start_lidar_rviz = DeclareLaunchArgument(
        'start_lidar_rviz', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run start_lidar_rviz')


    # Specify the actions
    lidar_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ydlidar_ros2_driver_dir, 'launch',
                                                       'ydlidar_g6_launch.py')),
            launch_arguments={'start_lidar_rviz': start_lidar_rviz}.items()),
    ])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_start_lidar_rviz)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(lidar_group)

    return ld
