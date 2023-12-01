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
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    arg_serial_port = DeclareLaunchArgument('serial_port', default_value='/dev/LeoBase',
                                         description='serial port name:/dev/ttyACM0 or /dev/LeoBase')
    arg_base_frame_id = DeclareLaunchArgument('base_frame_id', default_value='base_footprint',
                                           description='base fram id')
    arg_odom_frame_id = DeclareLaunchArgument('odom_frame_id', default_value='odom',
                                                description='Base link frame id')
    
    leo_base_node = launch_ros.actions.Node(
        package='leo_base',
        executable='leo_base_node',  
        output='screen',
        emulate_tty=True,
        parameters=[{
                'serial_port': launch.substitutions.LaunchConfiguration('serial_port'),                
                'base_frame_id': launch.substitutions.LaunchConfiguration('base_frame_id'),
                'odom_frame_id': launch.substitutions.LaunchConfiguration('odom_frame_id')
        }])

    return LaunchDescription([
        arg_serial_port,        
        arg_base_frame_id,
        arg_odom_frame_id,
        leo_base_node
    ])
