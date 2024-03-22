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
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')

    # Create the launch configuration variables

    dp_rgist = LaunchConfiguration('dp_rgist')   
    enable_rgbd = LaunchConfiguration('enable_rgbd')  
    pointcloud_enable = LaunchConfiguration('pointcloud.enable')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_color = LaunchConfiguration('enable_color')
    align_depth_enable = LaunchConfiguration('align_depth.enable')
    enable_sync = LaunchConfiguration('enable_sync')
    serial_no = LaunchConfiguration('serial_no')
    

    #remappings = [('/camera/depth/points', '/camera/depth_registered/points')]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_dp_rgist = DeclareLaunchArgument(
        'dp_rgist', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run dp_rgist')

    declare_enable_rgbd = DeclareLaunchArgument(
        'enable_rgbd', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run enable_rgbd')


    declare_pointcloud_enable = DeclareLaunchArgument(
        'pointcloud.enable', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run pointcloud.enable')

    declare_enable_depth = DeclareLaunchArgument(
        'enable_depth', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run enable_depth')
                
    declare_enable_color = DeclareLaunchArgument(
        'enable_color', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run enable_color')
        
    declare_align_depth_enable = DeclareLaunchArgument(
        'align_depth.enable', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run align_depth.enable')

         
    declare_enable_sync = DeclareLaunchArgument(
        'enable_sync', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run enable_sync')    
        
    declare_serial_no = DeclareLaunchArgument(
        'serial_no', 
        default_value='Bus 004 Device 003',
        choices=['Bus 004 Device 003', 'Bus 004 Device 006'],
        description='serial_no')             
    # Specify the actions
    d435_camera_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense2_camera_dir, 'launch',
                                                       'rs_launch.py')),
            launch_arguments={'enable_rgbd': enable_rgbd,
                                'pointcloud.enable': pointcloud_enable,
                                'enable_depth': enable_depth,
                                'enable_color': enable_color,
                                'align_depth.enable': align_depth_enable,
                                'enable_sync': enable_sync,
                                'serial_no':serial_no,
                                }.items()),
    ])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_dp_rgist)
    ld.add_action(declare_enable_rgbd)
    ld.add_action(declare_pointcloud_enable)
    ld.add_action(declare_enable_depth)
    ld.add_action(declare_enable_color)
    ld.add_action(declare_align_depth_enable)
    ld.add_action(declare_enable_sync)
    ld.add_action(declare_serial_no)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(d435_camera_group)

    return ld
