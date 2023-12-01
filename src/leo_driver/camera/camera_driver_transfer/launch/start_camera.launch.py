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

#摄像头列表字典
camera_type_dict = {"2bc5:0403": "astrapro", "2bc5:0401": "astra", "8086:0b07": "d435"}

#执行命令行，返回输出值
def execCmd(cmd):
    r = os.popen(cmd)
    text = r.read()
    r.close()
    return text

#执行命令行，返回输出值  
def get_camera_type():
    for i in camera_type_dict:
        # print(i, camera_type_dict[i])
        cmd = "lsusb -d " + i
        # print(cmd)
        result = execCmd(cmd)

        if len(result) !=0:
            # print("check camera is ",camera_type_dict[i])
            return camera_type_dict[i]
    print("can not find camera,set d435 as default!")
    return "d435"    



def generate_launch_description():
    # Get the launch directory
    camera_driver_transfer_dir = get_package_share_directory('camera_driver_transfer')

    default_camera_name = get_camera_type()
    # Create the launch configuration variables
    camera_type_tel = LaunchConfiguration('camera_type_tel')

    dp_rgist = LaunchConfiguration('dp_rgist')   
    start_camera_rviz = LaunchConfiguration('start_camera_rviz')   
    #remappings = [('/camera/depth/points', '/camera/depth_registered/points')]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_dp_rgist = DeclareLaunchArgument(
        'dp_rgist', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run dp_rgist')
        
    declare_start_camera_rviz = DeclareLaunchArgument(
        'start_camera_rviz', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run start_camera_rviz')

    declare_camera_type_tel = DeclareLaunchArgument(
        'camera_type_tel', 
        default_value=default_camera_name,
        #choices=['d435', 'astra_pro'],
        description='camera type')



        
    camera_type_launch = (camera_driver_transfer_dir, '/launch/', camera_type_tel, '.launch.py')
    camera_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_type_launch))
    ])
    # Specify the actions
    # camera_group = GroupAction([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(camera_driver_transfer_dir, 'launch',
    #                                                    'd435.launch.py')),
    #         condition=LaunchConfigurationEquals('camera_type_tel', 'd435'),
    #         launch_arguments={'start_camera_rviz': start_camera_rviz,
    #                           'dp_rgist': dp_rgist}.items()),

    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(camera_driver_transfer_dir, 'launch',
    #                                                    'astrapro_launch.py')),
    #         condition=LaunchConfigurationEquals('camera_type_tel', 'astrapro'),
    #         launch_arguments={'start_camera_rviz': start_camera_rviz,
    #                           'dp_rgist': dp_rgist}.items()),
    # ])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_start_camera_rviz)
    ld.add_action(declare_dp_rgist)
    ld.add_action(declare_camera_type_tel)
    ld.add_action(camera_group)

    return ld
