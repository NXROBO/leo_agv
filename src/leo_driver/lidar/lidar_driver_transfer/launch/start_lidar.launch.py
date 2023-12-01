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

#雷达列表字典
lidar_type_dict = {"10c4:ea60": ["ydlidar_g2","ydlidar_g4","ydlidar_g6"], "USB_key": "lidar_type"}

#读取雷达类型配置文件
def get_lidar_configure():
    all_the_text = None
    with open('/opt/lidar.txt', 'r') as f:
        all_the_text = f.read().strip()
    return all_the_text


#执行命令行，返回输出值
def execCmd(cmd):
    r = os.popen(cmd)
    text = r.read()
    r.close()
    return text

#执行命令行，返回输出值  
def get_lidar_type():
    lidar_c_name = get_lidar_configure()
    for i in lidar_type_dict:
        cmd = "lsusb -d " + i
        # print(cmd)
        result = execCmd(cmd)
        if len(result) !=0:
            if isinstance(lidar_type_dict[i],str):
                if(lidar_type_dict[i] == lidar_c_name):
                    print("check lidar is ",lidar_type_dict[i])
                    return lidar_c_name
            elif isinstance(lidar_type_dict[i],list): 
                if lidar_c_name in (lidar_type_dict[i]):
                    # print("check lidar in list is ",lidar_c_name)
                    return lidar_c_name

    print("can not find lidar, set ydlidar_g6 as default!")
    return "ydlidar_g6"    

def generate_launch_description():
    # Get the launch directory
    lidar_driver_transfer_dir = get_package_share_directory('lidar_driver_transfer')

    default_lidar_name = get_lidar_type()

    # Create the launch configuration variables
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')

    start_lidar_rviz = LaunchConfiguration('start_lidar_rviz')   

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
        
    declare_start_lidar_rviz = DeclareLaunchArgument(
        'start_lidar_rviz', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run start_lidar_rviz')

    declare_lidar_type_tel = DeclareLaunchArgument(
        'lidar_type_tel', 
        default_value=default_lidar_name,
        #choices=['ydlidar_g6', 'ydlidar_g2'],
        description='lidar type')
        
    lidar_type_launch = (lidar_driver_transfer_dir, '/launch/', lidar_type_tel, '.launch.py')
    lidar_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_type_launch))
    ])
    # Specify the actions
    # lidar_group = GroupAction([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(lidar_driver_transfer_dir, 'launch',
    #                                                    'ydlidar_g6.launch.py')),
    #         condition=LaunchConfigurationEquals('lidar_type_tel', 'ydlidar_g6'),
    #         launch_arguments={'start_lidar_rviz': start_lidar_rviz}.items()),

    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(lidar_driver_transfer_dir, 'launch',
    #                                                    'ydlidar_g2.launch.py')),
    #         condition=LaunchConfigurationEquals('lidar_type_tel', 'ydlidar_g2'),
    #         launch_arguments={'start_lidar_rviz': start_lidar_rviz}.items()),
    # ])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_start_lidar_rviz)
    ld.add_action(declare_lidar_type_tel)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(lidar_group)

    return ld
