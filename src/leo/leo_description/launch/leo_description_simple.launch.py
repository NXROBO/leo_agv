# Copyright (c) 2022 NXROBO
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
#        Author: litian.zhuang   
#        Email: <litian.zhuang@nxrobo.com>  
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription,LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from typing import List, Optional, Text, Union
from launch import SomeSubstitutionsType
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    '''
      1.Declare the arguments in lauch flie
    '''

    declared_arg = []
    declared_arg.append(  
    DeclareLaunchArgument(
            'enable_arm_tel',
            default_value=TextSubstitution(text='false'),
            description=('Whether to enable arm'),
        )
    )
     

    declared_arg.append(  
        DeclareLaunchArgument(
            'arm_type_tel',
            default_value=TextSubstitution(text='uarm'),
            # default_value='uarm',
            description=('name of the arm.such as uarm , sagittarius_arm '),
        )
    )
       
    declared_arg.append(  
        DeclareLaunchArgument(
            'camera_type_tel',
            default_value=TextSubstitution(text='d435'),
            # default_value='d435',
            description=('choose the camera type. such as d435, astrapro'),
        )
    )
    

    declared_arg.append(  
    DeclareLaunchArgument(
            'lidar_type_tel',
            default_value=TextSubstitution(text='ydlidar_g6'),
            # default_value='ydlidar_g6',
            description='choose the lidar type. such as ydlidar_g6, ydlidar_g2',
        )
    )
    
    context = LaunchContext()

    enable_arm_tel =  LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')

    '''
    declared_arguments = {}
    declared_arguments.update({                               
                            'enable_arm_tel': str(enable_arm_tel), # 'Whether to enable arm'
                            'arm_type_tel': str(arm_type_tel), # 'name of the arm.such as uarm , sagittarius_arm'
                            'camera_type_tel': camera_type_tel, # 'choose the camera type. such as d435, astrapro'
                            'lidar_type_tel': str(lidar_type_tel), #choose the lidar type. such as ydlidar_g6, ydlidar_g2
                               })
    '''


    '''
    2.define the robot description
    '''

    pkg_path = os.path.join(get_package_share_directory('leo_description'))
    xacro_file = os.path.join(pkg_path,'urdf','leo_agv.urdf.xacro')
    # robot_description_file = xacro.process_file(xacro_file,mappings=declared_arguments)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "enable_arm_tel:=",
            enable_arm_tel,
            " ",
            "arm_type_tel:=",
            arm_type_tel,
            " ",
            "camera_type_tel:=",
            camera_type_tel,
            " ",
            "lidar_type_tel:=",
            lidar_type_tel,
            " ",
            'gazebo:=ignition',
            '',
           
        ]
    )
    
    '''
      3.deifine the node
    '''

    DeclareLaunchArgument(
        'start_description_rviz', 
        default_value='false', 
        choices=['true', 'false'],
        description=('Whether to start rviz'),
    )   


    rviz_config_dir = os.path.join(get_package_share_directory('leo_description'), 'rviz', 'urdf.rviz')
    start_description_rviz = LaunchConfiguration('start_description_rviz')   

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
                # 'robot_description': robot_description_file.toxml(),
                'robot_description': robot_description_content,
            }],
            output={'both': 'log'},
        )
    
    joint_state_publisher= Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(start_description_rviz),
        arguments=[
            '-d', rviz_config_dir,
        ],
        output={'both': 'log'},
    )

    nodes_to_start = [
        robot_state_publisher_node,
        rviz2_node,
        joint_state_publisher
    ]

    return LaunchDescription(declared_arg + nodes_to_start)
