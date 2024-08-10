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
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue


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
    
    declared_arg.append(
    DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    declared_arg.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.47.101",
            description="IP of robot computer. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arg.append(
        DeclareLaunchArgument(
            'rtu_device_name',
            default_value='/dev/ttyUSB0,115200,N,8,1',
            description='Modbus RTU device info',
        )
    )
    declared_arg.append(
        DeclareLaunchArgument(
            'base_type_tel',
            default_value='leo_base_normal',
            description='choose leo base type urdf',
        )
    )
    declared_arg.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )
    declared_arg.append(
        DeclareLaunchArgument(
            'start_description_rviz', 
            default_value='false', 
            choices=['true', 'false'],
            description=('Whether to start rviz'),
        )   
    )

    declared_arg.append(
        DeclareLaunchArgument(# -----------修改---------------
        'rviz_config', 
        default_value='leo_base.rviz',
        description='rviz_config',
        )
    )   


    enable_arm_tel =  LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')
    tf_prefix = LaunchConfiguration("tf_prefix")
    robot_ip = LaunchConfiguration("robot_ip")
    rtu_device_name = LaunchConfiguration("rtu_device_name")
    base_type_tel = LaunchConfiguration("base_type_tel")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    start_description_rviz = LaunchConfiguration('start_description_rviz') 
    rviz_config = LaunchConfiguration('rviz_config')    


    '''
    2.define the robot description
    '''

    pkg_path = os.path.join(get_package_share_directory('leo_description'))
    xacro_file = os.path.join(pkg_path,'urdf','leo_agv.urdf.xacro')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            ' ',
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
            'tf_prefix:=',
            tf_prefix,
            " ",
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'rtu_device_name:=',
            rtu_device_name,
            ' ',
            'base_type_tel:=',
            base_type_tel,
            ' ',
        ])


    robot_description = {"robot_description": robot_description_content}
    moveit_config_package = 'leo_aubo_moveit_config'
    robot_description_semantic_content = ParameterValue(
        Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", "leo_agv.srdf.xacro"]
            ),
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    ),value_type=str)

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content} 

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    
    '''
      3.deifine the node
    '''
    # 构建 rviz_config_dir 路径
    rviz_config_dir = PathJoinSubstitution([
        get_package_share_directory('leo_description'), 
        'rviz', 
        rviz_config
    ])
     

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # namespace='/',
        # condition=UnlessCondition(enable_arm_tel),
        parameters=[{
                # 'robot_description': robot_description_file.toxml(),
                'robot_description': robot_description_content,
            }],
            output={'both': 'log'},
        )
    
    joint_state_publisher= Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(enable_arm_tel)
        )
        


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(enable_arm_tel),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
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
