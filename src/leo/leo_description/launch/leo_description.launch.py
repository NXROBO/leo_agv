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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from typing import List, Optional, Text, Union
from launch import LaunchContext, SomeSubstitutionsType
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


class DeclareSparkRobotDescriptionLaunchArgument(DeclareLaunchArgument):
    def __init__(
        self,
        *,
        default_value: Optional[SomeSubstitutionsType] = Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('leo_description'),
                'urdf',
                'leo_agv'
            ]), '.urdf.xacro ',
            'enable_arm_tel:=', LaunchConfiguration('enable_arm_tel'), ' ',
            'arm_type_tel:=', LaunchConfiguration('arm_type_tel'), ' ',
            'camera_type_tel:=', LaunchConfiguration('camera_type_tel'), ' ',
            'lidar_type_tel:=', LaunchConfiguration('lidar_type_tel'), ' ',
            'base_type_tel:=', LaunchConfiguration('base_type_tel'), ' ',
        ]),
        **kwargs
    ) -> None:
        super().__init__(
            name='robot_description',
            default_value=default_value,
            description=(
                'urdf of SPARK , the xacro command, support the DeclareLaunchArgument.'
            ),
            choices=None,
            **kwargs
        )


def declare_leo_robot_description_launch_arguments(
    *,
    enable_arm_tel: Text = 'false',
    arm_type_tel: Text = 'uarm',
    camera_type_tel: Text = 'd435',
    lidar_type_tel: Text = 'ydlidar_g6',
) -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'enable_arm_tel',
            default_value=enable_arm_tel,
            choices=('true', 'false'),
            description=('whether to enable the arm-model displayed'),
        ),
        DeclareLaunchArgument(
            'arm_type_tel',
            default_value=arm_type_tel,
            description=('choose the arm type. such as uarm , sagittarius_arm'),
        ),
        DeclareLaunchArgument(
            'camera_type_tel',
            default_value=camera_type_tel,
            description=(
                'choose the camera type. such as d435, astrapro'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_type_tel',
            default_value=lidar_type_tel,
            description=(
                'choose the lidar type. such as ydlidar_g6, ydlidar_g2'
            ),
        ),
        DeclareSparkRobotDescriptionLaunchArgument(),
    ]

#==================================================================================

def launch_setup(context, *args, **kwargs):

    rviz_config_dir = os.path.join(get_package_share_directory('leo_description'), 'rviz', 'urdf.rviz')
    enable_arm_tel = LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')
    robot_description = LaunchConfiguration('robot_description')
    start_description_rviz = LaunchConfiguration('start_description_rviz')   
 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
        }],
        output={'both': 'log'},
    )

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

    return [
        robot_state_publisher_node,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
         DeclareLaunchArgument(
            'start_description_rviz', 
            default_value='false',
            choices=['true', 'false'],
            description='Whether to start_description_rviz'
        )   
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_arm_tel',
            default_value='false',
            choices=['true', 'false'],
            description='model type of the leo Arm such as `uarm` or `sgr532`.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_type_tel',
            default_value='uarm',
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_type_tel',
            default_value='d435',
            description='model type of the leo Arm such as `d435` or `astra`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_type_tel',
            default_value='ydlidar_g6',
            description='model type of the leo lidar such as `ydlidar_g6` or `ydlidar_g2`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_type_tel',
            default_value='leo_base_normal',
            description='choose leo base type urdf',
        )
    )

    declared_arguments.extend(
        declare_leo_robot_description_launch_arguments(),
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
