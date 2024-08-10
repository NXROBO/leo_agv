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
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions
from typing import List, Optional, Text, Union
from launch import SomeSubstitutionsType
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch_ros.substitutions import FindPackageShare
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess

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
            default_value=TextSubstitution(text='aubo_C3'),
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
    xacro_file = os.path.join(pkg_path,'urdf','leo_aubo_agv_gazebo.urdf.xacro')
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

    
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'    


    declared_arg.append(  
    DeclareLaunchArgument(
        'start_description_rviz', 
        default_value='true', 
        choices=['true', 'false'],
        description=('Whether to start rviz'),
    )   
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz2_node],
        )
    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )
    
    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_leo",
        arguments=["-entity", "leo", "-topic", "robot_description"],
        output="screen",
    )

    leo_teleop_node = launch_ros.actions.Node(
        package='leo_teleop',
        executable='keyboard_control.sh',  
        output='screen',
        emulate_tty=True,
        )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        # parameters=[{'use_sim_time:true'}]
                        arguments=[
                                   '-topic', 'robot_description',
                                   '-entity', 'leo',
                                #    '-file',leo_description_dir+'/urdf/leo_agv.urdf.xacro'
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val, 
                                   '-Y', spawn_yaw_val,
                                #    '-flie',xacro_file
                                   ],
                        output='screen',
                        # remappings=[('/gazebo/spawn_entity','/spawn_entity')]
                        )
                        

    nodes_to_start = [
        robot_state_publisher_node,
        # rviz2_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        joint_state_publisher,
        # joint_state_broadcaster_spawner,
        # gazebo,
        start_gazebo_cmd,
        # gazebo_spawn_robot,
        spawn_entity,
        leo_teleop_node

    ]

    return LaunchDescription(declared_arg + nodes_to_start)
