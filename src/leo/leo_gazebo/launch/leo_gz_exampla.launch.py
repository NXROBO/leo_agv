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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    # Configure ROS nodes for launch

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
 

    enable_arm_tel =  LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('leo_gazebo')
    leo_description_dir = get_package_share_directory('leo_description')
    pkg_project_sim = get_package_share_directory('ros_gz_sim')

    pkg_path = os.path.join(get_package_share_directory('leo_description'))
    xacro_file = os.path.join(pkg_path,'urdf','leo_agv.urdf.xacro')
    
    bridge_file = os.path.join(pkg_project_bringup,'config','ros_gz_example_bridge.yaml')

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
           
        ]
    )


    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_bringup,
            'world',
            'Tugbot_in_Warehouse/'
            'tugbot_warehouse.sdf'])
        }.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    leo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_description_dir, 'launch','leo_description_simple.launch.py')),
                        launch_arguments={'start_description_rviz' : 'true',
                                          'use_sim_time': 'True',
                                          'camera_type_tel': 'd435',
                                          'lidar_type_tel': 'ydlidar_g6'}.items())


    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': "/home/nxrobo/leo_agv/src/leo_app/leo_gazebo/config/ros_gz_example_bridge.yaml",
            # 'config_file': bridge_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        *declared_arg, 
        gz_sim,
        bridge,
        leo
    ])


