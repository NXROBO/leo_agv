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
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.actions import Node
from launch.conditions import IfCondition,UnlessCondition

def generate_launch_description():
    # Get the launch directory
    leo_teleop_dir = get_package_share_directory('leo_teleop')
    leo_bringup_dir = get_package_share_directory('leo_bringup')

    # Create the launch configuration variables
    serial_port = LaunchConfiguration('serial_port')
    enable_arm_tel = LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    start_base = LaunchConfiguration('start_base')
    start_camera = LaunchConfiguration('start_camera')
    start_lidar = LaunchConfiguration('start_lidar')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')   
    dp_rgist = LaunchConfiguration('dp_rgist')   
    rviz_config = LaunchConfiguration('rviz_config')   
    use_sim_time = False

    rviz_config_dir = os.path.join(get_package_share_directory('leo_slam_transfer'),
                                   'rviz', 'leo_slam.rviz')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', 
        default_value='/dev/LeoBase',
        description='serial port name:/dev/ttyACM0 or /dev/LeoBase')
    declare_enable_arm_tel = DeclareLaunchArgument(
        'enable_arm_tel', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run arm')
    declare_arm_type_tel = DeclareLaunchArgument(
        'arm_type_tel', 
        default_value='aubo_ES3',
        description='arm name')
    declare_start_base = DeclareLaunchArgument(
        'start_base', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run base')
    declare_start_camera = DeclareLaunchArgument(
        'start_camera', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run camera')
    declare_start_lidar = DeclareLaunchArgument(
        'start_lidar', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run lidar')
    declare_camera_type_tel = DeclareLaunchArgument(
        'camera_type_tel', 
        default_value='d435',
        #choices=['d435', 'astra_pro'],
        description='camera type')
    declare_lidar_type_tel = DeclareLaunchArgument(
        'lidar_type_tel', 
        default_value='ydlidar_g6',
        #choices=['ydlidar_g2', 'ydlidar_g6'],
        description='lidar type')
    declare_dp_rgist = DeclareLaunchArgument(
        'dp_rgist', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run dp_rgist')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', 
        default_value='leo_base.rviz',
        description='rviz_config')  

   
    # Specify the actions
    camera_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_bringup_dir, 'launch',
                                                       'driver_bringup.launch.py')),
            launch_arguments={'serial_port': serial_port,
                              'enable_arm_tel': enable_arm_tel,
                              'arm_type_tel': arm_type_tel,
                              'start_base' : start_base,
                              'start_camera': start_camera,
							  'start_lidar': start_lidar,
                              'camera_type_tel' : camera_type_tel,
                              'lidar_type_tel': lidar_type_tel,
							  'dp_rgist': dp_rgist,
                              'rviz_config' : rviz_config,}.items()),
    ])
    
    leo_teleop_node = launch_ros.actions.Node(
        package='leo_teleop',
        executable='keyboard_control.sh',  
        output='screen',
        emulate_tty=True,
        )
    
    leo_test_node = launch_ros.actions.Node(
        package='leo_test',
        executable='leo_aging_test',  
        output='screen',
        )
    
    yolov8_pose_node = launch_ros.actions.Node(
        package='leo_yolov8',
        executable='camera_object',  
        output='screen',
        )

    gmapping_slam_node = launch_ros.actions.Node(
        package='slam_gmapping',
        executable='slam_gmapping',  
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}]
        )

    leo_slam_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(enable_arm_tel),
        )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_serial_port)
    ld.add_action(declare_enable_arm_tel)
    ld.add_action(declare_arm_type_tel)
    ld.add_action(declare_start_base)
    ld.add_action(declare_start_lidar)
    ld.add_action(declare_start_camera)
    ld.add_action(declare_camera_type_tel)
    ld.add_action(declare_lidar_type_tel)
    ld.add_action(declare_dp_rgist)
    ld.add_action(declare_rviz_config)
    ld.add_action(camera_group)
    # ld.add_action(leo_teleop_node)
    ld.add_action(leo_test_node)
    ld.add_action(yolov8_pose_node)
    ld.add_action(gmapping_slam_node)
    ld.add_action(leo_slam_rviz_node)
    return ld
