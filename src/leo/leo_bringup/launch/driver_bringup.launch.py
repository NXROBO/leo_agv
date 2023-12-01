# Copyright 2022 Clearpath Robotics, Inc.
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
import launch

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
    leo_base_dir = get_package_share_directory('leo_base')
    leo_description_dir = get_package_share_directory('leo_description')
    camera_driver_transfer_dir = get_package_share_directory('camera_driver_transfer')
    lidar_driver_transfer_dir = get_package_share_directory('lidar_driver_transfer')
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')

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
    start_bringup_rviz = LaunchConfiguration('start_bringup_rviz')   
    joy_config = LaunchConfiguration('joy_config')
    config_filepath = LaunchConfiguration('config_filepath')
    start_joy = LaunchConfiguration('start_joy')

    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', 
        default_value='/dev/LeoBase',
        description='serial port name:/dev/LeoBase or /dev/ttyACM0')
    declare_enable_arm_tel = DeclareLaunchArgument(
        'enable_arm_tel', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run arm')
    declare_arm_type_tel = DeclareLaunchArgument(
        'arm_type_tel', 
        default_value='uarm',
        choices=['uarm', 'sagittarius_arm'],
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
        choices=['d435', 'astra_pro'],
        description='camera type')
    declare_lidar_type_tel = DeclareLaunchArgument(
        'lidar_type_tel', 
        default_value='ydlidar_g6',
        choices=['ydlidar_g2', 'ydlidar_g6'],
        description='lidar type')
    declare_dp_rgist = DeclareLaunchArgument(
        'dp_rgist', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run dp_rgist')
    declare_start_bringup_rviz = DeclareLaunchArgument(
        'start_bringup_rviz', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to start_bringup_rviz')    
    declare_joy_config = DeclareLaunchArgument(
        'joy_config', 
        default_value='xbox',
        #choices=['xbox', 'ps2'],
        description='joy type')
    declare_config_filepath = DeclareLaunchArgument(
        'config_filepath', 
        default_value=[launch.substitutions.TextSubstitution(text=os.path.join(get_package_share_directory('leo_teleop'), 'config', '')),joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')],
        description='config file path')
    declare_start_joy = DeclareLaunchArgument(
        'start_joy', 
        default_value='true',
        choices=['true', 'false'],
        description='whether to use joy')
    
    rviz_config_dir = os.path.join(get_package_share_directory('leo_bringup'), 'rviz', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(start_bringup_rviz),
        parameters=[{'use_sim_time': False}]
        )
    # Specify the actions
    bringup_cmd_group = GroupAction([
        # PushRosNamespace(
        #     condition=IfCondition(use_namespace),
        #     namespace=namespace),

        # Node(
        #     condition=IfCondition(use_composition),
        #     name='xxx',
        #     package='yyy',
        #     executable='zzz',
        #     parameters=[configured_params, {'autostart': autostart}],
        #     remappings=remappings,
        #     output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_description_dir, 'launch',
                                                       'leo_description.launch.py')),
            launch_arguments={'camera_type_tel': camera_type_tel,
                              'lidar_type_tel': lidar_type_tel,
                              'enable_arm_tel': enable_arm_tel,
                              'start_description_rviz' : 'false',
                              'arm_type_tel': arm_type_tel,}.items()),
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_base_dir, 'launch',
                                                       'leo_base.launch.py')),
            condition=IfCondition(start_base),
            launch_arguments={'serial_port': serial_port,}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(camera_driver_transfer_dir, 'launch',
                                                       'start_camera.launch.py')),
            condition=IfCondition(start_camera),
            launch_arguments={'dp_rgist': dp_rgist,}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_driver_transfer_dir, 'launch',
                                                       'start_lidar.launch.py')),
            condition=IfCondition(start_lidar)),
                              
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(teleop_twist_joy_dir, 'launch',
                                                       'teleop-launch.py')),
            condition=IfCondition(start_joy),                                                       
            launch_arguments={'joy_config': joy_config,
                              'config_filepath': config_filepath,}.items()),

    ])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_serial_port)
    ld.add_action(declare_enable_arm_tel)
    ld.add_action(declare_arm_type_tel)
    ld.add_action(declare_start_base)
    ld.add_action(declare_start_lidar)
    ld.add_action(declare_start_camera)
    ld.add_action(declare_camera_type_tel)
    ld.add_action(declare_lidar_type_tel)
    ld.add_action(declare_dp_rgist)
    ld.add_action(declare_start_bringup_rviz)
    ld.add_action(declare_joy_config)
    ld.add_action(declare_config_filepath)
    ld.add_action(declare_start_joy)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    ld.add_action(rviz_node)
    

    return ld
