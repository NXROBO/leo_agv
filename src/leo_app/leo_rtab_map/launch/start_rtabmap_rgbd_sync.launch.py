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
from launch.actions import TimerAction

def generate_launch_description():
    # Get the launch directory
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_launch_file_dir = get_package_share_directory('nav2_bringup')

    leo_rtab_map_dir = get_package_share_directory('leo_rtab_map')
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
    start_slam_rviz = LaunchConfiguration('start_slam_rviz')   
    localization = LaunchConfiguration('localization')
    base_type = LaunchConfiguration('base_type')   # -----------新增---------------
    robot_ip = LaunchConfiguration('robot_ip')   # -----------新增---------------
    use_planning = LaunchConfiguration('use_planning')   # -----------新增---------------
    namespace = LaunchConfiguration('namespace')    # -----------新增------------
    tf_prefix = LaunchConfiguration("tf_prefix")     # -----------新增------------
    rtu_device_name = LaunchConfiguration("rtu_device_name") # -----------新增---------------


    rviz_config_dir = os.path.join(get_package_share_directory('leo_rtab_map'),
                                   'rviz', 'leo_rtabmap.rviz')
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Launch in localization mode.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_serial_port = DeclareLaunchArgument(
        'serial_port', 
        default_value='/dev/LeoBase',
        description='serial port name:/dev/LeoBase or /dev/ttyUSBx')
    declare_enable_arm_tel = DeclareLaunchArgument(
        'enable_arm_tel', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run arm')
    declare_arm_type_tel = DeclareLaunchArgument(
        'arm_type_tel', 
        default_value='uarm',
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
    declare_rviz_config = DeclareLaunchArgument(# -----------修改---------------
        'rviz_config', 
        default_value='leo_base.rviz',
        description='rviz_config')    
    
    declare_base_type = DeclareLaunchArgument(   # -----------新增---------------
        'base_type', 
        default_value='diff',
        choices=['diff', 'omni'],
        description='Leo base type')
    declare_robot_ip = DeclareLaunchArgument(   # -----------新增---------------
        'robot_ip', 
        default_value='192.168.47.101',
        description='choose Aubo IP')
    declare_namespace = DeclareLaunchArgument( # -----------新增---------------
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
    )
    declare_tf_prefix =DeclareLaunchArgument(# -----------新增---------------
            "tf_prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    declare_rtu_device_name = DeclareLaunchArgument(   # -----------新增---------------
        'rtu_device_name', 
        default_value='/dev/ttyUSB0,115200,N,8,0 ',
        description='Modbus RTU device info')
    declare_use_planning = DeclareLaunchArgument(   # -----------新增---------------
        'use_planning', 
        default_value='true',
        description='whether to use planing')
    declare_start_slam_rviz = DeclareLaunchArgument(
        'start_slam_rviz', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to start_slam_rviz')       
    
    declare_base_type = DeclareLaunchArgument(   # -----------新增---------------
        'base_type', 
        default_value='diff',
        choices=['diff', 'omni'],
        description='Leo base type')

    # Specify the group
    driver_bringup_group = GroupAction([
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
                              'base_type': base_type,   # -----------新增---------------
                              'robot_ip': robot_ip,     # -----------新增---------------
                              'tf_prefix':tf_prefix,    # -----------新增---------------
                              'rviz_config' : rviz_config, # -----------修改---------------
                              }.items()),

    ])
    map_nav_group = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_rtab_map_dir, 'launch',
            #                                           'leo_rtabmap_rgbd.launch.py')),
                                                       'leo_rtabmap_rgbd_sync.launch.py')),
            launch_arguments={'localization': localization,}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'launch',
                                                       'navigation_launch.py')),
        ),
    ])
 
    leo_teleop_node = launch_ros.actions.Node(
        package='leo_teleop',
        executable='keyboard_control.sh',  
        output='screen',
        emulate_tty=True,
        )


    leo_map_nav_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(enable_arm_tel),
        )

    leo_delay_map_nav_action = TimerAction(period=5.0, actions=[map_nav_group])
    #leo_delay_save_map_action = TimerAction(period=10.0, actions=[leo_save_map_node])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
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
    ld.add_action(declare_start_slam_rviz)
    ld.add_action(declare_localization)
    ld.add_action(declare_base_type)   # -----------新增---------------
    ld.add_action(declare_robot_ip)   # -----------新增---------------
    ld.add_action(declare_namespace)     # -----------新增---------------
    ld.add_action(declare_tf_prefix) # -----------新增---------------
    ld.add_action(declare_rtu_device_name)   # -----------新增---------------
    ld.add_action(declare_use_planning)   # -----------新增---------------


    ld.add_action(driver_bringup_group)
    ld.add_action(leo_teleop_node)
    ld.add_action(leo_map_nav_rviz_node)
    ld.add_action(leo_delay_map_nav_action)
    #ld.add_action(leo_delay_save_map_action)
    return ld
