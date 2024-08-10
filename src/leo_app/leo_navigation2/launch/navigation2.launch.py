# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition,UnlessCondition
from launch.event_handlers import OnProcessStart,OnProcessExit


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_arm_tel = LaunchConfiguration('enable_arm_tel')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('leo_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = 'leo_navigation.yaml'
    collision_param_file_name = 'collision_monitor_params.yaml'

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('leo_navigation2'),
            'param',
            param_file_name))

    collision_param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('leo_navigation2'),
            'param',
            collision_param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_collision_launch_file_dir = os.path.join(get_package_share_directory('nav2_collision_monitor'), 'launch')

    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('nav2_bringup'),
    #     'rviz',
    #     'nav2_default_view.rviz')

    rviz_config_dir = os.path.join(
        get_package_share_directory('leo_navigation2'),
        'rviz',
        'leo_navigation2.rviz')

    declare_map_type_tel = DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load')
    
    declare_parms_file = DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load')

    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    declare_enable_arm_tel = DeclareLaunchArgument(
            'enable_arm_tel', 
            default_value='false',
            choices=['true', 'false'],
            description='Whether to run arm')

    nav2_bringup_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'namespace':'',
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
    ])

    rviz2_node_group = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=UnlessCondition(enable_arm_tel),
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])


    ####################################################
    ####################################################

    collision_monitor_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_collision_launch_file_dir, '/collision_monitor_node.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': collision_param_dir}.items(),
        ),
        ])
    leo_delay_nav2_action = TimerAction(period=20.0, actions=[collision_monitor_group])

    # # \u521b\u5efa\u4e00\u4e2a ExecuteProcess \u5b9e\u4f8b\u4f5c\u4e3a delayed_action_on_start \u7684 on_exit \u53c2\u6570
    # # \u5728 nav2_bringup_group \u542f\u52a8\u540e\u6267\u884c collision_monitor_group \u4e2d\u7684\u64cd\u4f5c
    # collision_monitor_process = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'nav2_collision_monitor', 'collision_monitor_node.launch.py','use_sim_time',use_sim_time,'params_file',collision_param_dir],
    #     output='screen',
    # )

    # # \u6ce8\u518c\u4e00\u4e2a\u4e8b\u4ef6\u5904\u7406\u5668\uff0c\u5728\u6240\u6709 nav2_bringup_group \u8fdb\u7a0b\u542f\u52a8\u540e\u8c03\u7528 create_delayed_action \u51fd\u6570
    # delayed_action_on_start = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=nav2_bringup_group,  # \u76d1\u542c nav2_bringup_group \u7684\u542f\u52a8
    #         on_exit=[collision_monitor_process],  # \u5728 nav2_bringup_group \u542f\u52a8\u540e\u8c03\u7528 create_delayed_action \u51fd\u6570
    #     )
    # )

    ####################################################
    ####################################################


    ld = LaunchDescription()
    ld.add_action(declare_map_type_tel)
    ld.add_action(declare_parms_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_arm_tel )
    ld.add_action(nav2_bringup_group)
    ld.add_action(rviz2_node_group)
    ld.add_action(leo_delay_nav2_action)

    return ld

