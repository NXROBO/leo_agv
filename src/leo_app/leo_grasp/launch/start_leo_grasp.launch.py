# Copyright (c) 2024 NXROBO
#
# /* Author: haijie.huo */
# /* email: haijie.huo@nxrobo.com */
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
    leo_grasp_dir = get_package_share_directory('leo_grasp')
    table_h = LaunchConfiguration('table_h')
    declare_table_h = DeclareLaunchArgument('table_h', default_value="'0.255'")

    # 启动视觉抓取
    keyboard_group = GroupAction([
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource(os.path.join(leo_grasp_dir, 'launch','grasp_start.launch.py')),
                      launch_arguments={
                'table_h': table_h,}.items(),
        ),
])

    # 延迟2秒启动
    leo_delay_keyboard_action = TimerAction(period=2.0, actions=[keyboard_group])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_table_h)
    ld.add_action(leo_delay_keyboard_action)

    return ld
