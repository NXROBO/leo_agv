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
from launch.event_handlers import OnProcessStart,OnProcessExit
import launch_ros

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    
    aubo_type = LaunchConfiguration("aubo_type")
    # use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="aubo_C3", package_name="aubo_description"
        )
        .robot_description_semantic(file_path=get_package_share_directory("leo_aubo_moveit_config")
            + "/srdf/leo_agv.srdf.xacro")
        .robot_description(file_path=get_package_share_directory("leo_description")
            + "/urdf/leo_aubo_agv.urdf.xacro"
            ,mappings={
                    # "robot_ip": robot_ip.perform(context),
                    "aubo_type": "aubo_C3",
                    # "use_fake_hardware": use_fake_hardware.perform(context)
                    }
            )
        .planning_pipelines("ompl", ["ompl",  "pilz_industrial_motion_planner"])
        .moveit_cpp(
            file_path=get_package_share_directory("motion_planning_python_tutorial")
            + "/config/motion_planning_python_api_tutorial.yaml"
        )
        .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    table_h = LaunchConfiguration('table_h')
    declare_table_h = DeclareLaunchArgument('table_h', default_value="'0.255'")

    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    
    declare_aubo_type = DeclareLaunchArgument(
            'aubo_type',
            default_value='aubo_C3',
            description='Description with aubo robot type')

    
    load_controllers = []
    for controller in [
        "joint_trajectory_controller",
        "joint_state_broadcaster",
        ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    # 任务执行节点
    run_grasp_node = launch_ros.actions.Node(
        package='leo_grasp',
        executable='grasp_start',  
        output='screen',
        parameters=[{'table_h':table_h,},moveit_config.to_dict()],
        )
   
    # 物体识别执行节点
    rgb_detect_node = launch_ros.actions.Node(
        package='leo_grasp',
        executable='rgb_detect',
        output='screen',
        )

    
    # 延迟2秒启动
    leo_delay_run_grasp_node = TimerAction(period=2.0, actions=[run_grasp_node])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_aubo_type)
    ld.add_action(declare_table_h)
    

    ld.add_action(rgb_detect_node)
    ld.add_action(leo_delay_run_grasp_node)

    return ld
