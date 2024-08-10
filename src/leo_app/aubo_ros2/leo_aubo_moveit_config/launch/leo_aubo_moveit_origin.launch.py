# Copyright (c) 2024 AUBO, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from leo_aubo_moveit_config.launch_common import *
from ament_index_python.packages import get_package_share_directory
def launch_setup(context, *args, **kwargs):
    leo_bringup_dir = get_package_share_directory('leo_bringup')
    

    # Initialize Arguments
    aubo_type = LaunchConfiguration("aubo_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    namespace = LaunchConfiguration('namespace')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    controllers_file = LaunchConfiguration("controllers_file")
    # General arguments
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    robot_ip = LaunchConfiguration("robot_ip")
    rtu_device_name = LaunchConfiguration("rtu_device_name")
    base_type = LaunchConfiguration('base_type')   # -----------新增---------------

   
    leo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(leo_bringup_dir,'launch',
                                                   'driver_bringup_no_description.launch.py'),),
        launch_arguments={
            'start_bringup_rviz': 'false',
            'arm_type_tel': aubo_type,
            'base_type': base_type,   # -----------新增---------------

        }.items(),
    )
    robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            " ", 
            PathJoinSubstitution(
                [FindPackageShare("leo_description"), "urdf", "leo_aubo_agv.urdf.xacro"]
            ),
            ' ',
            'tf_prefix:=',
            tf_prefix,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'rtu_device_name:=',
            rtu_device_name,
            ' ',
            'aubo_type:=',
            aubo_type,
            ' ',
            ])
    
    pkg_path = os.path.join(get_package_share_directory('leo_description'))
    xacro_file = os.path.join(pkg_path,'urdf','leo_agv.urdf.xacro')
    robot_description_content1 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            ' ',
            'tf_prefix:=',
            tf_prefix,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'rtu_device_name:=',
            rtu_device_name,
            ' ',
            'aubo_type:=',
            aubo_type,
            ' ',
        ])    

    robot_description = {"robot_description": robot_description_content}

    print("-----------",robot_description,"--------------")

    # MoveIt Configuration
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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package),
            "config",
            controllers_file
        ]
    )

    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "aubo_joint_limits.yaml",
        ]
    )

    # Planning Configuration
    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "ompl_planning.yaml",
        ]
    )
    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "config", "pilz_cartesian_limits.yaml",
        ]
    )
    
    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("leo_aubo_moveit_config", "config/"  + "aubo_controllers.yaml")

    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = context.perform_substitution(use_fake_hardware)
    if change_controllers == "true":
        controllers_yaml["io_and_status_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True


    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package),
            "config", "aubo_moveit_controller_config.yaml"]
    )
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.0, # 1.2
        "trajectory_execution.allowed_goal_duration_margin": 0.5, # 0.5
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ParameterFile(robot_description_planning_joint_limits, allow_substs=True),
            robot_description_planning_cartesian_limits,
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            ParameterFile(moveit_controllers, allow_substs=True),
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_planning),
    )
    # Get parameters for the Servo node
    servo_params = PathJoinSubstitution([
            FindPackageShare(moveit_config_package),
            'config',
            'aubo_servo_config.yaml',
        ]
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_kinematics,
            robot_description_semantic,
            ParameterFile(servo_params, allow_substs=True),
        ],
        condition=IfCondition(use_servoing),
    )



    aubo_control_node = Node(
        package='aubo_hardware',
        executable='aubo_ros2_control_node',
        parameters=[robot_description, 
                    ParameterFile(robot_controllers, allow_substs=True),
                    ],
        output='both',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )
    # # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(moveit_config_package), "rviz", "moveit.rviz"]
    # )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("leo_grasp"), "rviz", "leo_grasp.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
    )

    # Spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags,
            condition=UnlessCondition(use_fake_hardware),
        )
    controller_spawner_names = [
        "io_and_status_controller",
    ]
    controller_spawner_inactive_names = ["forward_command_controller_position"]

    # robot_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[robot_controller, '--controller-manager', [namespace, 'controller_manager']],
    # )
    gpio_controller_spawner = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
    )



    delay_joint_state_broadcaster_spawner_after_aubo_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=aubo_control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_fake_hardware),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_joint_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name='moveit_servo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='aubo_moveit2',
                plugin='aubo_servo::JoyToServoPub',
                name='controller_to_servo_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        condition=IfCondition(use_servoing),
    )

    nodes_to_start = [ 
        aubo_control_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_aubo_control_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_controller_spawner_after_joint_state_broadcaster_spawner,
        container,
     servo_node,
     move_group_node,
     leo_launch,
     ] + gpio_controller_spawner

    nodes_to_start1 = [ 
        robot_state_pub_node,
        rviz_node
     ]

    return nodes_to_start


def generate_launch_description():
    
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_C3",
            description="Type/series of used UR robot.",
            choices=["aubo_C3", "aubo_ES3"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="169.254.3.8",
            description="IP of robot computer. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="aubo_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rtu_device_name',
            default_value='/dev/ttyUSB0,115200,N,8,1',
            description='Modbus RTU device info',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_servoing',
            default_value='false',
            description='Start robot with Moveit2 servoing.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="leo_aubo_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(   # -----------新增---------------
            'base_type', 
            default_value='diff',
            choices=['diff', 'omni'],
            description='Leo base type',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
