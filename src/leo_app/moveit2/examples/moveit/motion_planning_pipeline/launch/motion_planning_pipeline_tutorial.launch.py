# Copyright 2024 AUBO Robotics
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="aubo_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="aubo.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<tf_prefix>/"',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="/",
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    namespace = LaunchConfiguration("namespace")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "description_package:=",
            description_package,
            " ",
            "namespace:=",
            namespace,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "srdf", "aubo.srdf.xacro"]
            ),
            " ",
            "name:=",
            "aubo",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "description_package:=",
            description_package,
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "aubo_joint_limits.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "pilz_cartesian_limits.yaml",
        ]
    )

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(description_package), "moveit2", "kinematics.yaml"]
    )

    planning_pipelines_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "ompl_planning.yaml",
        ]
    )

    moveit_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "aubo_moveit_controller_config.yaml",
        ]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            ParameterFile(robot_description_planning_joint_limits, allow_substs=True),
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            ParameterFile(moveit_controllers, allow_substs=True),
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning_cartesian_limits,
            ParameterFile(robot_description_planning_joint_limits, allow_substs=True),
            robot_description_kinematics,
            planning_pipelines_config,
            ompl_planning_config,
        ],
        condition=IfCondition(launch_rviz),
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "aubo_base"],
    )

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    motion_planning_pipeline_tutorial = Node(
        package="motion_planning_pipeline_tutorial",
        executable="motion_planning_pipeline_tutorial",
        name="motion_planning_pipeline_tutorial",
        output="both",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ParameterFile(robot_description_planning_joint_limits, allow_substs=True),
            planning_pipelines_config,
            ompl_planning_config,
            planning_plugin,
        ],
    )

    nodes = [
        move_group_node,
        rviz_node,
        static_tf_node,
        motion_planning_pipeline_tutorial,
    ]

    return LaunchDescription(declared_arguments + nodes)
