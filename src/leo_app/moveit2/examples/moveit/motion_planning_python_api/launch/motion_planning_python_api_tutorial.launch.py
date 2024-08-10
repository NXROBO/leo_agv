"""
A launch file for running the motion planning python api tutorial
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import load_yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    # robot_ip = LaunchConfiguration("robot_ip")
    aubo_type = LaunchConfiguration("aubo_type")
    # use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=aubo_type.perform(context), package_name="aubo_description"
        )
        .robot_description_semantic(file_path=get_package_share_directory("aubo_description")
            + "/srdf/aubo.srdf.xacro")
        .robot_description(file_path=get_package_share_directory("aubo_description")
            + "/urdf/aubo_ES3.urdf.xacro"
            ,mappings={
                    # "robot_ip": robot_ip.perform(context),
                    "aubo_type": aubo_type.perform(context),
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

    example_file = DeclareLaunchArgument(
        "example_file",
        default_value="motion_planning_python_api_tutorial.py",
        # default_value="motion_planning_python_api_planning_scene.py",
        description="Python API tutorial file name",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="motion_planning_python_tutorial",
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("motion_planning_python_tutorial"),
        "config",
        "motion_planning_python_api_tutorial.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "aubo_base"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("aubo_description"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description,ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )

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
    nodes_to_start =[
            example_file,
            moveit_py_node,
            # robot_state_publisher,
            # ros2_control_node,
            # rviz_node,
            # static_tf,
        # ]+ load_controllers
    ]
    return nodes_to_start


def generate_launch_description():
    # declare parameter for using robot ip
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_C3",
            description='Description with aubo robot type.',
        )
    )


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])