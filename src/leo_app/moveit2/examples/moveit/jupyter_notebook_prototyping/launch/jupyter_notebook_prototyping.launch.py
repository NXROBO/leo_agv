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
import json
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    aubo_type = LaunchConfiguration("aubo_type")
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=aubo_type.perform(context), package_name="aubo_description"
        )
        # .trajectory_execution(file_path="moveit2/aubo_moveit_controller_config.yaml")
        .robot_description_semantic(file_path=get_package_share_directory("aubo_description") 
            + "/srdf/aubo.srdf.xacro")
        .robot_description(file_path=get_package_share_directory("aubo_description") 
            + "/urdf/aubo.urdf.xacro"
            ,mappings={
                    "robot_ip": robot_ip.perform(context),
                    "aubo_type": aubo_type.perform(context),
                    "use_fake_hardware": use_fake_hardware.perform(context)
                    }
            )
        .planning_pipelines("ompl", ["ompl",  "pilz_industrial_motion_planner"])
        .moveit_cpp(
            file_path=get_package_share_directory("jupyter_python_tutorial")
            + "/config/jupyter_notebook_prototyping.yaml"
        )
        .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    # with open("moveitconfigs_param.json", "w") as file:
    #     json.dump(moveit_config.to_dict(), file)


    rviz_config_file = os.path.join(
        get_package_share_directory("jupyter_python_tutorial"),
        "config",
        "jupyter_notebook_prototyping.rviz",
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
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    aubo_control_node = Node(
        package='aubo_hardware',
        executable='aubo_ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path
        ],
        output='both',
        condition=UnlessCondition(use_fake_hardware),
    )

    load_controllers = []
    controllers_name =[]
    if use_fake_hardware.perform(context):
        controllers_name = [
        "joint_trajectory_controller",
        "joint_state_broadcaster",
        ]
    else:
        controllers_name = [
        "joint_trajectory_controller",
        "io_and_status_controller",
        "joint_state_broadcaster",
        ]
    for controller in controllers_name:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]
    # We can start a notebook from a launch file
    notebook_dir = os.path.join(get_package_share_directory("jupyter_python_tutorial"), "src")
    start_notebook = ExecuteProcess(
        cmd=["cd {} && python3 -m notebook".format(notebook_dir)],
        shell=True,
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    nodes_to_start =[
            start_notebook,
            move_group_node,
            robot_state_publisher,
            control_node,
            aubo_control_node,
            rviz_node,
            static_tf,
        ]+ load_controllers
    return nodes_to_start


def generate_launch_description():
    # declare parameter for using robot ip
    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1",
        description="Robot IP",
    ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_C3",
            description='Description with aubo robot type.',
        )
    )

    # declare parameter for using fake controller
    declared_arguments.append(
        DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true", # default to fake hardware (Important: that user is explicit with intention of launching real hardware!)
        description="Use fake hardware",
    ))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
