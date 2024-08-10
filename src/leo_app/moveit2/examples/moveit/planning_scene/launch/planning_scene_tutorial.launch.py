from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Planning Scene Tutorial executable
    planning_scene_constraints = Node(
        name="planning_scene_constraints",
        package="planning_scene_constraints",
        executable="planning_scene_constraints",
        output="screen",
    )

    return LaunchDescription([planning_scene_constraints])
