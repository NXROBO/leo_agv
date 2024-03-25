from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='yolov8_object_detector', executable='yolov8_ros2_yolov8n_pt.py', output='screen', remappings = [('/camera/color/image_raw', '/camera/color/image_raw')]),
    ])