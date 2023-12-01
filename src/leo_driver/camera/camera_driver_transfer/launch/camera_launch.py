import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch directory
    camera_driver_transfer_dir = get_package_share_directory('camera_driver_transfer')
    camera_type_tel = LaunchConfiguration('camera_type_tel')

    declare_camera_type_tel = DeclareLaunchArgument(
        'camera_type_tel', 
        default_value='d435',
        #choices=['d435', 'astra_pro'],
        description='camera type')        
    #camera_type_launch = camera_type_tel+'_launch.py'
    #camera_type_launch = "d435"+'.launch.py'
    camera_type_launch = (camera_driver_transfer_dir, '/launch/', camera_type_tel, '.launch.py')
    camera_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_type_launch))
    ])
    # Specify the actions
    # camera_group = GroupAction([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(camera_driver_transfer_dir, 'launch',  camera_type_launch))) #this is error                
    # ])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_camera_type_tel)
    ld.add_action(camera_group)

    return ld