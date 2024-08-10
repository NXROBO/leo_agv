# Copyright (c) 2022 NXROBO
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
#        Author: litian.zhuang   
#        Email: <litian.zhuang@nxrobo.com>  
#
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


from launch_ros.actions import Node


def generate_launch_description():
    
    leo_description_dir = get_package_share_directory('leo_description')

    # package_name='leo_gazebo' 
    # world_file_path = 'worlds/neighborhood.world'

    # pkg_path = os.path.join(get_package_share_directory(package_name))
    # world_path = os.path.join(pkg_path, world_file_path)  

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'
    
    leo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_description_dir, 'launch','leo_description_simple.launch.py')),
                        launch_arguments={'start_description_rviz' : 'true',
                                          'use_sim_time': 'false',
                                          'camera_type_tel': 'd435',
                                          'lidar_type_tel': 'ydlidar_g6'}.items())

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'leo',
                                #    '-file',leo_description_dir+'/urdf/leo_agv.urdf.xacro'
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val, 
                                   '-Y', spawn_yaw_val],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        leo,
        gazebo,
        spawn_entity,
    ])

