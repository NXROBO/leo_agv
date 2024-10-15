# Copyright 2022 Clearpath Robotics, Inc.
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
import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import pyrealsense2 as rs

# 获取D435 摄像头设备号
num = []
ctx = rs.context()
if len(ctx.devices) > 0:
    for d in ctx.devices:
        # print(d.get_info(rs.camera_info.serial_number))
        num.append(d.get_info(rs.camera_info.serial_number))
else:
    print("No Intel Device connected")

if len(num) > 1:
    start_camera = 'false'   # false
else:
    start_camera = 'true'
    num = [" ", " "]

# 提取驱动文件内容
with open('/opt/leo_driver.txt', 'r', encoding='utf-8') as file:
    lines = file.readlines()  
second_string = lines[1].strip() 
if second_string != "None":
    start_device = 'leo_base'
else:
    start_device = 'leo_base_normal'
    
leo_bringup_dir = get_package_share_directory('leo_bringup')
leo_bringup_dir = os.path.join(*leo_bringup_dir.split(os.sep)[:4])
with open('/'+leo_bringup_dir+'/doc/camera.txt', 'r') as file:
    lines = file.readlines()
servo1 = lines[1].strip()  
servo2 = lines[2].strip()


def generate_launch_description():
    # --------------------Get the launch directory--------------------
    leo_base_dir = get_package_share_directory('leo_base')
    leo_description_dir = get_package_share_directory('leo_description')
    camera_driver_transfer_dir = get_package_share_directory('camera_driver_transfer')
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')  # -----------新增---------------
    moveit_dir = get_package_share_directory('leo_aubo_moveit_config')  # -----------新增---------------
    aubo_moveit_dir = get_package_share_directory('aubo_bringup')  # -----------新增---------------
    lidar_driver_transfer_dir = get_package_share_directory('lidar_driver_transfer')
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')

    # --------------------Create the launch configuration variables--------------------
    serial_port = LaunchConfiguration('serial_port')
    enable_arm_tel = LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    start_base = LaunchConfiguration('start_base')
    # start_camera = LaunchConfiguration('start_camera')
    start_lidar = LaunchConfiguration('start_lidar')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')   
    dp_rgist = LaunchConfiguration('dp_rgist')   
    rviz_config = LaunchConfiguration('rviz_config')   
    joy_config = LaunchConfiguration('joy_config')
    config_filepath = LaunchConfiguration('config_filepath')
    start_joy = LaunchConfiguration('start_joy')
    base_type = LaunchConfiguration('base_type')   # -----------新增---------------
    robot_ip = LaunchConfiguration('robot_ip')   # -----------新增---------------
    use_planning = LaunchConfiguration('use_planning')   # -----------新增---------------
    rtu_device_name = LaunchConfiguration("rtu_device_name") # -----------新增---------------
    namespace = LaunchConfiguration('namespace')    # -----------新增------------
    tf_prefix = LaunchConfiguration("tf_prefix")     # -----------新增------------
    leo_base_tel = LaunchConfiguration('leo_base_tel')
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    start_description_rviz = LaunchConfiguration('start_description_rviz')   
    


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', 
        default_value='/dev/LeoBase',
        description='serial port name:/dev/LeoBase or /dev/ttyACM0')
    declare_enable_arm_tel = DeclareLaunchArgument(
        'enable_arm_tel', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run arm')
    declare_arm_type_tel = DeclareLaunchArgument(
        'arm_type_tel', 
        default_value='aubo_ES3',
        description='arm name')
    declare_start_base = DeclareLaunchArgument(
        'start_base', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run base')
    declare_start_camera = DeclareLaunchArgument(
        'start_camera', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run camera')
    declare_start_lidar = DeclareLaunchArgument(
        'start_lidar', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run lidar')
    declare_camera_type_tel = DeclareLaunchArgument(
        'camera_type_tel', 
        default_value='d435',
        choices=['d435', 'astra_pro'],
        description='camera type')
    declare_lidar_type_tel = DeclareLaunchArgument(
        'lidar_type_tel', 
        default_value='ydlidar_g6',
        choices=['ydlidar_g2', 'ydlidar_g6'],
        description='lidar type')
    declare_dp_rgist = DeclareLaunchArgument(
        'dp_rgist', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run dp_rgist')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', 
        default_value='leo_base.rviz',
        description='rviz_config')    
    declare_joy_config = DeclareLaunchArgument(
        'joy_config', 
        default_value='xbox',
        #choices=['xbox', 'ps2'],
        description='joy type')
    declare_config_filepath = DeclareLaunchArgument(
        'config_filepath', 
        default_value=[launch.substitutions.TextSubstitution(text=os.path.join(get_package_share_directory('leo_teleop'), 'config', '')),joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')],
        description='config file path')
    declare_start_joy = DeclareLaunchArgument(
        'start_joy', 
        default_value='true',
        choices=['true', 'false'],
        description='whether to use joy')
    
    declare_base_type = DeclareLaunchArgument(   # -----------新增---------------
        'base_type', 
        default_value='diff',
        choices=['diff', 'omni'],
        description='Leo base type')
    declare_leo_base_tel = DeclareLaunchArgument(   # -----------新增---------------
        'leo_base_tel', 
        default_value='leo_base_normal',
        description='choose Leo base urdf')
    declare_robot_ip = DeclareLaunchArgument(   # -----------新增---------------
        'robot_ip', 
        default_value='192.168.47.101',
        description='choose Aubo IP')
    declare_use_planning = DeclareLaunchArgument(   # -----------新增---------------
        'use_planning', 
        default_value='true',
        description='whether to use planing')
    declare_rtu_device_name = DeclareLaunchArgument(   # -----------新增---------------
        'rtu_device_name', 
        default_value='/dev/ttyS7,115200,N,8,0 ',
        description='Modbus RTU device info')
    declare_namespace = DeclareLaunchArgument( # -----------新增---------------
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
    )
    declare_tf_prefix =DeclareLaunchArgument(# -----------新增---------------
            "tf_prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    declared_use_fake_hardware = DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    declared_moveit_config_package=DeclareLaunchArgument(
            "moveit_config_package",
            default_value="leo_aubo_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    

    # Specify the actions
    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_description_dir, 'launch',
                                                       'leo_description_simple.launch.py')),
            launch_arguments={'camera_type_tel': camera_type_tel,
                              'lidar_type_tel': lidar_type_tel,
                              'enable_arm_tel': enable_arm_tel, # 根据是否有机械臂显示不同的模型
                              'rviz_config' : rviz_config,
                              'tf_prefix':tf_prefix,
                              'rtu_device_name':rtu_device_name,
                              'arm_type_tel': arm_type_tel,
                              'robot_ip': robot_ip,
                              'base_type_tel': start_device,
                              'namespace':namespace,
                              }.items(),
            # condition=UnlessCondition(enable_arm_tel) # 判断是否有机械臂
            ), 
            
# -------------------------------------------------------------------新增 or 改动------------------------------------------------------------------------------------
        # 差速底盘驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_base_dir, 'launch',
                                                       'leo_base.launch.py')),
            condition=LaunchConfigurationEquals('base_type', 'diff'),
            launch_arguments={'serial_port': serial_port,}.items()),

        # 麦轮底盘驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(leo_base_dir, 'launch',
                                                       'leo_base.launch.py')),
            condition=LaunchConfigurationEquals('base_type', "omni"),
            launch_arguments={'serial_port': serial_port,}.items()),


        # 单相机
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(camera_driver_transfer_dir, 'launch',
                                                    'start_camera.launch.py')),
            condition=IfCondition(start_camera),
            launch_arguments={'dp_rgist': dp_rgist,}.items()),
        
        # 双相机
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(camera_driver_transfer_dir, 'launch',
                                                    'rs_multi_camera.launch.py')),
            condition=UnlessCondition(start_camera),
            launch_arguments={'serial_no1': "_"+servo2,
                                            'serial_no2': "_"+servo1,
                                            }.items()),
                    
# -------------------------------------------------------------------------------------------------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_driver_transfer_dir, 'launch',
                                                       'start_lidar.launch.py')),
            condition=IfCondition(start_lidar)),
                              
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(teleop_twist_joy_dir, 'launch',
                                                       'teleop-launch.py')),
            condition=IfCondition(start_joy),                                                       
            launch_arguments={'joy_config': joy_config,
                              'config_filepath': config_filepath,}.items()),

    ])

    arm_group = GroupAction([#  机械臂驱动             
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit_dir, 'launch',
                                                    'leo_aubo_moveit.launch.py')),
            launch_arguments={
                        'enable_arm_tel': enable_arm_tel, # 根据是否有机械臂显示不同的模型
                        'arm_type_tel': arm_type_tel,
                        'camera_type_tel': camera_type_tel,
                        'lidar_type_tel': lidar_type_tel,
                        'robot_ip': robot_ip,
                        'rtu_device_name':rtu_device_name,
                        'base_type_tel': start_device,
                        'tf_prefix':tf_prefix,
                        'namespace':namespace,
                        'use_planning':use_planning,
                    }.items(),

            condition=IfCondition(enable_arm_tel)), # 判断是否有机械臂
    ])
    arm_delay_moveit_action = TimerAction(period=5.0,actions=[arm_group])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_serial_port)
    ld.add_action(declare_enable_arm_tel)
    ld.add_action(declare_arm_type_tel)
    ld.add_action(declare_start_base)
    ld.add_action(declare_start_lidar)
    ld.add_action(declare_start_camera)
    ld.add_action(declare_camera_type_tel)
    ld.add_action(declare_lidar_type_tel)
    ld.add_action(declare_dp_rgist)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_joy_config)
    ld.add_action(declare_config_filepath)
    ld.add_action(declare_start_joy)
    
    
    ld.add_action(declare_base_type)   # -----------新增---------------
    ld.add_action(declare_leo_base_tel)   # -----------新增---------------
    ld.add_action(declare_robot_ip)   # -----------新增---------------
    ld.add_action(declare_use_planning)   # -----------新增---------------
    ld.add_action(declare_rtu_device_name)   # -----------新增---------------
    ld.add_action(declare_namespace)     # -----------新增---------------
    ld.add_action(declare_tf_prefix)     # -----------新增---------------
    # ld.add_action(declared_moveit_config_package)     # -----------新增---------------
    

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    ld.add_action(arm_delay_moveit_action)
    

    return ld
