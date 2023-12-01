from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_imu':qos,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    
    remappings=[
          ('scan', '/scan')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch
        
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=[parameters],
            remappings=remappings),
    ])
