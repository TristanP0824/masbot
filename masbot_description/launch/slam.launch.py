from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('masbot_description').find('masbot_description')
    rtabmap_pkg_share = FindPackageShare('rtabmap_launch').find('rtabmap_launch')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_qos_cmd = DeclareLaunchArgument(
        'qos',
        default_value='1',
        description='QoS used for sensor topics')
        
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode')
    
    # Include the robot launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'masbot.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_joints': 'true'
        }.items()
    )
    
    # Include the RTAB-Map launch file
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([rtabmap_pkg_share, 'launch', 'rtabmap.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'qos': qos,
            'localization': localization,
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'visual_odometry': 'true',
            'odom_topic': '/odom'
        }.items()
    )
    
    # Start RViz
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'slam.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_qos_cmd,
        declare_localization_cmd,
        robot_launch,
        rtabmap_launch,
        rviz_node
    ])
