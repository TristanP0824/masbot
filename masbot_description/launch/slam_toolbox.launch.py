from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('masbot_description').find('masbot_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
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
    
    # Start SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_laser_range': 12.0,  # Max range of RPLidar A1 is around 12m
            'resolution': 0.05,
            'map_update_interval': 5.0,
            'scan_topic': '/scan',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'transform_publish_period': 0.02,
            'map_start_pose': [0.0, 0.0, 0.0]  # Starting position in the map
        }]
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
        robot_launch,
        slam_toolbox_node,
        rviz_node
    ])