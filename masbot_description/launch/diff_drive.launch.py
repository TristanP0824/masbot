#!/usr/bin/env python3

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
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    max_motor_speed = LaunchConfiguration('max_motor_speed')
    encoder_resolution = LaunchConfiguration('encoder_resolution')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 communication')
        
    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication')
        
    declare_wheel_radius_cmd = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.025',
        description='Wheel radius in meters')
        
    declare_wheel_separation_cmd = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.28',
        description='Wheel separation (track width) in meters')
        
    declare_max_motor_speed_cmd = DeclareLaunchArgument(
        'max_motor_speed',
        default_value='200',
        description='Maximum motor speed in controller units')
        
    declare_encoder_resolution_cmd = DeclareLaunchArgument(
        'encoder_resolution',
        default_value='360',
        description='Encoder resolution in ticks per revolution')
        
    declare_publish_rate_cmd = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Rate at which to publish odometry in Hz')
    
    # Include the robot launch file to publish URDF and joint states
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'masbot.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_joints': 'true'
        }.items()
    )
    
    # Differential Drive Controller node
    diff_drive_controller_node = Node(
        package='masbot_description',
        executable='scripts/diff_drive_controller.py',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'wheel_radius': wheel_radius,
            'wheel_separation': wheel_separation,
            'max_motor_speed': max_motor_speed,
            'encoder_resolution': encoder_resolution,
            'publish_rate': publish_rate,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link'
        }]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_serial_port_cmd,
        declare_baud_rate_cmd,
        declare_wheel_radius_cmd,
        declare_wheel_separation_cmd,
        declare_max_motor_speed_cmd,
        declare_encoder_resolution_cmd,
        declare_publish_rate_cmd,
        robot_launch,
        diff_drive_controller_node
    ])