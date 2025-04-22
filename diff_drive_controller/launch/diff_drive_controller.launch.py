#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for the differential drive controller node.
    """
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for ESP32 communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.025',
        description='Wheel radius in meters'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.28',
        description='Wheel separation (track width) in meters'
    )
    
    max_motor_speed_arg = DeclareLaunchArgument(
        'max_motor_speed',
        default_value='200',
        description='Maximum motor speed in controller units'
    )
    
    encoder_resolution_arg = DeclareLaunchArgument(
        'encoder_resolution',
        default_value='360',
        description='Encoder resolution in ticks per revolution'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Rate at which to publish odometry in Hz'
    )
    
    # Create the differential drive controller node
    diff_drive_node = Node(
        package='diff_drive_controller',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'max_motor_speed': LaunchConfiguration('max_motor_speed'),
            'encoder_resolution': LaunchConfiguration('encoder_resolution'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link'
        }]
    )
    
    # Create and return launch description
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        max_motor_speed_arg,
        encoder_resolution_arg,
        publish_rate_arg,
        diff_drive_node
    ])
