from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF via xacro
    pkg_path = get_package_share_directory('masbot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'masbot.urdf.xacro')
    urdf_file = os.path.join(pkg_path, 'urdf', 'masbot.urdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_joints = LaunchConfiguration('publish_joints')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_publish_joints_cmd = DeclareLaunchArgument(
        'publish_joints',
        default_value='true',
        description='Publish joint states')
        
    # Start robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'robot_description': open(urdf_file, 'r').read()}],
    )

    # Start joint state publisher
    joint_state_publisher_node = Node(
        condition=IfCondition(publish_joints),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Start static transform publisher for map
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Start RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'frame_id': 'lidar_link',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'scan_mode': 'Standard',
            'angle_compensate': True,
            'scan_frequency': 10.0
        }],
        output='screen',
    )
    
    '''
    # IMU node (optional, if you decide to use it)
    imu_node = Node(
        package='mpu6050_driver',
        executable='mpu6050_driver_node',
        name='imu_node',
        parameters=[{
            'frame_id': 'imu_link',
            'i2c_bus': 1,
            'i2c_addr': 0x68,
            'update_rate': 50.0
        }],
        output='screen',
    )
    '''
    # Camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'camera_frame_id': 'camera_link',
            'pixel_format': 'YUYV',
            'image_width': 640,
            'image_height': 480,
            'camera_info_url': 'file:///path/to/camera_info.yaml'  # You'll need to calibrate your camera
        }],
        output='screen',
    )
    
    # Differential drive controller
    diff_drive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_publish_joints_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        map_to_odom,
        rplidar_node,
        #imu_node,  # Uncomment if using IMU
        camera_node,
        diff_drive_controller_node,
        joint_state_broadcaster_node
    ])
