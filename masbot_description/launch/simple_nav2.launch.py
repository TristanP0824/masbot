from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('masbot_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')
    
    # Start robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(os.path.join(pkg_dir, 'urdf', 'masbot.urdf'), 'r').read()
        }]
    )
    
    # Start joint state publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Start static transform publisher for map
    map_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Start RPLidar node
    rplidar_cmd = Node(
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
        }]
    )
    
    # Start RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'navigation.rviz')]
    )
    
    # Start Map Server
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # Start AMCL
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame_id': 'map',
            'robot_base_frame': 'base_link',
            'odom_frame_id': 'odom'
        }]
    )
    
    # Start Controller Server
    controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller_frequency': 20.0,
            'controller_plugins': ['FollowPath']
        }]
    )
    
    # Start Planner Server
    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planner_plugins': ['GridBased']
        }]
    )
    
    # Start Lifecycle Manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server']
        }]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_file_cmd,
        robot_state_publisher_cmd,
        joint_state_publisher_cmd,
        map_transform_cmd,
        rplidar_cmd,
        rviz_cmd,
        map_server_cmd,
        amcl_cmd,
        controller_server_cmd,
        planner_server_cmd,
        lifecycle_manager_cmd
    ])