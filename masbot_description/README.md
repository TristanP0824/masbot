# MasBot Description Package

This ROS2 package provides the description, hardware interface, and launch files for the MasBot robot. It includes a URDF model, TF tree, and navigation capabilities.

## Package Structure

```
masbot_description/
├── config/                 # Configuration files
│   └── diff_drive_controller.yaml
├── include/                # Header files
│   └── masbot_description/
│       └── masbot_hardware_interface.hpp
├── launch/                 # Launch files
│   ├── masbot.launch.py    # Main robot launch file
│   ├── test_tf.launch.py   # TF tree testing launch file
│   └── view_robot.launch.py # Visualization launch file
├── rviz/                   # RViz configuration
│   └── masbot.rviz
├── src/                    # Source files
│   └── masbot_hardware_interface.cpp
├── urdf/                   # URDF model files
│   ├── masbot.urdf         # Static URDF file
│   └── masbot.urdf.xacro   # XACRO file with hardware interface
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── masbot_hardware_interface.xml # Hardware interface plugin description
└── README.md               # This file
```

## Robot Specifications

- Base dimensions: 53cm x 20.75cm
- Track width: 28cm
- Track length: 55cm
- Total height: 118cm
- Vertical aluminum profile mast
- RPLidar A1 at the top
- ONN camera mounted above the lidar
- LCD screen in portrait orientation
- Ultrasonic sensors mounted at 26cm height

## Building the Package

1. Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/masbot_description.git
```

2. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select masbot_description
```

4. Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Launching the Robot

### Basic Launch

To launch the robot with all necessary nodes:

```bash
ros2 launch masbot_description masbot.launch.py
```

### Visualizing in RViz

To launch the robot and visualize it in RViz:

```bash
ros2 launch masbot_description view_robot.launch.py
```

### Testing the TF Tree

To test the TF tree and generate a PDF visualization:

```bash
ros2 launch masbot_description test_tf.launch.py
```

This will generate a `frames.pdf` file in your current directory showing the complete TF tree.

You can also use the TF2 tools to view the TF tree in real-time:

```bash
ros2 run tf2_tools view_frames
```

## Hardware Interface

The hardware interface provides communication between ROS2 control system and the robot's hardware. It reads encoder data from the front motors and controls them via a dual h-bridge motor driver.

The rear motors are wired in parallel with the front motors, so they follow the same commands.

## Differential Drive Controller

The differential drive controller is configured to use the front sprocket joints with encoders. The configuration can be found in `config/diff_drive_controller.yaml`.

## SLAM and Navigation

This package includes launch files for SLAM and navigation capabilities.

### SLAM with RTAB-Map

To run SLAM using RTAB-Map:

1. Install RTAB-Map ROS2 packages:

```bash
sudo apt install ros-humble-rtabmap-ros
```

2. Launch the SLAM system:

```bash
ros2 launch masbot_description slam.launch.py
```

3. Drive the robot around to build a map of the environment.

4. To save the map:

```bash
# Save map
ros2 service call /rtabmap/save_map std_srvs/srv/Empty
```

The map will be saved in the RTAB-Map database format. You can also export it to a 2D occupancy grid:

```bash
# Export to 2D grid map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/masbot_description/maps/map
```

### Navigation with Nav2

To run navigation using Nav2:

1. Install Nav2 packages:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

2. Launch the navigation system with a pre-built map:

```bash
ros2 launch masbot_description navigation.launch.py map_file:=/path/to/map.yaml
```

3. In RViz, use the "2D Pose Estimate" tool to set the initial pose of the robot.

4. Use the "2D Nav Goal" tool to set navigation goals for the robot.

### Testing the TF Tree

To visualize and test the TF tree:

```bash
ros2 launch masbot_description test_tf.launch.py
```

This will generate a `frames.pdf` file showing the complete TF tree.

## Customization

- Modify the URDF files in the `urdf/` directory to change the robot model
- Adjust the hardware interface parameters in `urdf/masbot.urdf.xacro`
- Update the controller parameters in `config/diff_drive_controller.yaml`

## License

This package is licensed under the Apache License 2.0.
