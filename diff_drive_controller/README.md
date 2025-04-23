# Differential Drive Controller for ROS2

This package provides a ROS2 node for controlling a differential drive robot with an ESP32 microcontroller. The node subscribes to `/cmd_vel` topics, converts Twist messages to wheel velocities using differential drive kinematics, and sends commands to the ESP32 via serial communication. It also reads encoder feedback from the ESP32, calculates odometry, and publishes it on the `/odom` topic while broadcasting the necessary TF transforms.

## Features

- Subscribes to `/cmd_vel` topic to receive velocity commands
- Converts linear and angular velocities to left and right wheel velocities
- Sends motor commands to ESP32 via serial communication
- Reads encoder feedback from ESP32
- Calculates and publishes odometry on `/odom` topic
- Broadcasts TF transforms (base_link to odom)
- Configurable parameters for wheel radius, track width, etc.

## Prerequisites

- ROS2 (tested on ROS2 Jazzy)
- Python 3
- pyserial package
- tf_transformations package

## Installation

1. Make sure the Python scripts are executable:
   ```bash
   chmod +x src/diff_drive_controller_node.py src/diff_drive_controller_launch.py
   ```

## Usage

### Running the Node Directly

You can run the node directly with:

```bash
ros2 run --prefix 'python3 -u' /home/tristan/ros2_ws/src/diff_drive_controller_node.py
```

### Using the Launch File

For easier configuration, use the provided launch file:

```bash
ros2 launch /home/tristan/ros2_ws/src/diff_drive_controller_launch.py
```

You can override default parameters:

```bash
ros2 launch /home/tristan/ros2_ws/src/diff_drive_controller_launch.py serial_port:=/dev/ttyUSB0 wheel_radius:=0.03
```

## Parameters

The node can be configured with the following parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| serial_port | /dev/ttyUSB1 | Serial port for ESP32 communication |
| baud_rate | 115200 | Baud rate for serial communication |
| wheel_radius | 0.025 | Wheel radius in meters |
| wheel_separation | 0.28 | Wheel separation (track width) in meters |
| max_motor_speed | 200 | Maximum motor speed in controller units |
| encoder_resolution | 360 | Encoder resolution in ticks per revolution |
| publish_rate | 50.0 | Rate at which to publish odometry in Hz |
| odom_frame_id | odom | Frame ID for the odometry message |
| base_frame_id | base_link | Frame ID for the robot base |

## ESP32 Communication Protocol

The node communicates with the ESP32 using the following commands:

- `CMD:STOP` - Stop all motors
- `CMD:FORWARD,<speed>` - Move forward at the specified speed
- `CMD:BACKWARD,<speed>` - Move backward at the specified speed
- `CMD:LEFT,<speed>` - Rotate left at the specified speed
- `CMD:RIGHT,<speed>` - Rotate right at the specified speed
- `CMD:CUSTOM,<left_speed>,<right_speed>` - Set custom speeds for left and right motors

The ESP32 sends encoder data in the format:
```
E:<right_encoder_count>,<right_encoder_speed>,<left_encoder_count>,<left_encoder_speed>
```

## Differential Drive Kinematics

The node implements these differential drive equations:
- Left wheel velocity = (linear_velocity - (track_width/2) * angular_velocity) / wheel_radius
- Right wheel velocity = (linear_velocity + (track_width/2) * angular_velocity) / wheel_radius

## License

This software is provided under the Apache License 2.0.
