#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import time
import math
import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations

class DiffDriveController(Node):
    """
    ROS2 Node for differential drive control.
    Subscribes to /cmd_vel, controls motors via serial, and publishes odometry.
    """
    
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.025)  # meters
        self.declare_parameter('wheel_separation', 0.28)  # meters (track width)
        self.declare_parameter('max_motor_speed', 200)  # motor controller units
        self.declare_parameter('encoder_resolution', 360)  # ticks per revolution
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_motor_speed = self.get_parameter('max_motor_speed').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
            time.sleep(2)  # Wait for connection to establish
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise
        
        # Initialize subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Initialize publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        # Initialize encoder variables
        self.right_encoder_count = 0
        self.right_encoder_speed = 0.0
        self.left_encoder_count = 0
        self.left_encoder_speed = 0.0
        self.prev_right_encoder_count = 0
        self.prev_left_encoder_count = 0
        self.last_encoder_time = self.get_clock().now()
        
        # Send initial stop command
        self.send_command("CMD:STOP")
        
        # Create timer for reading encoder data and publishing odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_odometry)
        
        self.get_logger().info('Differential Drive Controller initialized')
    
    def cmd_vel_callback(self, msg):
        """
        Callback for /cmd_vel topic.
        Converts Twist message to wheel velocities and sends commands to motors.
        """
        # Extract linear and angular velocities
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Calculate wheel velocities using differential drive kinematics
        left_wheel_velocity = (linear_velocity - (self.wheel_separation / 2.0) * angular_velocity) / self.wheel_radius
        right_wheel_velocity = (linear_velocity + (self.wheel_separation / 2.0) * angular_velocity) / self.wheel_radius
        
        # Convert wheel velocities to motor commands
        self.set_wheel_velocities(left_wheel_velocity, right_wheel_velocity)
    
    def set_wheel_velocities(self, left_wheel_velocity, right_wheel_velocity):
        """
        Convert wheel velocities to motor commands and send to ESP32.
        """
        # Scale wheel velocities to motor speed units
        # Assuming max_motor_speed corresponds to some maximum wheel velocity
        # This scaling factor would need to be calibrated for your specific robot
        max_wheel_velocity = 2.0  # rad/s (example value, adjust based on your robot)
        
        left_motor_speed = int((left_wheel_velocity / max_wheel_velocity) * self.max_motor_speed)
        right_motor_speed = int((right_wheel_velocity / max_wheel_velocity) * self.max_motor_speed)
        
        # Clamp motor speeds to valid range
        left_motor_speed = max(min(left_motor_speed, self.max_motor_speed), -self.max_motor_speed)
        right_motor_speed = max(min(right_motor_speed, self.max_motor_speed), -self.max_motor_speed)
        
        # Determine direction and send appropriate commands
        if left_motor_speed == 0 and right_motor_speed == 0:
            self.send_command("CMD:STOP")
        elif left_motor_speed >= 0 and right_motor_speed >= 0:
            # Both wheels forward
            if left_motor_speed == right_motor_speed:
                self.send_command(f"CMD:FORWARD,{abs(left_motor_speed)}")
            else:
                self.send_command(f"CMD:CUSTOM,{abs(left_motor_speed)},{abs(right_motor_speed)}")
        elif left_motor_speed <= 0 and right_motor_speed <= 0:
            # Both wheels backward
            if left_motor_speed == right_motor_speed:
                self.send_command(f"CMD:BACKWARD,{abs(left_motor_speed)}")
            else:
                self.send_command(f"CMD:CUSTOM,{-abs(left_motor_speed)},{-abs(right_motor_speed)}")
        elif left_motor_speed < 0 and right_motor_speed > 0:
            # Rotate counterclockwise (left)
            self.send_command(f"CMD:LEFT,{max(abs(left_motor_speed), abs(right_motor_speed))}")
        elif left_motor_speed > 0 and right_motor_speed < 0:
            # Rotate clockwise (right)
            self.send_command(f"CMD:RIGHT,{max(abs(left_motor_speed), abs(right_motor_speed))}")
        else:
            # Custom speeds for differential turning
            self.send_command(f"CMD:CUSTOM,{left_motor_speed},{right_motor_speed}")
    
    def send_command(self, command):
        """
        Format and send command to ESP32.
        """
        try:
            command = f"{command}\n"
            self.ser.write(command.encode())
            self.get_logger().debug(f"Sent command: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {e}")
    
    def process_encoder_data(self, data):
        """
        Process encoder data received from ESP32.
        """
        if data.startswith("E:"):
            parts = data.strip().split(',')
            if len(parts) >= 4:
                try:
                    self.right_encoder_count = int(parts[0][2:])
                    self.right_encoder_speed = float(parts[1])
                    self.left_encoder_count = int(parts[2])
                    self.left_encoder_speed = float(parts[3])
                except ValueError as e:
                    self.get_logger().warning(f"Error parsing encoder data: {e}")
    
    def update_odometry(self):
        """
        Read encoder data, update odometry, and publish.
        """
        # Read encoder data from serial
        self.read_encoder_data()
        
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_encoder_time).nanoseconds / 1e9  # Convert to seconds
        self.last_encoder_time = current_time
        
        if dt <= 0:
            return
        
        # Calculate wheel rotations from encoder ticks
        right_wheel_delta = (self.right_encoder_count - self.prev_right_encoder_count) / self.encoder_resolution
        left_wheel_delta = (self.left_encoder_count - self.prev_left_encoder_count) / self.encoder_resolution
        
        # Update previous encoder counts
        self.prev_right_encoder_count = self.right_encoder_count
        self.prev_left_encoder_count = self.left_encoder_count
        
        # Calculate wheel distances
        right_wheel_distance = right_wheel_delta * 2 * math.pi * self.wheel_radius
        left_wheel_distance = left_wheel_delta * 2 * math.pi * self.wheel_radius
        
        # Calculate robot's linear and angular displacement
        linear_distance = (right_wheel_distance + left_wheel_distance) / 2.0
        angular_distance = (right_wheel_distance - left_wheel_distance) / self.wheel_separation
        
        # Update robot's position and orientation
        self.theta += angular_distance
        self.x += linear_distance * math.cos(self.theta)
        self.y += linear_distance * math.sin(self.theta)
        
        # Calculate velocities
        self.vx = linear_distance / dt
        self.vy = 0.0  # Assuming no sideways motion
        self.vtheta = angular_distance / dt
        
        # Publish odometry
        self.publish_odometry(current_time)
    
    def read_encoder_data(self):
        """
        Read encoder data from serial port.
        """
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8', errors='ignore')
                self.process_encoder_data(data)
        except serial.SerialException as e:
            self.get_logger().error(f"Error reading encoder data: {e}")
    
    def publish_odometry(self, current_time):
        """
        Publish odometry message and broadcast transform.
        """
        # Create quaternion from yaw
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta
        
        # Set covariance (from diff-drive-config.txt)
        pose_covariance_diagonal = [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
        twist_covariance_diagonal = [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
        
        for i in range(6):
            odom.pose.covariance[i*6+i] = pose_covariance_diagonal[i]
            odom.twist.covariance[i*6+i] = twist_covariance_diagonal[i]
        
        self.odom_pub.publish(odom)
    
    def destroy_node(self):
        """
        Clean up when node is destroyed.
        """
        # Stop motors
        self.send_command("CMD:STOP")
        
        # Close serial connection
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        diff_drive_controller = DiffDriveController()
        rclpy.spin(diff_drive_controller)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if 'diff_drive_controller' in locals():
            diff_drive_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
