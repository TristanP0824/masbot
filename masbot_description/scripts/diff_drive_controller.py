#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import time
import math
import threading
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class DiffDriveController(Node):
    """
    ROS2 Node for differential drive control.
    Subscribes to /cmd_vel, controls motors via serial, and publishes odometry.
    """
    
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
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
        
        # Initialize serial connection with error handling
        self.ser = None
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                write_timeout=1.0
            )
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
            time.sleep(2)  # Wait for connection to establish
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.get_logger().warn("Controller will run in simulation mode without actual hardware")
            
        # Thread lock for serial communication
        self.serial_lock = threading.Lock()
        
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
        
        # Variables for wheel slippage detection
        self.expected_right_encoder_count = 0
        self.expected_left_encoder_count = 0
        self.slippage_threshold = 0.2  # 20% difference between expected and actual
        
        # Send initial stop command
        self.send_command("STOP")
        
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
        
        # Scale wheel velocities to motor commands
        left_motor_speed = self.scale_to_motor_units(left_wheel_velocity)
        right_motor_speed = self.scale_to_motor_units(right_wheel_velocity)
        
        # Send commands to the motors
        self.set_motor_speeds(left_motor_speed, right_motor_speed)
        
        # Update expected encoder counts (for wheel slippage detection)
        dt = 1.0 / self.publish_rate  # Time since last update
        expected_left_ticks = left_wheel_velocity * dt * self.encoder_resolution / (2 * math.pi)
        expected_right_ticks = right_wheel_velocity * dt * self.encoder_resolution / (2 * math.pi)
        self.expected_left_encoder_count += expected_left_ticks
        self.expected_right_encoder_count += expected_right_ticks
    
    def scale_to_motor_units(self, wheel_velocity):
        """Convert wheel velocity (rad/s) to motor command units."""
        # Assuming a linear relationship between wheel velocity and motor command
        max_wheel_velocity = 2.0  # rad/s (example value, adjust based on your robot)
        normalized_velocity = wheel_velocity / max_wheel_velocity
        motor_speed = int(normalized_velocity * self.max_motor_speed)
        
        # Clamp to valid range
        return max(min(motor_speed, self.max_motor_speed), -self.max_motor_speed)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Send motor speed commands to the ESP32."""
        if left_speed == 0 and right_speed == 0:
            command = "STOP"
        else:
            command = f"SPEED,{left_speed},{right_speed}"
            
        self.send_command(command)
    
    def send_command(self, command):
        """Send a command to the ESP32 with error handling."""
        if self.ser is None:
            # Simulation mode - just log the command
            self.get_logger().debug(f"Simulating command: {command}")
            return
            
        try:
            with self.serial_lock:
                formatted_command = f"{command}\n"
                self.ser.write(formatted_command.encode())
                self.ser.flush()
                self.get_logger().debug(f"Sent command: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")
    
    def read_encoder_data(self):
        """Read and process encoder data from the ESP32."""
        if self.ser is None:
            # Simulation mode - simulate encoder readings
            return self.simulate_encoder_readings()
            
        try:
            with self.serial_lock:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("ENC:"):
                        self.process_encoder_data(line)
        except serial.SerialException as e:
            self.get_logger().error(f"Error reading encoder data: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error reading encoder data: {e}")
    
    def process_encoder_data(self, data):
        """Process encoder data from ESP32 (format: ENC:left_count,left_speed,right_count,right_speed)."""
        try:
            parts = data.strip().split(":")
            if len(parts) == 2:
                values = parts[1].split(",")
                if len(values) == 4:
                    self.left_encoder_count = int(values[0])
                    self.left_encoder_speed = float(values[1])
                    self.right_encoder_count = int(values[2])
                    self.right_encoder_speed = float(values[3])
                    
                    # Check for wheel slippage
                    self.detect_wheel_slippage()
        except ValueError as e:
            self.get_logger().warning(f"Error parsing encoder data '{data}': {e}")
    
    def simulate_encoder_readings(self):
        """Simulate encoder readings based on expected values."""
        # Add some noise to simulate real-world conditions
        noise_factor = 0.05  # 5% noise
        left_noise = 1.0 + (2.0 * noise_factor * (0.5 - random.random()))
        right_noise = 1.0 + (2.0 * noise_factor * (0.5 - random.random()))
        
        # Update encoder counts with noise
        self.left_encoder_count = int(self.expected_left_encoder_count * left_noise)
        self.right_encoder_count = int(self.expected_right_encoder_count * right_noise)
    
    def detect_wheel_slippage(self):
        """Detect wheel slippage by comparing expected and actual encoder counts."""
        left_diff = abs(self.left_encoder_count - self.expected_left_encoder_count)
        right_diff = abs(self.right_encoder_count - self.expected_right_encoder_count)
        
        left_threshold = self.expected_left_encoder_count * self.slippage_threshold
        right_threshold = self.expected_right_encoder_count * self.slippage_threshold
        
        if left_diff > left_threshold:
            self.get_logger().warning(f"Possible LEFT wheel slippage detected! Expected: {self.expected_left_encoder_count:.2f}, Actual: {self.left_encoder_count}")
            # Recalibrate expected value to prevent continuous warnings
            self.expected_left_encoder_count = self.left_encoder_count
            
        if right_diff > right_threshold:
            self.get_logger().warning(f"Possible RIGHT wheel slippage detected! Expected: {self.expected_right_encoder_count:.2f}, Actual: {self.right_encoder_count}")
            # Recalibrate expected value to prevent continuous warnings
            self.expected_right_encoder_count = self.right_encoder_count
    
    def update_odometry(self):
        """Read encoder data, update odometry, and publish."""
        # Read encoder data
        self.read_encoder_data()
        
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_encoder_time).nanoseconds / 1e9  # Convert to seconds
        
        if dt <= 0.0:
            self.get_logger().debug("Zero time difference, skipping odometry update")
            return
            
        self.last_encoder_time = current_time
        
        # Calculate wheel rotations from encoder ticks
        left_ticks = self.left_encoder_count - self.prev_left_encoder_count
        right_ticks = self.right_encoder_count - self.prev_right_encoder_count
        
        self.prev_left_encoder_count = self.left_encoder_count
        self.prev_right_encoder_count = self.right_encoder_count
        
        # Convert ticks to radians
        left_wheel_angle = 2.0 * math.pi * left_ticks / self.encoder_resolution
        right_wheel_angle = 2.0 * math.pi * right_ticks / self.encoder_resolution
        
        # Calculate wheel distances
        left_distance = left_wheel_angle * self.wheel_radius
        right_distance = right_wheel_angle * self.wheel_radius
        
        # Calculate robot's linear and angular displacement
        distance = (left_distance + right_distance) / 2.0
        rotation = (right_distance - left_distance) / self.wheel_separation
        
        # Update robot's position and orientation
        if abs(rotation) < 0.0001:
            # Moving straight
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
        else:
            # Moving in an arc
            radius = distance / rotation
            old_theta = self.theta
            self.theta += rotation
            
            self.x += radius * (math.sin(self.theta) - math.sin(old_theta))
            self.y -= radius * (math.cos(self.theta) - math.cos(old_theta))
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        self.vx = distance / dt
        self.vy = 0.0  # Assuming no sideways motion
        self.vtheta = rotation / dt
        
        # Publish odometry
        self.publish_odometry(current_time)
    
    def publish_odometry(self, current_time):
        """Publish odometry message and broadcast transform."""
        # Create quaternion from yaw
        q = quaternion_from_euler(0, 0, self.theta)
        
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
        
        # Set covariance
        # Pose covariance (6x6 matrix stored as array of 36 elements)
        # X, Y, Z, rotation about X, rotation about Y, rotation about Z
        pose_covariance = [0.0] * 36
        # Main diagonal values for position uncertainty
        pose_covariance[0] = 0.01  # x
        pose_covariance[7] = 0.01  # y
        pose_covariance[14] = 0.01  # z (very certain as we're not moving in z)
        pose_covariance[21] = 0.01  # rotation about X (roll)
        pose_covariance[28] = 0.01  # rotation about Y (pitch)
        pose_covariance[35] = 0.03  # rotation about Z (yaw) - less certain
        
        # Twist covariance (6x6 matrix stored as array of 36 elements)
        twist_covariance = [0.0] * 36
        # Main diagonal values for velocity uncertainty
        twist_covariance[0] = 0.01  # x velocity
        twist_covariance[7] = 0.01  # y velocity
        twist_covariance[14] = 0.01  # z velocity
        twist_covariance[21] = 0.01  # angular velocity about X
        twist_covariance[28] = 0.01  # angular velocity about Y
        twist_covariance[35] = 0.03  # angular velocity about Z
        
        # Set the covariance matrices
        odom.pose.covariance = pose_covariance
        odom.twist.covariance = twist_covariance
        
        # Publish
        self.odom_pub.publish(odom)
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        # Stop motors
        self.send_command("STOP")
        
        # Close serial connection
        if self.ser is not None:
            try:
                self.ser.close()
                self.get_logger().info("Serial connection closed")
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    import random  # Import here for the simulate_encoder_readings method
    
    try:
        controller = DiffDriveController()
        rclpy.spin(controller)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()