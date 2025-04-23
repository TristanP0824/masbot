#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import csv
import numpy as np
import os
from datetime import datetime

class DiffDriveTest(Node):
    """
    Test node for differential drive controller.
    Publishes Twist messages to /cmd_vel to test all movement directions
    and collects encoder data from /odom for analysis.
    """
    
    def __init__(self):
        super().__init__('diff_drive_test')
        
        # Create publisher for /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscriber for /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Initialize data collection
        self.odom_data = []
        self.start_time = None
        self.test_running = False
        self.current_movement = "IDLE"
        
        # Create timer for test sequence
        self.timer = self.create_timer(0.1, self.test_sequence)
        
        # Initialize test sequence state
        self.test_state = 0
        self.state_start_time = None
        
        self.get_logger().info('Differential Drive Test Node initialized')
        self.get_logger().info('Starting test sequence in 2 seconds...')
        time.sleep(2)  # Give time for subscribers to connect
        
        # Start the test
        self.start_time = self.get_clock().now()
        self.state_start_time = self.get_clock().now()
        self.test_running = True
    
    def odom_callback(self, msg):
        """
        Callback for /odom topic.
        Collects odometry data for analysis.
        """
        if not self.test_running:
            return
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        # Extract position and velocity data
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        vel_angular = msg.twist.twist.angular.z
        
        # Store data
        self.odom_data.append({
            'time': elapsed_time,
            'movement': self.current_movement,
            'pos_x': pos_x,
            'pos_y': pos_y,
            'vel_x': vel_x,
            'vel_y': vel_y,
            'vel_angular': vel_angular
        })
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """
        Publish a Twist message to /cmd_vel.
        """
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
    
    def test_sequence(self):
        """
        Execute the test sequence.
        """
        if not self.test_running:
            return
        
        current_time = self.get_clock().now()
        state_elapsed_time = (current_time - self.state_start_time).nanoseconds / 1e9
        
        # State machine for test sequence
        if self.test_state == 0:
            # Forward for 6 seconds
            if state_elapsed_time < 6.0:
                if state_elapsed_time < 0.1:
                    self.get_logger().info('Moving FORWARD for 6 seconds')
                self.publish_cmd_vel(0.2, 0.0)
                self.current_movement = "FORWARD"
            else:
                self.test_state = 1
                self.state_start_time = current_time
                self.publish_cmd_vel(0.0, 0.0)  # Stop before next movement
                time.sleep(1.0)  # Pause between movements
        
        elif self.test_state == 1:
            # Backward for 6 seconds
            if state_elapsed_time < 6.0:
                if state_elapsed_time < 0.1:
                    self.get_logger().info('Moving BACKWARD for 6 seconds')
                self.publish_cmd_vel(-0.2, 0.0)
                self.current_movement = "BACKWARD"
            else:
                self.test_state = 2
                self.state_start_time = current_time
                self.publish_cmd_vel(0.0, 0.0)  # Stop before next movement
                time.sleep(1.0)  # Pause between movements
        
        elif self.test_state == 2:
            # Left for 3 seconds
            if state_elapsed_time < 3.0:
                if state_elapsed_time < 0.1:
                    self.get_logger().info('Turning LEFT for 3 seconds')
                self.publish_cmd_vel(0.0, 0.2)
                self.current_movement = "LEFT"
            else:
                self.test_state = 3
                self.state_start_time = current_time
                self.publish_cmd_vel(0.0, 0.0)  # Stop before next movement
                time.sleep(1.0)  # Pause between movements
        
        elif self.test_state == 3:
            # Right for 3 seconds
            if state_elapsed_time < 3.0:
                if state_elapsed_time < 0.1:
                    self.get_logger().info('Turning RIGHT for 3 seconds')
                self.publish_cmd_vel(0.0, -0.2)
                self.current_movement = "RIGHT"
            else:
                self.test_state = 4
                self.state_start_time = current_time
                self.publish_cmd_vel(0.0, 0.0)  # Stop at the end
                time.sleep(1.0)  # Pause before finishing
        
        elif self.test_state == 4:
            # Test complete
            self.get_logger().info('Test sequence complete')
            self.test_running = False
            self.save_data()
            self.timer.cancel()
    
    def save_data(self):
        """
        Save collected data to a CSV file.
        """
        if not self.odom_data:
            self.get_logger().error('No data collected during test')
            return
        
        # Create a timestamp for the filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"diff_drive_test_{timestamp}.csv"
        filepath = os.path.join(os.getcwd(), filename)
        
        # Write data to CSV
        with open(filepath, 'w', newline='') as csvfile:
            fieldnames = ['time', 'movement', 'pos_x', 'pos_y', 'vel_x', 'vel_y', 'vel_angular']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for data_point in self.odom_data:
                writer.writerow(data_point)
        
        self.get_logger().info(f'Data saved to {filepath}')
        
        # Print summary statistics
        self.print_summary()
    
    def print_summary(self):
        """
        Print summary statistics of the collected data.
        """
        if not self.odom_data:
            return
        
        # Group data by movement type
        movements = {}
        for data in self.odom_data:
            movement = data['movement']
            if movement not in movements:
                movements[movement] = []
            movements[movement].append(data)
        
        # Print summary for each movement
        self.get_logger().info('===== TEST SUMMARY =====')
        for movement, data in movements.items():
            if movement == "IDLE":
                continue
                
            # Calculate statistics
            pos_x_values = [d['pos_x'] for d in data]
            pos_y_values = [d['pos_y'] for d in data]
            vel_x_values = [d['vel_x'] for d in data]
            vel_angular_values = [d['vel_angular'] for d in data]
            
            # Calculate distance traveled
            if len(pos_x_values) >= 2:
                start_x, start_y = pos_x_values[0], pos_y_values[0]
                end_x, end_y = pos_x_values[-1], pos_y_values[-1]
                distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            else:
                distance = 0.0
            
            # Calculate average velocities
            avg_vel_x = np.mean(vel_x_values) if vel_x_values else 0.0
            avg_vel_angular = np.mean(vel_angular_values) if vel_angular_values else 0.0
            
            # Print summary
            self.get_logger().info(f'Movement: {movement}')
            self.get_logger().info(f'  Distance traveled: {distance:.4f} meters')
            self.get_logger().info(f'  Average linear velocity: {avg_vel_x:.4f} m/s')
            self.get_logger().info(f'  Average angular velocity: {avg_vel_angular:.4f} rad/s')
            self.get_logger().info('------------------------')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        diff_drive_test = DiffDriveTest()
        rclpy.spin(diff_drive_test)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if 'diff_drive_test' in locals():
            diff_drive_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
