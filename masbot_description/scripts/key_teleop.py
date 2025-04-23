#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading
import time

msg = """
MasBot Keyboard Controller
---------------------------
Use arrow keys to control the robot:
    UP:    Move forward
    DOWN:  Move backward
    LEFT:  Turn left
    RIGHT: Turn right
    SPACE: Stop

Press 'q' to quit
"""

# Key codes
UP_KEY = '\x1b[A'
DOWN_KEY = '\x1b[B'
RIGHT_KEY = '\x1b[C'
LEFT_KEY = '\x1b[D'
SPACE_KEY = ' '
QUIT_KEY = 'q'

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Create publisher for cmd_vel topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement settings
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.get_logger().info(msg)
        
        # Create a timer to repeatedly publish the last command
        # This helps maintain movement when a key is held down
        self.last_twist = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Flag to control the keyboard reading thread
        self.running = True
        
        # Start keyboard reading in a separate thread
        self.key_thread = threading.Thread(target=self.read_keys_loop)
        self.key_thread.daemon = True
        self.key_thread.start()
    
    def timer_callback(self):
        """Publish the last twist command periodically"""
        self.publisher.publish(self.last_twist)
    
    def read_keys_loop(self):
        """Loop reading keys and updating twist command"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                # Check if a key is available
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    # Check for arrow keys (escape sequence)
                    if key == '\x1b':
                        key += sys.stdin.read(2)
                    
                    self.process_key(key)
                
                # Small sleep to prevent CPU hogging
                time.sleep(0.05)
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def process_key(self, key):
        """Process keypresses and update twist command"""
        twist = Twist()
        
        if key == UP_KEY:
            twist.linear.x = self.linear_speed
            self.get_logger().info('Forward')
        elif key == DOWN_KEY:
            twist.linear.x = -self.linear_speed
            self.get_logger().info('Backward')
        elif key == LEFT_KEY:
            twist.angular.z = self.angular_speed
            self.get_logger().info('Left')
        elif key == RIGHT_KEY:
            twist.angular.z = -self.angular_speed
            self.get_logger().info('Right')
        elif key == SPACE_KEY:
            # All values are 0 by default
            self.get_logger().info('Stop')
        elif key == QUIT_KEY:
            self.get_logger().info('Quitting...')
            self.running = False
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        
        # Update the last twist command
        self.last_twist = twist
        # Publish immediately (also published by timer)
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    controller = KeyboardController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt detected, shutting down')
    finally:
        # Make sure we stop the robot before exiting
        stop_twist = Twist()
        controller.publisher.publish(stop_twist)
        controller.running = False
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()