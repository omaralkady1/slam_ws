#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import math
import time
import sys

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # Create publisher directly to the controller to bypass twist_mux
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
            
        # Create odom subscriber to monitor movement
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        # Initialize variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_orientation = 0.0
        self.have_start_position = False
        
        self.get_logger().info('Motor tester node initialized.')
        self.get_logger().info('Waiting for odometry data...')
    
    def odom_callback(self, msg):
        """Store current position from odometry"""
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation (yaw) from quaternion
        q = msg.pose.pose.orientation
        self.current_orientation = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Store initial position
        if not self.have_start_position:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_orientation = self.current_orientation
            self.have_start_position = True
            self.get_logger().info('Initial position recorded.')
    
    def send_cmd_vel(self, linear_x, angular_z, duration):
        """Send a velocity command for a specified duration"""
        if not self.have_start_position:
            self.get_logger().error('No odometry data received yet. Cannot execute command.')
            return
        
        self.get_logger().info(f'Sending command: linear_x={linear_x}, angular_z={angular_z}, duration={duration}s')
        
        # Create message
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        
        # Record position before motion
        start_x = self.current_x
        start_y = self.current_y
        start_orientation = self.current_orientation
        
        # Send command for the specified duration
        start_time = time.time()
        rate = self.create_rate(20)  # 20 Hz refresh rate
        
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(msg)
            self.print_status(start_x, start_y, start_orientation)
            rate.sleep()
        
        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('Command completed.')
        
        # Print final displacement
        dx = self.current_x - start_x
        dy = self.current_y - start_y
        dyaw = self.current_orientation - start_orientation
        distance = math.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info(f'Total movement: distance={distance:.2f}m, rotation={dyaw:.2f}rad')
    
    def print_status(self, start_x, start_y, start_orientation):
        """Print current status"""
        dx = self.current_x - start_x
        dy = self.current_y - start_y
        dyaw = self.current_orientation - start_orientation
        distance = math.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info(f'Current position: x={self.current_x:.2f}, y={self.current_y:.2f}, θ={self.current_orientation:.2f}')
        self.get_logger().info(f'Moved: distance={distance:.2f}m, rotation={dyaw:.2f}rad')
    
    def run_test_pattern(self):
        """Run a test pattern to verify motor control"""
        # Wait for odometry
        while not self.have_start_position:
            self.get_logger().info('Waiting for odometry data...')
            time.sleep(1.0)
        
        # Execute test pattern
        self.get_logger().info('=== Starting test pattern ===')
        
        # Forward
        self.get_logger().info('Test 1: Moving forward')
        self.send_cmd_vel(0.2, 0.0, 3.0)
        time.sleep(1.0)
        
        # Backward
        self.get_logger().info('Test 2: Moving backward')
        self.send_cmd_vel(-0.2, 0.0, 3.0)
        time.sleep(1.0)
        
        # Rotate left
        self.get_logger().info('Test 3: Rotating left')
        self.send_cmd_vel(0.0, 0.5, 3.0)
        time.sleep(1.0)
        
        # Rotate right
        self.get_logger().info('Test 4: Rotating right')
        self.send_cmd_vel(0.0, -0.5, 3.0)
        time.sleep(1.0)
        
        # Arc forward-left
        self.get_logger().info('Test 5: Arc forward-left')
        self.send_cmd_vel(0.2, 0.3, 3.0)
        time.sleep(1.0)
        
        # Arc forward-right
        self.get_logger().info('Test 6: Arc forward-right')
        self.send_cmd_vel(0.2, -0.3, 3.0)
        time.sleep(1.0)
        
        # Return to start
        dx = self.start_x - self.current_x
        dy = self.start_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > 0.1:
            self.get_logger().info('Returning to start position...')
            
            # Calculate angle to start
            angle_to_start = math.atan2(dy, dx)
            angle_diff = angle_to_start - self.current_orientation
            
            # Normalize angle to -pi..pi
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi
            
            # Rotate to face start
            self.send_cmd_vel(0.0, 0.5 if angle_diff > 0 else -0.5, abs(angle_diff) / 0.5)
            
            # Move to start
            self.send_cmd_vel(0.2, 0.0, distance / 0.2)
        
        self.get_logger().info('=== Test pattern complete ===')

def main(args=None):
    rclpy.init(args=args)
    tester = MotorTester()
    
    # Parse command line arguments
    if len(sys.argv) == 1:
        # No arguments - run test pattern
        thread = threading.Thread(target=tester.run_test_pattern)
        thread.daemon = True
        thread.start()
        
        rclpy.spin(tester)
    
    elif len(sys.argv) == 4:
        # Arguments: linear_x, angular_z, duration
        linear_x = float(sys.argv[1])
        angular_z = float(sys.argv[2])
        duration = float(sys.argv[3])
        
        # Create a thread for sending commands
        thread = threading.Thread(target=tester.send_cmd_vel, 
                                 args=(linear_x, angular_z, duration))
        thread.daemon = True
        thread.start()
        
        rclpy.spin(tester)
    
    else:
        print("Usage:")
        print("  ros2 run my_slam_pkg test_motor_control.py")
        print("    - Run a predefined test pattern")
        print("  ros2 run my_slam_pkg test_motor_control.py linear_x angular_z duration")
        print("    - Send a specific velocity command")
        print("      linear_x: Forward/backward velocity in m/s")
        print("      angular_z: Rotational velocity in rad/s")
        print("      duration: Command duration in seconds")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()