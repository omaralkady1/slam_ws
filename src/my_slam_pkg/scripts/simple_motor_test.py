#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import sys

class SimpleMotorTest(Node):
    def __init__(self):
        super().__init__('simple_motor_test')
        
        # Publisher to cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # State tracking
        self.joint_states_received = False
        self.latest_joint_state = None
        
        self.get_logger().info('Simple Motor Test initialized')
        self.get_logger().info('Waiting for joint states...')
        
    def joint_state_callback(self, msg):
        """Store latest joint state"""
        if not self.joint_states_received:
            self.get_logger().info('First joint state received!')
            self.joint_states_received = True
            
        self.latest_joint_state = msg
        
        # Print joint info periodically
        if hasattr(self, '_last_print_time'):
            if time.time() - self._last_print_time > 2.0:
                self.print_joint_states()
                self._last_print_time = time.time()
        else:
            self._last_print_time = time.time()
    
    def print_joint_states(self):
        """Print current joint states"""
        if self.latest_joint_state:
            self.get_logger().info('Current joint states:')
            for i, name in enumerate(self.latest_joint_state.name):
                pos = self.latest_joint_state.position[i]
                vel = self.latest_joint_state.velocity[i]
                self.get_logger().info(f'  {name}: pos={pos:.3f}, vel={vel:.3f}')
    
    def send_velocity_command(self, linear_x, angular_z, duration=2.0):
        """Send velocity command for specified duration"""
        self.get_logger().info(f'Sending: linear_x={linear_x}, angular_z={angular_z} for {duration}s')
        
        # Create command
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        
        # Send command for duration
        start_time = time.time()
        rate = self.create_rate(10)  # 10Hz
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Command complete - robot stopped')
        
        # Show final joint states
        time.sleep(0.5)
        self.print_joint_states()
    
    def run_test_sequence(self):
        """Run a sequence of motor tests"""
        # Wait for joint states
        timeout = 10.0
        start_time = time.time()
        while not self.joint_states_received and time.time() - start_time < timeout:
            self.get_logger().info('Waiting for joint states from ESP32...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        if not self.joint_states_received:
            self.get_logger().error('No joint states received! Check ESP32 connection.')
            return False
        
        self.get_logger().info('Starting motor test sequence...')
        
        # Test 1: Forward
        self.get_logger().info('\n=== Test 1: Forward Movement ===')
        self.send_velocity_command(0.1, 0.0, 2.0)
        time.sleep(1.0)
        
        # Test 2: Backward
        self.get_logger().info('\n=== Test 2: Backward Movement ===')
        self.send_velocity_command(-0.1, 0.0, 2.0)
        time.sleep(1.0)
        
        # Test 3: Rotate left
        self.get_logger().info('\n=== Test 3: Rotate Left ===')
        self.send_velocity_command(0.0, 0.5, 2.0)
        time.sleep(1.0)
        
        # Test 4: Rotate right
        self.get_logger().info('\n=== Test 4: Rotate Right ===')
        self.send_velocity_command(0.0, -0.5, 2.0)
        time.sleep(1.0)
        
        self.get_logger().info('\n=== Test Sequence Complete ===')
        return True
    
    def manual_control(self):
        """Allow manual control via keyboard input"""
        self.get_logger().info('Manual control mode')
        self.get_logger().info('Commands: w=forward, s=backward, a=left, d=right, x=stop, q=quit')
        
        while True:
            try:
                command = input('Enter command: ').strip().lower()
                
                if command == 'q':
                    break
                elif command == 'w':
                    self.send_velocity_command(0.2, 0.0, 1.0)
                elif command == 's':
                    self.send_velocity_command(-0.2, 0.0, 1.0)
                elif command == 'a':
                    self.send_velocity_command(0.0, 0.5, 1.0)
                elif command == 'd':
                    self.send_velocity_command(0.0, -0.5, 1.0)
                elif command == 'x':
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    self.get_logger().info('Robot stopped')
                else:
                    self.get_logger().info('Unknown command')
                    
            except KeyboardInterrupt:
                break
        
        # Stop robot on exit
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Manual control ended')

def main():
    rclpy.init()
    
    test_node = SimpleMotorTest()
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        
        if mode == 'manual':
            # Manual control mode
            import threading
            spin_thread = threading.Thread(target=lambda: rclpy.spin(test_node))
            spin_thread.daemon = True
            spin_thread.start()
            
            test_node.manual_control()
            
        elif mode == 'test':
            # Run test sequence
            success = test_node.run_test_sequence()
            if not success:
                sys.exit(1)
        else:
            # Custom command: linear_x angular_z duration
            try:
                linear_x = float(sys.argv[1])
                angular_z = float(sys.argv[2])
                duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
                
                test_node.send_velocity_command(linear_x, angular_z, duration)
            except (ValueError, IndexError):
                print("Invalid arguments")
                print_usage()
                sys.exit(1)
    else:
        print_usage()
        sys.exit(1)
    
    test_node.destroy_node()
    rclpy.shutdown()

def print_usage():
    print("Usage:")
    print("  ros2 run my_slam_pkg simple_motor_test.py test")
    print("    - Run predefined test sequence")
    print("  ros2 run my_slam_pkg simple_motor_test.py manual")
    print("    - Manual control mode")
    print("  ros2 run my_slam_pkg simple_motor_test.py <linear_x> <angular_z> [duration]")
    print("    - Send specific velocity command")

if __name__ == '__main__':
    main()