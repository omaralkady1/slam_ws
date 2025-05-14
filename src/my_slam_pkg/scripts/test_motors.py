#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # Create cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel',
            10
        )
        
        # Create joint states subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Initialize joint states
        self.joint_positions = {}
        self.joint_velocities = {}
        self.received_joint_state = False
        
        self.get_logger().info('Motor tester started')
        
    def joint_state_callback(self, msg):
        """Store joint states from the message"""
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = msg.velocity[i]
        
        self.received_joint_state = True
    
    def send_cmd_vel(self, linear_x, angular_z, duration=2.0):
        """Send a cmd_vel command for the specified duration"""
        self.get_logger().info(f'Sending cmd_vel: linear_x={linear_x}, angular_z={angular_z}')
        
        # Create and send command
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        
        # Time to start measuring
        start_time = time.time()
        
        # Send the command repeatedly for the duration
        rate = self.create_rate(10)  # 10Hz
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Command complete')
        
        # Give time for the robot to stop
        time.sleep(0.5)
    
    def print_joint_states(self):
        """Print current joint states"""
        if not self.received_joint_state:
            self.get_logger().warn('No joint states received yet')
            return
            
        self.get_logger().info('Joint states:')
        for name in sorted(self.joint_positions.keys()):
            pos = self.joint_positions[name]
            vel = self.joint_velocities[name]
            self.get_logger().info(f'  {name}: position={pos:.4f}, velocity={vel:.4f}')
    
    def run_test_sequence(self):
        """Run a sequence of motor tests"""
        self.get_logger().info('Starting motor test sequence')
        
        # Wait for initial joint states
        timeout = 10.0  # 10 seconds
        start_time = time.time()
        while not self.received_joint_state and time.time() - start_time < timeout:
            self.get_logger().info('Waiting for joint states...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        if not self.received_joint_state:
            self.get_logger().error('No joint states received. Check hardware connection.')
            return False
            
        # Print initial state
        self.get_logger().info('Initial state:')
        self.print_joint_states()
        
        # Test sequence
        tests = [
            # Forward
            {'linear_x': 0.1, 'angular_z': 0.0, 'duration': 2.0, 'desc': 'Forward'},
            # Backward
            {'linear_x': -0.1, 'angular_z': 0.0, 'duration': 2.0, 'desc': 'Backward'},
            # Turn left
            {'linear_x': 0.0, 'angular_z': 0.5, 'duration': 2.0, 'desc': 'Turn left'},
            # Turn right
            {'linear_x': 0.0, 'angular_z': -0.5, 'duration': 2.0, 'desc': 'Turn right'},
            # Forward + left
            {'linear_x': 0.1, 'angular_z': 0.2, 'duration': 2.0, 'desc': 'Forward + left'},
            # Forward + right
            {'linear_x': 0.1, 'angular_z': -0.2, 'duration': 2.0, 'desc': 'Forward + right'}
        ]
        
        # Run each test
        for test in tests:
            self.get_logger().info(f"\n=== Test: {test['desc']} ===")
            self.send_cmd_vel(test['linear_x'], test['angular_z'], test['duration'])
            
            # Spin some to get joint state updates
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)
                
            # Print joint states after the test
            self.print_joint_states()
            
            # Wait a moment between tests
            time.sleep(1.0)
        
        self.get_logger().info('\n=== Motor test sequence complete ===')
        return True

def main():
    rclpy.init()
    tester = MotorTester()
    
    if len(sys.argv) > 1 and sys.argv[1] == 'sequence':
        # Run the test sequence
        tester.run_test_sequence()
    elif len(sys.argv) > 3:
        # Manual command: linear_x angular_z duration
        linear_x = float(sys.argv[1])
        angular_z = float(sys.argv[2])
        duration = float(sys.argv[3])
        
        tester.send_cmd_vel(linear_x, angular_z, duration)
        
        # Wait to receive joint states
        for _ in range(10):
            rclpy.spin_once(tester, timeout_sec=0.2)
            
        tester.print_joint_states()
    else:
        print("Usage:")
        print("  ros2 run my_slam_pkg test_motors.py sequence")
        print("    - Run a predefined test sequence")
        print("  ros2 run my_slam_pkg test_motors.py linear_x angular_z duration")
        print("    - Send a specific velocity command")
        print("      linear_x: Forward/backward velocity in m/s")
        print("      angular_z: Rotational velocity in rad/s")
        print("      duration: Command duration in seconds")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()