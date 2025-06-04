#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import math

class MockESP32(Node):
    def __init__(self):
        super().__init__('mock_esp32')
        
        # Joint states publisher (simulating data from ESP32)
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Cmd_vel subscriber (commands that would go to ESP32)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Initialize joint states
        self.positions = [0.0, 0.0, 0.0, 0.0]  # Four wheels
        self.velocities = [0.0, 0.0, 0.0, 0.0]
        
        # Parameters
        self.wheel_radius = 0.05
        self.wheel_separation = 0.34
        
        # Create timer for publishing joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        
        # Store last command
        self.last_cmd_vel = Twist()
        self.last_cmd_time = time.time()
        
        self.get_logger().info('Mock ESP32 node started')
    
    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_time = time.time()
        self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
    
    def publish_joint_states(self):
        # Calculate wheel velocities from last command
        linear_x = self.last_cmd_vel.linear.x
        angular_z = self.last_cmd_vel.angular.z
        
        # Simple differential drive kinematics
        left_vel = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        right_vel = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Set wheel velocities
        self.velocities[0] = left_vel    # Front left
        self.velocities[1] = right_vel   # Front right
        self.velocities[2] = left_vel    # Rear left
        self.velocities[3] = right_vel   # Rear right
        
        # Timeout - stop if no commands received recently
        if time.time() - self.last_cmd_time > 0.5:
            self.velocities = [0.0, 0.0, 0.0, 0.0]
        
        # Update positions based on velocities
        dt = 0.02  # 50Hz
        for i in range(4):
            self.positions[i] += self.velocities[i] * dt
        
        # Create joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint', 
                    'rear_left_wheel_joint', 'rear_right_wheel_joint']
        msg.position = self.positions
        msg.velocity = self.velocities
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        
        # Publish joint states
        self.joint_state_pub.publish(msg)

def main():
    rclpy.init()
    node = MockESP32()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()