#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10
        )
        
        self.cmd_sub = self.create_subscription(
            Twist,
            '/diff_drive_controller/cmd_vel',
            self.cmd_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Data storage
        self.joint_data = {}
        self.last_cmd = None
        self.odom_data = None
        
        # Status timer
        self.timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('Joint state monitor started')
        
    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_data[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0
            }
            
    def cmd_callback(self, msg):
        self.last_cmd = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
            'time': time.time()
        }
        
    def odom_callback(self, msg):
        self.odom_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        }
        
    def print_status(self):
        self.get_logger().info('='*50)
        
        # Print command
        if self.last_cmd:
            age = time.time() - self.last_cmd['time']
            self.get_logger().info(
                f'Command: linear={self.last_cmd["linear_x"]:.2f}, '
                f'angular={self.last_cmd["angular_z"]:.2f} '
                f'(age: {age:.1f}s)'
            )
        
        # Print joint states
        if self.joint_data:
            self.get_logger().info('Joint States:')
            for name in sorted(self.joint_data.keys()):
                data = self.joint_data[name]
                self.get_logger().info(
                    f'  {name}: pos={data["position"]:.3f}, '
                    f'vel={data["velocity"]:.3f}'
                )
        
        # Print odometry
        if self.odom_data:
            self.get_logger().info(
                f'Odometry: x={self.odom_data["x"]:.3f}, '
                f'y={self.odom_data["y"]:.3f}, '
                f'vx={self.odom_data["linear_x"]:.3f}, '
                f'wz={self.odom_data["angular_z"]:.3f}'
            )
            
def main(args=None):
    rclpy.init(args=args)
    node = JointStateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()