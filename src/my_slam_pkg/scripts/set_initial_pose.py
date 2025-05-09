#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
import math
import sys

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10)
        # Wait a moment for publisher to be established
        self.get_logger().info('Waiting to publish initial pose...')
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # Parse command line arguments for position and orientation
        x = 0.0
        y = 0.0
        theta = 0.0
        
        if len(sys.argv) > 1:
            x = float(sys.argv[1])
        if len(sys.argv) > 2:
            y = float(sys.argv[2])
        if len(sys.argv) > 3:
            theta = float(sys.argv[3])
            
        self.publish_initial_pose(x, y, theta)
        
    def publish_initial_pose(self, x=0.0, y=0.0, theta=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set the position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Set the orientation (yaw in radians)
        q = quaternion_from_euler(0.0, 0.0, theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Set the covariance matrix (diagonal matrix with small variance)
        # This represents uncertainty in the position estimate
        covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        msg.pose.covariance = covariance
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published initial pose: x={x}, y={y}, theta={theta}')

def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()