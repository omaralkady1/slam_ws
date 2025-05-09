#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import sys

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Navigation Goal Sender node initialized')

    def send_goal(self, x, y, theta=0.0):
        self.get_logger().info(f'Waiting for navigation server...')
        self._action_client.wait_for_server()
        
        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert euler angle to quaternion)
        q = quaternion_from_euler(0.0, 0.0, float(theta))
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
        
        # Shutdown after receiving result
        rclpy.shutdown()
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    
    # Check command line arguments
    if len(sys.argv) < 3:
        print("Usage: ros2 run my_slam_pkg nav_goal_sender.py x y [theta]")
        print("  x, y: coordinates in meters")
        print("  theta: orientation in radians (optional, default: 0.0)")
        rclpy.shutdown()
        return
        
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    nav_goal_sender = NavGoalSender()
    nav_goal_sender.send_goal(x, y, theta)
    
    rclpy.spin(nav_goal_sender)

if __name__ == '__main__':
    main()