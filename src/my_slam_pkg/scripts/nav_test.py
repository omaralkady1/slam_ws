#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time

class NavTest(Node):
    def __init__(self):
        super().__init__('nav_test')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav test node initialized')
        
        # Define waypoints [x, y, theta]
        self.waypoints = [
            (1.0, 1.0, 0.0),
            (2.0, 1.0, 1.57),
            (2.0, 2.0, 3.14),
            (1.0, 2.0, -1.57),
            (1.0, 1.0, 0.0)
        ]
        
        self.current_waypoint = 0
        self.goal_complete = True
        
        # Create a timer for sending goals
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        if self.goal_complete and self.current_waypoint < len(self.waypoints):
            x, y, theta = self.waypoints[self.current_waypoint]
            self.send_goal(x, y, theta)
            self.current_waypoint += 1
    
    def send_goal(self, x, y, theta):
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0.0, 0.0, theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        self.goal_complete = False
        
        # Send the goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.goal_complete = True
            return
            
        self.get_logger().info('Goal accepted')
        
        # Get the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
        
        # Mark goal as complete
        self.goal_complete = True
        
        # If we've completed all waypoints, shutdown
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            self.timer.cancel()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    nav_test = NavTest()
    
    try:
        rclpy.spin(nav_test)
    except KeyboardInterrupt:
        pass
    
    nav_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()