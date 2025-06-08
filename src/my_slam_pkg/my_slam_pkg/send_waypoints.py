#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import sys
import time

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info('Waypoint Sender node initialized')
        self.goal_succeeded = True

    def send_single_goal(self, x, y, theta=0.0):
        self.get_logger().info('Waiting for navigation server...')
        self._nav_to_pose_client.wait_for_server()
        
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
        self._send_goal_future = self._nav_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def send_waypoints(self, waypoints):
        self.get_logger().info('Waiting for follow_waypoints server...')
        self._follow_waypoints_client.wait_for_server()
        
        # Create waypoints message
        goal_msg = FollowWaypoints.Goal()
        
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set position
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            
            # Set orientation (convert euler angle to quaternion)
            theta = float(wp[2]) if len(wp) > 2 else 0.0
            q = quaternion_from_euler(0.0, 0.0, theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            goal_msg.poses.append(pose)
        
        self.get_logger().info(f'Sending {len(waypoints)} waypoints')
        
        # Send waypoints
        self._send_waypoints_future = self._follow_waypoints_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.waypoints_feedback_callback
        )
        self._send_waypoints_future.add_done_callback(self.waypoints_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.goal_succeeded = False
            return
            
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
            self.goal_succeeded = True
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
            self.goal_succeeded = False
    
    def waypoints_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoints rejected')
            self.goal_succeeded = False
            return
            
        self.get_logger().info('Waypoints accepted')
        self._get_waypoints_result_future = goal_handle.get_result_async()
        self._get_waypoints_result_future.add_done_callback(self.get_waypoints_result_callback)
        
    def get_waypoints_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoints complete! Missed waypoints: {result.missed_waypoints}')
        self.goal_succeeded = True
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
        
    def waypoints_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current waypoint: {feedback.current_waypoint}')

def main(args=None):
    rclpy.init(args=args)
    
    waypoint_sender = WaypointSender()
    
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Single waypoint:   ros2 run my_slam_pkg send_waypoints.py --pose x y [theta]")
        print("  Multiple waypoints: ros2 run my_slam_pkg send_waypoints.py --waypoints x1 y1 [theta1] x2 y2 [theta2] ...")
        rclpy.shutdown()
        return
    
    if sys.argv[1] == '--pose' and len(sys.argv) >= 4:
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        theta = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        
        waypoint_sender.send_single_goal(x, y, theta)
        rclpy.spin(waypoint_sender)
        
    elif sys.argv[1] == '--waypoints' and len(sys.argv) >= 4:
        # Parse waypoints from command line
        args = sys.argv[2:]
        waypoints = []
        
        i = 0
        while i < len(args):
            if i + 1 < len(args):  # At least x and y
                x = float(args[i])
                y = float(args[i+1])
                
                if i + 2 < len(args) and args[i+2][0] != '-':  # There's a theta
                    theta = float(args[i+2])
                    waypoints.append([x, y, theta])
                    i += 3
                else:
                    waypoints.append([x, y, 0.0])
                    i += 2
            else:
                break
        
        if waypoints:
            waypoint_sender.send_waypoints(waypoints)
            rclpy.spin(waypoint_sender)
        else:
            print("No valid waypoints provided")
            
    else:
        print("Invalid arguments")
        print("Usage:")
        print("  Single waypoint:   ros2 run my_slam_pkg send_waypoints.py --pose x y [theta]")
        print("  Multiple waypoints: ros2 run my_slam_pkg send_waypoints.py --waypoints x1 y1 [theta1] x2 y2 [theta2] ...")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()