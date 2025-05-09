#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
import math
import sys

class NavVisualization(Node):
    def __init__(self):
        super().__init__('nav_visualization')
        
        # Create action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for visualization markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            10)
            
        # Initialize marker id
        self.marker_id = 0
        
        self.get_logger().info('Navigation Visualization node initialized')
        
    def wait_for_navigation_server(self):
        self.get_logger().info('Waiting for navigation server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Navigation server available!')
        
    def send_goal(self, x, y, theta=0.0):
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
        
        # Visualize the goal
        self.visualize_waypoint(x, y, theta)
        
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
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} meters')
        
    def visualize_waypoint(self, x, y, theta):
        # Create marker array message
        marker_array = MarkerArray()
        
        # Create a sphere marker for the position
        position_marker = Marker()
        position_marker.header.frame_id = 'map'
        position_marker.header.stamp = self.get_clock().now().to_msg()
        position_marker.ns = 'waypoints'
        position_marker.id = self.marker_id
        self.marker_id += 1
        
        position_marker.type = Marker.SPHERE
        position_marker.action = Marker.ADD
        
        # Set position
        position_marker.pose.position.x = x
        position_marker.pose.position.y = y
        position_marker.pose.position.z = 0.1
        
        # Set marker scale
        position_marker.scale.x = 0.2
        position_marker.scale.y = 0.2
        position_marker.scale.z = 0.2
        
        # Set marker color (red)
        position_marker.color.r = 1.0
        position_marker.color.g = 0.0
        position_marker.color.b = 0.0
        position_marker.color.a = 1.0
        
        # Make it last forever
        position_marker.lifetime.sec = 0
        position_marker.lifetime.nanosec = 0
        
        # Create an arrow marker for the orientation
        orientation_marker = Marker()
        orientation_marker.header.frame_id = 'map'
        orientation_marker.header.stamp = self.get_clock().now().to_msg()
        orientation_marker.ns = 'orientations'
        orientation_marker.id = self.marker_id
        self.marker_id += 1
        
        orientation_marker.type = Marker.ARROW
        orientation_marker.action = Marker.ADD
        
        # Set position
        orientation_marker.pose.position.x = x
        orientation_marker.pose.position.y = y
        orientation_marker.pose.position.z = 0.1
        
        # Set orientation
        q = quaternion_from_euler(0.0, 0.0, theta)
        orientation_marker.pose.orientation.x = q[0]
        orientation_marker.pose.orientation.y = q[1]
        orientation_marker.pose.orientation.z = q[2]
        orientation_marker.pose.orientation.w = q[3]
        
        # Set marker scale (arrow dimensions)
        orientation_marker.scale.x = 0.5  # arrow length
        orientation_marker.scale.y = 0.1  # arrow width
        orientation_marker.scale.z = 0.1  # arrow height
        
        # Set marker color (blue)
        orientation_marker.color.r = 0.0
        orientation_marker.color.g = 0.0
        orientation_marker.color.b = 1.0
        orientation_marker.color.a = 1.0
        
        # Make it last forever
        orientation_marker.lifetime.sec = 0
        orientation_marker.lifetime.nanosec = 0
        
        # Create a text marker for the coordinates
        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = 'coordinates'
        text_marker.id = self.marker_id
        self.marker_id += 1
        
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Set position slightly above the waypoint
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = 0.3
        
        # Set text content
        text_marker.text = f"({x:.2f}, {y:.2f}, {theta:.2f})"
        
        # Set marker scale (text size)
        text_marker.scale.z = 0.2  # text height
        
        # Set marker color (white)
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        # Make it last forever
        text_marker.lifetime.sec = 0
        text_marker.lifetime.nanosec = 0
        
        # Add markers to array
        marker_array.markers.append(position_marker)
        marker_array.markers.append(orientation_marker)
        marker_array.markers.append(text_marker)
        
        # Publish the marker array
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    # Check command line arguments
    if len(sys.argv) < 3:
        print("Usage: ros2 run my_slam_pkg nav_visualization.py x y [theta]")
        print("  x, y: coordinates in meters")
        print("  theta: orientation in radians (optional, default: 0.0)")
        rclpy.shutdown()
        return
        
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    visualization = NavVisualization()
    visualization.wait_for_navigation_server()
    visualization.send_goal(x, y, theta)
    
    rclpy.spin(visualization)
    rclpy.shutdown()

if __name__ == '__main__':
    main()