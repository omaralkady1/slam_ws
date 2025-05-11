#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            'goal_pose_cmd',
            10)
            
        # Create subscription to status updates
        self.subscription = self.create_subscription(
            String,
            'goal_status',
            self.status_callback,
            10)
            
        self.get_logger().info('Goal Publisher initialized')
        
    def publish_goal(self, x, y, theta=0.0):
        msg = String()
        goal_data = {
            'x': float(x),
            'y': float(y),
            'theta': float(theta)
        }
        msg.data = json.dumps(goal_data)
        
        self.get_logger().info(f'Publishing goal: x={x}, y={y}, theta={theta}')
        self.publisher.publish(msg)
        
    def status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status', '')
            message = status_data.get('message', '')
            
            self.get_logger().info(f'Goal Status: {status} - {message}')
            
            # Automatically shut down if the goal succeeded or failed
            if status in ['succeeded', 'failed', 'rejected', 'error']:
                self.get_logger().info('Navigation complete, shutting down...')
                rclpy.shutdown()
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse status JSON: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    # Check command line arguments
    if len(sys.argv) < 3:
        print("Usage: ros2 run my_slam_pkg nav_goal_client.py x y [theta]")
        print("  x, y: coordinates in meters")
        print("  theta: orientation in radians (optional, default: 0.0)")
        rclpy.shutdown()
        return
        
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    publisher = GoalPublisher()
    publisher.publish_goal(x, y, theta)
    
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()