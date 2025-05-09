#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import json
from std_msgs.msg import String

class NavGoalListener(Node):
    def __init__(self):
        super().__init__('nav_goal_listener')
        
        # Create action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create subscription to goal commands
        self.subscription = self.create_subscription(
            String,
            'goal_pose_cmd',
            self.goal_callback,
            10)
            
        # Status publisher
        self.status_publisher = self.create_publisher(
            String,
            'goal_status',
            10)
            
        self.get_logger().info('Navigation Goal Listener node initialized')
        self.get_logger().info('Listening for goals on topic: goal_pose_cmd')
        self.get_logger().info('Expected format: {"x": 1.0, "y": 2.0, "theta": 0.0}')
        
        # Wait for Nav2 action server
        self._action_client.wait_for_server()
        self.get_logger().info('Navigation server is available')

    def goal_callback(self, msg):
        try:
            # Parse goal from JSON
            goal_data = json.loads(msg.data)
            x = float(goal_data.get('x', 0.0))
            y = float(goal_data.get('y', 0.0))
            theta = float(goal_data.get('theta', 0.0))
            
            self.get_logger().info(f'Received goal: x={x}, y={y}, theta={theta}')
            self.send_goal(x, y, theta)
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse JSON: {msg.data}')
            self.publish_status('error', 'Invalid JSON format')
        except KeyError as e:
            self.get_logger().error(f'Missing key in JSON: {e}')
            self.publish_status('error', f'Missing key in JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing goal: {e}')
            self.publish_status('error', str(e))

    def send_goal(self, x, y, theta=0.0):
        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert euler angle to quaternion)
        q = quaternion_from_euler(0.0, 0.0, theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        self.publish_status('sending', f'x={x}, y={y}, theta={theta}')
        
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
            self.publish_status('rejected', 'Goal was rejected by navigation server')
            return
            
        self.get_logger().info('Goal accepted')
        self.publish_status('accepted', 'Goal accepted by navigation server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
            self.publish_status('succeeded', 'Navigation complete')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
            self.publish_status('failed', f'Navigation failed with status: {status}')
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
        self.publish_status('in_progress', f'Distance remaining: {feedback.distance_remaining:.2f} meters')
    
    def publish_status(self, status_type, message):
        status_msg = String()
        status_data = {
            'status': status_type,
            'message': message,
            'timestamp': self.get_clock().now().seconds_nanoseconds()[0]
        }
        status_msg.data = json.dumps(status_data)
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    nav_goal_listener = NavGoalListener()
    
    try:
        rclpy.spin(nav_goal_listener)
    except KeyboardInterrupt:
        pass
    finally:
        nav_goal_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()