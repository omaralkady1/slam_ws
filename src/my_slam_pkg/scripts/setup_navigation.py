#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time
import sys

class NavigationSetup(Node):
    def __init__(self):
        super().__init__('navigation_setup')
        
        # Define lifecycle nodes
        self.lifecycle_nodes = [
            '/map_server',
            '/amcl',
            '/controller_server',
            '/planner_server',
            '/behavior_server',
            '/bt_navigator'
        ]
        
        # Create service clients
        self.change_state_clients = {}
        self.get_state_clients = {}
        
        for node_name in self.lifecycle_nodes:
            self.change_state_clients[node_name] = self.create_client(
                ChangeState, f'{node_name}/change_state')
            self.get_state_clients[node_name] = self.create_client(
                GetState, f'{node_name}/get_state')
        
        # Create publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
        
        self.get_logger().info('Navigation setup node initialized')
    
    def configure_lifecycle_nodes(self):
        """Configure all lifecycle nodes"""
        self.get_logger().info('Configuring lifecycle nodes...')
        
        for node_name in self.lifecycle_nodes:
            if not self.change_state_clients[node_name].wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {node_name}/change_state not available')
                continue
            
            # Check current state
            state = self.get_node_state(node_name)
            self.get_logger().info(f'Node {node_name} state: {state}')
            
            # Configure if unconfigured
            if state == 'unconfigured':
                self.change_state(node_name, Transition.TRANSITION_CONFIGURE)
                self.get_logger().info(f'Configured {node_name}')
            
            # Activate if inactive
            state = self.get_node_state(node_name)
            if state == 'inactive':
                self.change_state(node_name, Transition.TRANSITION_ACTIVATE)
                self.get_logger().info(f'Activated {node_name}')
            
            # Final check
            state = self.get_node_state(node_name)
            self.get_logger().info(f'Node {node_name} final state: {state}')
    
    def get_node_state(self, node_name):
        """Get the state of a lifecycle node"""
        if not self.get_state_clients[node_name].wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {node_name}/get_state not available')
            return 'unknown'
        
        req = GetState.Request()
        future = self.get_state_clients[node_name].call_async(req)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            state_id = response.current_state.id
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                return 'unconfigured'
            elif state_id == State.PRIMARY_STATE_INACTIVE:
                return 'inactive'
            elif state_id == State.PRIMARY_STATE_ACTIVE:
                return 'active'
            else:
                return f'unknown state {state_id}'
        else:
            self.get_logger().error(f'Failed to get state for {node_name}')
            return 'unknown'
    
    def change_state(self, node_name, transition_id):
        """Change the state of a lifecycle node"""
        req = ChangeState.Request()
        req.transition.id = transition_id
        
        future = self.change_state_clients[node_name].call_async(req)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            if response.success:
                return True
            else:
                self.get_logger().error(f'Failed to change state for {node_name}')
                return False
        else:
            self.get_logger().error(f'Service call failed for {node_name}')
            return False
    
    def set_initial_pose(self, x, y, theta):
        """Set the initial pose for AMCL"""
        self.get_logger().info(f'Setting initial pose: x={x}, y={y}, theta={theta}')
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Orientation
        q = quaternion_from_euler(0.0, 0.0, theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Covariance (small values = high certainty)
        covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        msg.pose.covariance = covariance
        
        # Publish the pose
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('Initial pose published')
    
    def send_goal(self, x, y, theta):
        """Send a navigation goal"""
        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        # Orientation
        q = quaternion_from_euler(0.0, 0.0, theta)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        # Publish the goal
        self.goal_pub.publish(msg)
        self.get_logger().info('Goal published')

def main(args=None):
    rclpy.init(args=args)
    setup_node = NavigationSetup()
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == 'configure':
            # Configure and activate nodes
            setup_node.configure_lifecycle_nodes()
        
        elif command == 'initial_pose':
            # Set initial pose
            x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
            y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            theta = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
            
            setup_node.set_initial_pose(x, y, theta)
            # Wait to ensure the pose is published
            time.sleep(1.0)
        
        elif command == 'goal':
            # Send navigation goal
            x = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            theta = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
            
            setup_node.send_goal(x, y, theta)
            # Wait to ensure the goal is published
            time.sleep(1.0)
        
        elif command == 'setup_all':
            # Do everything
            setup_node.configure_lifecycle_nodes()
            
            # Wait for nodes to be fully active
            time.sleep(2.0)
            
            # Set initial pose
            x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
            y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            theta = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
            
            setup_node.set_initial_pose(x, y, theta)
            
            # Wait for AMCL to process initial pose
            time.sleep(2.0)
            
            # Send a navigation goal (optional)
            if len(sys.argv) > 5 and sys.argv[5] == 'withgoal':
                goal_x = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0
                goal_y = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
                goal_theta = float(sys.argv[8]) if len(sys.argv) > 8 else 0.0
                
                setup_node.send_goal(goal_x, goal_y, goal_theta)
        
        else:
            print("Unknown command. Use: configure, initial_pose, goal, or setup_all")
    else:
        print("Usage: setup_navigation.py [configure|initial_pose|goal|setup_all] [args...]")
        print("  configure: Configure and activate all lifecycle nodes")
        print("  initial_pose x y theta: Set initial pose")
        print("  goal x y theta: Send navigation goal")
        print("  setup_all x y theta [withgoal goal_x goal_y goal_theta]: Do all of the above")
    
    setup_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()