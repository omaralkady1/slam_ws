#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
import time

class ActivateNavigation(Node):
    def __init__(self):
        super().__init__('activate_navigation')
        # Define nodes in order of dependency
        self.lifecycle_nodes = [
            '/map_server',
            '/amcl',
            '/controller_server',
            '/planner_server',
            '/behavior_server',
            '/bt_navigator'
        ]
        
        # Create service clients
        self.get_state_clients = {}
        self.change_state_clients = {}
        
        for node_name in self.lifecycle_nodes:
            self.get_state_clients[node_name] = self.create_client(
                GetState, f'{node_name}/get_state')
            self.change_state_clients[node_name] = self.create_client(
                ChangeState, f'{node_name}/change_state')
    
    def activate_all_nodes(self):
        """Ensure all navigation nodes are active"""
        self.get_logger().info('Checking all navigation nodes...')
        
        for node_name in self.lifecycle_nodes:
            if not self.get_state_clients[node_name].wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{node_name} not available, skipping')
                continue
                
            state = self.get_node_state(node_name)
            self.get_logger().info(f'{node_name} is in state: {state}')
            
            # Only try to configure if unconfigured
            if state == 'unconfigured':
                self.configure_node(node_name)
                self.get_logger().info(f'Configured {node_name}')
                # Brief delay after configuring
                time.sleep(0.5)
                
                # Get updated state
                state = self.get_node_state(node_name)
            
            # Only try to activate if inactive
            if state == 'inactive':
                self.activate_node(node_name)
                self.get_logger().info(f'Activated {node_name}')
                # Brief delay after activating
                time.sleep(0.5)
            
            # Final check
            state = self.get_node_state(node_name)
            self.get_logger().info(f'{node_name} final state: {state}')
            
            # Extra delay after AMCL to ensure it's ready
            if node_name == '/amcl':
                time.sleep(1.0)
    
    def get_node_state(self, node_name):
        """Get the current state of a lifecycle node"""
        req = GetState.Request()
        future = self.get_state_clients[node_name].call_async(req)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is None:
            self.get_logger().error(f'Failed to get state for {node_name}')
            return 'unknown'
            
        state_id = future.result().current_state.id
        
        if state_id == State.PRIMARY_STATE_UNCONFIGURED:
            return 'unconfigured'
        elif state_id == State.PRIMARY_STATE_INACTIVE:
            return 'inactive'
        elif state_id == State.PRIMARY_STATE_ACTIVE:
            return 'active'
        else:
            return f'unknown state {state_id}'
    
    def configure_node(self, node_name):
        """Configure a lifecycle node"""
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        
        future = self.change_state_clients[node_name].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is None or not future.result().success:
            self.get_logger().error(f'Failed to configure {node_name}')
    
    def activate_node(self, node_name):
        """Activate a lifecycle node"""
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        
        future = self.change_state_clients[node_name].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is None or not future.result().success:
            self.get_logger().error(f'Failed to activate {node_name}')

def main(args=None):
    rclpy.init(args=args)
    
    activator = ActivateNavigation()
    activator.activate_all_nodes()
    
    activator.get_logger().info('Done! All available nodes should now be active')
    activator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()