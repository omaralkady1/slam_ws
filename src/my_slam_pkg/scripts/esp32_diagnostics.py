#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import subprocess
import os

class ESP32Diagnostics(Node):
    def __init__(self):
        super().__init__('esp32_diagnostics')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.diagnostics_pub = self.create_publisher(String, '/esp32_diagnostics', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # State tracking
        self.joint_states_received = False
        self.last_joint_state_time = None
        self.cmd_vel_sent_count = 0
        self.joint_state_count = 0
        
        # Timer for diagnostics
        self.timer = self.create_timer(2.0, self.run_diagnostics)
        
        self.get_logger().info('ESP32 Diagnostics started')
        
    def joint_state_callback(self, msg):
        """Handle joint state messages"""
        self.joint_states_received = True
        self.last_joint_state_time = time.time()
        self.joint_state_count += 1
        
        if self.joint_state_count <= 5:  # Log first few messages
            self.get_logger().info(f'Joint states received: {len(msg.name)} joints')
            for i, name in enumerate(msg.name):
                self.get_logger().info(f'  {name}: pos={msg.position[i]:.3f}, vel={msg.velocity[i]:.3f}')
    
    def send_test_command(self):
        """Send a test velocity command"""
        cmd = Twist()
        cmd.linear.x = 0.1  # Small forward velocity
        cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
        self.cmd_vel_sent_count += 1
        self.get_logger().info(f'Sent test cmd_vel #{self.cmd_vel_sent_count}')
        
        # Stop after 1 second
        time.sleep(1.0)
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def check_serial_connections(self):
        """Check available serial ports"""
        try:
            result = subprocess.run(['ls', '/dev/ttyUSB*'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                ports = result.stdout.strip().split('\n')
                self.get_logger().info(f'Available USB ports: {ports}')
                return ports
            else:
                self.get_logger().warn('No USB serial ports found')
                return []
        except Exception as e:
            self.get_logger().error(f'Error checking serial ports: {e}')
            return []
    
    def check_micro_ros_agent(self):
        """Check if micro-ROS agent is running"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True)
            if 'micro_ros_agent' in result.stdout:
                self.get_logger().info('✅ micro-ROS agent is running')
                return True
            else:
                self.get_logger().warn('❌ micro-ROS agent not found')
                return False
        except Exception as e:
            self.get_logger().error(f'Error checking micro-ROS agent: {e}')
            return False
    
    def check_controllers(self):
        """Check controller status"""
        try:
            result = subprocess.run(['ros2', 'control', 'list_controllers'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info('Controller status:')
                self.get_logger().info(result.stdout)
                return True
            else:
                self.get_logger().warn('Failed to get controller status')
                return False
        except Exception as e:
            self.get_logger().error(f'Error checking controllers: {e}')
            return False
    
    def run_diagnostics(self):
        """Run comprehensive diagnostics"""
        self.get_logger().info('=== ESP32 Diagnostics ===')
        
        # Check serial connections
        ports = self.check_serial_connections()
        
        # Check micro-ROS agent
        agent_running = self.check_micro_ros_agent()
        
        # Check controllers
        controllers_ok = self.check_controllers()
        
        # Check joint states
        if self.joint_states_received:
            time_since_last = time.time() - self.last_joint_state_time if self.last_joint_state_time else 0
            self.get_logger().info(f'✅ Joint states: {self.joint_state_count} received, last {time_since_last:.1f}s ago')
        else:
            self.get_logger().warn('❌ No joint states received from ESP32')
        
        # Send test command if everything looks good
        if agent_running and self.joint_states_received:
            self.send_test_command()
        else:
            self.get_logger().warn('Skipping test command - system not ready')
        
        # Publish diagnostic summary
        diagnostic_msg = String()
        diagnostic_msg.data = f'Ports: {len(ports)}, Agent: {agent_running}, JointStates: {self.joint_states_received}'
        self.diagnostics_pub.publish(diagnostic_msg)
        
        self.get_logger().info('=== Diagnostics Complete ===\n')

def main():
    rclpy.init()
    diagnostics = ESP32Diagnostics()
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()