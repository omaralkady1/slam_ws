#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time

class VelocityMonitor(Node):
    def __init__(self):
        super().__init__('velocity_monitor')
        
        # Subscribers for different velocity topics
        self.teleop_sub = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.teleop_callback, 10)
        self.joy_sub = self.create_subscription(
            Twist, '/joy_cmd_vel', self.joy_callback, 10)
        self.nav_sub = self.create_subscription(
            Twist, '/cmd_vel', self.nav_callback, 10)
        self.output_sub = self.create_subscription(
            Twist, '/cmd_vel_out', self.output_callback, 10)
        self.controller_sub = self.create_subscription(
            Twist, '/diff_drive_controller/cmd_vel', self.controller_callback, 10)
        
        # Joint states subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # State tracking
        self.last_teleop_time = 0
        self.last_joy_time = 0
        self.last_nav_time = 0
        self.last_output_time = 0
        self.last_controller_time = 0
        self.last_joint_state_time = 0
        
        self.teleop_active = False
        self.joy_active = False
        self.nav_active = False
        self.output_active = False
        self.controller_active = False
        self.joint_states_active = False
        
        # Latest velocities
        self.latest_teleop = Twist()
        self.latest_joy = Twist()
        self.latest_nav = Twist()
        self.latest_output = Twist()
        self.latest_controller = Twist()
        
        # Timer for status reporting
        self.timer = self.create_timer(1.0, self.report_status)
        
        self.get_logger().info('Velocity Monitor started - monitoring teleop data flow')
    
    def teleop_callback(self, msg):
        self.latest_teleop = msg
        self.last_teleop_time = time.time()
        self.teleop_active = True
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'Teleop: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def joy_callback(self, msg):
        self.latest_joy = msg
        self.last_joy_time = time.time()
        self.joy_active = True
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'Joy: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def nav_callback(self, msg):
        self.latest_nav = msg
        self.last_nav_time = time.time()
        self.nav_active = True
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'Nav: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def output_callback(self, msg):
        self.latest_output = msg
        self.last_output_time = time.time()
        self.output_active = True
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'Output: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def controller_callback(self, msg):
        self.latest_controller = msg
        self.last_controller_time = time.time()
        self.controller_active = True
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'Controller: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def joint_state_callback(self, msg):
        self.last_joint_state_time = time.time()
        self.joint_states_active = True
        
        # Log joint velocities if any wheel is moving
        if len(msg.velocity) >= 4:
            max_vel = max(abs(v) for v in msg.velocity[:4])
            if max_vel > 0.01:
                wheel_vels = [f'{v:.2f}' for v in msg.velocity[:4]]
                self.get_logger().info(f'Wheel velocities: {wheel_vels}')
    
    def report_status(self):
        current_time = time.time()
        
        # Check active status (received message in last 2 seconds)
        self.teleop_active = (current_time - self.last_teleop_time) < 2.0
        self.joy_active = (current_time - self.last_joy_time) < 2.0
        self.nav_active = (current_time - self.last_nav_time) < 2.0
        self.output_active = (current_time - self.last_output_time) < 2.0
        self.controller_active = (current_time - self.last_controller_time) < 2.0
        self.joint_states_active = (current_time - self.last_joint_state_time) < 2.0
        
        # Create status report
        status = []
        status.append(f"Teleop: {'✅' if self.teleop_active else '❌'}")
        status.append(f"Joy: {'✅' if self.joy_active else '❌'}")
        status.append(f"Nav: {'✅' if self.nav_active else '❌'}")
        status.append(f"Output: {'✅' if self.output_active else '❌'}")
        status.append(f"Controller: {'✅' if self.controller_active else '❌'}")
        status.append(f"JointStates: {'✅' if self.joint_states_active else '❌'}")
        
        self.get_logger().info(f"Status: {' | '.join(status)}")
        
        # Warn about potential issues
        if self.teleop_active and not self.output_active:
            self.get_logger().warn("Teleop active but no output - check twist_mux!")
        
        if self.output_active and not self.controller_active:
            self.get_logger().warn("Output active but controller not receiving - check remapping!")
        
        if self.controller_active and not self.joint_states_active:
            self.get_logger().warn("Controller active but no joint states - check ESP32 connection!")

def main():
    rclpy.init()
    monitor = VelocityMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()