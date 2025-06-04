#!/usr/bin/env python3
# scripts/hardware_diagnostics.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import time
import math
import numpy as np

class HardwareDiagnostics(Node):
    def __init__(self):
        super().__init__('hardware_diagnostics')
        
        # Parameters
        self.declare_parameter('update_rate', 2.0)  # Hz
        self.declare_parameter('watchdog_timeout', 1.0)  # seconds
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/diff_drive_controller/cmd_vel', self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.esp32_diag_sub = self.create_subscription(
            String, '/esp32/diagnostics', self.esp32_diag_callback, 10)
        self.ros2_control_status_sub = self.create_subscription(
            String, '/ros2_control/esp32_status', self.ros2_control_status_callback, 10)
        
        # Publishers
        self.diag_pub = self.create_publisher(
            String, '/hardware_diagnostics', 10)
        
        # Data storage
        self.joint_data = {}
        self.cmd_data = None
        self.odom_data = None
        self.esp32_diag = None
        self.ros2_control_status = None
        
        # Timing
        self.last_joint_time = None
        self.last_cmd_time = None
        self.last_odom_time = None
        self.last_esp32_diag_time = None
        
        # Statistics
        self.joint_msg_count = 0
        self.cmd_msg_count = 0
        self.odom_msg_count = 0
        self.joint_msg_rate = 0.0
        self.cmd_msg_rate = 0.0
        self.odom_msg_rate = 0.0
        
        # Performance tracking
        self.velocity_errors = []
        self.position_errors = []
        self.control_latency = []
        
        # Timers
        update_rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0/update_rate, self.diagnostics_callback)
        self.rate_timer = self.create_timer(1.0, self.calculate_rates)
        
        self.get_logger().info('Hardware diagnostics started')
        
    def joint_callback(self, msg):
        self.joint_msg_count += 1
        self.last_joint_time = self.get_clock().now()
        
        # Store joint data
        for i, name in enumerate(msg.name):
            if name not in self.joint_data:
                self.joint_data[name] = {'position': [], 'velocity': [], 'effort': []}
            
            # Keep last 100 samples
            self.joint_data[name]['position'].append(msg.position[i] if i < len(msg.position) else 0.0)
            self.joint_data[name]['velocity'].append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
            self.joint_data[name]['effort'].append(msg.effort[i] if i < len(msg.effort) else 0.0)
            
            # Limit history
            for key in ['position', 'velocity', 'effort']:
                if len(self.joint_data[name][key]) > 100:
                    self.joint_data[name][key].pop(0)
    
    def cmd_callback(self, msg):
        self.cmd_msg_count += 1
        self.last_cmd_time = self.get_clock().now()
        
        self.cmd_data = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
            'time': self.last_cmd_time
        }
        
        # Calculate control latency if we have joint states
        if self.last_joint_time:
            latency = (self.last_cmd_time - self.last_joint_time).nanoseconds / 1e6  # ms
            self.control_latency.append(latency)
            if len(self.control_latency) > 100:
                self.control_latency.pop(0)
    
    def odom_callback(self, msg):
        self.odom_msg_count += 1
        self.last_odom_time = self.get_clock().now()
        
        self.odom_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        }
    
    def esp32_diag_callback(self, msg):
        self.esp32_diag = msg.data
        self.last_esp32_diag_time = self.get_clock().now()
    
    def ros2_control_status_callback(self, msg):
        self.ros2_control_status = msg.data
    
    def calculate_rates(self):
        # Calculate message rates
        self.joint_msg_rate = self.joint_msg_count
        self.cmd_msg_rate = self.cmd_msg_count
        self.odom_msg_rate = self.odom_msg_count
        
        # Reset counters
        self.joint_msg_count = 0
        self.cmd_msg_count = 0
        self.odom_msg_count = 0
    
    def diagnostics_callback(self):
        current_time = self.get_clock().now()
        watchdog_timeout = self.get_parameter('watchdog_timeout').value
        
        # Create diagnostic message
        diag = {
            'timestamp': current_time.nanoseconds,
            'system_status': 'OK',
            'warnings': [],
            'errors': []
        }
        
        # Check joint state connection
        if self.last_joint_time:
            joint_age = (current_time - self.last_joint_time).nanoseconds / 1e9
            if joint_age > watchdog_timeout:
                diag['errors'].append(f'Joint states timeout: {joint_age:.2f}s')
                diag['system_status'] = 'ERROR'
        else:
            diag['errors'].append('No joint states received')
            diag['system_status'] = 'ERROR'
        
        # Check command velocity
        if self.last_cmd_time:
            cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9
            if cmd_age > watchdog_timeout and self.cmd_data:
                if abs(self.cmd_data['linear_x']) > 0.01 or abs(self.cmd_data['angular_z']) > 0.01:
                    diag['warnings'].append(f'Command velocity stale: {cmd_age:.2f}s')
        
        # Check odometry
        if self.last_odom_time:
            odom_age = (current_time - self.last_odom_time).nanoseconds / 1e9
            if odom_age > watchdog_timeout:
                diag['warnings'].append(f'Odometry timeout: {odom_age:.2f}s')
        else:
            diag['warnings'].append('No odometry received')
        
        # Check ESP32 diagnostics
        if self.last_esp32_diag_time:
            esp32_age = (current_time - self.last_esp32_diag_time).nanoseconds / 1e9
            if esp32_age > 5.0:  # ESP32 sends diagnostics every 5 seconds
                diag['warnings'].append(f'ESP32 diagnostics stale: {esp32_age:.2f}s')
        
        # Message rates
        diag['message_rates'] = {
            'joint_states': f'{self.joint_msg_rate:.1f} Hz',
            'cmd_vel': f'{self.cmd_msg_rate:.1f} Hz',
            'odom': f'{self.odom_msg_rate:.1f} Hz'
        }
        
        # Joint statistics
        if self.joint_data:
            joint_stats = {}
            for name, data in self.joint_data.items():
                if data['velocity']:
                    vel_array = np.array(data['velocity'][-10:])  # Last 10 samples
                    joint_stats[name] = {
                        'avg_velocity': float(np.mean(vel_array)),
                        'std_velocity': float(np.std(vel_array)),
                        'max_velocity': float(np.max(np.abs(vel_array)))
                    }
            diag['joint_statistics'] = joint_stats
        
        # Control performance
        if self.control_latency:
            latency_array = np.array(self.control_latency)
            diag['control_performance'] = {
                'avg_latency_ms': float(np.mean(latency_array)),
                'max_latency_ms': float(np.max(latency_array)),
                'min_latency_ms': float(np.min(latency_array))
            }
        
        # Current state
        if self.cmd_data:
            diag['current_command'] = {
                'linear_x': self.cmd_data['linear_x'],
                'angular_z': self.cmd_data['angular_z']
            }
        
        if self.odom_data:
            diag['current_odom'] = {
                'position': {'x': self.odom_data['x'], 'y': self.odom_data['y']},
                'velocity': {'linear_x': self.odom_data['linear_x'], 
                           'angular_z': self.odom_data['angular_z']}
            }
        
        # ESP32 status
        if self.esp32_diag:
            diag['esp32_status'] = self.esp32_diag
        
        # ROS2 Control status
        if self.ros2_control_status:
            diag['ros2_control_status'] = self.ros2_control_status
        
        # Publish diagnostic summary
        self.publish_diagnostics(diag)
        
        # Log summary
        if diag['system_status'] == 'ERROR':
            self.get_logger().error(f"System errors: {', '.join(diag['errors'])}")
        elif diag['warnings']:
            self.get_logger().warn(f"System warnings: {', '.join(diag['warnings'])}")
        else:
            self.get_logger().info('System status: OK')
    
    def publish_diagnostics(self, diag):
        # Format as human-readable string
        msg = String()
        lines = [
            f"=== Hardware Diagnostics ===",
            f"Status: {diag['system_status']}",
            f"Time: {time.time():.2f}",
            ""
        ]
        
        if diag.get('errors'):
            lines.append("ERRORS:")
            for error in diag['errors']:
                lines.append(f"  - {error}")
            lines.append("")
        
        if diag.get('warnings'):
            lines.append("WARNINGS:")
            for warning in diag['warnings']:
                lines.append(f"  - {warning}")
            lines.append("")
        
        if diag.get('message_rates'):
            lines.append("MESSAGE RATES:")
            for topic, rate in diag['message_rates'].items():
                lines.append(f"  {topic}: {rate}")
            lines.append("")
        
        if diag.get('current_command'):
            cmd = diag['current_command']
            lines.append(f"CURRENT COMMAND: linear={cmd['linear_x']:.3f}, angular={cmd['angular_z']:.3f}")
        
        if diag.get('current_odom'):
            odom = diag['current_odom']
            lines.append(f"CURRENT POSITION: x={odom['position']['x']:.3f}, y={odom['position']['y']:.3f}")
            lines.append(f"CURRENT VELOCITY: linear={odom['velocity']['linear_x']:.3f}, angular={odom['velocity']['angular_z']:.3f}")
        
        if diag.get('control_performance'):
            perf = diag['control_performance']
            lines.append(f"\nCONTROL LATENCY: avg={perf['avg_latency_ms']:.1f}ms, max={perf['max_latency_ms']:.1f}ms")
        
        if diag.get('joint_statistics'):
            lines.append("\nJOINT VELOCITIES (rad/s):")
            for name, stats in diag['joint_statistics'].items():
                lines.append(f"  {name}: avg={stats['avg_velocity']:.3f}, std={stats['std_velocity']:.3f}")
        
        msg.data = '\n'.join(lines)
        self.diag_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()