#!/usr/bin/env python3
"""
EKF Diagnostics Node for ESP32 Robot
Monitors EKF performance and sensor fusion quality
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import json
import math
import time
from collections import deque


class EKFDiagnostics(Node):
    def __init__(self):
        super().__init__('ekf_diagnostics')
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',  # Raw wheel odometry
            self.wheel_odom_callback,
            qos_profile
        )
        
        self.ekf_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # EKF filtered odometry
            self.ekf_odom_callback,
            qos_profile
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_profile
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        # Publisher for diagnostics
        self.diagnostics_pub = self.create_publisher(
            String,
            '/ekf/diagnostics',
            10
        )
        
        # Data storage
        self.wheel_odom_data = deque(maxlen=100)
        self.ekf_odom_data = deque(maxlen=100)
        self.imu_data = deque(maxlen=100)
        self.cmd_vel_data = deque(maxlen=50)
        
        # State variables
        self.last_wheel_odom = None
        self.last_ekf_odom = None
        self.last_imu = None
        self.last_cmd_vel = None
        
        # Performance metrics
        self.wheel_odom_freq = 0.0
        self.ekf_odom_freq = 0.0
        self.imu_freq = 0.0
        self.cmd_vel_freq = 0.0
        
        # Fusion quality metrics
        self.position_difference = 0.0
        self.orientation_difference = 0.0
        self.velocity_difference = 0.0
        
        # Timer for diagnostics publishing
        self.diagnostics_timer = self.create_timer(2.0, self.publish_diagnostics)
        
        # Timer for frequency calculation
        self.freq_timer = self.create_timer(1.0, self.calculate_frequencies)
        
        self.get_logger().info("EKF Diagnostics node started")
    
    def wheel_odom_callback(self, msg):
        """Process wheel odometry data"""
        current_time = time.time()
        self.wheel_odom_data.append(current_time)
        self.last_wheel_odom = msg
        
        # Calculate difference with EKF if available
        if self.last_ekf_odom is not None:
            self.calculate_fusion_quality()
    
    def ekf_odom_callback(self, msg):
        """Process EKF filtered odometry data"""
        current_time = time.time()
        self.ekf_odom_data.append(current_time)
        self.last_ekf_odom = msg
        
        # Calculate difference with wheel odometry if available
        if self.last_wheel_odom is not None:
            self.calculate_fusion_quality()
    
    def imu_callback(self, msg):
        """Process IMU data"""
        current_time = time.time()
        self.imu_data.append(current_time)
        self.last_imu = msg
    
    def cmd_vel_callback(self, msg):
        """Process command velocity data"""
        current_time = time.time()
        self.cmd_vel_data.append(current_time)
        self.last_cmd_vel = msg
    
    def calculate_frequencies(self):
        """Calculate publishing frequencies for all topics"""
        current_time = time.time()
        
        # Calculate wheel odometry frequency
        if len(self.wheel_odom_data) > 1:
            time_span = current_time - self.wheel_odom_data[0]
            if time_span > 0:
                self.wheel_odom_freq = len(self.wheel_odom_data) / time_span
        
        # Calculate EKF odometry frequency
        if len(self.ekf_odom_data) > 1:
            time_span = current_time - self.ekf_odom_data[0]
            if time_span > 0:
                self.ekf_odom_freq = len(self.ekf_odom_data) / time_span
        
        # Calculate IMU frequency
        if len(self.imu_data) > 1:
            time_span = current_time - self.imu_data[0]
            if time_span > 0:
                self.imu_freq = len(self.imu_data) / time_span
        
        # Calculate command velocity frequency
        if len(self.cmd_vel_data) > 1:
            time_span = current_time - self.cmd_vel_data[0]
            if time_span > 0:
                self.cmd_vel_freq = len(self.cmd_vel_data) / time_span
    
    def calculate_fusion_quality(self):
        """Calculate quality metrics for sensor fusion"""
        if self.last_wheel_odom is None or self.last_ekf_odom is None:
            return
        
        # Position difference
        wheel_pos = self.last_wheel_odom.pose.pose.position
        ekf_pos = self.last_ekf_odom.pose.pose.position
        
        self.position_difference = math.sqrt(
            (wheel_pos.x - ekf_pos.x)**2 + 
            (wheel_pos.y - ekf_pos.y)**2
        )
        
        # Orientation difference (yaw only for 2D)
        wheel_quat = self.last_wheel_odom.pose.pose.orientation
        ekf_quat = self.last_ekf_odom.pose.pose.orientation
        
        wheel_yaw = self.quaternion_to_yaw(wheel_quat)
        ekf_yaw = self.quaternion_to_yaw(ekf_quat)
        
        self.orientation_difference = abs(self.normalize_angle(wheel_yaw - ekf_yaw))
        
        # Velocity difference
        wheel_vel = self.last_wheel_odom.twist.twist.linear
        ekf_vel = self.last_ekf_odom.twist.twist.linear
        
        self.velocity_difference = math.sqrt(
            (wheel_vel.x - ekf_vel.x)**2 + 
            (wheel_vel.y - ekf_vel.y)**2
        )
    
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_sensor_status(self):
        """Get status of all sensors"""
        current_time = time.time()
        status = {}
        
        # Check if sensors are publishing
        status['wheel_odom_active'] = len(self.wheel_odom_data) > 0 and (current_time - self.wheel_odom_data[-1]) < 1.0
        status['ekf_odom_active'] = len(self.ekf_odom_data) > 0 and (current_time - self.ekf_odom_data[-1]) < 1.0
        status['imu_active'] = len(self.imu_data) > 0 and (current_time - self.imu_data[-1]) < 1.0
        status['cmd_vel_active'] = len(self.cmd_vel_data) > 0 and (current_time - self.cmd_vel_data[-1]) < 2.0
        
        return status
    
    def assess_fusion_quality(self):
        """Assess the quality of sensor fusion"""
        quality = "UNKNOWN"
        
        if self.position_difference < 0.05 and self.orientation_difference < 0.1:
            quality = "EXCELLENT"
        elif self.position_difference < 0.1 and self.orientation_difference < 0.2:
            quality = "GOOD"
        elif self.position_difference < 0.2 and self.orientation_difference < 0.3:
            quality = "FAIR"
        else:
            quality = "POOR"
        
        return quality
    
    def publish_diagnostics(self):
        """Publish comprehensive diagnostics"""
        status = self.get_sensor_status()
        fusion_quality = self.assess_fusion_quality()
        
        # Create diagnostics message
        diagnostics = {
            'timestamp': time.time(),
            'node': 'ekf_diagnostics',
            'sensors': {
                'wheel_odometry': {
                    'active': status['wheel_odom_active'],
                    'frequency': round(self.wheel_odom_freq, 2),
                    'status': 'OK' if status['wheel_odom_active'] and self.wheel_odom_freq > 50 else 'WARNING'
                },
                'ekf_odometry': {
                    'active': status['ekf_odom_active'],
                    'frequency': round(self.ekf_odom_freq, 2),
                    'status': 'OK' if status['ekf_odom_active'] and self.ekf_odom_freq > 50 else 'WARNING'
                },
                'imu': {
                    'active': status['imu_active'],
                    'frequency': round(self.imu_freq, 2),
                    'status': 'OK' if status['imu_active'] and self.imu_freq > 50 else 'WARNING'
                },
                'command_velocity': {
                    'active': status['cmd_vel_active'],
                    'frequency': round(self.cmd_vel_freq, 2),
                    'status': 'OK' if status['cmd_vel_active'] else 'INFO'
                }
            },
            'fusion_quality': {
                'overall': fusion_quality,
                'position_difference': round(self.position_difference, 4),
                'orientation_difference': round(self.orientation_difference, 4),
                'velocity_difference': round(self.velocity_difference, 4)
            },
            'current_state': {}
        }
        
        # Add current state information
        if self.last_ekf_odom:
            pos = self.last_ekf_odom.pose.pose.position
            vel = self.last_ekf_odom.twist.twist.linear
            ang_vel = self.last_ekf_odom.twist.twist.angular
            
            diagnostics['current_state'] = {
                'position': {
                    'x': round(pos.x, 3),
                    'y': round(pos.y, 3),
                    'yaw': round(self.quaternion_to_yaw(self.last_ekf_odom.pose.pose.orientation), 3)
                },
                'velocity': {
                    'linear_x': round(vel.x, 3),
                    'linear_y': round(vel.y, 3),
                    'angular_z': round(ang_vel.z, 3)
                }
            }
        
        # Add IMU state if available
        if self.last_imu:
            diagnostics['imu_state'] = {
                'angular_velocity': {
                    'x': round(self.last_imu.angular_velocity.x, 3),
                    'y': round(self.last_imu.angular_velocity.y, 3),
                    'z': round(self.last_imu.angular_velocity.z, 3)
                },
                'linear_acceleration': {
                    'x': round(self.last_imu.linear_acceleration.x, 3),
                    'y': round(self.last_imu.linear_acceleration.y, 3),
                    'z': round(self.last_imu.linear_acceleration.z, 3)
                }
            }
        
        # Publish diagnostics
        msg = String()
        msg.data = json.dumps(diagnostics, indent=2)
        self.diagnostics_pub.publish(msg)
        
        # Log summary to console
        self.get_logger().info(
            f"EKF Status: Fusion={fusion_quality}, "
            f"Pos_diff={self.position_difference:.3f}m, "
            f"Ori_diff={self.orientation_difference:.3f}rad, "
            f"Freqs: Wheel={self.wheel_odom_freq:.1f}Hz, "
            f"EKF={self.ekf_odom_freq:.1f}Hz, "
            f"IMU={self.imu_freq:.1f}Hz"
        )


def main(args=None):
    rclpy.init(args=args)
    
    ekf_diagnostics = EKFDiagnostics()
    
    try:
        rclpy.spin(ekf_diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        ekf_diagnostics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()