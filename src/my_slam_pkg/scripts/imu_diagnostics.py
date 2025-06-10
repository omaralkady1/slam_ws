#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf_transformations
import numpy as np
import time
import math

class IMUDiagnostics(Node):
    def __init__(self):
        super().__init__('imu_diagnostics')
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Data storage
        self.imu_data = None
        self.joint_data = None
        self.odom_data = None
        self.cmd_vel_data = None
        
        # Health tracking
        self.imu_received = False
        self.joint_received = False
        self.odom_received = False
        self.cmd_vel_received = False
        
        self.imu_count = 0
        self.joint_count = 0
        self.odom_count = 0
        
        # Timing tracking
        self.start_time = time.time()
        self.last_imu_time = 0
        self.last_joint_time = 0
        self.last_odom_time = 0
        
        # Quality metrics
        self.imu_frequencies = []
        self.joint_frequencies = []
        self.odom_frequencies = []
        
        # Fusion quality tracking
        self.wheel_angular_velocities = []
        self.imu_angular_velocities = []
        self.velocity_differences = []
        
        # Timer for periodic reports
        self.timer = self.create_timer(2.0, self.publish_diagnostics)
        
        self.get_logger().info('IMU Diagnostics node started')
        self.get_logger().info('Monitoring IMU-wheel odometry fusion quality')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        current_time = time.time()
        
        if self.last_imu_time > 0:
            frequency = 1.0 / (current_time - self.last_imu_time)
            self.imu_frequencies.append(frequency)
            if len(self.imu_frequencies) > 50:
                self.imu_frequencies.pop(0)
        
        self.last_imu_time = current_time
        self.imu_count += 1
        self.imu_received = True
        self.imu_data = msg
        
        # Store angular velocity for fusion analysis
        self.imu_angular_velocities.append(msg.angular_velocity.z)
        if len(self.imu_angular_velocities) > 100:
            self.imu_angular_velocities.pop(0)
        
        # Check for IMU health issues
        self.check_imu_health(msg)
    
    def joint_callback(self, msg):
        """Process joint state data"""
        current_time = time.time()
        
        if self.last_joint_time > 0:
            frequency = 1.0 / (current_time - self.last_joint_time)
            self.joint_frequencies.append(frequency)
            if len(self.joint_frequencies) > 50:
                self.joint_frequencies.pop(0)
        
        self.last_joint_time = current_time
        self.joint_count += 1
        self.joint_received = True
        self.joint_data = msg
        
        # Calculate wheel-derived angular velocity
        if len(msg.velocity) >= 4:
            # Assuming differential drive: angular_vel = (right - left) / wheel_separation
            wheel_separation = 0.34  # meters
            left_avg = (msg.velocity[0] + msg.velocity[2]) / 2.0  # FL + RL
            right_avg = (msg.velocity[1] + msg.velocity[3]) / 2.0  # FR + RR
            
            wheel_angular_vel = (right_avg - left_avg) * 0.05 / wheel_separation  # wheel_radius = 0.05
            self.wheel_angular_velocities.append(wheel_angular_vel)
            if len(self.wheel_angular_velocities) > 100:
                self.wheel_angular_velocities.pop(0)
    
    def odom_callback(self, msg):
        """Process odometry data"""
        current_time = time.time()
        
        if self.last_odom_time > 0:
            frequency = 1.0 / (current_time - self.last_odom_time)
            self.odom_frequencies.append(frequency)
            if len(self.odom_frequencies) > 50:
                self.odom_frequencies.pop(0)
        
        self.last_odom_time = current_time
        self.odom_count += 1
        self.odom_received = True
        self.odom_data = msg
    
    def cmd_vel_callback(self, msg):
        """Process command velocity data"""
        self.cmd_vel_received = True
        self.cmd_vel_data = msg
    
    def check_imu_health(self, msg):
        """Check IMU data for health issues"""
        issues = []
        
        # Check for unrealistic values
        if abs(msg.angular_velocity.x) > 10.0:
            issues.append(f"High gyro X: {msg.angular_velocity.x:.3f} rad/s")
        if abs(msg.angular_velocity.y) > 10.0:
            issues.append(f"High gyro Y: {msg.angular_velocity.y:.3f} rad/s")
        if abs(msg.angular_velocity.z) > 10.0:
            issues.append(f"High gyro Z: {msg.angular_velocity.z:.3f} rad/s")
        
        if abs(msg.linear_acceleration.x) > 50.0:
            issues.append(f"High accel X: {msg.linear_acceleration.x:.3f} m/s²")
        if abs(msg.linear_acceleration.y) > 50.0:
            issues.append(f"High accel Y: {msg.linear_acceleration.y:.3f} m/s²")
        if abs(msg.linear_acceleration.z) > 50.0:
            issues.append(f"High accel Z: {msg.linear_acceleration.z:.3f} m/s²")
        
        # Check for NaN values
        if math.isnan(msg.angular_velocity.z) or math.isnan(msg.linear_acceleration.x):
            issues.append("NaN values detected in IMU data")
        
        # Check orientation quaternion validity
        quat_norm = math.sqrt(
            msg.orientation.x**2 + msg.orientation.y**2 + 
            msg.orientation.z**2 + msg.orientation.w**2
        )
        if abs(quat_norm - 1.0) > 0.1:
            issues.append(f"Invalid quaternion norm: {quat_norm:.3f}")
        
        if issues:
            self.get_logger().warn("IMU Health Issues:")
            for issue in issues:
                self.get_logger().warn(f"  - {issue}")
    
    def analyze_fusion_quality(self):
        """Analyze the quality of IMU-wheel fusion"""
        if (len(self.wheel_angular_velocities) < 10 or 
            len(self.imu_angular_velocities) < 10):
            return "Insufficient data for fusion analysis"
        
        # Compare recent angular velocities
        recent_wheel = self.wheel_angular_velocities[-10:]
        recent_imu = self.imu_angular_velocities[-10:]
        
        # Calculate correlation and differences
        wheel_array = np.array(recent_wheel)
        imu_array = np.array(recent_imu[-len(recent_wheel):])
        
        if len(wheel_array) != len(imu_array):
            return "Mismatched data lengths"
        
        # Calculate RMS difference
        differences = wheel_array - imu_array
        rms_diff = np.sqrt(np.mean(differences**2))
        
        # Store for trending
        self.velocity_differences.append(rms_diff)
        if len(self.velocity_differences) > 50:
            self.velocity_differences.pop(0)
        
        # Calculate correlation
        if np.std(wheel_array) > 0.01 and np.std(imu_array) > 0.01:
            correlation = np.corrcoef(wheel_array, imu_array)[0, 1]
        else:
            correlation = 0.0
        
        # Fusion quality assessment
        if rms_diff < 0.1 and correlation > 0.7:
            quality = "EXCELLENT"
        elif rms_diff < 0.2 and correlation > 0.5:
            quality = "GOOD"
        elif rms_diff < 0.5 and correlation > 0.3:
            quality = "FAIR"
        else:
            quality = "POOR"
        
        return f"{quality} (RMS diff: {rms_diff:.3f} rad/s, correlation: {correlation:.3f})"
    
    def publish_diagnostics(self):
        """Publish periodic diagnostic information"""
        runtime = time.time() - self.start_time
        
        self.get_logger().info("=== IMU-Enhanced SLAM Diagnostics ===")
        self.get_logger().info(f"Runtime: {runtime:.1f}s")
        
        # Data reception status
        self.get_logger().info(f"Data Reception:")
        self.get_logger().info(f"  IMU: {'✅' if self.imu_received else '❌'} ({self.imu_count} msgs)")
        self.get_logger().info(f"  Joints: {'✅' if self.joint_received else '❌'} ({self.joint_count} msgs)")
        self.get_logger().info(f"  Odometry: {'✅' if self.odom_received else '❌'} ({self.odom_count} msgs)")
        self.get_logger().info(f"  Cmd Vel: {'✅' if self.cmd_vel_received else '❌'}")
        
        # Frequency analysis
        if len(self.imu_frequencies) > 5:
            avg_imu_freq = np.mean(self.imu_frequencies)
            self.get_logger().info(f"IMU Frequency: {avg_imu_freq:.1f} Hz")
            if avg_imu_freq < 20:
                self.get_logger().warn("⚠️  LOW IMU FREQUENCY - May affect fusion quality")
        
        if len(self.joint_frequencies) > 5:
            avg_joint_freq = np.mean(self.joint_frequencies)
            self.get_logger().info(f"Joint States Frequency: {avg_joint_freq:.1f} Hz")
            if avg_joint_freq < 20:
                self.get_logger().warn("⚠️  LOW JOINT FREQUENCY - May affect odometry")
        
        if len(self.odom_frequencies) > 5:
            avg_odom_freq = np.mean(self.odom_frequencies)
            self.get_logger().info(f"Odometry Frequency: {avg_odom_freq:.1f} Hz")
        
        # Fusion quality analysis
        fusion_quality = self.analyze_fusion_quality()
        self.get_logger().info(f"Fusion Quality: {fusion_quality}")
        
        # Current sensor values
        if self.imu_data:
            euler = tf_transformations.euler_from_quaternion([
                self.imu_data.orientation.x,
                self.imu_data.orientation.y,
                self.imu_data.orientation.z,
                self.imu_data.orientation.w
            ])
            self.get_logger().info(f"IMU: Yaw={math.degrees(euler[2]):.1f}°, " +
                                  f"Gyro Z={self.imu_data.angular_velocity.z:.3f} rad/s")
        
        if self.joint_data and len(self.joint_data.velocity) >= 4:
            velocities = [f"{v:.2f}" for v in self.joint_data.velocity[:4]]
            self.get_logger().info(f"Wheel Velocities: {velocities} rad/s")
        
        if self.odom_data:
            euler = tf_transformations.euler_from_quaternion([
                self.odom_data.pose.pose.orientation.x,
                self.odom_data.pose.pose.orientation.y,
                self.odom_data.pose.pose.orientation.z,
                self.odom_data.pose.pose.orientation.w
            ])
            self.get_logger().info(f"Odom: Pos=({self.odom_data.pose.pose.position.x:.2f}, " +
                                  f"{self.odom_data.pose.pose.position.y:.2f}), " +
                                  f"Yaw={math.degrees(euler[2]):.1f}°")
        
        # Issues and recommendations
        self.check_system_health()
        
        self.get_logger().info("=====================================")
    
    def check_system_health(self):
        """Check overall system health and provide recommendations"""
        issues = []
        recommendations = []
        
        if not self.imu_received:
            issues.append("No IMU data received")
            recommendations.append("Check IMU wiring and ESP32 firmware")
        
        if not self.joint_received:
            issues.append("No joint states received")
            recommendations.append("Check encoder connections and ESP32 communication")
        
        if not self.odom_received:
            issues.append("No odometry received")
            recommendations.append("Check ESP32 odometry publishing")
        
        # Check data rates
        if len(self.imu_frequencies) > 5 and np.mean(self.imu_frequencies) < 20:
            issues.append("Low IMU frequency")
            recommendations.append("Increase IMU update rate in ESP32 firmware")
        
        if len(self.joint_frequencies) > 5 and np.mean(self.joint_frequencies) < 20:
            issues.append("Low joint state frequency")
            recommendations.append("Increase encoder processing rate")
        
        # Check fusion quality
        if len(self.velocity_differences) > 10:
            avg_diff = np.mean(self.velocity_differences)
            if avg_diff > 0.3:
                issues.append("Poor IMU-wheel fusion")
                recommendations.append("Check IMU calibration and wheel parameters")
        
        if issues:
            self.get_logger().warn("System Health Issues:")
            for issue in issues:
                self.get_logger().warn(f"  ❌ {issue}")
            
            self.get_logger().info("Recommendations:")
            for rec in recommendations:
                self.get_logger().info(f"  💡 {rec}")

def main(args=None):
    rclpy.init(args=args)
    diagnostics = IMUDiagnostics()
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()