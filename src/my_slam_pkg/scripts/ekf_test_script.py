#!/usr/bin/env python3

"""
EKF Test Script for ESP32 SLAM System
This script tests the Extended Kalman Filter functionality for sensor fusion
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
import time


class EKFTester(Node):
    """Test EKF functionality with IMU and wheel odometry fusion"""
    
    def __init__(self):
        super().__init__('ekf_test_script')
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odom_callback, 10)
        
        # Data storage
        self.imu_data = []
        self.wheel_odom_data = []
        self.ekf_odom_data = []
        
        # Test parameters
        self.test_duration = 30.0  # seconds
        self.start_time = time.time()
        
        # Timer for periodic reporting
        self.timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info("EKF Test Script started")
        self.get_logger().info(f"Testing for {self.test_duration} seconds...")
    
    def imu_callback(self, msg):
        """Process IMU data"""
        current_time = time.time()
        self.imu_data.append({
            'timestamp': current_time,
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
        })
        
        # Keep only recent data (last 100 samples)
        if len(self.imu_data) > 100:
            self.imu_data.pop(0)
    
    def odom_callback(self, msg):
        """Process wheel odometry data"""
        current_time = time.time()
        self.wheel_odom_data.append({
            'timestamp': current_time,
            'position': [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ],
            'velocity': [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z
            ]
        })
        
        # Keep only recent data
        if len(self.wheel_odom_data) > 100:
            self.wheel_odom_data.pop(0)
    
    def ekf_odom_callback(self, msg):
        """Process EKF-filtered odometry data"""
        current_time = time.time()
        self.ekf_odom_data.append({
            'timestamp': current_time,
            'position': [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ],
            'velocity': [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z
            ]
        })
        
        # Keep only recent data
        if len(self.ekf_odom_data) > 100:
            self.ekf_odom_data.pop(0)
    
    def report_status(self):
        """Report current test status"""
        elapsed_time = time.time() - self.start_time
        remaining_time = max(0, self.test_duration - elapsed_time)
        
        self.get_logger().info("="*50)
        self.get_logger().info(f"EKF Test Status - Time remaining: {remaining_time:.1f}s")
        self.get_logger().info(f"IMU samples collected: {len(self.imu_data)}")
        self.get_logger().info(f"Wheel odometry samples: {len(self.wheel_odom_data)}")
        self.get_logger().info(f"EKF odometry samples: {len(self.ekf_odom_data)}")
        
        # Check data rates
        if len(self.imu_data) > 1:
            imu_rate = len(self.imu_data) / min(5.0, elapsed_time)
            self.get_logger().info(f"IMU data rate: {imu_rate:.1f} Hz")
        
        if len(self.wheel_odom_data) > 1:
            odom_rate = len(self.wheel_odom_data) / min(5.0, elapsed_time)
            self.get_logger().info(f"Wheel odometry rate: {odom_rate:.1f} Hz")
        
        if len(self.ekf_odom_data) > 1:
            ekf_rate = len(self.ekf_odom_data) / min(5.0, elapsed_time)
            self.get_logger().info(f"EKF odometry rate: {ekf_rate:.1f} Hz")
        
        # Check for data quality issues
        self.check_data_quality()
        
        # Stop test after duration
        if elapsed_time >= self.test_duration:
            self.generate_report()
            rclpy.shutdown()
    
    def check_data_quality(self):
        """Check for data quality issues"""
        issues = []
        
        # Check IMU data
        if len(self.imu_data) < 10:
            issues.append("Low IMU data rate")
        
        # Check wheel odometry
        if len(self.wheel_odom_data) < 5:
            issues.append("Low wheel odometry rate")
        
        # Check EKF output
        if len(self.ekf_odom_data) < 5:
            issues.append("Low EKF output rate")
        
        # Check for stuck values
        if len(self.wheel_odom_data) > 5:
            recent_positions = [d['position'] for d in self.wheel_odom_data[-5:]]
            if all(np.allclose(pos, recent_positions[0], atol=1e-6) for pos in recent_positions):
                issues.append("Wheel odometry appears stuck")
        
        if issues:
            self.get_logger().warn("Data quality issues detected:")
            for issue in issues:
                self.get_logger().warn(f"  - {issue}")
    
    def generate_report(self):
        """Generate final test report"""
        self.get_logger().info("="*60)
        self.get_logger().info("EKF TEST COMPLETE - FINAL REPORT")
        self.get_logger().info("="*60)
        
        total_time = time.time() - self.start_time
        
        self.get_logger().info(f"Test duration: {total_time:.1f} seconds")
        self.get_logger().info(f"Total IMU samples: {len(self.imu_data)}")
        self.get_logger().info(f"Total wheel odometry samples: {len(self.wheel_odom_data)}")
        self.get_logger().info(f"Total EKF odometry samples: {len(self.ekf_odom_data)}")
        
        # Calculate average rates
        if total_time > 0:
            avg_imu_rate = len(self.imu_data) / total_time
            avg_odom_rate = len(self.wheel_odom_data) / total_time
            avg_ekf_rate = len(self.ekf_odom_data) / total_time
            
            self.get_logger().info(f"Average IMU rate: {avg_imu_rate:.1f} Hz")
            self.get_logger().info(f"Average wheel odometry rate: {avg_odom_rate:.1f} Hz")
            self.get_logger().info(f"Average EKF rate: {avg_ekf_rate:.1f} Hz")
        
        # Test results
        success_criteria = [
            ("IMU data available", len(self.imu_data) > 50),
            ("Wheel odometry available", len(self.wheel_odom_data) > 10),
            ("EKF output available", len(self.ekf_odom_data) > 10),
            ("IMU rate adequate", len(self.imu_data) / total_time > 10 if total_time > 0 else False),
            ("Odometry rate adequate", len(self.wheel_odom_data) / total_time > 1 if total_time > 0 else False)
        ]
        
        passed_tests = sum(1 for _, passed in success_criteria if passed)
        total_tests = len(success_criteria)
        
        self.get_logger().info("Test Results:")
        for test_name, passed in success_criteria:
            status = "PASS" if passed else "FAIL"
            self.get_logger().info(f"  {test_name}: {status}")
        
        self.get_logger().info(f"Overall: {passed_tests}/{total_tests} tests passed")
        
        if passed_tests == total_tests:
            self.get_logger().info("🎉 EKF TEST PASSED - System is working correctly!")
        else:
            self.get_logger().warn("⚠️  EKF TEST FAILED - Check system configuration")
        
        self.get_logger().info("="*60)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        ekf_tester = EKFTester()
        rclpy.spin(ekf_tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
