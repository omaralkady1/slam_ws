#!/usr/bin/env python3
"""
Enhanced Motor and Odometry Calibration for SLAM
This script helps you calibrate wheel parameters for accurate odometry
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf_transformations

class SLAMOdometryCalibrator(Node):
    def __init__(self):
        super().__init__('slam_odometry_calibrator')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # State tracking
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.joint_positions = {}
        self.joint_velocities = {}
        
        # Calibration results
        self.test_results = {
            'wheel_radius_tests': [],
            'wheel_separation_tests': [],
            'linearity_tests': [],
            'angular_tests': []
        }
        
        self.get_logger().info('SLAM Odometry Calibrator initialized')
    
    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0
    
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_theta = euler[2]
    
    def send_command(self, linear_x, angular_z, duration):
        """Send velocity command and measure result"""
        initial_x, initial_y, initial_theta = self.odom_x, self.odom_y, self.odom_theta
        
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        
        start_time = time.time()
        rate = self.create_rate(20)  # 20Hz
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        time.sleep(0.5)
        
        # Measure final position
        final_x, final_y, final_theta = self.odom_x, self.odom_y, self.odom_theta
        
        # Calculate results
        distance = math.sqrt((final_x - initial_x)**2 + (final_y - initial_y)**2)
        rotation = self.normalize_angle(final_theta - initial_theta)
        
        return {
            'command': {'linear_x': linear_x, 'angular_z': angular_z, 'duration': duration},
            'initial': {'x': initial_x, 'y': initial_y, 'theta': initial_theta},
            'final': {'x': final_x, 'y': final_y, 'theta': final_theta},
            'measured': {'distance': distance, 'rotation': rotation},
            'expected': {'distance': linear_x * duration, 'rotation': angular_z * duration}
        }
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def test_wheel_radius(self):
        """Test linear motion to calibrate wheel radius"""
        self.get_logger().info("=== Testing Wheel Radius (Linear Motion) ===")
        
        test_speeds = [0.1, 0.2, 0.3, -0.1, -0.2]  # m/s
        duration = 3.0  # seconds
        
        for speed in test_speeds:
            self.get_logger().info(f"Testing linear speed: {speed} m/s")
            result = self.send_command(speed, 0.0, duration)
            
            expected_distance = abs(speed * duration)
            measured_distance = result['measured']['distance']
            error = measured_distance - expected_distance
            error_percent = (error / expected_distance) * 100 if expected_distance > 0 else 0
            
            self.get_logger().info(f"  Expected: {expected_distance:.3f}m, Measured: {measured_distance:.3f}m")
            self.get_logger().info(f"  Error: {error:.3f}m ({error_percent:.1f}%)")
            
            # Calculate effective wheel radius
            if abs(speed) > 0.01:
                effective_radius = measured_distance / (expected_distance / 0.05)  # Assuming 0.05m nominal
                self.test_results['wheel_radius_tests'].append(effective_radius)
            
            time.sleep(2.0)  # Rest between tests
    
    def test_wheel_separation(self):
        """Test angular motion to calibrate wheel separation"""
        self.get_logger().info("=== Testing Wheel Separation (Angular Motion) ===")
        
        test_speeds = [0.5, 1.0, -0.5, -1.0]  # rad/s
        duration = 2.0  # seconds
        
        for speed in test_speeds:
            self.get_logger().info(f"Testing angular speed: {speed} rad/s")
            result = self.send_command(0.0, speed, duration)
            
            expected_rotation = abs(speed * duration)
            measured_rotation = abs(result['measured']['rotation'])
            error = measured_rotation - expected_rotation
            error_percent = (error / expected_rotation) * 100 if expected_rotation > 0 else 0
            
            self.get_logger().info(f"  Expected: {expected_rotation:.3f}rad, Measured: {measured_rotation:.3f}rad")
            self.get_logger().info(f"  Error: {error:.3f}rad ({error_percent:.1f}%)")
            
            # Calculate effective wheel separation
            if abs(speed) > 0.01:
                effective_separation = measured_rotation / (expected_rotation / 0.34)  # Assuming 0.34m nominal
                self.test_results['wheel_separation_tests'].append(effective_separation)
            
            time.sleep(2.0)
    
    def test_motion_accuracy(self):
        """Test combined motions for overall accuracy"""
        self.get_logger().info("=== Testing Motion Accuracy ===")
        
        # Test patterns for SLAM accuracy
        test_patterns = [
            {'linear': 0.2, 'angular': 0.0, 'duration': 2.0, 'name': 'Forward'},
            {'linear': -0.2, 'angular': 0.0, 'duration': 2.0, 'name': 'Backward'},
            {'linear': 0.0, 'angular': 0.5, 'duration': 3.14, 'name': 'Right Turn'},
            {'linear': 0.0, 'angular': -0.5, 'duration': 3.14, 'name': 'Left Turn'},
            {'linear': 0.1, 'angular': 0.2, 'duration': 3.0, 'name': 'Arc Motion'}
        ]
        
        for pattern in test_patterns:
            self.get_logger().info(f"Testing {pattern['name']}")
            result = self.send_command(pattern['linear'], pattern['angular'], pattern['duration'])
            
            distance_error = result['measured']['distance'] - abs(pattern['linear'] * pattern['duration'])
            rotation_error = abs(result['measured']['rotation']) - abs(pattern['angular'] * pattern['duration'])
            
            self.get_logger().info(f"  Distance error: {distance_error:.4f}m")
            self.get_logger().info(f"  Rotation error: {rotation_error:.4f}rad")
            
            time.sleep(2.0)
    
    def analyze_results(self):
        """Analyze calibration results and provide recommendations"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("CALIBRATION ANALYSIS FOR SLAM")
        self.get_logger().info("="*50)
        
        # Wheel radius analysis
        if self.test_results['wheel_radius_tests']:
            radii = np.array(self.test_results['wheel_radius_tests'])
            avg_radius = np.mean(radii)
            std_radius = np.std(radii)
            
            self.get_logger().info(f"\nWheel Radius Analysis:")
            self.get_logger().info(f"  Average: {avg_radius:.5f}m")
            self.get_logger().info(f"  Std Dev: {std_radius:.5f}m")
            self.get_logger().info(f"  Recommended: wheel_radius: {avg_radius:.5f}")
        
        # Wheel separation analysis
        if self.test_results['wheel_separation_tests']:
            separations = np.array(self.test_results['wheel_separation_tests'])
            avg_separation = np.mean(separations)
            std_separation = np.std(separations)
            
            self.get_logger().info(f"\nWheel Separation Analysis:")
            self.get_logger().info(f"  Average: {avg_separation:.5f}m")
            self.get_logger().info(f"  Std Dev: {std_separation:.5f}m")
            self.get_logger().info(f"  Recommended: wheel_separation: {avg_separation:.5f}")
        
        # Generate YAML configuration
        self.generate_config()
    
    def generate_config(self):
        """Generate optimized controller configuration"""
        config = """
# SLAM-Optimized Controller Configuration
# Generated by odometry calibration

diff_drive_controller:
  ros__parameters:
    # Calibrated physical parameters
"""
        
        if self.test_results['wheel_radius_tests']:
            avg_radius = np.mean(self.test_results['wheel_radius_tests'])
            config += f"    wheel_radius: {avg_radius:.6f}  # Calibrated value\n"
        
        if self.test_results['wheel_separation_tests']:
            avg_separation = np.mean(self.test_results['wheel_separation_tests'])
            config += f"    wheel_separation: {avg_separation:.6f}  # Calibrated value\n"
        
        config += """
    # SLAM-optimized settings
    publish_rate: 100.0
    odom_publish_frequency: 100.0
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    publish_odom: true
    publish_odom_tf: true
"""
        
        # Save to file
        with open('slam_calibrated_config.yaml', 'w') as f:
            f.write(config)
        
        self.get_logger().info(f"\nConfiguration saved to: slam_calibrated_config.yaml")
        self.get_logger().info("Copy these values to your controller configuration file.")
    
    def run_full_calibration(self):
        """Run complete calibration sequence"""
        self.get_logger().info("Starting SLAM Odometry Calibration")
        self.get_logger().info("Make sure robot has clear space to move!")
        
        input("Press Enter to start wheel radius tests...")
        self.test_wheel_radius()
        
        input("Press Enter to start wheel separation tests...")
        self.test_wheel_separation()
        
        input("Press Enter to start accuracy tests...")
        self.test_motion_accuracy()
        
        self.analyze_results()

def main():
    rclpy.init()
    calibrator = SLAMOdometryCalibrator()
    
    # Wait for connections
    time.sleep(2.0)
    
    try:
        calibrator.run_full_calibration()
    except KeyboardInterrupt:
        print("Calibration interrupted by user")
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
