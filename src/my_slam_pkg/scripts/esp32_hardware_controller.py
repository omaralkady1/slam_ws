#!/usr/bin/env python3
"""
ESP32 Hardware Controller Node with IMU Integration
This node interfaces between ROS2 control and the ESP32 via micro-ROS
Provides IMU-enhanced odometry for SLAM
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np
import time

class ESP32HardwareController(Node):
    def __init__(self):
        super().__init__('esp32_hardware_controller')
        
        # Parameters
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('esp32_port', '/dev/ttyUSB0')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('use_imu_fusion', True)
        self.declare_parameter('imu_weight', 0.7)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_imu_fusion = self.get_parameter('use_imu_fusion').value
        self.imu_weight = self.get_parameter('imu_weight').value
        
        # State variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_vtheta = 0.0
        
        # Previous states for integration
        self.prev_time = time.time()
        self.prev_joint_state = None
        self.prev_wheel_positions = [0.0, 0.0, 0.0, 0.0]
        
        # IMU data
        self.latest_imu = None
        self.imu_calibrated = False
        self.imu_yaw_offset = 0.0
        
        # Data quality tracking
        self.joint_state_received = False
        self.imu_received = False
        self.cmd_vel_timeout = 0.5  # seconds
        self.last_cmd_vel_time = time.time()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom_enhanced', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.diagnostics_pub = self.create_publisher(String, 'esp32_diagnostics', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_input', self.cmd_vel_callback, 10)
        
        # Timers
        self.odom_timer = self.create_timer(1.0/self.publish_rate, self.publish_odometry)
        self.diagnostics_timer = self.create_timer(2.0, self.publish_diagnostics)
        
        # Initialize IMU calibration
        self.calibration_samples = []
        self.calibration_sample_count = 100
        
        self.get_logger().info('ESP32 Hardware Controller with IMU initialized')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, separation: {self.wheel_separation}m')
        self.get_logger().info(f'IMU fusion: {self.use_imu_fusion}, weight: {self.imu_weight}')
    
    def joint_state_callback(self, msg):
        """Process joint states from ESP32"""
        if not self.joint_state_received:
            self.get_logger().info('First joint state received from ESP32')
            self.joint_state_received = True
        
        # Extract wheel positions and velocities
        if len(msg.position) >= 4 and len(msg.velocity) >= 4:
            self.update_odometry_from_wheels(msg)
            self.prev_joint_state = msg
    
    def imu_callback(self, msg):
        """Process IMU data from ESP32"""
        if not self.imu_received:
            self.get_logger().info('First IMU data received from ESP32')
            self.imu_received = True
            
        self.latest_imu = msg
        
        # Calibrate IMU if needed
        if not self.imu_calibrated:
            self.calibrate_imu(msg)
    
    def cmd_vel_callback(self, msg):
        """Forward cmd_vel to ESP32"""
        self.cmd_vel_pub.publish(msg)
        self.last_cmd_vel_time = time.time()
    
    def calibrate_imu(self, imu_msg):
        """Calibrate IMU gyroscope offset"""
        # Collect samples when robot should be stationary
        current_time = time.time()
        if current_time - self.last_cmd_vel_time > 2.0:  # Robot stationary for 2 seconds
            self.calibration_samples.append(imu_msg.angular_velocity.z)
            
            if len(self.calibration_samples) >= self.calibration_sample_count:
                self.imu_yaw_offset = np.mean(self.calibration_samples)
                self.imu_calibrated = True
                self.get_logger().info(f'IMU calibrated. Gyro Z offset: {self.imu_yaw_offset:.4f} rad/s')
    
    def update_odometry_from_wheels(self, joint_state):
        """Update odometry using wheel encoder data"""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0 or self.prev_joint_state is None:
            self.prev_time = current_time
            return
        
        # Extract wheel positions (assuming order: FL, FR, RL, RR)
        try:
            fl_idx = joint_state.name.index('front_left_wheel_joint')
            fr_idx = joint_state.name.index('front_right_wheel_joint')
            rl_idx = joint_state.name.index('rear_left_wheel_joint')
            rr_idx = joint_state.name.index('rear_right_wheel_joint')
            
            # Current positions
            fl_pos = joint_state.position[fl_idx]
            fr_pos = joint_state.position[fr_idx]
            rl_pos = joint_state.position[rl_idx]
            rr_pos = joint_state.position[rr_idx]
            
            # Previous positions
            prev_fl_pos = self.prev_joint_state.position[fl_idx]
            prev_fr_pos = self.prev_joint_state.position[fr_idx]
            prev_rl_pos = self.prev_joint_state.position[rl_idx]
            prev_rr_pos = self.prev_joint_state.position[rr_idx]
            
            # Calculate wheel displacements
            dfl = fl_pos - prev_fl_pos
            dfr = fr_pos - prev_fr_pos
            drl = rl_pos - prev_rl_pos
            drr = rr_pos - prev_rr_pos
            
            # Average left and right wheel displacements
            dl = (dfl + drl) / 2.0  # Left side average
            dr = (dfr + drr) / 2.0  # Right side average
            
            # Calculate robot motion
            dc = (dl + dr) / 2.0  # Center displacement
            dtheta_wheels = (dr - dl) / self.wheel_separation  # Angular displacement from wheels
            
            # Calculate velocities
            wheel_vx = (dc * self.wheel_radius) / dt
            wheel_vtheta = (dtheta_wheels * self.wheel_radius) / dt
            
            # IMU fusion for angular velocity
            if self.use_imu_fusion and self.imu_calibrated and self.latest_imu:
                # Get calibrated IMU angular velocity
                imu_vtheta = self.latest_imu.angular_velocity.z - self.imu_yaw_offset
                
                # Sanity check on IMU data
                if abs(imu_vtheta) < 10.0:  # Reasonable angular velocity limit
                    # Weighted fusion
                    self.robot_vtheta = (1.0 - self.imu_weight) * wheel_vtheta + self.imu_weight * imu_vtheta
                else:
                    self.robot_vtheta = wheel_vtheta
                    self.get_logger().warn(f'IMU angular velocity out of range: {imu_vtheta:.3f} rad/s')
            else:
                self.robot_vtheta = wheel_vtheta
            
            self.robot_vx = wheel_vx
            self.robot_vy = 0.0  # Differential drive assumption
            
            # Integrate to get position
            if abs(dtheta_wheels) < 1e-6:
                # Straight line motion
                dx = dc * self.wheel_radius * math.cos(self.robot_theta)
                dy = dc * self.wheel_radius * math.sin(self.robot_theta)
                dtheta = self.robot_vtheta * dt
            else:
                # Arc motion
                radius = dc / dtheta_wheels if abs(dtheta_wheels) > 1e-6 else float('inf')
                if abs(radius) < 1e6:  # Reasonable radius
                    dtheta = self.robot_vtheta * dt
                    dx = radius * self.wheel_radius * (math.sin(self.robot_theta + dtheta) - math.sin(self.robot_theta))
                    dy = -radius * self.wheel_radius * (math.cos(self.robot_theta + dtheta) - math.cos(self.robot_theta))
                else:
                    dx = dc * self.wheel_radius * math.cos(self.robot_theta)
                    dy = dc * self.wheel_radius * math.sin(self.robot_theta)
                    dtheta = self.robot_vtheta * dt
            
            # Update robot pose
            self.robot_x += dx
            self.robot_y += dy
            self.robot_theta += dtheta
            
            # Normalize angle
            self.robot_theta = self.normalize_angle(self.robot_theta)
            
            self.prev_time = current_time
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error processing joint states: {e}')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def publish_odometry(self):
        """Publish enhanced odometry message"""
        if not self.joint_state_received:
            return
        
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.robot_theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom.twist.twist.linear.x = self.robot_vx
        odom.twist.twist.linear.y = self.robot_vy
        odom.twist.twist.angular.z = self.robot_vtheta
        
        # Covariance (lower values = more confident)
        # Adjust based on IMU availability and data quality
        pos_var = 0.01 if self.imu_calibrated else 0.05
        ang_var = 0.005 if self.imu_calibrated else 0.02
        
        # Position covariance (x, y, z, roll, pitch, yaw)
        odom.pose.covariance[0] = pos_var    # x
        odom.pose.covariance[7] = pos_var    # y
        odom.pose.covariance[14] = 1e6       # z (not used)
        odom.pose.covariance[21] = 1e6       # roll (not used)
        odom.pose.covariance[28] = 1e6       # pitch (not used)
        odom.pose.covariance[35] = ang_var   # yaw
        
        # Velocity covariance
        odom.twist.covariance[0] = pos_var * 2   # vx
        odom.twist.covariance[7] = pos_var * 2   # vy
        odom.twist.covariance[14] = 1e6          # vz (not used)
        odom.twist.covariance[21] = 1e6          # vroll (not used)
        odom.twist.covariance[28] = 1e6          # vpitch (not used)
        odom.twist.covariance[35] = ang_var * 2  # vyaw
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        diag_msg = String()
        
        status = []
        status.append(f"Joint States: {'✓' if self.joint_state_received else '✗'}")
        status.append(f"IMU: {'✓' if self.imu_received else '✗'}")
        status.append(f"IMU Calibrated: {'✓' if self.imu_calibrated else '✗'}")
        
        if self.imu_calibrated:
            status.append(f"IMU Offset: {self.imu_yaw_offset:.4f} rad/s")
        
        status.append(f"Position: ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.robot_theta:.3f})")
        status.append(f"Velocity: ({self.robot_vx:.3f}, {self.robot_vtheta:.3f})")
        
        # Data freshness
        current_time = time.time()
        if current_time - self.last_cmd_vel_time > self.cmd_vel_timeout:
            status.append("⚠ No recent cmd_vel")
        
        diag_msg.data = " | ".join(status)
        self.diagnostics_pub.publish(diag_msg)
        
        # Periodic info logging
        if self.joint_state_received:
            self.get_logger().info(
                f"Odom: pos=({self.robot_x:.3f}, {self.robot_y:.3f}), "
                f"yaw={self.robot_theta:.3f}, vel=({self.robot_vx:.3f}, {self.robot_vtheta:.3f}), "
                f"IMU: {'OK' if self.imu_calibrated else 'CALIBRATING'}"
            )

def main(args=None):
    rclpy.init(args=args)
    
    controller = ESP32HardwareController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down ESP32 Hardware Controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()