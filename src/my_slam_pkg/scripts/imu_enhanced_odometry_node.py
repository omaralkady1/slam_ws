#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np

class IMUEnhancedOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_enhanced_odometry_node')
        
        # Robot parameters - match your ESP32 code
        self.wheel_radius = 0.05  # meters
        self.wheel_base = 0.34    # meters between left and right wheels
        
        # Fusion parameters
        self.imu_weight = 0.7     # Weight for IMU yaw vs wheel odometry yaw
        self.wheel_slip_threshold = 0.1  # rad/s difference threshold for slip detection
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0          # Fused orientation
        self.wheel_theta = 0.0    # Wheel-only orientation for comparison
        
        # Velocity estimates
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        # Previous wheel positions and IMU data
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.prev_time = None
        self.prev_imu_yaw = None
        self.imu_yaw_offset = None  # Offset to align IMU with wheel odometry
        
        # Slip detection
        self.wheel_slip_detected = False
        self.slip_start_time = None
        
        # Publishers and subscribers
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Diagnostic timer
        self.diagnostic_timer = self.create_timer(2.0, self.publish_diagnostics)
        
        # State tracking
        self.joints_received = False
        self.imu_received = False
        self.latest_imu_data = None
        
        self.get_logger().info('IMU-Enhanced wheel odometry node started')
        self.get_logger().info(f'Using IMU weight: {self.imu_weight}, slip threshold: {self.wheel_slip_threshold}')
    
    def imu_callback(self, msg):
        """Process IMU data for orientation fusion"""
        self.latest_imu_data = msg
        self.imu_received = True
        
        # Extract yaw from quaternion
        quaternion = [
            msg.orientation.x,
            msg.orientation.y, 
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Convert quaternion to Euler angles
        euler = tf_transformations.euler_from_quaternion(quaternion)
        current_imu_yaw = euler[2]  # yaw
        
        # Initialize IMU offset on first valid reading
        if self.prev_imu_yaw is None and self.joints_received:
            self.imu_yaw_offset = self.wheel_theta - current_imu_yaw
            self.get_logger().info(f'IMU yaw offset initialized: {self.imu_yaw_offset:.3f} rad')
        
        self.prev_imu_yaw = current_imu_yaw
    
    def joint_callback(self, msg):
        try:
            # Extract wheel positions (assuming 4-wheel robot with front/rear pairs)
            fl_idx = msg.name.index('front_left_wheel_joint')
            fr_idx = msg.name.index('front_right_wheel_joint')
            rl_idx = msg.name.index('rear_left_wheel_joint')
            rr_idx = msg.name.index('rear_right_wheel_joint')
            
            left_pos = (msg.position[fl_idx] + msg.position[rl_idx]) / 2.0
            right_pos = (msg.position[fr_idx] + msg.position[rr_idx]) / 2.0
            
            # Extract wheel velocities for slip detection
            left_vel = (msg.velocity[fl_idx] + msg.velocity[rl_idx]) / 2.0
            right_vel = (msg.velocity[fr_idx] + msg.velocity[rr_idx]) / 2.0
            
            current_time = self.get_clock().now()
            
            # Initialize on first callback
            if self.prev_time is None:
                self.prev_left_pos = left_pos
                self.prev_right_pos = right_pos
                self.prev_time = current_time
                self.joints_received = True
                return
            
            # Calculate time step
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt <= 0:
                return
                
            # Calculate wheel displacement
            dl = left_pos - self.prev_left_pos
            dr = right_pos - self.prev_right_pos
            
            # Basic wheel odometry calculation
            dc = (dl + dr) / 2.0  # Center displacement
            wheel_dtheta = (dr - dl) / self.wheel_base  # Angular displacement from wheels
            
            # Update wheel-only orientation for comparison
            self.wheel_theta += wheel_dtheta
            self.wheel_theta = self.normalize_angle(self.wheel_theta)
            
            # Slip detection based on wheel velocity consistency
            expected_angular_vel = (right_vel - left_vel) / self.wheel_base
            self.detect_wheel_slip(left_vel, right_vel, expected_angular_vel)
            
            # Determine final orientation based on IMU availability and slip status
            if self.imu_received and self.imu_yaw_offset is not None:
                # Get corrected IMU yaw
                imu_yaw = self.prev_imu_yaw + self.imu_yaw_offset
                imu_yaw = self.normalize_angle(imu_yaw)
                
                # Fuse IMU and wheel odometry
                if self.wheel_slip_detected:
                    # During slip, trust IMU more
                    effective_imu_weight = min(0.9, self.imu_weight + 0.2)
                    self.get_logger().warn_throttle(2.0, 
                        f'Wheel slip detected! Using IMU weight: {effective_imu_weight:.2f}')
                else:
                    effective_imu_weight = self.imu_weight
                
                # Weighted fusion of orientations
                dtheta_fused = self.fuse_angular_measurements(
                    wheel_dtheta, 
                    imu_yaw - self.theta, 
                    effective_imu_weight
                )
                
                self.theta += dtheta_fused
                self.theta = self.normalize_angle(self.theta)
                
                # Update IMU offset slowly to handle drift
                yaw_error = self.normalize_angle(imu_yaw - self.theta)
                self.imu_yaw_offset += yaw_error * 0.001  # Very slow adaptation
                
            else:
                # Fall back to wheel-only odometry
                self.theta = self.wheel_theta
            
            # Calculate robot motion in world frame
            if abs(wheel_dtheta) < 1e-6:
                # Straight line motion
                dx = dc * math.cos(self.theta)
                dy = dc * math.sin(self.theta)
            else:
                # Arc motion - use wheel odometry for translation
                radius = dc / wheel_dtheta if abs(wheel_dtheta) > 1e-6 else float('inf')
                if abs(radius) < 1e6:  # Reasonable radius
                    dx = radius * (math.sin(self.theta) - math.sin(self.theta - wheel_dtheta))
                    dy = -radius * (math.cos(self.theta) - math.cos(self.theta - wheel_dtheta))
                else:
                    dx = dc * math.cos(self.theta)
                    dy = dc * math.sin(self.theta)
            
            self.x += dx
            self.y += dy
            
            # Calculate velocities
            self.vx = dx / dt
            self.vy = dy / dt
            
            # For angular velocity, use IMU if available and reliable
            if (self.imu_received and self.latest_imu_data and 
                not self.wheel_slip_detected):
                # Use IMU angular velocity (already filtered by ESP32)
                self.vtheta = self.latest_imu_data.angular_velocity.z
            else:
                # Fall back to wheel-derived angular velocity
                self.vtheta = wheel_dtheta / dt
            
            # Publish odometry
            self.publish_odometry(current_time)
            
            # Update previous values
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.prev_time = current_time
            self.joints_received = True
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error processing joint states: {e}')
    
    def detect_wheel_slip(self, left_vel, right_vel, expected_angular_vel):
        """Detect wheel slip based on velocity inconsistencies"""
        if not self.imu_received or not self.latest_imu_data:
            return
            
        # Compare expected angular velocity with IMU
        imu_angular_vel = self.latest_imu_data.angular_velocity.z
        angular_vel_diff = abs(expected_angular_vel - imu_angular_vel)
        
        # Check for slip
        slip_detected = angular_vel_diff > self.wheel_slip_threshold
        
        if slip_detected and not self.wheel_slip_detected:
            self.slip_start_time = self.get_clock().now()
            self.get_logger().warn(f'Wheel slip detected! Angular vel diff: {angular_vel_diff:.3f} rad/s')
        elif not slip_detected and self.wheel_slip_detected:
            slip_duration = (self.get_clock().now() - self.slip_start_time).nanoseconds / 1e9
            self.get_logger().info(f'Wheel slip ended after {slip_duration:.1f}s')
        
        self.wheel_slip_detected = slip_detected
    
    def fuse_angular_measurements(self, wheel_dtheta, imu_dtheta, imu_weight):
        """Fuse wheel and IMU angular measurements with proper angle wrapping"""
        # Normalize angle differences
        wheel_dtheta = self.normalize_angle(wheel_dtheta)
        imu_dtheta = self.normalize_angle(imu_dtheta)
        
        # Weighted fusion
        fused_dtheta = (1.0 - imu_weight) * wheel_dtheta + imu_weight * imu_dtheta
        return self.normalize_angle(fused_dtheta)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def publish_odometry(self, current_time):
        """Publish fused odometry message"""
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta
        
        # Covariance - adjust based on slip detection and IMU availability
        base_pos_cov = 0.05 if self.wheel_slip_detected else 0.01
        base_ang_cov = 0.02 if not self.imu_received else 0.005
        
        if self.wheel_slip_detected:
            base_pos_cov *= 3.0  # Increase uncertainty during slip
        
        # Position covariance
        odom.pose.covariance[0] = base_pos_cov   # x
        odom.pose.covariance[7] = base_pos_cov   # y
        odom.pose.covariance[35] = base_ang_cov  # theta
        
        # Velocity covariance
        odom.twist.covariance[0] = base_pos_cov * 2   # vx
        odom.twist.covariance[7] = base_pos_cov * 2   # vy
        odom.twist.covariance[35] = base_ang_cov * 2  # vtheta
        
        self.odom_pub.publish(odom)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        if not self.joints_received:
            return
            
        imu_status = "OK" if self.imu_received else "NO_DATA"
        slip_status = "SLIP" if self.wheel_slip_detected else "OK"
        
        yaw_diff = 0.0
        if self.imu_received and self.imu_yaw_offset is not None:
            corrected_imu_yaw = self.prev_imu_yaw + self.imu_yaw_offset
            yaw_diff = abs(self.normalize_angle(corrected_imu_yaw - self.theta))
        
        self.get_logger().info(
            f'Odom: pos=({self.x:.3f}, {self.y:.3f}), yaw={self.theta:.3f}rad, '
            f'IMU={imu_status}, slip={slip_status}, yaw_diff={yaw_diff:.3f}rad'
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUEnhancedOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()