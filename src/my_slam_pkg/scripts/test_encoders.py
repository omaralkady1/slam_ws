#!/usr/bin/env python3
"""
ESP32 Encoder Test Script
Tests encoder readings and motor responses to help debug SLAM issues
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import math

class EncoderTester(Node):
    def __init__(self):
        super().__init__('encoder_tester')
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Data storage
        self.latest_joint_state = None
        self.last_positions = [0.0, 0.0, 0.0, 0.0]
        self.test_start_time = time.time()
        
        # Test parameters
        self.test_duration = 5.0  # seconds
        self.test_linear_speed = 0.1  # m/s
        self.test_angular_speed = 0.2  # rad/s
        
        self.get_logger().info("Encoder Tester Node Started")
        self.get_logger().info("This will test forward, backward, and rotation movements")
        
        # Start the test sequence
        self.timer = self.create_timer(0.1, self.test_loop)
        self.test_phase = 0  # 0=stop, 1=forward, 2=backward, 3=rotate_left, 4=rotate_right
        self.phase_start_time = time.time()

    def joint_callback(self, msg):
        """Process joint state messages"""
        self.latest_joint_state = msg
        
        if len(msg.position) >= 4:
            # Calculate position changes
            pos_changes = []
            for i in range(4):
                change = msg.position[i] - self.last_positions[i]
                pos_changes.append(change)
                self.last_positions[i] = msg.position[i]
            
            # Print encoder status
            self.get_logger().info(
                f"Encoders - Pos: FL={msg.position[0]:.3f}, FR={msg.position[1]:.3f}, "
                f"RL={msg.position[2]:.3f}, RR={msg.position[3]:.3f}"
            )
            self.get_logger().info(
                f"Velocities - FL={msg.velocity[0]:.3f}, FR={msg.velocity[1]:.3f}, "
                f"RL={msg.velocity[2]:.3f}, RR={msg.velocity[3]:.3f} rad/s"
            )
            
            # Check for encoder issues
            self.check_encoder_health(msg)

    def check_encoder_health(self, msg):
        """Check for common encoder problems"""
        issues = []
        
        # Check for stuck encoders (no movement when commanded)
        if self.test_phase > 0:  # Only check when moving
            for i, vel in enumerate(msg.velocity):
                if abs(vel) < 0.01:  # Very low velocity when should be moving
                    issues.append(f"Motor {i} encoder may be stuck or motor not moving")
        
        # Check for excessive velocity differences between left/right
        left_avg = (msg.velocity[0] + msg.velocity[2]) / 2.0  # FL + RL
        right_avg = (msg.velocity[1] + msg.velocity[3]) / 2.0  # FR + RR
        
        if abs(left_avg - right_avg) > 0.5 and self.test_phase == 1:  # Forward movement
            issues.append(f"Large velocity difference: Left={left_avg:.3f}, Right={right_avg:.3f}")
        
        # Check for unrealistic high velocities
        for i, vel in enumerate(msg.velocity):
            if abs(vel) > 10.0:  # > 10 rad/s is likely noise
                issues.append(f"Motor {i} velocity too high: {vel:.3f} rad/s (possible noise)")
        
        if issues:
            self.get_logger().warn("ENCODER ISSUES DETECTED:")
            for issue in issues:
                self.get_logger().warn(f"  - {issue}")

    def test_loop(self):
        """Main test loop"""
        current_time = time.time()
        phase_duration = current_time - self.phase_start_time
        
        # Advance test phases every 3 seconds
        if phase_duration > 3.0:
            self.test_phase += 1
            self.phase_start_time = current_time
            
            if self.test_phase > 4:
                self.test_phase = 0  # Loop back to stop
        
        # Execute current test phase
        twist = Twist()
        
        if self.test_phase == 0:
            # Stop
            self.get_logger().info("Phase: STOP")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif self.test_phase == 1:
            # Forward
            self.get_logger().info("Phase: FORWARD")
            twist.linear.x = self.test_linear_speed
            twist.angular.z = 0.0
            
        elif self.test_phase == 2:
            # Backward
            self.get_logger().info("Phase: BACKWARD")
            twist.linear.x = -self.test_linear_speed
            twist.angular.z = 0.0
            
        elif self.test_phase == 3:
            # Rotate left
            self.get_logger().info("Phase: ROTATE LEFT")
            twist.linear.x = 0.0
            twist.angular.z = self.test_angular_speed
            
        elif self.test_phase == 4:
            # Rotate right
            self.get_logger().info("Phase: ROTATE RIGHT")
            twist.linear.x = 0.0
            twist.angular.z = -self.test_angular_speed
        
        # Publish the command
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    encoder_tester = EncoderTester()
    
    try:
        rclpy.spin(encoder_tester)
    except KeyboardInterrupt:
        encoder_tester.get_logger().info("Shutting down encoder tester...")
    finally:
        # Stop the robot
        stop_twist = Twist()
        encoder_tester.cmd_pub.publish(stop_twist)
        encoder_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
