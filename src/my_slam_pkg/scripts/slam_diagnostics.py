#!/usr/bin/env python3
"""
SLAM System Diagnostic Tool
Automatically detects and reports ESP32/SLAM issues
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import time
import subprocess
import sys

class SLAMDiagnostic(Node):
    def __init__(self):
        super().__init__('slam_diagnostic')
        
        # Diagnostic data
        self.joint_state_received = False
        self.laser_received = False
        self.tf_received = False
        self.joint_state_count = 0
        self.laser_count = 0
        self.tf_count = 0
        self.last_joint_time = None
        self.last_laser_time = None
        self.joint_issues = []
        
        # Subscribers for diagnostics
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        
        # Publisher for testing
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("SLAM Diagnostic Tool Started")
        self.get_logger().info("Collecting data for 10 seconds...")
        
        # Run diagnostics
        self.timer = self.create_timer(1.0, self.diagnostic_update)
        self.start_time = time.time()
        
    def joint_callback(self, msg):
        self.joint_state_received = True
        self.joint_state_count += 1
        self.last_joint_time = time.time()
        
        # Check for issues
        if len(msg.position) != 4:
            self.joint_issues.append(f"Wrong joint count: {len(msg.position)} (should be 4)")
        
        if len(msg.velocity) != 4:
            self.joint_issues.append(f"Wrong velocity count: {len(msg.velocity)} (should be 4)")
        
        # Check for unrealistic values
        for i, vel in enumerate(msg.velocity):
            if abs(vel) > 20.0:
                self.joint_issues.append(f"High velocity on joint {i}: {vel:.2f} rad/s")
            if str(vel) in ['nan', 'inf', '-inf']:
                self.joint_issues.append(f"Invalid velocity on joint {i}: {vel}")
    
    def laser_callback(self, msg):
        self.laser_received = True
        self.laser_count += 1
        self.last_laser_time = time.time()
    
    def tf_callback(self, msg):
        self.tf_received = True
        self.tf_count += 1
    
    def diagnostic_update(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < 10:
            self.get_logger().info(f"Collecting data... {elapsed:.0f}/10 seconds")
            return
        
        # Stop the timer and run full diagnostics
        self.timer.cancel()
        self.run_diagnostics()
    
    def run_diagnostics(self):
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("SLAM SYSTEM DIAGNOSTIC REPORT")
        self.get_logger().info("="*50)
        
        # Check ROS topics
        self.check_topics()
        
        # Check data rates
        self.check_data_rates()
        
        # Check hardware communication
        self.check_hardware()
        
        # Check specific issues
        self.check_specific_issues()
        
        # Provide recommendations
        self.provide_recommendations()
    
    def check_topics(self):
        self.get_logger().info("\n--- TOPIC AVAILABILITY ---")
        
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')
            
            required_topics = ['/joint_states', '/scan', '/cmd_vel', '/tf']
            missing_topics = []
            
            for topic in required_topics:
                if topic in topics:
                    self.get_logger().info(f"✓ {topic} - Available")
                else:
                    self.get_logger().error(f"✗ {topic} - Missing")
                    missing_topics.append(topic)
            
            if missing_topics:
                self.get_logger().error(f"Missing topics indicate hardware not running: {missing_topics}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to check topics: {e}")
    
    def check_data_rates(self):
        self.get_logger().info("\n--- DATA RATES ---")
        
        joint_rate = self.joint_state_count / 10.0
        laser_rate = self.laser_count / 10.0
        tf_rate = self.tf_count / 10.0
        
        self.get_logger().info(f"Joint States: {joint_rate:.1f} Hz (target: ~50 Hz)")
        self.get_logger().info(f"Laser Scan: {laser_rate:.1f} Hz (target: ~10 Hz)")
        self.get_logger().info(f"TF Updates: {tf_rate:.1f} Hz (target: ~50 Hz)")
        
        # Rate analysis
        if joint_rate < 10:
            self.get_logger().error("⚠️  LOW JOINT STATE RATE - ESP32 connection issue")
        elif joint_rate > 80:
            self.get_logger().warn("⚠️  HIGH JOINT STATE RATE - May cause system lag")
        else:
            self.get_logger().info("✓ Joint state rate looks good")
        
        if laser_rate < 5:
            self.get_logger().error("⚠️  LOW LASER RATE - RPLidar connection issue")
        else:
            self.get_logger().info("✓ Laser rate looks good")
    
    def check_hardware(self):
        self.get_logger().info("\n--- HARDWARE STATUS ---")
        
        # Check USB devices
        try:
            result = subprocess.run(['ls', '/dev/ttyUSB*'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                devices = result.stdout.strip().split('\n')
                self.get_logger().info(f"USB Serial devices: {devices}")
            else:
                self.get_logger().error("No USB serial devices found")
        except:
            pass
        
        # Check micro-ROS agent process
        try:
            result = subprocess.run(['pgrep', '-f', 'micro_ros_agent'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("✓ micro-ROS agent is running")
            else:
                self.get_logger().error("✗ micro-ROS agent not found")
        except:
            pass
    
    def check_specific_issues(self):
        self.get_logger().info("\n--- SPECIFIC ISSUES DETECTED ---")
        
        if not self.joint_state_received:
            self.get_logger().error("🔴 CRITICAL: No joint states received from ESP32")
            self.get_logger().error("   Possible causes:")
            self.get_logger().error("   - ESP32 not connected")
            self.get_logger().error("   - micro-ROS agent not running")
            self.get_logger().error("   - Firmware not uploaded")
            self.get_logger().error("   - Wrong serial port")
        
        if not self.laser_received:
            self.get_logger().error("🔴 CRITICAL: No laser data received")
            self.get_logger().error("   Possible causes:")
            self.get_logger().error("   - RPLidar not connected")
            self.get_logger().error("   - RPLidar driver not running")
            self.get_logger().error("   - Wrong RPLidar port")
        
        if self.joint_issues:
            self.get_logger().error("🟡 JOINT STATE ISSUES:")
            for issue in set(self.joint_issues):  # Remove duplicates
                self.get_logger().error(f"   - {issue}")
        
        if not self.joint_issues and self.joint_state_received:
            self.get_logger().info("✓ Joint states look healthy")
    
    def provide_recommendations(self):
        self.get_logger().info("\n--- RECOMMENDATIONS ---")
        
        if not self.joint_state_received:
            self.get_logger().info("1. Check ESP32 connection:")
            self.get_logger().info("   sudo dmesg | tail -20")
            self.get_logger().info("   ls /dev/ttyUSB*")
            self.get_logger().info("2. Start micro-ROS agent:")
            self.get_logger().info("   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0")
            self.get_logger().info("3. Flash fixed firmware using:")
            self.get_logger().info("   ./src/my_slam_pkg/scripts/flash_esp32_firmware.sh")
        
        elif self.joint_state_count < 100:
            self.get_logger().info("1. Low data rate detected. Try:")
            self.get_logger().info("   - Reset ESP32")
            self.get_logger().info("   - Check power supply")
            self.get_logger().info("   - Verify encoder connections")
        
        if not self.laser_received:
            self.get_logger().info("1. Check RPLidar:")
            self.get_logger().info("   ls /dev/ttyUSB*")
            self.get_logger().info("   ros2 launch rplidar_ros2 rplidar.launch.py")
        
        if self.joint_issues:
            self.get_logger().info("1. Test encoders individually:")
            self.get_logger().info("   ros2 run my_slam_pkg test_encoders.py")
        
        self.get_logger().info("\n2. For complete troubleshooting guide, see:")
        self.get_logger().info("   /home/alkady/slam_ws/ESP32_ENCODER_FIX_GUIDE.md")

def main(args=None):
    rclpy.init(args=args)
    
    diagnostic = SLAMDiagnostic()
    
    try:
        rclpy.spin(diagnostic)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
