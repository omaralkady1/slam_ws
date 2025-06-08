#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time

class SLAMDiagnostic(Node):
    def __init__(self):
        super().__init__('slam_diagnostic')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Status tracking
        self.map_received = False
        self.scan_received = False
        self.map_updates = 0
        self.scan_count = 0
        self.start_time = time.time()
        
        # Timer for status reports
        self.timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info('SLAM Diagnostic started - monitoring map and scan data')
        
    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info('✅ First map received!')
            self.get_logger().info(f'   Map size: {msg.info.width}x{msg.info.height}')
            self.get_logger().info(f'   Resolution: {msg.info.resolution}m/cell')
            self.get_logger().info(f'   Origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})')
            self.map_received = True
            
        self.map_updates += 1
        
    def scan_callback(self, msg):
        if not self.scan_received:
            self.get_logger().info('✅ First laser scan received!')
            self.get_logger().info(f'   Range: {msg.range_min:.2f} - {msg.range_max:.2f}m')
            self.get_logger().info(f'   Points per scan: {len(msg.ranges)}')
            self.scan_received = True
            
        self.scan_count += 1
        
    def report_status(self):
        runtime = time.time() - self.start_time
        
        self.get_logger().info('=== SLAM Status Report ===')
        self.get_logger().info(f'Runtime: {runtime:.1f}s')
        self.get_logger().info(f'Map received: {"✅" if self.map_received else "❌"} ({self.map_updates} updates)')
        self.get_logger().info(f'Scan received: {"✅" if self.scan_received else "❌"} ({self.scan_count} scans)')
        
        # Check TF tree
        try:
            # Check map -> odom transform (provided by SLAM)
            map_to_odom = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            self.get_logger().info('✅ map -> odom transform available (SLAM working)')
            
            # Check odom -> base_footprint (provided by diff_drive_controller)
            odom_to_base = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            self.get_logger().info('✅ odom -> base_footprint transform available')
            
            # Check base_footprint -> lidar_link (static)
            base_to_lidar = self.tf_buffer.lookup_transform('base_footprint', 'lidar_link', rclpy.time.Time())
            self.get_logger().info('✅ base_footprint -> lidar_link transform available')
            
        except Exception as e:
            self.get_logger().warn(f'❌ TF issue: {str(e)}')
            
        # Overall status
        if self.map_received and self.scan_received:
            self.get_logger().info('🎉 SLAM is working properly!')
        else:
            self.get_logger().warn('⚠️  SLAM issues detected - check above')
            
        self.get_logger().info('========================')

def main():
    rclpy.init()
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