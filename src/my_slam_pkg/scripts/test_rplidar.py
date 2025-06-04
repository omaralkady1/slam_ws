
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
import math
import numpy as np

class RPLidarTester(Node):
    def __init__(self):
        super().__init__('rplidar_tester')
        
        # Create subscription to laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Statistics
        self.scan_count = 0
        self.start_time = time.time()
        self.last_scan_time = 0
        self.scan_frequencies = []
        
        # Data analysis
        self.min_ranges = []
        self.max_ranges = []
        self.range_counts = []
        
        self.get_logger().info('RPLidar A1 tester started')
        self.get_logger().info('Waiting for scan data...')
        
        # Create timer for periodic reporting
        self.timer = self.create_timer(5.0, self.report_statistics)
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        current_time = time.time()
        
        # Calculate scan frequency
        if self.last_scan_time > 0:
            frequency = 1.0 / (current_time - self.last_scan_time)
            self.scan_frequencies.append(frequency)
            
            # Keep only last 100 measurements
            if len(self.scan_frequencies) > 100:
                self.scan_frequencies.pop(0)
        
        self.last_scan_time = current_time
        self.scan_count += 1
        
        # Analyze scan data
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        
        if valid_ranges:
            self.min_ranges.append(min(valid_ranges))
            self.max_ranges.append(max(valid_ranges))
            self.range_counts.append(len(valid_ranges))
        
        # First scan info
        if self.scan_count == 1:
            self.get_logger().info('First scan received!')
            self.get_logger().info(f'Scan parameters:')
            self.get_logger().info(f'  Range: {msg.range_min:.3f} - {msg.range_max:.3f} m')
            self.get_logger().info(f'  Angle: {msg.angle_min:.3f} - {msg.angle_max:.3f} rad')
            self.get_logger().info(f'  Angle increment: {msg.angle_increment:.6f} rad ({math.degrees(msg.angle_increment):.3f} deg)')
            self.get_logger().info(f'  Points per scan: {len(msg.ranges)}')
            self.get_logger().info(f'  Valid points: {len(valid_ranges)}')
    
    def report_statistics(self):
        """Report periodic statistics"""
        if self.scan_count == 0:
            self.get_logger().warn('No scan data received yet!')
            return
        
        # Calculate averages
        elapsed_time = time.time() - self.start_time
        avg_frequency = self.scan_count / elapsed_time
        
        if self.scan_frequencies:
            current_frequency = np.mean(self.scan_frequencies)
            frequency_std = np.std(self.scan_frequencies)
        else:
            current_frequency = 0
            frequency_std = 0
        
        if self.min_ranges:
            avg_min_range = np.mean(self.min_ranges)
            avg_max_range = np.mean(self.max_ranges)
            avg_valid_points = np.mean(self.range_counts)
        else:
            avg_min_range = 0
            avg_max_range = 0
            avg_valid_points = 0
        
        # Report
        self.get_logger().info('=== RPLidar A1 Statistics ===')
        self.get_logger().info(f'Total scans: {self.scan_count}')
        self.get_logger().info(f'Average frequency: {avg_frequency:.2f} Hz')
        self.get_logger().info(f'Current frequency: {current_frequency:.2f} ± {frequency_std:.2f} Hz')
        self.get_logger().info(f'Average range: {avg_min_range:.3f} - {avg_max_range:.3f} m')
        self.get_logger().info(f'Average valid points: {avg_valid_points:.0f}')
        self.get_logger().info('============================')
    
    def analyze_scan_quality(self, msg):
        """Analyze scan quality and detect issues"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        
        # Quality metrics
        total_points = len(ranges)
        valid_points = len(valid_ranges)
        validity_ratio = valid_points / total_points if total_points > 0 else 0
        
        # Check for issues
        issues = []
        
        if validity_ratio < 0.7:
            issues.append(f"Low validity ratio: {validity_ratio:.2f}")
        
        if valid_points < 200:
            issues.append(f"Too few valid points: {valid_points}")
        
        # Check for clustering (potential obstruction)
        if len(valid_ranges) > 50:
            range_std = np.std(valid_ranges)
            if range_std < 0.1:
                issues.append(f"Low range variation: {range_std:.3f} (possible obstruction)")
        
        if issues:
            self.get_logger().warn(f"Scan quality issues: {', '.join(issues)}")

def main():
    rclpy.init()
    tester = RPLidarTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()