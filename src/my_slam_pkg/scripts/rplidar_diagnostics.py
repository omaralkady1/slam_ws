
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
import time
import numpy as np
from collections import deque

class RPLidarDiagnostics(Node):
    def __init__(self):
        super().__init__('rplidar_diagnostics')
        
        # Diagnostics publisher
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, 
            '/diagnostics', 
            10
        )
        
        # Scan subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Health monitoring data
        self.scan_frequencies = deque(maxlen=50)
        self.scan_count = 0
        self.last_scan_time = 0
        self.start_time = time.time()
        
        # Quality metrics
        self.validity_ratios = deque(maxlen=50)
        self.range_variations = deque(maxlen=50)
        
        # Error tracking
        self.consecutive_missing_scans = 0
        self.max_missing_scans = 0
        
        # Timer for diagnostics publishing
        self.timer = self.create_timer(2.0, self.publish_diagnostics)
        
        self.get_logger().info('RPLidar diagnostics node started')
    
    def scan_callback(self, msg):
        """Process scan for diagnostics"""
        current_time = time.time()
        
        # Calculate frequency
        if self.last_scan_time > 0:
            frequency = 1.0 / (current_time - self.last_scan_time)
            self.scan_frequencies.append(frequency)
        
        self.last_scan_time = current_time
        self.scan_count += 1
        self.consecutive_missing_scans = 0
        
        # Analyze scan quality
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        
        # Calculate metrics
        validity_ratio = len(valid_ranges) / len(ranges) if len(ranges) > 0 else 0
        self.validity_ratios.append(validity_ratio)
        
        if len(valid_ranges) > 10:
            range_variation = np.std(valid_ranges)
            self.range_variations.append(range_variation)
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        current_time = time.time()
        
        # Check for missing scans
        if self.last_scan_time > 0 and (current_time - self.last_scan_time) > 1.0:
            self.consecutive_missing_scans += 1
            self.max_missing_scans = max(self.max_missing_scans, self.consecutive_missing_scans)
        
        # Create diagnostic message
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # RPLidar status
        status = DiagnosticStatus()
        status.name = "RPLidar A1"
        status.hardware_id = "rplidar_a1"
        
        # Determine status level
        if self.scan_count == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = "No scan data received"
        elif self.consecutive_missing_scans > 5:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Missing scans for {self.consecutive_missing_scans} cycles"
        elif len(self.scan_frequencies) > 0:
            avg_freq = np.mean(self.scan_frequencies)
            if avg_freq < 5.0:
                status.level = DiagnosticStatus.WARN
                status.message = f"Low scan frequency: {avg_freq:.1f} Hz"
            elif len(self.validity_ratios) > 0 and np.mean(self.validity_ratios) < 0.7:
                status.level = DiagnosticStatus.WARN
                status.message = f"Low data validity: {np.mean(self.validity_ratios):.2f}"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "RPLidar operating normally"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "Insufficient data for analysis"
        
        # Add diagnostic values
        status.values = []
        
        # Basic stats
        status.values.append(KeyValue(key="Total Scans", value=str(self.scan_count)))
        status.values.append(KeyValue(key="Uptime", value=f"{current_time - self.start_time:.1f} s"))
        
        # Frequency stats
        if len(self.scan_frequencies) > 0:
            avg_freq = np.mean(self.scan_frequencies)
            freq_std = np.std(self.scan_frequencies)
            status.values.append(KeyValue(key="Average Frequency", value=f"{avg_freq:.2f} Hz"))
            status.values.append(KeyValue(key="Frequency Std Dev", value=f"{freq_std:.2f} Hz"))
        
        # Quality stats
        if len(self.validity_ratios) > 0:
            avg_validity = np.mean(self.validity_ratios)
            status.values.append(KeyValue(key="Data Validity", value=f"{avg_validity:.3f}"))
        
        if len(self.range_variations) > 0:
            avg_variation = np.mean(self.range_variations)
            status.values.append(KeyValue(key="Range Variation", value=f"{avg_variation:.3f} m"))
        
        # Error stats
        status.values.append(KeyValue(key="Max Consecutive Missing", value=str(self.max_missing_scans)))
        status.values.append(KeyValue(key="Current Missing", value=str(self.consecutive_missing_scans)))
        
        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)

def main():
    rclpy.init()
    diagnostics = RPLidarDiagnostics()
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()