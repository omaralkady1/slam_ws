#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

class TFMonitor(Node):
    def __init__(self):
        super().__init__('tf_monitor')
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Define frames to monitor
        self.parent_frames = ['map', 'odom', 'base_footprint', 'base_link']
        self.child_frames = ['odom', 'base_footprint', 'base_link', 'lidar_link', 
                             'front_left_wheel', 'front_right_wheel', 
                             'rear_left_wheel', 'rear_right_wheel']
        
        # Create status publisher
        self.status_publisher = self.create_publisher(
            String, 'tf_monitor/status', 10)
        
        # Create timer for periodic checking
        self.timer = self.create_timer(1.0, self.check_transforms)
        
        # Stats for transforms
        self.transform_stats = {}
        
        self.get_logger().info('TF Monitor started')
        
    def check_transforms(self):
        """Check all relevant transforms in the TF tree"""
        # Get current time
        now = self.get_clock().now()
        status_msg = String()
        status_text = "===== TF Status =====\n"
        all_ok = True
        
        for parent_frame in self.parent_frames:
            for child_frame in self.child_frames:
                if parent_frame != child_frame:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            parent_frame, child_frame, rclpy.time.Time())
                        
                        # Store transform stats
                        key = f"{parent_frame} -> {child_frame}"
                        if key not in self.transform_stats:
                            self.transform_stats[key] = {
                                'count': 0,
                                'last_time': None
                            }
                        
                        # Update stats
                        self.transform_stats[key]['count'] += 1
                        
                        # Calculate delta time if we have a previous timestamp
                        dt_text = ""
                        if self.transform_stats[key]['last_time'] is not None:
                            dt = now.nanoseconds - self.transform_stats[key]['last_time']
                            dt_ms = dt / 1000000.0
                            dt_text = f", dt={dt_ms:.1f}ms"
                        
                        self.transform_stats[key]['last_time'] = now.nanoseconds
                        
                        # Add to status text
                        status_text += f"✅ {key} (x:{transform.transform.translation.x:.2f}, y:{transform.transform.translation.y:.2f}{dt_text})\n"
                    
                    except TransformException as ex:
                        # Add to status text if it's a transform we expect to exist
                        # Check if this is a required transform
                        required = False
                        if parent_frame == 'map' and child_frame == 'odom':
                            required = True
                        elif parent_frame == 'odom' and child_frame == 'base_footprint':
                            required = True
                        elif parent_frame == 'base_footprint' and child_frame == 'base_link':
                            required = True
                        elif parent_frame == 'base_link' and child_frame in ['lidar_link', 'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']:
                            required = True
                            
                        if required:
                            status_text += f"❌ {parent_frame} -> {child_frame}: {ex}\n"
                            all_ok = False
        
        # Add summary
        if all_ok:
            status_text += "\n✅ All required transforms found!\n"
        else:
            status_text += "\n❌ Missing some required transforms!\n"
            
        # Add diagnostics
        status_text += "\nFor detailed TF analysis run:\n"
        status_text += "  ros2 run tf2_tools view_frames.py\n"
        
        # Publish status
        status_msg.data = status_text
        self.status_publisher.publish(status_msg)
        
        # Print to console
        if not all_ok:
            self.get_logger().warn(status_text)
        else:
            self.get_logger().info("TF tree check completed: All transforms OK")

def main(args=None):
    rclpy.init(args=args)
    monitor = TFMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()