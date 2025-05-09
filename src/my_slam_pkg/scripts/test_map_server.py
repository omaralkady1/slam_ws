#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time

class MapTester(Node):
    def __init__(self):
        super().__init__('map_tester')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.map_received = False
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Map tester started')
    
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            self.get_logger().info(f'Received map: {width}x{height} cells, resolution: {resolution} m/cell')
    
    def timer_callback(self):
        elapsed = time.time() - self.start_time
        if not self.map_received:
            self.get_logger().info(f'Waiting for map... ({elapsed:.1f}s)')
            if elapsed > 10.0:
                self.get_logger().error('No map received after 10 seconds. Check your map server!')
                rclpy.shutdown()
        else:
            self.get_logger().info('Map test complete!')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tester = MapTester()
    rclpy.spin(tester)
    rclpy.shutdown()

if __name__ == '__main__':
    main()