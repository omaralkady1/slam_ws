#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        
        self.publisher = self.create_publisher(
            Twist, 
            '/diff_drive_controller/cmd_vel', 
            10
        )
        
        self.get_logger().info('Motor test node started')
        self.get_logger().info('Commands:')
        self.get_logger().info('  f: forward')
        self.get_logger().info('  b: backward')
        self.get_logger().info('  l: turn left')
        self.get_logger().info('  r: turn right')
        self.get_logger().info('  s: stop')
        self.get_logger().info('  t: run test sequence')
        self.get_logger().info('  q: quit')
        
    def send_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent: linear={linear_x}, angular={angular_z}')
        
    def run_test_sequence(self):
        self.get_logger().info('Running test sequence...')
        
        tests = [
            ('Forward', 0.2, 0.0, 2.0),
            ('Backward', -0.2, 0.0, 2.0),
            ('Turn left', 0.0, 0.5, 2.0),
            ('Turn right', 0.0, -0.5, 2.0),
            ('Arc left', 0.2, 0.3, 3.0),
            ('Arc right', 0.2, -0.3, 3.0),
        ]
        
        for name, linear, angular, duration in tests:
            self.get_logger().info(f'Test: {name}')
            self.send_cmd(linear, angular)
            time.sleep(duration)
            
        self.send_cmd(0.0, 0.0)
        self.get_logger().info('Test sequence complete')
        
    def interactive_control(self):
        while True:
            try:
                cmd = input('Command (f/b/l/r/s/t/q): ').lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'f':
                    self.send_cmd(0.2, 0.0)
                elif cmd == 'b':
                    self.send_cmd(-0.2, 0.0)
                elif cmd == 'l':
                    self.send_cmd(0.0, 0.5)
                elif cmd == 'r':
                    self.send_cmd(0.0, -0.5)
                elif cmd == 's':
                    self.send_cmd(0.0, 0.0)
                elif cmd == 't':
                    self.run_test_sequence()
                else:
                    self.get_logger().info('Unknown command')
                    
            except KeyboardInterrupt:
                break
                
        self.send_cmd(0.0, 0.0)
        self.get_logger().info('Exiting...')

def main(args=None):
    rclpy.init(args=args)
    node = MotorTest()
    
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        node.run_test_sequence()
    else:
        node.interactive_control()
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()