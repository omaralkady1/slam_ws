#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import time

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Create velocity publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Control parameters
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.5  # rad/s
        self.max_linear = 0.5
        self.max_angular = 1.0
        self.linear_increment = 0.05
        self.angular_increment = 0.1
        
        # Current velocity
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Print instructions
        self.print_instructions()
    
    def timer_callback(self):
        # Create and publish Twist message
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.vel_pub.publish(msg)
    
    def print_instructions(self):
        instructions = """
Control Your Robot:
------------------
   w    
a  s  d  - Basic movement (forward, left, back, right)

q/e     - Rotate left/right

+/-     - Increase/decrease linear speed
</,     - Increase/decrease angular speed

SPACE   - Full stop

CTRL+C  - Quit
"""
        print(instructions)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def print_status(self):
        status_msg = f"\rLinear: {self.linear_x:.2f} m/s | Angular: {self.angular_z:.2f} rad/s | "
        status_msg += f"Speed: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}   "
        print(status_msg, end='', flush=True)
    
    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while True:
                key = self.get_key()
                
                if key == '\x03':  # CTRL+C
                    break
                
                # Linear movement
                if key == 'w':
                    self.linear_x = self.linear_speed
                    self.angular_z = 0.0
                elif key == 's':
                    self.linear_x = -self.linear_speed
                    self.angular_z = 0.0
                
                # Angular movement
                elif key == 'a':
                    self.linear_x = 0.0
                    self.angular_z = self.angular_speed
                elif key == 'd':
                    self.linear_x = 0.0
                    self.angular_z = -self.angular_speed
                
                # Combined movement
                elif key == 'q':
                    self.linear_x = self.linear_speed
                    self.angular_z = self.angular_speed
                elif key == 'e':
                    self.linear_x = self.linear_speed
                    self.angular_z = -self.angular_speed
                
                # Speed control
                elif key == '+':
                    self.linear_speed = min(self.max_linear, self.linear_speed + self.linear_increment)
                elif key == '-':
                    self.linear_speed = max(0.0, self.linear_speed - self.linear_increment)
                elif key == '<':
                    self.angular_speed = max(0.0, self.angular_speed - self.angular_increment)
                elif key == '>':
                    self.angular_speed = min(self.max_angular, self.angular_speed + self.angular_increment)
                
                # Stop
                elif key == ' ':
                    self.linear_x = 0.0
                    self.angular_z = 0.0
                
                self.print_status()
                
                # Small sleep to avoid flooding
                time.sleep(0.01)
                
        finally:
            # Make sure to stop the robot when exiting
            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nExiting...")

def main():
    rclpy.init()
    controller = KeyboardController()
    
    # Run in a separate thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,))
    spin_thread.start()
    
    try:
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
