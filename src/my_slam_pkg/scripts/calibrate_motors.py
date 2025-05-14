#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import sys
import time
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf_transformations
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class MotorCalibrator(Node):
    def __init__(self):
        super().__init__('motor_calibrator')
        
        # Create cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel',
            10
        )
        
        # Create joint states subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Initialize joint states
        self.joint_positions = {}
        self.joint_velocities = {}
        self.received_joint_state = False
        
        # Initialize odometry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.received_odom = False
        
        # Store test results
        self.test_results = {
            'linear': [],
            'angular': [],
            'wheel_radius': [],
            'wheel_separation': []
        }
        
        # Path tracking for visualization
        self.path_x = []
        self.path_y = []
        
        self.get_logger().info('Motor calibrator started')
    
    def joint_state_callback(self, msg):
        """Store joint states from the message"""
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = msg.velocity[i]
        
        self.received_joint_state = True
    
    def odom_callback(self, msg):
        """Store odometry data"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_theta = euler[2]  # yaw
        
        self.received_odom = True
        
        # Store path for visualization
        self.path_x.append(self.odom_x)
        self.path_y.append(self.odom_y)
    
    def send_cmd_vel(self, linear_x, angular_z, duration=2.0):
        """Send a cmd_vel command for the specified duration"""
        self.get_logger().info(f'Sending cmd_vel: linear_x={linear_x}, angular_z={angular_z}')
        
        # Record initial position
        initial_x = self.odom_x
        initial_y = self.odom_y
        initial_theta = self.odom_theta
        
        # Clear path tracking
        self.path_x = [initial_x]
        self.path_y = [initial_y]
        
        # Create and send command
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        
        # Time to start measuring
        start_time = time.time()
        
        # Send the command repeatedly for the duration
        rate = self.create_rate(10)  # 10Hz
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
            
            # Process some callbacks
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Command complete')
        
        # Give time for the robot to stop and process final position
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)
            
        # Calculate distance and rotation
        final_x = self.odom_x
        final_y = self.odom_y
        final_theta = self.odom_theta
        
        distance = math.sqrt((final_x - initial_x)**2 + (final_y - initial_y)**2)
        rotation = self.normalize_angle(final_theta - initial_theta)
        
        # Calculate expected values
        expected_distance = linear_x * duration
        expected_rotation = angular_z * duration
        
        # Calculate wheel parameters
        if abs(linear_x) > 0.01 and abs(angular_z) < 0.01:
            # Linear motion - estimate wheel radius
            estimated_radius = distance / (duration * linear_x)
            if abs(estimated_radius) > 0.01:  # Avoid division by zero issues
                self.test_results['wheel_radius'].append(estimated_radius)
            
        if abs(angular_z) > 0.01 and abs(linear_x) < 0.01:
            # Rotational motion - estimate wheel separation
            estimated_separation = (rotation * 2) / (duration * angular_z)
            if abs(estimated_separation) > 0.01:  # Avoid division by zero issues
                self.test_results['wheel_separation'].append(estimated_separation)
        
        # Store results
        result = {
            'command': {'linear_x': linear_x, 'angular_z': angular_z, 'duration': duration},
            'initial': {'x': initial_x, 'y': initial_y, 'theta': initial_theta},
            'final': {'x': final_x, 'y': final_y, 'theta': final_theta},
            'measured': {'distance': distance, 'rotation': rotation},
            'expected': {'distance': expected_distance, 'rotation': expected_rotation},
            'error': {
                'distance': distance - expected_distance,
                'rotation': rotation - expected_rotation
            }
        }
        
        if abs(linear_x) > 0.01:
            self.test_results['linear'].append(result)
        
        if abs(angular_z) > 0.01:
            self.test_results['angular'].append(result)
        
        return result
        
    def normalize_angle(self, angle):
        """Normalize angle to -pi to pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def print_joint_states(self):
        """Print current joint states"""
        if not self.received_joint_state:
            self.get_logger().warn('No joint states received yet')
            return
            
        self.get_logger().info('Joint states:')
        for name in sorted(self.joint_positions.keys()):
            pos = self.joint_positions[name]
            vel = self.joint_velocities[name]
            self.get_logger().info(f'  {name}: position={pos:.4f}, velocity={vel:.4f}')
    
    def print_odometry(self):
        """Print current odometry"""
        if not self.received_odom:
            self.get_logger().warn('No odometry received yet')
            return
            
        self.get_logger().info(f'Odometry: x={self.odom_x:.4f}, y={self.odom_y:.4f}, theta={self.odom_theta:.4f}')
    
    def run_calibration(self):
        """Run calibration tests"""
        self.get_logger().info('Starting calibration')
        
        # Wait for initial data
        timeout = 10.0  # 10 seconds
        start_time = time.time()
        while (not self.received_joint_state or not self.received_odom) and time.time() - start_time < timeout:
            self.get_logger().info('Waiting for joint states and odometry...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        if not self.received_joint_state or not self.received_odom:
            self.get_logger().error('Missing joint states or odometry. Check hardware connection.')
            return False
            
        # Print initial state
        self.get_logger().info('Initial state:')
        self.print_joint_states()
        self.print_odometry()
        
        # Linear motion tests
        linear_speeds = [0.05, 0.1, 0.15, 0.2, -0.05, -0.1, -0.15]
        for speed in linear_speeds:
            self.get_logger().info(f"\n=== Linear motion test: {speed} m/s ===")
            result = self.send_cmd_vel(speed, 0.0, 3.0)
            self.print_results(result)
            time.sleep(1.0)
        
        # Rotational motion tests
        angular_speeds = [0.5, 0.75, 1.0, -0.5, -0.75, -1.0]
        for speed in angular_speeds:
            self.get_logger().info(f"\n=== Rotational motion test: {speed} rad/s ===")
            result = self.send_cmd_vel(0.0, speed, 3.0)
            self.print_results(result)
            time.sleep(1.0)
        
        # Combined motion tests
        combined_tests = [
            {'linear_x': 0.1, 'angular_z': 0.5},
            {'linear_x': 0.1, 'angular_z': -0.5},
            {'linear_x': -0.1, 'angular_z': 0.5},
            {'linear_x': -0.1, 'angular_z': -0.5}
        ]
        for test in combined_tests:
            self.get_logger().info(f"\n=== Combined motion test: linear_x={test['linear_x']}, angular_z={test['angular_z']} ===")
            result = self.send_cmd_vel(test['linear_x'], test['angular_z'], 3.0)
            self.print_results(result)
            time.sleep(1.0)
        
        # Analyze results
        self.analyze_results()
        
        # Plot path
        self.plot_path()
        
        self.get_logger().info('\n=== Calibration complete ===')
        return True
    
    def print_results(self, result):
        """Print test results"""
        cmd = result['command']
        measured = result['measured']
        expected = result['expected']
        error = result['error']
        
        self.get_logger().info(f"Command: linear_x={cmd['linear_x']}, angular_z={cmd['angular_z']}, duration={cmd['duration']}")
        self.get_logger().info(f"Measured: distance={measured['distance']:.4f}m, rotation={measured['rotation']:.4f}rad")
        self.get_logger().info(f"Expected: distance={expected['distance']:.4f}m, rotation={expected['rotation']:.4f}rad")
        
        distance_error_pct = 0.0
        if abs(expected['distance']) > 0.001:
            distance_error_pct = 100.0 * error['distance'] / expected['distance']
            
        rotation_error_pct = 0.0
        if abs(expected['rotation']) > 0.001:
            rotation_error_pct = 100.0 * error['rotation'] / expected['rotation']
            
        self.get_logger().info(f"Error: distance={error['distance']:.4f}m ({distance_error_pct:.1f}%), " +
                              f"rotation={error['rotation']:.4f}rad ({rotation_error_pct:.1f}%)")
        
        # Print current odometry for verification
        self.print_odometry()
    
    def analyze_results(self):
        """Analyze calibration results and suggest parameters"""
        self.get_logger().info("\n=== Calibration Analysis ===")
        
        # Analyze wheel radius
        if len(self.test_results['wheel_radius']) > 0:
            radii = np.array(self.test_results['wheel_radius'])
            avg_radius = np.mean(radii)
            std_radius = np.std(radii)
            
            self.get_logger().info(f"Wheel radius: average={avg_radius:.4f}m, stdev={std_radius:.4f}m")
            self.get_logger().info(f"Wheel radius tests: {len(radii)} measurements")
            
            # Suggest adjustments to wheel_radius parameter
            self.get_logger().info("Suggested wheel_radius parameter: " +
                                  f"wheel_radius: {avg_radius:.4f}  # Calibrated value")
        else:
            self.get_logger().info("No wheel radius measurements collected")
        
        # Analyze wheel separation
        if len(self.test_results['wheel_separation']) > 0:
            separations = np.array(self.test_results['wheel_separation'])
            avg_separation = np.mean(separations)
            std_separation = np.std(separations)
            
            self.get_logger().info(f"Wheel separation: average={avg_separation:.4f}m, stdev={std_separation:.4f}m")
            self.get_logger().info(f"Wheel separation tests: {len(separations)} measurements")
            
            # Suggest adjustments to wheel_separation parameter
            self.get_logger().info("Suggested wheel_separation parameter: " +
                                  f"wheel_separation: {avg_separation:.4f}  # Calibrated value")
        else:
            self.get_logger().info("No wheel separation measurements collected")
        
        # Analyze linear motion errors
        if len(self.test_results['linear']) > 0:
            linear_errors = np.array([r['error']['distance'] for r in self.test_results['linear']])
            avg_linear_error = np.mean(linear_errors)
            std_linear_error = np.std(linear_errors)
            
            self.get_logger().info(f"Linear motion error: average={avg_linear_error:.4f}m, stdev={std_linear_error:.4f}m")
            
            # Calculate multiplier for wheel_radius to compensate for linear error
            if abs(avg_linear_error) > 0.01:
                radius_multiplier = 1.0
                if avg_linear_error < 0:
                    # Robot moves less than expected, increase effective wheel radius
                    radius_multiplier = 1.0 / (1.0 + abs(avg_linear_error) / 0.5)
                else:
                    # Robot moves more than expected, decrease effective wheel radius
                    radius_multiplier = 1.0 / (1.0 - abs(avg_linear_error) / 0.5)
                
                radius_multiplier = max(0.9, min(1.1, radius_multiplier))  # Limit to reasonable range
                
                self.get_logger().info("Suggested wheel radius multiplier: " +
                                      f"wheel_radius_multiplier: {radius_multiplier:.4f}  # Calibration correction")
        
        # Analyze angular motion errors
        if len(self.test_results['angular']) > 0:
            angular_errors = np.array([r['error']['rotation'] for r in self.test_results['angular']])
            avg_angular_error = np.mean(angular_errors)
            std_angular_error = np.std(angular_errors)
            
            self.get_logger().info(f"Angular motion error: average={avg_angular_error:.4f}rad, stdev={std_angular_error:.4f}rad")
            
            # Calculate multiplier for wheel_separation to compensate for angular error
            if abs(avg_angular_error) > 0.01:
                separation_multiplier = 1.0
                if avg_angular_error < 0:
                    # Robot turns less than expected, decrease effective wheel separation
                    separation_multiplier = 1.0 - abs(avg_angular_error) / 1.0
                else:
                    # Robot turns more than expected, increase effective wheel separation
                    separation_multiplier = 1.0 + abs(avg_angular_error) / 1.0
                
                separation_multiplier = max(0.9, min(1.1, separation_multiplier))  # Limit to reasonable range
                
                self.get_logger().info("Suggested wheel separation multiplier: " +
                                      f"wheel_separation_multiplier: {separation_multiplier:.4f}  # Calibration correction")
        
        # Generate complete suggested configuration
        self.get_logger().info("\n=== Suggested Controller Configuration ===")
        self.get_logger().info("Add these values to your hardware_controllers.yaml file:")
        
        config = "diff_drive_controller:\n  ros__parameters:\n"
        
        if len(self.test_results['wheel_radius']) > 0:
            config += f"    wheel_radius: {np.mean(self.test_results['wheel_radius']):.4f}\n"
        
        if len(self.test_results['wheel_separation']) > 0:
            config += f"    wheel_separation: {np.mean(self.test_results['wheel_separation']):.4f}\n"
        
        # Add multipliers if we have error data
        if len(self.test_results['linear']) > 0:
            linear_errors = np.array([r['error']['distance'] for r in self.test_results['linear']])
            avg_linear_error = np.mean(linear_errors)
            
            if abs(avg_linear_error) > 0.01:
                radius_multiplier = 1.0
                if avg_linear_error < 0:
                    radius_multiplier = 1.0 / (1.0 + abs(avg_linear_error) / 0.5)
                else:
                    radius_multiplier = 1.0 / (1.0 - abs(avg_linear_error) / 0.5)
                
                radius_multiplier = max(0.9, min(1.1, radius_multiplier))
                config += f"    left_wheel_radius_multiplier: {radius_multiplier:.4f}\n"
                config += f"    right_wheel_radius_multiplier: {radius_multiplier:.4f}\n"
        
        if len(self.test_results['angular']) > 0:
            angular_errors = np.array([r['error']['rotation'] for r in self.test_results['angular']])
            avg_angular_error = np.mean(angular_errors)
            
            if abs(avg_angular_error) > 0.01:
                separation_multiplier = 1.0
                if avg_angular_error < 0:
                    separation_multiplier = 1.0 - abs(avg_angular_error) / 1.0
                else:
                    separation_multiplier = 1.0 + abs(avg_angular_error) / 1.0
                
                separation_multiplier = max(0.9, min(1.1, separation_multiplier))
                config += f"    wheel_separation_multiplier: {separation_multiplier:.4f}\n"
        
        self.get_logger().info(config)
        
        # Save configuration to file
        try:
            with open('calibrated_params.yaml', 'w') as f:
                f.write(config)
            self.get_logger().info("Calibrated parameters saved to calibrated_params.yaml")
        except Exception as e:
            self.get_logger().error(f"Failed to save calibration file: {e}")
    
    def plot_path(self):
        """Plot the robot's path during calibration"""
        try:
            plt.figure(figsize=(8, 8))
            plt.plot(self.path_x, self.path_y, 'b-', label='Robot Path')
            plt.plot(self.path_x[0], self.path_y[0], 'go', label='Start')
            plt.plot(self.path_x[-1], self.path_y[-1], 'ro', label='End')
            
            # Add arrows to show direction
            for i in range(0, len(self.path_x), max(1, len(self.path_x) // 20)):
                if i+1 < len(self.path_x):
                    dx = self.path_x[i+1] - self.path_x[i]
                    dy = self.path_y[i+1] - self.path_y[i]
                    plt.arrow(self.path_x[i], self.path_y[i], dx, dy, head_width=0.02, head_length=0.03, fc='blue', ec='blue')
            
            plt.grid(True)
            plt.axis('equal')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.title('Robot Path During Calibration')
            plt.legend()
            
            # Save the plot
            plt.savefig('calibration_path.png')
            self.get_logger().info("Path plot saved to calibration_path.png")
            
            # Try to display the plot
            plt.show()
        except Exception as e:
            self.get_logger().error(f"Failed to create path plot: {e}")

def main():
    rclpy.init()
    calibrator = MotorCalibrator()
    
    try:
        if len(sys.argv) > 1 and sys.argv[1] == 'calibrate':
            calibrator.run_calibration()
        else:
            print("Usage:")
            print("  ros2 run my_slam_pkg calibrate_motors.py calibrate")
            print("    - Run the calibration procedure")
    except KeyboardInterrupt:
        pass
    
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()