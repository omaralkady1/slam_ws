#!/usr/bin/env python3

import sys
import os
import argparse
import json
import subprocess
import signal
import time

def run_command(command):
    """Run a command and return its output"""
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    stdout, stderr = process.communicate()
    return stdout.decode('utf-8'), stderr.decode('utf-8'), process.returncode

def start_navigation():
    """Start the navigation stack if it's not already running"""
    print("Checking if navigation is running...")
    stdout, stderr, rc = run_command("ros2 node list | grep controller_server")
    
    if rc != 0:
        print("Starting navigation stack...")
        nav_cmd = "ros2 launch my_slam_pkg navigation_with_controllers.launch.py"
        nav_process = subprocess.Popen(nav_cmd, shell=True)
        print("Waiting for navigation to initialize...")
        time.sleep(10)  # Give Nav2 time to start
        return nav_process
    else:
        print("Navigation stack is already running.")
        return None

def start_goal_listener():
    """Start the goal listener if it's not already running"""
    print("Checking if goal listener is running...")
    stdout, stderr, rc = run_command("ros2 node list | grep nav_goal_listener")
    
    if rc != 0:
        print("Starting goal listener...")
        listener_cmd = "ros2 launch my_slam_pkg nav_goal_listener.launch.py"
        listener_process = subprocess.Popen(listener_cmd, shell=True)
        print("Waiting for goal listener to initialize...")
        time.sleep(5)  # Give the listener time to start
        return listener_process
    else:
        print("Goal listener is already running.")
        return None

def send_goal(x, y, theta=0.0):
    """Send a goal to the navigation stack"""
    print(f"Sending goal: x={x}, y={y}, theta={theta}")
    cmd = f"ros2 run my_slam_pkg nav_goal_client.py {x} {y} {theta}"
    process = subprocess.Popen(cmd, shell=True)
    return process

def send_waypoints(waypoints):
    """Send a series of waypoints to the navigation stack"""
    cmd = "ros2 run my_slam_pkg send_waypoints.py --waypoints"
    
    for wp in waypoints:
        if len(wp) == 2:
            cmd += f" {wp[0]} {wp[1]}"
        else:
            cmd += f" {wp[0]} {wp[1]} {wp[2]}"
    
    print(f"Sending waypoints: {waypoints}")
    process = subprocess.Popen(cmd, shell=True)
    return process

def main():
    parser = argparse.ArgumentParser(description='Navigate to coordinates')
    parser.add_argument('--x', type=float, help='X coordinate')
    parser.add_argument('--y', type=float, help='Y coordinate')
    parser.add_argument('--theta', type=float, default=0.0, help='Orientation in radians')
    parser.add_argument('--waypoints', type=str, help='JSON file with waypoints')
    parser.add_argument('--start-nav', action='store_true', help='Start navigation stack if not running')
    parser.add_argument('--visualize', action='store_true', help='Visualize waypoints in RViz')
    
    args = parser.parse_args()
    
    # Start processes
    nav_process = None
    listener_process = None
    
    if args.start_nav:
        nav_process = start_navigation()
    
    # Always start the goal listener
    listener_process = start_goal_listener()
    
    try:
        # Send goal or waypoints
        if args.waypoints:
            with open(args.waypoints, 'r') as f:
                waypoints = json.load(f)
            
            if args.visualize:
                # Start visualization for each waypoint
                for wp in waypoints:
                    if len(wp) == 2:
                        run_command(f"ros2 run my_slam_pkg nav_visualization.py {wp[0]} {wp[1]}")
                    else:
                        run_command(f"ros2 run my_slam_pkg nav_visualization.py {wp[0]} {wp[1]} {wp[2]}")
            
            goal_process = send_waypoints(waypoints)
            goal_process.wait()  # Wait for navigation to complete
            
        elif args.x is not None and args.y is not None:
            if args.visualize:
                run_command(f"ros2 run my_slam_pkg nav_visualization.py {args.x} {args.y} {args.theta}")
                
            goal_process = send_goal(args.x, args.y, args.theta)
            goal_process.wait()  # Wait for navigation to complete
            
        else:
            print("Error: Must specify either --x and --y or --waypoints")
            parser.print_help()
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("Navigation interrupted.")
    
    finally:
        # Clean up processes
        if nav_process:
            print("Stopping navigation stack...")
            nav_process.send_signal(signal.SIGINT)
            nav_process.wait()
            
        if listener_process:
            print("Stopping goal listener...")
            listener_process.send_signal(signal.SIGINT)
            listener_process.wait()

if __name__ == '__main__':
    main()