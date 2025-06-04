
#!/usr/bin/env python3

import os
import subprocess
import sys
import time

def run_command(command, check=True):
    """Run a shell command and return output"""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True, check=check)
        return result.stdout.strip(), result.stderr.strip(), result.returncode
    except subprocess.CalledProcessError as e:
        return e.stdout, e.stderr, e.returncode

def check_rplidar_package():
    """Check if rplidar_ros package is installed"""
    print("Checking for rplidar_ros package...")
    stdout, stderr, returncode = run_command("ros2 pkg list | grep rplidar", check=False)
    
    if returncode == 0:
        print("✅ rplidar_ros package found")
        return True
    else:
        print("❌ rplidar_ros package not found")
        return False

def install_rplidar_package():
    """Install rplidar_ros package"""
    print("Installing rplidar_ros package...")
    
    # Try apt install first
    stdout, stderr, returncode = run_command("sudo apt update && sudo apt install -y ros-humble-rplidar-ros", check=False)
    
    if returncode == 0:
        print("✅ rplidar_ros installed via apt")
        return True
    else:
        print("❌ Failed to install via apt. Trying from source...")
        
        # Install from source
        commands = [
            "cd ~/slam_ws/src",
            "git clone https://github.com/Slamtec/rplidar_ros.git",
            "cd ~/slam_ws",
            "colcon build --packages-select rplidar_ros",
            "source install/setup.bash"
        ]
        
        for cmd in commands:
            print(f"Running: {cmd}")
            stdout, stderr, returncode = run_command(cmd, check=False)
            if returncode != 0:
                print(f"❌ Failed: {cmd}")
                print(f"Error: {stderr}")
                return False
        
        print("✅ rplidar_ros built from source")
        return True

def check_serial_ports():
    """Check available serial ports"""
    print("Checking available serial ports...")
    
    # Check for USB serial devices
    stdout, stderr, returncode = run_command("ls /dev/ttyUSB* 2>/dev/null || echo 'No ttyUSB devices'", check=False)
    if "ttyUSB" in stdout:
        print(f"USB devices: {stdout}")
    else:
        print("No USB serial devices found")
    
    # Check for ACM devices
    stdout, stderr, returncode = run_command("ls /dev/ttyACM* 2>/dev/null || echo 'No ttyACM devices'", check=False)
    if "ttyACM" in stdout:
        print(f"ACM devices: {stdout}")
    else:
        print("No ACM serial devices found")
    
    return stdout

def setup_serial_permissions():
    """Setup serial port permissions"""
    print("Setting up serial port permissions...")
    
    # Add user to dialout group
    username = os.getenv('USER')
    stdout, stderr, returncode = run_command(f"sudo usermod -a -G dialout {username}", check=False)
    
    if returncode == 0:
        print("✅ Added user to dialout group")
        print("⚠️  You need to log out and log back in for this to take effect")
    else:
        print("❌ Failed to add user to dialout group")
        print(f"Error: {stderr}")

def test_rplidar_connection(port):
    """Test RPLidar connection"""
    print(f"Testing RPLidar connection on {port}...")
    
    # Simple connection test
    command = f"sudo chmod 666 {port} && echo 'Port accessible' || echo 'Port access failed'"
    stdout, stderr, returncode = run_command(command, check=False)
    
    if "accessible" in stdout:
        print(f"✅ Port {port} is accessible")
        return True
    else:
        print(f"❌ Cannot access port {port}")
        return False

def create_udev_rules():
    """Create udev rules for RPLidar"""
    print("Creating udev rules for RPLidar...")
    
    udev_rule = '''# RPLidar A1 udev rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="rplidar"
KERNEL=="ttyUSB*", ATTRS{product}=="CP2102 USB to UART Bridge Controller", MODE="0666", GROUP="dialout", SYMLINK+="rplidar"
'''
    
    try:
        with open('/tmp/99-rplidar.rules', 'w') as f:
            f.write(udev_rule)
        
        # Copy to udev rules directory
        stdout, stderr, returncode = run_command("sudo cp /tmp/99-rplidar.rules /etc/udev/rules.d/", check=False)
        
        if returncode == 0:
            # Reload udev rules
            run_command("sudo udevadm control --reload-rules", check=False)
            run_command("sudo udevadm trigger", check=False)
            print("✅ Udev rules created and reloaded")
            print("🔄 You may need to unplug and replug your RPLidar")
            return True
        else:
            print("❌ Failed to create udev rules")
            return False
    except Exception as e:
        print(f"❌ Error creating udev rules: {e}")
        return False

def test_rplidar_node(port):
    """Test RPLidar node launch"""
    print(f"Testing RPLidar node on {port}...")
    
    # Launch RPLidar node for testing
    command = f"timeout 10s ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:={port}"
    print(f"Running: {command}")
    
    stdout, stderr, returncode = run_command(command, check=False)
    
    if "successfully" in stdout.lower() or "initialized" in stdout.lower():
        print("✅ RPLidar node launched successfully")
        return True
    else:
        print("❌ RPLidar node failed to launch")
        print(f"Output: {stdout}")
        print(f"Error: {stderr}")
        return False

def main():
    print("=== RPLidar A1 Setup Script ===")
    
    # Check if running with correct ROS environment
    if not os.getenv('ROS_DISTRO'):
        print("❌ ROS environment not sourced. Please run:")
        print("source /opt/ros/humble/setup.bash")
        print("source ~/slam_ws/install/setup.bash")
        return
    
    print(f"ROS Distribution: {os.getenv('ROS_DISTRO')}")
    
    # Step 1: Check and install rplidar package
    if not check_rplidar_package():
        if not install_rplidar_package():
            print("❌ Failed to install rplidar_ros package")
            return
    
    # Step 2: Check serial ports
    ports = check_serial_ports()
    
    # Step 3: Setup permissions
    setup_serial_permissions()
    
    # Step 4: Create udev rules
    create_udev_rules()
    
    # Step 5: Test connection
    test_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/rplidar']
    working_port = None
    
    for port in test_ports:
        if os.path.exists(port):
            if test_rplidar_connection(port):
                working_port = port
                break
    
    if working_port:
        print(f"✅ Found working port: {working_port}")
        
        # Test RPLidar node
        if test_rplidar_node(working_port):
            print("\n=== Setup Complete ===")
            print(f"✅ RPLidar A1 is ready to use on {working_port}")
            print("\nTo test your setup, run:")
            print(f"ros2 launch my_slam_pkg esp32_real_robot.launch.py lidar_port:={working_port}")
            print("\nFor SLAM, run:")
            print(f"ros2 launch my_slam_pkg esp32_slam_real.launch.py lidar_port:={working_port}")
        else:
            print("❌ RPLidar node test failed")
    else:
        print("❌ No working RPLidar port found")
        print("Please check:")
        print("1. RPLidar is connected via USB")
        print("2. RPLidar power cable is connected")
        print("3. Try different USB ports")
        print("4. Check with: lsusb | grep -i 'cp210\\|silicon'")

if __name__ == '__main__':
    main()