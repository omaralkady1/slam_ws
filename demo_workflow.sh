#!/bin/bash

# ESP32 Hardware Interface Workflow Demonstration Script
# This script demonstrates the complete workflow of the ESP32 Hardware Interface

set -e  # Exit on any error

echo "🚀 ESP32 Hardware Interface Workflow Demonstration"
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_step() {
    echo -e "${BLUE}📋 Step $1: $2${NC}"
}

print_info() {
    echo -e "${GREEN}ℹ️  $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Check if we're in the right workspace
if [ ! -d "/home/alkady/slam_ws/src/my_slam_pkg" ]; then
    print_error "Please run this script from the slam_ws directory"
    exit 1
fi

cd /home/alkady/slam_ws

# Source the workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    print_info "Workspace sourced successfully"
else
    print_error "Workspace not built. Please run 'colcon build' first"
    exit 1
fi

print_step "1" "System Overview"
echo "This demonstration shows how the ESP32HardwareInterface works:"
echo "  • ROS 2 Navigation sends cmd_vel commands"
echo "  • DiffDriveController converts to wheel velocities"  
echo "  • ESP32HardwareInterface converts back to cmd_vel for ESP32"
echo "  • ESP32 firmware controls motors and reads encoders"
echo "  • Joint states flow back through the same path"
echo ""

print_step "2" "Checking System Components"

# Check if ROS 2 is available
if ! command -v ros2 &> /dev/null; then
    print_error "ROS 2 not found. Please source ROS 2 installation"
    exit 1
fi
print_info "✓ ROS 2 available"

# Check if package is built
if [ ! -f "install/my_slam_pkg/lib/libmy_slam_pkg__rosidl_typesupport_cpp.so" ]; then
    print_warning "Package may not be fully built. Building now..."
    colcon build --packages-select my_slam_pkg
fi
print_info "✓ Package built"

print_step "3" "Hardware Interface Configuration Analysis"

# Show the hardware interface configuration
echo "Current hardware interface configuration:"
echo "----------------------------------------"
if [ -f "src/my_slam_pkg/config/hardware_controllers.yaml" ]; then
    echo "📄 Key parameters from hardware_controllers.yaml:"
    grep -E "(wheel_radius|wheel_separation|cmd_vel_topic|joint_states_topic)" src/my_slam_pkg/config/hardware_controllers.yaml || true
else
    print_warning "Configuration file not found"
fi
echo ""

print_step "4" "Demonstrating Command Flow"
echo "Starting hardware interface in simulation mode..."

# Start the hardware interface in background
print_info "Launching hardware control system..."
ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=true &
LAUNCH_PID=$!

# Wait for system to start up
print_info "Waiting for system startup (10 seconds)..."
sleep 10

# Check if the system started successfully
if ! ps -p $LAUNCH_PID > /dev/null; then
    print_error "Hardware interface failed to start"
    exit 1
fi

print_info "✓ Hardware interface started successfully"

# Give it a moment to fully initialize
sleep 5

print_step "5" "Verifying System Status"

# Check available topics
echo "📡 Available topics related to our system:"
ros2 topic list | grep -E "(cmd_vel|joint_states|diff_drive)" || print_warning "Some topics may not be available yet"

# Check controller status
echo ""
echo "🎮 Controller status:"
ros2 control list_controllers 2>/dev/null || print_warning "Controllers may still be loading"

# Check hardware interfaces
echo ""
echo "🔧 Hardware interfaces:"
ros2 control list_hardware_interfaces 2>/dev/null || print_warning "Hardware interfaces may still be loading"

print_step "6" "Sending Test Commands"

# Send a simple forward command
print_info "Sending forward motion command (linear_x = 0.1 m/s)..."
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &

sleep 2

# Send a rotation command  
print_info "Sending rotation command (angular_z = 0.2 rad/s)..."
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}' &

sleep 2

# Send a combined command
print_info "Sending combined motion (forward + turn)..."
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' &

sleep 2

print_step "7" "Monitoring Data Flow"

# Show joint states
echo "📊 Current joint states (showing hardware interface output):"
timeout 3s ros2 topic echo /joint_states --once 2>/dev/null || print_warning "Joint states not available - this is normal in simulation mode with no feedback"

# Show diff drive controller commands
echo ""
echo "🚗 Diff drive controller commands (to individual wheels):"
timeout 3s ros2 topic echo /diff_drive_controller/cmd_vel --once 2>/dev/null || print_warning "Controller commands not available"

print_step "8" "Understanding the Data Flow"

cat << 'EOF'

📈 Data Flow Visualization:

Navigation/Teleop
       ↓ cmd_vel (Twist)
Twist Multiplexer  
       ↓ cmd_vel_out → /diff_drive_controller/cmd_vel
Diff Drive Controller
       ↓ Individual wheel velocities (4 wheels)
ESP32 Hardware Interface
       ↓ Aggregated cmd_vel → /cmd_vel (to ESP32)
ESP32 Firmware
       ↓ /joint_states (from ESP32)
ESP32 Hardware Interface
       ↓ Joint states to ROS 2 Control
Joint State Broadcaster
       ↓ /joint_states (published)
Robot State Publisher → TF transforms

Key Transformation in ESP32HardwareInterface::write():
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Input: 4 individual wheel velocities from DiffDriveController
  • front_left_wheel_joint:  3.66 rad/s
  • front_right_wheel_joint: 4.34 rad/s  
  • rear_left_wheel_joint:   3.66 rad/s
  • rear_right_wheel_joint:  4.34 rad/s

Processing:
  left_avg = (front_left + rear_left) / 2   = 3.66 rad/s
  right_avg = (front_right + rear_right) / 2 = 4.34 rad/s
  
  linear_x = wheel_radius * (left_avg + right_avg) / 2
  angular_z = wheel_radius * (right_avg - left_avg) / wheel_separation

Output: Single cmd_vel to ESP32
  • linear.x:  0.2 m/s
  • angular.z: 0.1 rad/s

EOF

print_step "9" "Testing Different Scenarios"

echo "🧪 Testing edge cases:"

# Test zero command (stop)
print_info "Testing stop command..."
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &
sleep 1

# Test maximum values (if any limits defined)
print_info "Testing high-speed command..."
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}' &
sleep 1

# Test negative values (reverse)
print_info "Testing reverse motion..."
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &
sleep 1

print_step "10" "Performance Analysis"

echo "📊 System performance metrics:"

# Check topic frequencies
echo "Topic update rates:"
echo "  • /joint_states: $(timeout 3s ros2 topic hz /joint_states 2>/dev/null | tail -1 || echo 'Not available')"
echo "  • /cmd_vel: Variable (depends on commands)"

# Check system load
echo ""
echo "System resource usage:"
echo "  • Hardware interface process: $(ps aux | grep ros2_control_node | grep -v grep | awk '{print $3"%"}' | head -1 || echo 'N/A')"

print_step "11" "Cleanup and Summary"

# Stop the hardware interface
print_info "Stopping hardware interface..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# Kill any remaining processes
pkill -f "ros2 launch my_slam_pkg hardware_control" 2>/dev/null || true
pkill -f "ros2_control_node" 2>/dev/null || true

print_info "✓ Cleanup completed"

echo ""
echo "🎯 WORKFLOW DEMONSTRATION COMPLETE"
echo "=================================="
echo ""
print_info "Key Takeaways:"
echo "  1. ESP32HardwareInterface bridges ROS 2 Control and ESP32 firmware"
echo "  2. Converts between 4-wheel individual commands and differential drive"
echo "  3. Handles bidirectional communication via ROS topics"
echo "  4. Supports both simulation and real hardware modes"
echo "  5. Integrates seamlessly with navigation and SLAM systems"
echo ""

print_info "Next Steps:"
echo "  • For real hardware: Set use_fake_hardware:=false and connect ESP32"
echo "  • For navigation: Launch navigation stack with this hardware interface"
echo "  • For SLAM: Use with slam_toolbox for mapping and localization"
echo ""

print_info "Configuration files to modify for your robot:"
echo "  • src/my_slam_pkg/config/hardware_controllers.yaml - wheel parameters"
echo "  • src/my_slam_pkg/urdf/car.urdf.xacro - robot geometry"
echo "  • ESP32 firmware - motor pins and encoder configuration"

echo ""
print_info "Demonstration completed successfully! 🎉"
