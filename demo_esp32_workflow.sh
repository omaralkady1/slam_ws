#!/bin/bash

# ESP32 Hardware Interface - Interactive Workflow Demonstration
# This script demonstrates the complete workflow of your ESP32 Hardware Interface

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to print colored output
print_header() {
    echo -e "\n${BLUE}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC} ${YELLOW}$1${NC} ${BLUE}║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════════════════════════════════════════════╝${NC}\n"
}

print_info() {
    echo -e "${CYAN}ℹ️  $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_step() {
    echo -e "\n${PURPLE}🔸 $1${NC}\n"
}

# Function to wait for user input
wait_for_user() {
    echo -e "\n${YELLOW}Press Enter to continue...${NC}"
    read -r
}

# Function to check if ROS 2 is sourced
check_ros_setup() {
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS 2 is not sourced. Please run: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_success "ROS 2 ($ROS_DISTRO) environment detected"
}

# Function to check workspace
check_workspace() {
    if [ ! -f "install/setup.bash" ]; then
        print_error "Not in ROS 2 workspace or workspace not built. Please run 'colcon build' first."
        exit 1
    fi
    source install/setup.bash
    print_success "Workspace sourced successfully"
}

# Function to demonstrate kinematics calculation
demonstrate_kinematics() {
    print_step "Kinematics Calculation Demonstration"
    
    echo "Let's calculate wheel velocities for a movement command:"
    echo ""
    echo "Robot Parameters:"
    echo "  • Wheel radius: 0.05m"
    echo "  • Wheel separation: 0.34m"
    echo ""
    echo "Input Command:"
    echo "  • Linear velocity: 0.2 m/s (forward)"
    echo "  • Angular velocity: 0.1 rad/s (turn left)"
    echo ""
    
    # Python calculation
    python3 << 'EOF'
import math

# Robot parameters
wheel_radius = 0.05  # meters
wheel_separation = 0.34  # meters

# Input command
linear_x = 0.2  # m/s
angular_z = 0.1  # rad/s

print("Differential Drive Kinematics:")
print("================================")

# Calculate wheel speeds
left_speed = (linear_x - angular_z * wheel_separation/2) / wheel_radius
right_speed = (linear_x + angular_z * wheel_separation/2) / wheel_radius

print(f"Left wheel speed  = ({linear_x} - {angular_z} × {wheel_separation}/2) / {wheel_radius}")
print(f"                  = ({linear_x} - {angular_z * wheel_separation/2:.3f}) / {wheel_radius}")
print(f"                  = {linear_x - angular_z * wheel_separation/2:.3f} / {wheel_radius}")
print(f"                  = {left_speed:.2f} rad/s")
print("")

print(f"Right wheel speed = ({linear_x} + {angular_z} × {wheel_separation}/2) / {wheel_radius}")
print(f"                  = ({linear_x} + {angular_z * wheel_separation/2:.3f}) / {wheel_radius}")
print(f"                  = {linear_x + angular_z * wheel_separation/2:.3f} / {wheel_radius}")
print(f"                  = {right_speed:.2f} rad/s")
print("")

# Verify reverse calculation
calc_linear = wheel_radius * (left_speed + right_speed) / 2
calc_angular = wheel_radius * (right_speed - left_speed) / wheel_separation

print("Verification (reverse calculation):")
print(f"Linear velocity  = {wheel_radius} × ({left_speed:.2f} + {right_speed:.2f}) / 2 = {calc_linear:.3f} m/s ✓")
print(f"Angular velocity = {wheel_radius} × ({right_speed:.2f} - {left_speed:.2f}) / {wheel_separation} = {calc_angular:.3f} rad/s ✓")
EOF
    
    wait_for_user
}

# Function to start hardware interface
start_hardware_interface() {
    print_step "Starting ESP32 Hardware Interface"
    
    print_info "Launching hardware control system in simulation mode..."
    echo "Command: ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=true"
    
    # Start in background
    ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=true &
    LAUNCH_PID=$!
    
    print_info "Waiting for system to initialize..."
    sleep 5
    
    # Check if launch is still running
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        print_error "Hardware interface failed to start"
        return 1
    fi
    
    print_success "Hardware interface started (PID: $LAUNCH_PID)"
    return 0
}

# Function to verify system status
verify_system() {
    print_step "Verifying System Status"
    
    print_info "Checking active controllers..."
    if ros2 control list_controllers 2>/dev/null | grep -q "active"; then
        print_success "Controllers are active:"
        ros2 control list_controllers | grep active
    else
        print_warning "No active controllers found"
        return 1
    fi
    
    echo ""
    print_info "Checking available topics..."
    if ros2 topic list | grep -q "/cmd_vel\|/joint_states"; then
        print_success "Key topics available:"
        ros2 topic list | grep -E "(cmd_vel|joint_states)" | sed 's/^/  • /'
    else
        print_warning "Key topics not found"
        return 1
    fi
    
    echo ""
    print_info "Checking hardware interfaces..."
    if ros2 control list_hardware_interfaces 2>/dev/null | grep -q "available"; then
        print_success "Hardware interfaces available:"
        ros2 control list_hardware_interfaces | grep available | head -4 | sed 's/^/  • /'
    else
        print_warning "No hardware interfaces found"
        return 1
    fi
    
    wait_for_user
}

# Function to demonstrate command flow
demonstrate_command_flow() {
    print_step "Live Command Flow Demonstration"
    
    print_info "Starting topic monitoring in background..."
    
    # Monitor joint states
    echo "Monitoring /joint_states for 3 samples..." > /tmp/joint_states_monitor.log
    timeout 10s ros2 topic echo /joint_states --max-count 3 >> /tmp/joint_states_monitor.log 2>&1 &
    
    # Monitor cmd_vel
    echo "Monitoring /cmd_vel for commands..." > /tmp/cmd_vel_monitor.log
    timeout 10s ros2 topic echo /cmd_vel >> /tmp/cmd_vel_monitor.log 2>&1 &
    
    sleep 2
    
    print_info "Sending test command: Forward 0.2 m/s + Turn left 0.1 rad/s"
    echo "ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.1}}'"
    
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
    
    sleep 3
    
    print_success "Command sent! Let's see the results..."
    
    echo ""
    print_info "Command that was sent:"
    echo "  Linear X:  0.2 m/s"
    echo "  Angular Z: 0.1 rad/s"
    
    echo ""
    print_info "Expected wheel velocities (calculated above):"
    echo "  Left wheels:  3.66 rad/s"
    echo "  Right wheels: 4.34 rad/s"
    
    # Show monitoring results
    sleep 2
    if [ -s /tmp/joint_states_monitor.log ]; then
        echo ""
        print_info "Joint states feedback from hardware interface:"
        tail -20 /tmp/joint_states_monitor.log | grep -A 2 "velocity:" || echo "  (Simulated - no real encoder feedback)"
    fi
    
    wait_for_user
}

# Function to demonstrate different movement patterns
demonstrate_movements() {
    print_step "Testing Different Movement Patterns"
    
    movements=(
        "Forward only:0.3:0.0:Move straight forward"
        "Backward only:-0.2:0.0:Move straight backward"
        "Turn left only:0.0:0.5:Rotate in place (left)"
        "Turn right only:0.0:-0.5:Rotate in place (right)"
        "Forward + right:0.2:-0.2:Arc movement (forward + right turn)"
        "Complex motion:0.15:0.3:Forward + sharp left turn"
    )
    
    for movement in "${movements[@]}"; do
        IFS=':' read -r name linear angular description <<< "$movement"
        
        print_info "Test: $description"
        echo "  Command: linear_x=$linear, angular_z=$angular"
        
        # Calculate expected wheel speeds
        python3 << EOF
wheel_radius = 0.05
wheel_separation = 0.34 
linear_x = $linear
angular_z = $angular

left_speed = (linear_x - angular_z * wheel_separation/2) / wheel_radius
right_speed = (linear_x + angular_z * wheel_separation/2) / wheel_radius

print(f"  Expected: Left={left_speed:.2f} rad/s, Right={right_speed:.2f} rad/s")
EOF
        
        # Send command
        ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: $linear}, angular: {z: $angular}}" > /dev/null
        
        echo "  ✓ Command sent"
        sleep 1.5
        echo ""
    done
    
    print_success "Movement pattern testing completed"
    wait_for_user
}

# Function to show system monitoring
show_monitoring() {
    print_step "System Monitoring and Diagnostics"
    
    print_info "Real-time system status:"
    
    echo ""
    echo "📊 Controller Status:"
    ros2 control list_controllers 2>/dev/null || echo "  Controllers not available"
    
    echo ""
    echo "📈 Topic Activity:"
    echo "  /cmd_vel hz:" 
    timeout 3s ros2 topic hz /cmd_vel 2>/dev/null | head -1 | sed 's/^/    /' || echo "    No activity"
    echo "  /joint_states hz:"
    timeout 3s ros2 topic hz /joint_states 2>/dev/null | head -1 | sed 's/^/    /' || echo "    No activity"
    
    echo ""
    echo "🔧 Hardware Interface Status:"
    ros2 control list_hardware_interfaces 2>/dev/null | grep -E "(front_left|front_right)" | head -2 | sed 's/^/  /'
    
    echo ""
    print_info "Performance tips:"
    echo "  • Monitor /rosout for error messages"
    echo "  • Check topic frequencies with 'ros2 topic hz'"
    echo "  • Use 'ros2 topic echo' to inspect message content"
    echo "  • Controller manager provides service interfaces for control"
    
    wait_for_user
}

# Function to cleanup
cleanup() {
    print_step "Cleanup"
    
    print_info "Stopping all ROS 2 processes..."
    
    # Stop any running launches
    if [ ! -z "$LAUNCH_PID" ] && kill -0 $LAUNCH_PID 2>/dev/null; then
        print_info "Stopping hardware interface (PID: $LAUNCH_PID)"
        kill $LAUNCH_PID
        sleep 2
    fi
    
    # Kill any remaining ROS 2 processes
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "controller_manager" 2>/dev/null || true
    
    # Clean up temp files
    rm -f /tmp/joint_states_monitor.log /tmp/cmd_vel_monitor.log
    
    print_success "Cleanup completed"
}

# Main demonstration function
main() {
    print_header "ESP32 Hardware Interface - Complete Workflow Demonstration"
    
    print_info "This demonstration will show you:"
    echo "  1. 🧮 Kinematics calculations with real numbers"
    echo "  2. 🚀 System startup and verification"  
    echo "  3. 📡 Live command flow monitoring"
    echo "  4. 🎮 Different movement patterns"
    echo "  5. 📊 System monitoring and diagnostics"
    echo ""
    print_info "Duration: ~5-10 minutes depending on your pace"
    
    wait_for_user
    
    # Setup trap for cleanup
    trap cleanup EXIT
    
    # Check prerequisites
    print_header "System Prerequisites Check"
    check_ros_setup
    check_workspace
    print_success "All prerequisites met!"
    
    wait_for_user
    
    # Demonstrate kinematics
    print_header "1. Differential Drive Kinematics"
    demonstrate_kinematics
    
    # Start system
    print_header "2. System Startup"
    if ! start_hardware_interface; then
        print_error "Failed to start hardware interface. Check your installation."
        exit 1
    fi
    
    # Verify system
    print_header "3. System Verification"
    if ! verify_system; then
        print_warning "System verification had issues, but continuing..."
        sleep 2
    fi
    
    # Demonstrate command flow
    print_header "4. Live Command Flow"
    demonstrate_command_flow
    
    # Test different movements
    print_header "5. Movement Pattern Testing"
    demonstrate_movements
    
    # Show monitoring
    print_header "6. System Monitoring"
    show_monitoring
    
    # Summary
    print_header "🎉 Demonstration Complete!"
    
    print_success "You've seen how the ESP32 Hardware Interface:"
    echo "  1. ✅ Converts high-level cmd_vel to individual wheel commands"
    echo "  2. ✅ Handles differential drive kinematics automatically"
    echo "  3. ✅ Provides bidirectional communication with ESP32"
    echo "  4. ✅ Integrates seamlessly with ROS 2 Control framework"
    echo "  5. ✅ Supports both simulation and real hardware modes"
    echo ""
    
    print_info "Next Steps:"
    echo "  • For real hardware: Set use_fake_hardware:=false and connect ESP32"
    echo "  • For navigation: Launch nav2 with this hardware interface"
    echo "  • For SLAM: Use with slam_toolbox or other SLAM packages"
    echo ""
    
    print_info "Key files to examine:"
    echo "  • ESP32_COMPLETE_EXAMPLE.md - Detailed workflow documentation"
    echo "  • src/my_slam_pkg/hardware_interface/ - Hardware interface source code"
    echo "  • src/my_slam_pkg/config/hardware_controllers.yaml - Controller config"
    
    echo ""
    print_success "Demonstration completed successfully! 🚀"
}

# Run the demonstration
main "$@"
