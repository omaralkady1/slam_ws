/home/alkady/Arduino/sketch_may6a/sketch_may6a.ino: In function 'void updateAndPublish()':
/home/alkady/Arduino/sketch_may6a/sketch_may6a.ino:483:19: error: 'velocity_history' was not declared in this scope
  483 |                   velocity_history[0][0], velocity_history[1][0], velocity_history[2][0], velocity_history[3][0]);
      |                   ^~~~~~~~~~~~~~~~
exit status 1

Compilation error: 'velocity_history' was not declared in this scope#!/bin/bash

# ESP32 SLAM Encoder Test Script
# This script helps verify encoder accuracy and rotation for SLAM

echo "=== ESP32 SLAM Encoder Test ==="
echo "This script will help you verify encoder accuracy for SLAM"
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

# Function to run a test command
run_test() {
    local test_name="$1"
    local cmd="$2"
    local duration="$3"
    
    echo "=== $test_name ==="
    echo "Publishing: $cmd"
    echo "Duration: ${duration}s"
    echo "Watch the robot and RViz odometry..."
    echo ""
    
    # Run the command in background
    timeout ${duration}s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "$cmd" &
    local pid=$!
    
    # Wait for the test to complete
    wait $pid 2>/dev/null
    
    echo "Test complete. Press Enter to continue..."
    read -r
    echo ""
}

echo "Make sure your ESP32 is connected and ros2_control is running!"
echo "Also open RViz to monitor the odometry visualization"
echo "Press Enter when ready..."
read -r

# Test 1: Forward movement
run_test "Forward Movement Test" \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    5

# Test 2: Backward movement  
run_test "Backward Movement Test" \
    "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    5

# Test 3: Clockwise rotation
run_test "Clockwise Rotation Test" \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}" \
    6

# Test 4: Counter-clockwise rotation
run_test "Counter-Clockwise Rotation Test" \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
    6

# Test 5: Combined movement (arc)
run_test "Arc Movement Test" \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" \
    8

echo "=== Test Complete ==="
echo ""
echo "Encoder Calibration Tips:"
echo "1. If the robot doesn't move straight during forward/backward tests:"
echo "   - Adjust left_wheel_radius_multiplier or right_wheel_radius_multiplier"
echo "   - Start with small increments (±0.02)"
echo ""
echo "2. If rotation rate is wrong:"
echo "   - Adjust wheel_separation_multiplier"
echo "   - Increase if robot rotates too slowly, decrease if too fast"
echo ""
echo "3. If encoder directions are wrong:"
echo "   - Uncomment calibrateEncoderDirections() in ESP32 setup()"
echo "   - Update encoder_directions[] array based on calibration results"
echo ""
echo "4. Check RViz /odom topic to verify odometry matches actual movement"
echo ""
echo "Edit /home/alkady/slam_ws/src/my_slam_pkg/config/esp32_controllers.yaml to adjust parameters"
