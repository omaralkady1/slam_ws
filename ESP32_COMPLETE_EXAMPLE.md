# ESP32 Hardware Interface - Complete Practical Example

## 🎯 What This Example Demonstrates

This example shows **exactly** how your ESP32 Hardware Interface works by walking through:
1. **Real command flow** with actual numbers
2. **Bidirectional communication** between ROS 2 and ESP32
3. **Kinematics transformations** step-by-step
4. **Live monitoring** and debugging techniques
5. **Complete system integration** from navigation to motors

---

## 🚀 Quick Start Demo

### **Step 1: Launch the System**
```bash
# Terminal 1: Start the hardware interface
cd /home/alkady/slam_ws
source install/setup.bash

# For testing with simulation:
ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=true

# For real ESP32 hardware:
ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=false
```

### **Step 2: Verify System is Running**
```bash
# Terminal 2: Check controllers
ros2 control list_controllers

# Expected output:
# diff_drive_controller[diff_drive_controller/DiffDriveController] active    
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### **Step 3: Send a Test Command**
```bash
# Send forward + turn command
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

---

## 🔍 Complete Data Flow Analysis

### **Input Command Analysis**
When you send this command:
```json
{
  "linear": {"x": 0.2},    // Move forward 0.2 m/s
  "angular": {"z": 0.1}    // Turn left 0.1 rad/s
}
```

### **Step-by-Step Transformation**

#### **1. Diff Drive Controller Processing**
```cpp
// Using your robot parameters:
// wheel_radius = 0.05m, wheel_separation = 0.34m

// Calculate individual wheel speeds:
left_speed = (linear_x - angular_z * wheel_separation/2) / wheel_radius
right_speed = (linear_x + angular_z * wheel_separation/2) / wheel_radius

// With your values:
left_speed = (0.2 - 0.1 * 0.34/2) / 0.05 = 3.66 rad/s
right_speed = (0.2 + 0.1 * 0.34/2) / 0.05 = 4.34 rad/s
```

#### **2. Hardware Interface Receives Commands**
```cpp
// ESP32HardwareInterface::write() receives:
hw_commands_[0] = 3.66;  // front_left_wheel_joint
hw_commands_[1] = 4.34;  // front_right_wheel_joint  
hw_commands_[2] = 3.66;  // rear_left_wheel_joint
hw_commands_[3] = 4.34;  // rear_right_wheel_joint
```

#### **3. Convert Back to Robot Velocities**
```cpp
// Average left and right wheels (4WD differential drive)
double left_wheel_vel = (hw_commands_[0] + hw_commands_[2]) / 2.0;   // 3.66
double right_wheel_vel = (hw_commands_[1] + hw_commands_[3]) / 2.0;  // 4.34

// Convert to robot motion
double linear_x = wheel_radius_ * (left_wheel_vel + right_wheel_vel) / 2.0;
double angular_z = wheel_radius_ * (right_wheel_vel - left_wheel_vel) / wheel_separation_;

// Results:
linear_x = 0.05 * (3.66 + 4.34) / 2 = 0.2 m/s   ✓ Matches input
angular_z = 0.05 * (4.34 - 3.66) / 0.34 = 0.1 rad/s   ✓ Matches input
```

#### **4. Published to ESP32**
```json
// Topic: /cmd_vel (to ESP32)
{
  "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
}
```

---

## 📊 Live Monitoring Example

### **Monitor Command Flow**
```bash
# Terminal 3: Watch commands to ESP32
ros2 topic echo /cmd_vel

# Terminal 4: Watch feedback from ESP32  
ros2 topic echo /joint_states

# Terminal 5: Monitor controller internals
ros2 topic echo /diff_drive_controller/cmd_vel
```

### **Expected Output Sequence**

**1. Input Command:**
```yaml
# /cmd_vel (input)
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1
```

**2. Controller Processing:**
```yaml
# /diff_drive_controller/cmd_vel (internal)
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1
```

**3. ESP32 Feedback:**
```yaml
# /joint_states (from ESP32)
header:
  stamp: {sec: 1704905523, nanosec: 123456789}
  frame_id: ''
name: 
  - front_left_wheel_joint
  - front_right_wheel_joint
  - rear_left_wheel_joint
  - rear_right_wheel_joint
position: [1.234, 1.456, 1.234, 1.456]  # Accumulated wheel rotations
velocity: [3.66, 4.34, 3.66, 4.34]      # Current wheel speeds
effort: [0.0, 0.0, 0.0, 0.0]            # Motor currents (if available)
```

---

## 🔧 Real Hardware Integration

### **ESP32 Firmware Requirements**
Your ESP32 must publish and subscribe to these topics:

```cpp
// ESP32 micro-ROS setup
// Publisher: /joint_states
sensor_msgs__msg__JointState joint_state_msg;

// Subscriber: /cmd_vel  
geometry_msgs__msg__Twist cmd_vel_msg;

// Example ESP32 callback:
void cmd_vel_callback(const void * msgin) {
    geometry_msgs__msg__Twist * cmd_vel = (geometry_msgs__msg__Twist *)msgin;
    
    // Convert to motor commands
    float linear_x = cmd_vel->linear.x;
    float angular_z = cmd_vel->angular.z;
    
    // Your differential drive logic here
    float left_speed = linear_x - angular_z * WHEEL_SEPARATION / 2.0;
    float right_speed = linear_x + angular_z * WHEEL_SEPARATION / 2.0;
    
    // Send to motors
    setMotorSpeeds(left_speed, right_speed);
}
```

### **Hardware Interface Configuration**
```yaml
# config/hardware_controllers.yaml
hardware:
  - name: esp32_robot
    type: my_slam_pkg/ESP32HardwareInterface
    parameters:
      use_sim_hardware: false           # Set to true for testing
      wheel_radius: 0.05               # Your wheel radius in meters
      wheel_separation: 0.34           # Distance between wheel centers
      cmd_vel_topic: "/cmd_vel"        # Topic to send commands to ESP32
      joint_states_topic: "/joint_states"  # Topic to receive feedback from ESP32
```

---

## 🧪 Testing Scenarios

### **1. Forward Motion Test**
```bash
# Move forward only
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'

# Expected wheel velocities: All wheels same speed
# left_speed = right_speed = 0.3 / 0.05 = 6.0 rad/s
```

### **2. Pure Rotation Test**
```bash
# Rotate in place
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'

# Expected wheel velocities: Left negative, right positive
# left_speed = -0.5 * 0.34 / 2 / 0.05 = -1.7 rad/s
# right_speed = +0.5 * 0.34 / 2 / 0.05 = +1.7 rad/s
```

### **3. Complex Motion Test**
```bash
# Forward + turn right
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: -0.2}}'

# Expected: Left wheels faster than right wheels
```

---

## 🔍 Debugging Tools

### **Check Hardware Interface Status**
```bash
# List all hardware interfaces
ros2 control list_hardware_interfaces

# Expected output:
# esp32_robot/front_left_wheel_joint/velocity [available] [claimed]
# esp32_robot/front_right_wheel_joint/velocity [available] [claimed]
# ...
```

### **Monitor Performance**
```bash
# Check controller manager status
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers

# Check for errors in logs
ros2 topic echo /rosout | grep -i error
```

### **Manual Controller Testing**
```bash
# Stop current controllers
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{stop_controllers: ['diff_drive_controller']}"

# Start controllers
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{start_controllers: ['diff_drive_controller']}"
```

---

## 📈 Advanced Usage

### **Integration with Navigation**
```bash
# 1. Start hardware interface
ros2 launch my_slam_pkg hardware_control.launch.py

# 2. Start navigation
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/path/to/nav2_params.yaml

# 3. Set navigation goal
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}}}'
```

### **SLAM Integration**
```bash
# Start SLAM with hardware interface
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false

# Your ESP32 hardware interface provides odometry through joint_states
# which gets converted to /odom by the diff_drive_controller
```

---

## 🎛️ Configuration for Different Robots

### **Small Robot (10cm wheels, 20cm separation)**
```yaml
wheel_radius: 0.05
wheel_separation: 0.20
```

### **Large Robot (15cm wheels, 50cm separation)**  
```yaml
wheel_radius: 0.075
wheel_separation: 0.50
```

### **Mecanum Wheels Setup**
For mecanum wheels, you'd modify the hardware interface to handle X/Y/rotation independently:
```cpp
// In write() function, convert cmd_vel to individual wheel velocities
// using mecanum kinematics instead of differential drive
```

---

## 🚨 Common Issues & Solutions

### **Problem: No joint_states received**
```bash
# Check ESP32 connection
ros2 topic list | grep joint_states
ros2 topic hz /joint_states

# Solution: Verify micro-ROS agent and ESP32 firmware
```

### **Problem: Robot moves incorrectly**
```bash
# Check wheel parameters
ros2 param get /diff_drive_controller wheel_radius
ros2 param get /diff_drive_controller wheel_separation

# Solution: Measure and update actual robot dimensions
```

### **Problem: Controllers not starting**
```bash
# Check hardware interface status
ros2 control list_hardware_interfaces

# Solution: Verify URDF joint names match hardware interface
```

---

## 🎉 Success Indicators

When everything works correctly, you should see:

1. ✅ **Controllers active**: `ros2 control list_controllers` shows active controllers
2. ✅ **Bidirectional communication**: `/joint_states` updating from ESP32
3. ✅ **Command flow**: `/cmd_vel` commands reach ESP32  
4. ✅ **Correct kinematics**: Robot moves as expected
5. ✅ **Stable feedback**: Joint states update consistently

---

This example demonstrates how your ESP32 Hardware Interface seamlessly bridges the gap between ROS 2's sophisticated control framework and your ESP32's real-time motor control, handling all the complex transformations and communications automatically.
