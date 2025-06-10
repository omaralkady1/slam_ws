# ESP32 Hardware Interface - Complete Workflow Example

## 📋 Overview
This document demonstrates the complete workflow of the ESP32 Hardware Interface, showing real examples of how commands flow from ROS 2 Navigation to the ESP32 hardware and back.

## 🏗️ Architecture Overview

```
┌─────────────────┐    ┌────────────────────┐    ┌──────────────────┐    ┌─────────────┐
│   Navigation    │    │   ROS 2 Control    │    │  ESP32 Hardware  │    │    ESP32    │
│     Stack       │    │    Framework       │    │    Interface     │    │  Firmware   │
│                 │    │                    │    │                  │    │             │
│ - Nav2          │───▶│ - Controller Mgr   │───▶│ - Bridge Layer   │───▶│ - micro-ROS │
│ - SLAM          │    │ - Diff Drive Ctrl  │    │ - Kinematics     │    │ - Motors    │
│ - Planner       │    │ - Joint State BC   │    │ - ROS Topics     │    │ - Encoders  │
└─────────────────┘    └────────────────────┘    └──────────────────┘    └─────────────┘
```

## 🔄 Complete Data Flow

### **Commands Flow (ROS 2 → ESP32)**
```
1. Navigation Goal          → cmd_vel (geometry_msgs/Twist)
2. Twist Multiplexer       → /diff_drive_controller/cmd_vel  
3. Diff Drive Controller   → Individual wheel velocities
4. ESP32 Hardware Interface → Aggregated /cmd_vel to ESP32
5. ESP32 Firmware          → Motor PWM signals
```

### **Feedback Flow (ESP32 → ROS 2)**
```
1. ESP32 Encoders          → Read wheel positions
2. ESP32 Firmware          → /joint_states (sensor_msgs/JointState)
3. ESP32 Hardware Interface → Update ROS 2 Control state
4. Joint State Broadcaster → /joint_states topic
5. Robot State Publisher   → TF transforms
6. Navigation Stack        → Odometry for localization
```

---

## 🎯 Practical Example Walkthrough

### **Step 1: System Initialization**

#### **Launch the Hardware Interface**
```bash
# Terminal 1: Start the hardware control system
cd /home/alkady/slam_ws
source install/setup.bash

# For real ESP32 hardware:
ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=false

# For simulation/testing:
ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=true
```

#### **What Happens During Initialization:**

1. **URDF Processing**: Robot description loaded with 4 wheel joints
   ```
   Joint 0: front_left_wheel_joint
   Joint 1: front_right_wheel_joint  
   Joint 2: rear_left_wheel_joint
   Joint 3: rear_right_wheel_joint
   ```

2. **Hardware Interface Configuration**:
   ```cpp
   // From your config file
   wheel_radius: 0.05m
   wheel_separation: 0.34m
   cmd_vel_topic: "/cmd_vel"
   joint_states_topic: "/joint_states"
   ```

3. **Controller Manager Startup**:
   - Loads ESP32HardwareInterface plugin
   - Spawns diff_drive_controller
   - Spawns joint_state_broadcaster

---

### **Step 2: Real Command Example**

#### **Send a Movement Command**
```bash
# Terminal 2: Send a movement command
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

#### **Detailed Flow Trace:**

**🔹 Input Processing:**
```
Input: linear_x = 0.2 m/s, angular_z = 0.1 rad/s
```

**🔹 Diff Drive Controller Calculations:**
```cpp
// Inside diff_drive_controller
left_wheel_speed = (linear_x - angular_z * wheel_separation/2) / wheel_radius
right_wheel_speed = (linear_x + angular_z * wheel_separation/2) / wheel_radius

// With your values:
left_wheel_speed = (0.2 - 0.1 * 0.34/2) / 0.05 = 3.66 rad/s
right_wheel_speed = (0.2 + 0.1 * 0.34/2) / 0.05 = 4.34 rad/s
```

**🔹 Hardware Interface Processing:**
```cpp
// In ESP32HardwareInterface::write()

// Commands received for each wheel:
front_left_wheel_joint:  3.66 rad/s
rear_left_wheel_joint:   3.66 rad/s  
front_right_wheel_joint: 4.34 rad/s
rear_right_wheel_joint:  4.34 rad/s

// Convert back to robot velocities:
left_avg = (3.66 + 3.66) / 2 = 3.66 rad/s
right_avg = (4.34 + 4.34) / 2 = 4.34 rad/s

// Final output to ESP32:
linear_x = 0.05 * (3.66 + 4.34) / 2 = 0.2 m/s  ✓
angular_z = 0.05 * (4.34 - 3.66) / 0.34 = 0.1 rad/s  ✓
```

**🔹 ESP32 Receives:**
```json
{
  "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
}
```

---

### **Step 3: Feedback Processing**

#### **ESP32 Sends Joint States**
```cpp
// ESP32 firmware publishes:
sensor_msgs::msg::JointState joint_state;
joint_state.name = ["front_left_wheel_joint", "front_right_wheel_joint", 
                    "rear_left_wheel_joint", "rear_right_wheel_joint"];
joint_state.position = [1.2, 1.4, 1.2, 1.4];  // rad
joint_state.velocity = [3.6, 4.3, 3.6, 4.3];  // rad/s
```

#### **Hardware Interface Updates State**
```cpp
// In ESP32HardwareInterface::read()
hw_positions_[0] = 1.2;  // front_left
hw_positions_[1] = 1.4;  // front_right
hw_positions_[2] = 1.2;  // rear_left  
hw_positions_[3] = 1.4;  // rear_right

hw_velocities_[0] = 3.6; // front_left
hw_velocities_[1] = 4.3; // front_right
hw_velocities_[2] = 3.6; // rear_left
hw_velocities_[3] = 4.3; // rear_right
```

#### **ROS 2 Control Publishes Updates**
```bash
# Check the published joint states
ros2 topic echo /joint_states
```

Output:
```yaml
header:
  stamp: {sec: 1704905523, nanosec: 123456789}
  frame_id: ''
name: 
  - front_left_wheel_joint
  - front_right_wheel_joint
  - rear_left_wheel_joint
  - rear_right_wheel_joint
position: [1.2, 1.4, 1.2, 1.4]
velocity: [3.6, 4.3, 3.6, 4.3]
effort: [0.0, 0.0, 0.0, 0.0]
```

---

## 🔧 **Debugging Commands**

### **Monitor the Hardware Interface**
```bash
# Check if hardware interface is loaded
ros2 control list_hardware_interfaces

# Check controller status
ros2 control list_controllers

# Monitor command flow
ros2 topic echo /diff_drive_controller/cmd_vel

# Monitor joint commands (internal)
ros2 topic echo /joint_states

# Check TF transforms
ros2 run tf2_tools view_frames
```

### **Manual Testing**
```bash
# Test individual wheel movement
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{start_controllers: ['diff_drive_controller'], 
    stop_controllers: [], 
    strictness: 1}"

# Test rotation only
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: 0.5}}' --once

# Test forward motion only  
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}}' --once
```

---

## 📊 **Performance Monitoring**

### **Key Metrics to Watch**
```bash
# Update rates
ros2 topic hz /joint_states          # Should be ~50 Hz
ros2 topic hz /cmd_vel              # Variable based on commands
ros2 topic hz /diff_drive_controller/cmd_vel

# Latency check
ros2 topic delay /joint_states       # Should be < 20ms
```

### **Common Issues and Solutions**

| Issue | Symptom | Solution |
|-------|---------|----------|
| No joint states | `joint_state_received_ = false` | Check ESP32 micro-ROS connection |
| Wrong wheel direction | Robot turns opposite | Check wheel joint mapping in URDF |
| Jerky movement | Irregular cmd_vel | Adjust controller update rate |
| Poor rotation | Circular drift | Tune `wheel_separation_multiplier` |

---

## 🚀 **Complete Startup Sequence**

### **Real Hardware Workflow**
```bash
# 1. Start micro-ROS agent (if using serial)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# 2. Power on ESP32 and verify connection
# Check that ESP32 publishes /joint_states and subscribes to /cmd_vel

# 3. Launch hardware interface
ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=false

# 4. Verify system is working
ros2 topic list | grep -E "(joint_states|cmd_vel)"
ros2 control list_hardware_interfaces

# 5. Test movement
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### **Simulation Workflow**
```bash
# 1. Launch in simulation mode
ros2 launch my_slam_pkg hardware_control.launch.py use_fake_hardware:=true

# 2. Test with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

# 3. Visualize in RViz
rviz2 -d $(ros2 pkg prefix my_slam_pkg)/share/my_slam_pkg/rviz/robot_view.rviz
```

---

## 🎛️ **Configuration Examples**

### **For Different Robot Configurations**

#### **Small Robot (0.1m wheels, 0.2m separation)**
```yaml
# hardware_controllers.yaml
wheel_radius: 0.05  # 5cm wheels
wheel_separation: 0.2  # 20cm between wheels  
wheel_separation_multiplier: 1.0  # Start with 1.0, tune if needed
```

#### **Large Robot (0.15m wheels, 0.5m separation)**
```yaml
wheel_radius: 0.075  # 7.5cm wheels
wheel_separation: 0.5  # 50cm between wheels
wheel_separation_multiplier: 1.1  # May need tuning
```

### **Topic Remapping Examples**
```xml
<!-- For multiple robots -->
<remap from="/cmd_vel" to="/robot1/cmd_vel"/>
<remap from="/joint_states" to="/robot1/joint_states"/>
```

This workflow shows exactly how your ESP32 Hardware Interface acts as the critical bridge between ROS 2's high-level navigation and your ESP32's low-level motor control, handling all the complex kinematics and communication protocols seamlessly.
