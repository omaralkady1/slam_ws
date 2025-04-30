# ROS 2 SLAM Workspace

A ROS 2 workspace for Simultaneous Localization and Mapping (SLAM).

## Features

- SLAM navigation in simulated environments
- Map server for saved maps
- Navigation stack integration
- Hospital environment simulation

## Requirements

- ROS 2 
- Navigation2
- SLAM Toolbox
- Gazebo

## Usage

### Building the workspace

```bash
cd ~/slam_ws
colcon build
source install/setup.bash
```

### Running SLAM

```bash
ros2 launch my_slam_pkg slam_complete.launch.py
```

### Running Navigation

```bash
ros2 launch my_slam_pkg navigation.launch.py map:=my_map.yaml
```