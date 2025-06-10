#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('my_slam_pkg')
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_hardware',
            default_value='false',
            description='Use simulated hardware instead of real ESP32',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'esp32_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ESP32 connection',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for RPLidar connection',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Enable SLAM mapping',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='false',
            description='Enable Nav2 navigation (requires existing map)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ekf_config_file',
            default_value=os.path.join(pkg_share, 'config', 'esp32_ekf_params.yaml'),
            description='EKF parameters file',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_config_file',
            default_value=os.path.join(pkg_share, 'config', 'esp32_ros2_control_ekf.yaml'),
            description='ros2_control parameters file for EKF integration',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_share, 'config', 'esp32_ekf_slam.rviz'),
            description='RViz configuration file',
        )
    )
    
    # Get arguments
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_navigation = LaunchConfiguration('enable_navigation')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    controllers_config_file = LaunchConfiguration('controllers_config_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # === CORE ROBOT HARDWARE WITH ros2_control ===
    # Robot State Publisher
    robot_description_content = open(os.path.join(pkg_share, 'urdf', 'esp32_complete_robot.urdf')).read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )
    
    # Controller Manager with ESP32 hardware interface
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_config_file,
            {'use_sim_time': False}
        ],
        output='screen',
    )
    
    # micro-ROS agent for ESP32 communication
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', esp32_port, '--baud', '115200'],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Differential Drive Controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # RPLidar A1 driver
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )
    
    # === SENSOR FUSION WITH EKF ===
    # Extended Kalman Filter for fusing wheel odometry + IMU
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': False}
        ],
        remappings=[
            ('/odometry/filtered', '/odom_filtered'),  # EKF output
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # Static transforms for sensor mounting
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0.0', '0.0', '0.15', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': False}]
    )
    
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0.0', '0.0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': False}]
    )
    
    static_tf_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': False}]
    )
    
    # === SLAM TOOLBOX (Uses EKF filtered odometry) ===
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            ekf_config_file,  # Contains both EKF and SLAM config
            {'use_sim_time': False}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/map', '/map')
        ],
        condition=LaunchConfiguration('enable_slam')
    )
    
    # === TWIST MULTIPLEXER FOR SAFE CONTROL ===
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel')  # Output to diff_drive_controller
        ],
        output='screen',
    )
    
    # === NAVIGATION STACK (Optional) ===
    # Map server (for navigation mode)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': os.path.join(pkg_share, 'maps', 'my_map.yaml')
        }],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    # AMCL for localization (uses EKF filtered odometry)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'esp32_nav2_params.yaml'),
            {'use_sim_time': False}
        ],
        remappings=[
            ('/initialpose', '/initialpose'),
            ('/odom', '/odom_filtered')  # Use EKF filtered odometry
        ],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'esp32_nav2_params.yaml'),
            {'use_sim_time': False}
        ],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'esp32_nav2_params.yaml'),
            {'use_sim_time': False}
        ],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'esp32_nav2_params.yaml'),
            {'use_sim_time': False}
        ],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'esp32_nav2_params.yaml'),
            {'use_sim_time': False}
        ],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    # Lifecycle managers
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server', 
                           'behavior_server', 'bt_navigator']
        }],
        condition=LaunchConfiguration('enable_navigation')
    )
    
    # === VISUALIZATION AND CONTROL ===
    # RViz for visualization with EKF topics
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # Teleop for manual control
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_keyboard')],  # Goes to twist_mux
        parameters=[{'use_sim_time': False}]
    )
    
    # === DIAGNOSTIC AND MONITORING NODES ===
    # System diagnostics
    system_diagnostics = Node(
        package='my_slam_pkg',
        executable='slam_diagnostics.py',
        name='system_diagnostics',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # TF monitor
    tf_monitor = Node(
        package='my_slam_pkg',
        executable='tf_monitor.py',
        name='tf_monitor',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # EKF diagnostics
    ekf_diagnostics = Node(
        package='my_slam_pkg',
        executable='ekf_diagnostics.py',
        name='ekf_diagnostics',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # === LAUNCH SEQUENCE WITH DELAYS ===
    # Start core hardware first
    hardware_startup_info = LogInfo(
        msg="Starting ESP32 4-Motor Robot with EKF Sensor Fusion..."
    )
    
    hardware_config_info = LogInfo(
        msg=["ESP32 Port: ", esp32_port, ", LiDAR Port: ", lidar_port]
    )
    
    # Delayed controller spawning (wait for controller manager)
    delayed_joint_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_diff_drive = TimerAction(
        period=5.0,
        actions=[diff_drive_controller_spawner]
    )
    
    # Delayed EKF (wait for controllers and sensors)
    delayed_ekf = TimerAction(
        period=7.0,
        actions=[ekf_filter_node]
    )
    
    # Delayed SLAM (wait for EKF to stabilize)
    delayed_slam = TimerAction(
        period=10.0,
        actions=[slam_toolbox_node]
    )
    
    # Delayed navigation (wait for SLAM to initialize)
    delayed_navigation = TimerAction(
        period=15.0,
        actions=[
            map_server,
            amcl,
            lifecycle_manager_localization,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_navigation
        ]
    )
    
    # Delayed visualization
    delayed_rviz = TimerAction(
        period=12.0,
        actions=[rviz_node]
    )
    
    # Delayed teleop
    delayed_teleop = TimerAction(
        period=8.0,
        actions=[teleop_keyboard]
    )
    
    # Delayed diagnostics
    delayed_diagnostics = TimerAction(
        period=5.0,
        actions=[system_diagnostics, tf_monitor, ekf_diagnostics]
    )
    
    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add info messages
    ld.add_action(hardware_startup_info)
    ld.add_action(hardware_config_info)
    
    # Add immediate hardware nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(micro_ros_agent)
    ld.add_action(rplidar_node)
    ld.add_action(static_tf_base_to_lidar)
    ld.add_action(static_tf_base_to_imu)
    ld.add_action(static_tf_footprint_to_base)
    ld.add_action(twist_mux)
    
    # Add delayed components
    ld.add_action(delayed_joint_broadcaster)
    ld.add_action(delayed_diff_drive)
    ld.add_action(delayed_ekf)
    ld.add_action(delayed_slam)
    ld.add_action(delayed_navigation)
    ld.add_action(delayed_rviz)
    ld.add_action(delayed_teleop)
    ld.add_action(delayed_diagnostics)
    
    return ld