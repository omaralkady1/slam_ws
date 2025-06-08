from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
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
            'esp32_baudrate',
            default_value='115200',
            description='Baudrate for ESP32 serial connection',
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
            'enable_lidar',
            default_value='true',
            description='Enable RPLidar A1',
        )
    )
    
    # Get package directory
    pkg_share = FindPackageShare('my_slam_pkg').find('my_slam_pkg')
    
    # Get arguments
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')
    esp32_port = LaunchConfiguration('esp32_port')
    esp32_baudrate = LaunchConfiguration('esp32_baudrate')
    lidar_port = LaunchConfiguration('lidar_port')
    enable_lidar = LaunchConfiguration('enable_lidar')
    
    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'urdf', 'car.urdf.xacro'), ' ',
        'use_fake_hardware:=', use_sim_hardware
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Choose appropriate controller configuration
    controller_manager_config = os.path.join(pkg_share, 'config', 'esp32_controllers.yaml')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False}  # Real hardware doesn't use sim time
        ],
    )
    
    # Controller Manager with ESP32 hardware interface
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_manager_config,
            {'use_sim_time': False}
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # micro-ROS agent for ESP32 communication (only for real hardware)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', esp32_port, '--baud', esp32_baudrate],
        output='screen',
        condition=UnlessCondition(use_sim_hardware),
        emulate_tty=True,
    )
    
    # RPLidar node (only when enabled and using real hardware)
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
        output='screen',
        condition=IfCondition(enable_lidar),
        emulate_tty=True,
    )
    
    # Start joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Start diff drive controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay diff drive controller after joint state broadcaster
    diff_drive_controller_spawner_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    # Twist multiplexer using your existing configuration
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ],
        output='screen',
    )
    
    # Static transform for base_footprint to base_link
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': False}]
    )
    
    # Optional: Add diagnostics for hardware monitoring
    hardware_diagnostics = Node(
        package='my_slam_pkg',
        executable='rplidar_diagnostics.py',
        name='hardware_diagnostics',
        output='screen',
        condition=IfCondition(enable_lidar),
        parameters=[{'use_sim_time': False}]
    )
    
    # Add a delay to ensure micro-ROS agent starts first
    delayed_nodes = TimerAction(
        period=3.0,
        actions=[
            controller_manager,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner_delay,
        ]
    )
    
    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add all nodes in order
    ld.add_action(robot_state_publisher)
    ld.add_action(micro_ros_agent)  # Start micro-ROS agent first
    ld.add_action(rplidar_node)     # Start lidar
    ld.add_action(delayed_nodes)    # Start controllers after delay
    ld.add_action(twist_mux)
    ld.add_action(static_tf_base_footprint_to_base_link)
    ld.add_action(hardware_diagnostics)
    
    return ld