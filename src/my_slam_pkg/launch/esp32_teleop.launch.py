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
            'enable_lidar',
            default_value='true',
            description='Enable RPLidar',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Enable RViz visualization',
        )
    )
    
    # Get package directory
    pkg_share = FindPackageShare('my_slam_pkg').find('my_slam_pkg')
    
    # Get arguments
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'urdf', 'car.urdf.xacro'), ' ',
        'use_fake_hardware:=false'
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # ESP32 controllers configuration
    controller_manager_config = os.path.join(pkg_share, 'config', 'esp32_controllers.yaml')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False}
        ],
    )
    
    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_manager_config,
            {'use_sim_time': False}
        ],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ]
    )
    
    # micro-ROS agent for ESP32
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', esp32_port, '--baud', '115200'],
        output='screen'
    )
    
    # RPLidar node
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
        condition=IfCondition(enable_lidar)
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Diff drive controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay diff drive controller
    diff_drive_controller_spawner_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    # Simple twist multiplexer (using existing config)
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
    
    # Keyboard teleop - Direct to diff_drive_controller for now
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel'),  # Use the main topic that twist_mux handles
        ],
        parameters=[{'use_sim_time': False}]
    )
    
    # Static transforms
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': False}]
    )
    
    static_tf_base_link_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lidar',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': False}]
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'esp32_slam_real.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(enable_rviz)
    )
    
    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    
    # Core robot nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(micro_ros_agent)
    ld.add_action(rplidar_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner_delay)
    ld.add_action(twist_mux)
    ld.add_action(static_tf_base_footprint_to_base_link)
    ld.add_action(static_tf_base_link_to_lidar)
    
    # Teleop with delay
    ld.add_action(TimerAction(
        period=4.0,  # Wait for controllers to be ready
        actions=[teleop_keyboard]
    ))
    
    # RViz with delay
    ld.add_action(TimerAction(
        period=2.0,
        actions=[rviz_node]
    ))
    
    return ld