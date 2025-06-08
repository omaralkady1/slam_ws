from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'esp32_slam_params.yaml'),
            description='Full path to the ROS2 parameters file for SLAM',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_share, 'config', 'esp32_slam_real.rviz'),
            description='Full path to the RViz config file',
        )
    )
    
    # Get arguments
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # Include the ESP32 robot launch
    esp32_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'esp32_real_robot.launch.py')
        ]),
        launch_arguments={
            'use_sim_hardware': use_sim_hardware,
            'esp32_port': esp32_port,
            'lidar_port': lidar_port,
            'enable_lidar': 'true'
        }.items()
    )
    
    # SLAM Toolbox node for online mapping - CORRECTED
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False}  # Real hardware doesn't use sim time
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/map', '/map')
        ]
    )
    
    # RViz2 for visualization - CORRECTED config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # Teleop node for manual control during mapping
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel'),  # Goes through twist_mux
        ],
        parameters=[{'use_sim_time': False}]
    )
    
    # Optional: TF monitor for debugging
    tf_monitor = Node(
        package='my_slam_pkg',
        executable='tf_monitor.py',
        name='tf_monitor',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Map saver service (to save maps when needed)
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Delay SLAM and visualization to ensure robot is ready
    delayed_slam = TimerAction(
        period=8.0,  # Increased delay to ensure all hardware is ready
        actions=[slam_toolbox_node]
    )
    
    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz_node]
    )
    
    delayed_teleop = TimerAction(
        period=12.0,
        actions=[teleop_node]
    )
    
    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add all components
    ld.add_action(esp32_robot_launch)      # Start robot hardware first
    ld.add_action(map_saver_server)        # Start map saver service
    ld.add_action(delayed_slam)            # Start SLAM after robot is ready
    ld.add_action(delayed_rviz)            # Start RViz after SLAM
    ld.add_action(delayed_teleop)          # Start teleop for manual driving
    ld.add_action(tf_monitor)              # TF monitoring
    
    return ld