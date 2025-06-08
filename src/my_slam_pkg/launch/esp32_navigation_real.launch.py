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
            'map_yaml_file',
            default_value=os.path.join(pkg_share, 'maps', 'my_map.yaml'),
            description='Full path to map yaml file to load',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'esp32_nav2_params.yaml'),
            description='Full path to the ROS2 parameters file for navigation',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_share, 'config', 'esp32_navigation_real.rviz'),
            description='Full path to the RViz config file',
        )
    )
    
    # Get arguments
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
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
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ]
    )
    
    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ]
    )
    
    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ]
    )
    
    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ]
    )
    
    # Lifecycle manager for localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # Lifecycle manager for navigation
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
        }]
    )
    
    # RViz2 for visualization and goal setting
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # Optional: Navigation goal listener for programmatic control
    nav_goal_listener = Node(
        package='my_slam_pkg',
        executable='nav_goal_listener.py',
        name='nav_goal_listener',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Optional: Set initial pose automatically
    set_initial_pose = Node(
        package='my_slam_pkg',
        executable='set_initial_pose.py',
        name='set_initial_pose',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['0.0', '0.0', '0.0']  # Adjust as needed
    )
    
    # Delay navigation components to ensure robot is ready
    delayed_localization = TimerAction(
        period=5.0,
        actions=[
            map_server,
            amcl,
            lifecycle_manager_localization
        ]
    )
    
    delayed_navigation = TimerAction(
        period=8.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_navigation
        ]
    )
    
    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz_node]
    )
    
    delayed_initial_pose = TimerAction(
        period=12.0,
        actions=[set_initial_pose]
    )
    
    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add all components in order
    ld.add_action(esp32_robot_launch)      # Start robot hardware first
    ld.add_action(delayed_localization)    # Start localization components
    ld.add_action(delayed_navigation)      # Start navigation components
    ld.add_action(delayed_rviz)            # Start RViz
    ld.add_action(delayed_initial_pose)    # Set initial pose
    ld.add_action(nav_goal_listener)       # Goal listener for programmatic control
    
    return ld