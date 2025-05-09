from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('my_slam_pkg')
    
    # Define the path to map file
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    
    # Check if map file exists
    if not os.path.exists(map_file_path):
        return LaunchDescription([
            LogInfo(msg=f"ERROR: Map file not found at {map_file_path}. Please check your file path.")
        ])
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_file = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=map_file_path,
        description='Full path to map yaml file to load'
    )
    
    # Log info about the map file
    log_map_info = LogInfo(
        msg=["Using map file: ", LaunchConfiguration('map_yaml_file')]
    )
    
    # Launch map server node with simplified parameters
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'yaml_filename': LaunchConfiguration('map_yaml_file')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # Launch lifecycle manager for map server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml_file,
        log_map_info,
        map_server_node,
        lifecycle_manager_node
    ])