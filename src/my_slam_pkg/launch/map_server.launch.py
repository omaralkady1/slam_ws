from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Create launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'slam_ws/src/my_slam_pkg/maps/my_map.yaml'
        ),
        description='Full path to map yaml file to load'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Start map server
    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'yaml_filename': map_yaml_file}]
    )
    
    # Start lifecycle manager
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': True},
                   {'node_names': ['map_server']}]
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Add declared arguments
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    
    return ld