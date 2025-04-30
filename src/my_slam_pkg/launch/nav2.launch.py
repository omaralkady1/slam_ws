from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('my_slam_pkg')
    workspace_dir = os.path.join(pkg_share, '..', '..')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')
    
    # Launch argument declarations
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(workspace_dir, 'my_new_map.yaml'),
        description='Full path to map yaml file'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file for navigation'
    )
    
    # Start Nav2 nodes
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )
    
    # Start RViz2 with existing config
    rviz_config_file = os.path.join(pkg_share, 'config', 'nav2_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        declare_params_file,
        nav2_bringup,
        rviz_node,
    ])