from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('my_slam_pkg')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_file = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(pkg_share, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    
    
    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # AMCL - minimal config
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': '/scan',
            'transform_tolerance': 1.0,
            'robot_model_type': 'differential',
            'tf_broadcast': True
        }]
    )
    
    # Manual setup of the lifecycle nodes
    # We use ExecuteProcess to sleep before setting up the nodes
    setup_map_server = ExecuteProcess(
        cmd=['sleep', '2', '&&', 
             'ros2', 'lifecycle', 'set', '/map_server', 'configure', '&&',
             'ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        shell=True
    )
    
    setup_amcl = ExecuteProcess(
        cmd=['sleep', '4', '&&', 
             'ros2', 'lifecycle', 'set', '/amcl', 'configure', '&&',
             'ros2', 'lifecycle', 'set', '/amcl', 'activate'],
        shell=True
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'simple_nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Set initial pose
    set_initial_pose = ExecuteProcess(
        cmd=['sleep', '10', '&&', 
             'ros2', 'run', 'my_slam_pkg', 'set_initial_pose.py', '0.0', '0.0', '0.0'],
        shell=True
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml_file,
        
        static_tf,
        amcl,
        setup_map_server,
        setup_amcl,
        rviz_node,
        set_initial_pose
    ])