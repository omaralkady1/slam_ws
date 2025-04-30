from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Package share directory
    pkg_share = FindPackageShare('my_slam_pkg').find('my_slam_pkg')
    
    # Include robot_gazebo.launch.py
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_gazebo.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # Launch twist_mux
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel')
        ],
        output='screen'
    )
    
    # Include teleop.launch.py
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'teleop.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_gazebo_launch,
        twist_mux_node,
        teleop_launch
    ])