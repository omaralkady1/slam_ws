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
    
    declare_cmd_vel_target = DeclareLaunchArgument(
        'cmd_vel_target',
        default_value='/cmd_vel',
        description='Target topic for twist_mux cmd_vel_out remapping'
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
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'twist_mux.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
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
        declare_cmd_vel_target,
        robot_gazebo_launch,
    
        twist_mux,
        teleop_launch
    ])