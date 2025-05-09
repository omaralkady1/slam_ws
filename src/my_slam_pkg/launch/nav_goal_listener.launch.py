from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Navigation goal listener node
    nav_goal_listener_node = Node(
        package='my_slam_pkg',
        executable='nav_goal_listener.py',
        name='nav_goal_listener',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        nav_goal_listener_node
    ])