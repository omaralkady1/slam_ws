from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
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
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to navigation parameters file'
    )
    
    # ROBOT COMPONENTS
    # Robot in Gazebo
    robot_gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'my_slam_pkg', 'robot_gazebo.launch.py'],
        output='screen'
    )
    
    # LOCALIZATION COMPONENTS
    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map_yaml_file')
        }]
    )
    
    # AMCL
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
            'tf_broadcast': True
        }]
    )
    
    # NAVIGATION COMPONENTS
    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )
    
    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # LIFECYCLE MANAGEMENT
    # Lifecycle manager for localization
    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # Lifecycle manager for navigation
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['controller_server', 'planner_server', 
                           'behavior_server', 'bt_navigator']
        }]
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
    
    # Activation helpers
    # Set initial pose after delay
    set_initial_pose = Node(
        package='my_slam_pkg',
        executable='set_initial_pose.py',
        name='set_initial_pose',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Ensure navigation components are active
    ensure_active = Node(
        package='my_slam_pkg',
        executable='ensure_nav_active.py',
        name='ensure_nav_active',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml_file,
        declare_params_file,
        
        # Step 1: Start robot
        robot_gazebo,
        
        # Step 2: Start localization components
        static_tf,
        map_server,
        amcl,
        lifecycle_manager_loc,
        
        # Step 3: Start navigation components after localization (with delay)
        TimerAction(
            period=5.0,
            actions=[
                controller_server,
                planner_server,
                behavior_server,
                bt_navigator,
                lifecycle_manager_nav
            ]
        ),
        
        # Step 4: Start RViz (with delay)
        TimerAction(
            period=2.0,
            actions=[rviz_node]
        ),
        
        # Step 5: Set initial pose (with delay)
        TimerAction(
            period=10.0,
            actions=[set_initial_pose]
        ),
        
        # Step 6: Ensure all components are active (with extra delay)
        TimerAction(
            period=15.0,
            actions=[ensure_active]
        )
    ])