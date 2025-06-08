from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # Robot in Gazebo with controllers
    robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_gazebo.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # LOCALIZATION COMPONENTS
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
            ('cmd_vel', '/cmd_vel')  # Make sure this aligns with twist_mux output
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
    
    # Twist multiplexer to manage velocity commands
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            os.path.join(pkg_share, 'config', 'twist_mux.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')
        ],
        output='screen'
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
    rviz_config_file = os.path.join(pkg_share, 'config', 'nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Set initial pose
    set_initial_pose = Node(
        package='my_slam_pkg',
        executable='set_initial_pose.py',
        name='set_initial_pose',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # Add your default initial pose coordinates here
        arguments=['0.0', '0.0', '0.0']
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
        
        # Start robot
        robot_gazebo,
        
        # Start localization components
        map_server,
        amcl,
        lifecycle_manager_loc,
        
        # Start twist mux right after robot for command management
        twist_mux_node,
        
        # Start navigation components (with delay)
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
        
        # Start RViz (with delay)
        TimerAction(
            period=2.0,
            actions=[rviz_node]
        ),
        
        # Set initial pose (with delay)
        TimerAction(
            period=10.0,
            actions=[set_initial_pose]
        ),
        
        # Ensure all components are active (with extra delay)
        TimerAction(
            period=15.0,
            actions=[ensure_active]
        )
    ])