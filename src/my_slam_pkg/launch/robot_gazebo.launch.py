from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Set Gazebo model path
    additional_model_path = '/home/alkady/slam_ws/src/my_slam_pkg/worlds/models'
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        new_model_path = f"{existing_model_path}:{additional_model_path}"
    else:
        new_model_path = additional_model_path

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Paths and configurations
    pkg_share = FindPackageShare('my_slam_pkg').find('my_slam_pkg')
    urdf_path = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')
    gazebo_ros_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    world_file = '/home/alkady/slam_ws/src/my_slam_pkg/worlds/hos.world'
    
    # Set Gazebo model path environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
    )
    
    # Get URDF content with more detailed inertial parameters
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        urdf_path, ' ',
        'use_sim_time:=', LaunchConfiguration('use_sim_time')
    ])
    
    # Include Gazebo launch with world file - Ensure realistic physics settings
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'gui': 'true',
            'pause': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'extra_gazebo_args': '--physics=ode --max_step_size=0.001 --real_time_factor=1.0 --real_time_update_rate=1000'
        }.items()
    )
    
    # Robot State Publisher with higher publish frequency
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0,  # Increased from default for smoother TF updates
        }],
        output='screen'
    )

    # Spawn robot in Gazebo with specific pose
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Load controllers from config file
    controllers_config = os.path.join(pkg_share, "config", "controllers.yaml")

    # Launch controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            controllers_config
        ],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Diff Drive Controller - make sure we wait for joint state broadcaster to be active
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Static TF publisher for base_footprint to base_link
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Delay diff_drive_controller after joint_state_broadcaster
    delay_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        set_gazebo_model_path,
        declare_use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner,
        static_tf_base_footprint_to_base_link
    ])