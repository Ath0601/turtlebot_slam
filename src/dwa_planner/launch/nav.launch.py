import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Directories
    pkg_dwa_planner = get_package_share_directory('dwa_planner')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    xacro_file = PathJoinSubstitution([pkg_dwa_planner, 'urdf', 'turtlebot3_waffle.urdf.xacro'])
    world_file = os.path.join(pkg_dwa_planner, 'world', 'environment.world')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'turtlebot3_waffle',
            '-topic', '/robot_description',
            '-x', '-0.30', '-y', '0.0', '-z', '0.1'
        ]
    )

    # BRIDGES
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # CONTROLLER SPAWNERS
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # CUSTOM DWA PLANNER
    dwa_planner = Node(
        package='dwa_planner',
        executable='planner',
        name='dwa_planner',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Goal Publisher
    goal_publisher = Node(
        package='dwa_planner',
        executable='goal',
        name='goal_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    wheel_odometry = Node(
        package='dwa_planner',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen'
    )

    twist_joint = Node(
        package='dwa_planner',
        executable='converter',
        name='twist_to_effort',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        
        # Use event handlers for proper sequencing
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[bridge_clock, bridge_scan, bridge_imu]
                    )
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=4.0,
                        actions=[joint_state_spawner]
                    )
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[drive_controller_spawner]
                    )
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=drive_controller_spawner,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[wheel_odometry,dwa_planner, goal_publisher, twist_joint]
                    )
                ]
            )
        )
    ])