import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. SETUP
    qcar2_pkg_dir = get_package_share_directory('qcar2_nodes')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    # 2. HARDWARE & SENSORS
    qcar2_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(qcar2_pkg_dir, 'launch', 'qcar2_launch.py')])
    )

    tf_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0.15', '3.14', '0', '0', 'base_link', 'base_scan'],
        output='screen'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0}],
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/home/nvidia/Documents/ekf.yaml']
    )

    # 3. TOPIC RELAYS (The Fix for Communication)
    # Bridge 1: Force Nav2 commands to reach the hardware
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/cmd_vel_nav']
    )

    # Bridge 2: Force EKF data into the topic Nav2 expects
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        output='screen',
        arguments=['/odometry/filtered', '/odom']
    )

    # 4. NAV2 STACK (Controller Only)
    nav2_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']

    nav2_group = GroupAction([
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': nav2_nodes}]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            # FORCE REMAPPING: Nav2 Output -> Hardware Input
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'),    # <--- CRITICAL: Sends commands to wheels
                ('/odom', '/odometry/filtered')  # <--- CRITICAL: Reads speed from EKF
            ]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params]),
    ])

    # 5. RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('params_file', 
            default_value='/home/nvidia/Documents/Quanser/5_research/sdcs/qcar2/ros2/src/qcar2_nodes/config/qcar2_slam_and_nav.yaml'),

        qcar2_hardware_launch,
        tf_lidar_node,
        rf2o_node,
        robot_localization_node,
        cmd_vel_relay,  # <--- Relay 1
        odom_relay,     # <--- Relay 2
        nav2_group,
        rviz_node
    ])