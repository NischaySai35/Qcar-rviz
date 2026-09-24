import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directories
    qcar2_pkg_dir = get_package_share_directory('qcar2_nodes')
    nav2_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(nav2_dir, 'launch')

    # ---------------------------------------------------------
    # 1. HARDWARE LAUNCH (Lidar + Odom + Camera)
    # ---------------------------------------------------------
    qcar2_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            qcar2_pkg_dir, 'launch', 'qcar2_launch.py')]
        )
    )

    # ---------------------------------------------------------
    # 2. CONFIGURATION VARIABLES
    # ---------------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # ---------------------------------------------------------
    # 3. DECLARE ARGUMENTS
    # ---------------------------------------------------------
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        # UPDATED: Pointing to your specific map file
        default_value='/home/nvidia/maps/qcar_map.yaml',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(qcar2_pkg_dir, 'config', 'qcar2_slam_and_nav.yaml'),
        description='Full path to the ROS2 parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # ---------------------------------------------------------
    # 4. FIXES (Static TF & Converter)
    # ---------------------------------------------------------
    
    # Fix for "queue full" error: Connect base_link to base_scan
    tf_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'base_scan'],
        output='screen'
    )

    qcar2_nav2_converter = Node(
        package='qcar2_nodes',
        executable='nav2_qcar2_converter',
        name='nav2_qcar2_converter',
    )

    # ---------------------------------------------------------
    # 5. RVIZ NODE
    # ---------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ---------------------------------------------------------
    # 6. NAV2 BRINGUP GROUP
    # ---------------------------------------------------------
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        # A. LOCALIZATION (Runs when SLAM is FALSE)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=UnlessCondition(slam),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        # B. SLAM (Runs when SLAM is TRUE)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        # C. NAVIGATION (Always Runs)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # ---------------------------------------------------------
    # 7. BUILD LAUNCH DESCRIPTION
    # ---------------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(qcar2_hardware_launch)
    ld.add_action(tf_lidar_node)
    ld.add_action(qcar2_nav2_converter)
    ld.add_action(bringup_cmd_group)
    ld.add_action(rviz_node)

    return ld