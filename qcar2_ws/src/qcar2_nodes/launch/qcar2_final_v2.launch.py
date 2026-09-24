import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    qcar2_pkg_dir = get_package_share_directory('qcar2_nodes')
    nav2_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(nav2_dir, 'launch')

    # 1. HARDWARE
    qcar2_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(qcar2_pkg_dir, 'launch', 'qcar2_launch.py')])
    )

    # 2. CONFIGURATION
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')

    param_substitutions = {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # 3. ARGUMENTS
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('use_namespace', default_value='false'))
    ld.add_action(DeclareLaunchArgument('slam', default_value='False'))
    ld.add_action(DeclareLaunchArgument('map', default_value='/home/nvidia/maps/qcar_map.yaml'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    
    # POINTING TO OUR NEW PARAMS FILE
    ld.add_action(DeclareLaunchArgument('params_file', 
        default_value='/home/nvidia/Documents/Quanser/5_research/sdcs/qcar2/ros2/src/qcar2_nodes/config/qcar2_slam_and_nav.yaml'))
        
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_composition', default_value='True'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info'))

    # 4. NODES
    
    # STATIC TF: connects base_link -> base_scan (Corrected based on your echo command)
    tf_lidar_node = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0.15', '3.14', '0', '0', 'base_link', 'base_scan'],
        output='screen'
    )

    qcar2_nav2_converter = Node(
        package='qcar2_nodes', executable='nav2_qcar2_converter', name='nav2_qcar2_converter'
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )

    """tf_odom_node = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )"""

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
   	parameters=['/home/nvidia/Documents/ekf.yaml']
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False, # We let EKF handle the TF
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0}],
    )

    bringup_cmd_group = GroupAction([
        PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
        
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'),

        # Load Map & AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=UnlessCondition(slam),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': configured_params,
                              'use_composition': use_composition,
                              'container_name': 'nav2_container'}.items()),

        # Navigation Logic (Planner/Controller)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': configured_params,
                              'use_composition': use_composition,
                              'container_name': 'nav2_container'}.items()),
    ])

    ld.add_action(qcar2_hardware_launch)
    ld.add_action(tf_lidar_node)
    #ld.add_action(tf_odom_node)
    ld.add_action(rf2o_node)
    ld.add_action(robot_localization_node)
    ld.add_action(qcar2_nav2_converter)
    ld.add_action(bringup_cmd_group)
    ld.add_action(rviz_node)

    return ld
