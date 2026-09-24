import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    bringup_dir = get_package_share_directory('qcar2_nodes')
    nav2_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(nav2_dir, 'launch')

    # === PARAMETERS ===
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/nvidia/maps/qcar_map.yaml',
        description='Full path to map yaml file')

    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'qcar2_slam_and_nav.yaml'),
        description='Nav2 parameters file')

    # === NAV2 MAP SERVER + AMCL + CONTROLLERS ===
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file,
            'map': map_file,
            'slam': 'false'
        }.items()
    )

    # === QCAR2 NAV2 CONVERTER ===
    qcar2_nav2_converter = Node(
        package='qcar2_nodes',
        executable='nav2_qcar2_converter',
        name='nav2_qcar2_converter',
        output='screen'
    )

    # === LAUNCH DESCRIPTION ===
    ld = LaunchDescription()

    # ENV fix
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(navigation)
    ld.add_action(qcar2_nav2_converter)

    return ld

