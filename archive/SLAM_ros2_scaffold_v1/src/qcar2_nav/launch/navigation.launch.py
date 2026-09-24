import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('qcar2_nav')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))

    # Nav2 Bringup Setup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # SLAM Toolbox Setup
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Launch Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # RViz (optional)
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Custom CSI Hub (Fixes camera error)
    csi_hub = Node(
        package='qcar2_nav',
        executable='csi_hub',
        name='csi_hub',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')))
    
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_cmd)
    ld.add_action(csi_hub)

    return ld
