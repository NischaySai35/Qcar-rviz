"""
MAPPING MODE: drive the car around and build a brand new SLAM map with
Cartographer, watching everything live in our custom RViz layout
(360 cameras + LiDAR + growing map).

Usage:
  ros2 launch qcar2_rviz_gui mapping.launch.py
  ros2 launch qcar2_rviz_gui mapping.launch.py use_rviz:=false use_cameras:=false

When you're happy with the map, in another terminal run scripts/save_map.sh
(or the map_saver_cli command from the README) BEFORE shutting this down.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_cameras = LaunchConfiguration('use_cameras')
    use_front_camera = LaunchConfiguration('use_front_camera')
    use_drive_gui = LaunchConfiguration('use_drive_gui')
    sensor_fusion = LaunchConfiguration('sensor_fusion')
    resolution = LaunchConfiguration('resolution')

    # The browser console (qcar2_web_gui.py) is the default operator UI and
    # includes the hold-to-drive pad; RViz and the Tk drive window are opt-in.
    declare_use_web_gui_cmd = DeclareLaunchArgument(
        'use_web_gui', default_value='true',
        description='Serve the QCar2 browser console (map, drive pad, save map, cameras, voice)')
    declare_web_port_cmd = DeclareLaunchArgument(
        'web_port', default_value='8080', description='Port for the browser console')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='Also launch RViz (debug only)')
    declare_use_cameras_cmd = DeclareLaunchArgument(
        'use_cameras', default_value='false',
        description='Launch all 4 CSI cameras for a stitched 360 view '
                     '(disabled by default -- this car only needs the single '
                     'front preview below for a normal drive; pass '
                     'use_cameras:=true only if you actually want all four)')
    declare_use_front_camera_cmd = DeclareLaunchArgument(
        'use_front_camera', default_value='true',
        description='Start the single front-camera preview relay shown in '
                     'the RViz "Front Camera" panel. On by default -- it is '
                     'just a display feed (no analysis), throttled to a few '
                     'FPS, and costs one CSI camera instead of all four.')
    declare_use_drive_gui_cmd = DeclareLaunchArgument(
        'use_drive_gui', default_value='false',
        description='Open the old Tk hold-to-drive window (the browser console has a drive pad)')
    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution', default_value='0.05', description='Map grid resolution (m/cell)')
    declare_sensor_fusion_cmd = DeclareLaunchArgument(
        'sensor_fusion', default_value='false',
        description='Use LiDAR-primary mapping with wheel-encoder/gyro odometry')

    qcar2_gui_dir = get_package_share_directory('qcar2_rviz_gui')
    qcar2_nodes_dir = get_package_share_directory('qcar2_nodes')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(qcar2_gui_dir, 'launch', 'hardware_base.launch.py')]),
        launch_arguments={
            'use_cameras': use_cameras,
            'use_front_camera': use_front_camera,
        }.items(),
    )

    scan_only_cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', os.path.join(qcar2_nodes_dir, 'config'),
            '-configuration_basename', 'qcar2_2d.lua',
        ],
        condition=UnlessCondition(sensor_fusion),
    )

    # Cartographer collates every enabled sensor. Start it only after the
    # independent encoder/gyro odometry node has calibrated at rest; this
    # avoids a blank /map at startup.
    fused_cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', os.path.join(qcar2_nodes_dir, 'config'),
            '-configuration_basename', 'qcar2_2d_fused.lua',
        ],
        remappings=[('imu', '/qcar2_imu'), ('odom', '/wheel_imu_odom')],
        condition=IfCondition(sensor_fusion),
    )
    fused_cartographer_start = TimerAction(period=4.0, actions=[fused_cartographer_node])

    wheel_imu_odometry_node = Node(
        package='qcar2_rviz_gui',
        executable='wheel_imu_odometry.py',
        name='wheel_imu_odometry',
        output='screen',
        condition=IfCondition(sensor_fusion),
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', resolution, '-publish_period_sec', '1.0'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(qcar2_gui_dir, 'rviz', 'qcar2_full_gui.rviz')],
        condition=IfCondition(use_rviz),
    )

    drive_gui_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_drive_gui.py',
        name='qcar2_drive_gui',
        output='screen',
        condition=IfCondition(use_drive_gui),
    )

    # Browser console -- see scripts/qcar2_web_gui.py and web/index.html.
    # Needs no DISPLAY; open http://<car-ip>:<web_port> from any laptop.
    web_gui_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_web_gui.py',
        name='qcar2_web_gui',
        output='screen',
        parameters=[{'mode': 'mapping', 'port': LaunchConfiguration('web_port')}],
        condition=IfCondition(LaunchConfiguration('use_web_gui')),
    )

    # "Mapping started" plus nearby-obstacle callouts through the onboard
    # speaker -- see qcar2_announcer.py.
    announcer_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_announcer.py',
        name='qcar2_announcer',
        output='screen',
        parameters=[{'mode': 'mapping'}],
    )

    return LaunchDescription([
        declare_use_web_gui_cmd,
        declare_web_port_cmd,
        declare_use_rviz_cmd,
        declare_use_cameras_cmd,
        declare_use_front_camera_cmd,
        declare_use_drive_gui_cmd,
        declare_resolution_cmd,
        declare_sensor_fusion_cmd,
        hardware_launch,
        wheel_imu_odometry_node,
        scan_only_cartographer_node,
        fused_cartographer_start,
        cartographer_occupancy_grid_node,
        drive_gui_node,
        rviz_node,
        web_gui_node,
        announcer_node,
    ])
