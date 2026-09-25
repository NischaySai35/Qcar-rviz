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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_cameras = LaunchConfiguration('use_cameras')
    use_front_camera = LaunchConfiguration('use_front_camera')
    use_drive_gui = LaunchConfiguration('use_drive_gui')
    sensor_fusion = LaunchConfiguration('sensor_fusion')
    resolution = LaunchConfiguration('resolution')
    detect_objects = LaunchConfiguration('detect_objects')
    explore = LaunchConfiguration('explore')

    # Object detection needs all four cameras to get 360 coverage, so asking
    # for it implies use_cameras even when the operator did not pass it. The
    # single front preview is not enough: an object first seen ahead has to
    # stay observable as the car turns past it, which is what lets sightings
    # from different cameras fuse into one landmark instead of several.
    cameras_on = PythonExpression([
        "'true' if '", use_cameras, "' == 'true' or '",
        detect_objects, "' == 'true' else 'false'",
    ])

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
        'sensor_fusion', default_value='true',
        description='Use LiDAR-primary mapping with wheel-encoder/gyro odometry')
    declare_detect_objects_cmd = DeclareLaunchArgument(
        'detect_objects', default_value='true',
        description='Label the map with objects seen by the 4 cameras '
                    '(implies use_cameras:=true). Pass false for a plain '
                    'geometric map with no detector running.')
    declare_explore_cmd = DeclareLaunchArgument(
        'explore', default_value='false',
        description='Drive the car automatically: run Nav2 on the live SLAM map '
                    'and send it frontier goals until the room is covered. '
                    'Use scripts/start_mapping_auto.sh rather than this flag.')
    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('qcar2_nodes'), 'config', 'qcar2_slam_and_nav.yaml'),
        description='Nav2 parameters used by explore mode (costmaps, planner, controller)')
    declare_time_budget_cmd = DeclareLaunchArgument(
        'time_budget_sec', default_value='900.0',
        description='Hard stop for autonomous exploration, seconds')
    declare_use_voice_cmd = DeclareLaunchArgument(
        'use_voice', default_value='true',
        description='Run the spoken-command listener (microphone stays OFF '
                    'until you turn it on in the browser console)')

    qcar2_gui_dir = get_package_share_directory('qcar2_rviz_gui')
    qcar2_nodes_dir = get_package_share_directory('qcar2_nodes')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(qcar2_gui_dir, 'launch', 'hardware_base.launch.py')]),
        launch_arguments={
            'use_cameras': cameras_on,
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
        # publish_tf must be true: this node is the only thing that publishes
        # odom -> base_link, and qcar2_2d_fused.lua's published_frame = "odom"
        # depends on that transform existing to compute map -> odom. Without
        # it there is no odom frame at all, Nav2's local costmap never starts,
        # and autonomous exploration sits still. See the note in that .lua.
        parameters=[{'publish_tf': True}],
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

    # Semantic mapping: YOLO-World on the 4 cameras, fused with the LiDAR into
    # named map-frame landmarks -- see qcar2_object_mapper.py.  Delayed so the
    # CSI cameras have opened before the first inference pass; the node itself
    # tolerates missing frames, this just avoids a noisy first few seconds.
    object_mapper_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_object_mapper.py',
        name='qcar2_object_mapper',
        output='screen',
        condition=IfCondition(detect_objects),
    )
    object_mapper_start = TimerAction(period=6.0, actions=[object_mapper_node])

    # ------------------------------------------------------------- explore
    # Autonomous exploration runs the SAME Nav2 stack navigation mode uses,
    # but against Cartographer's live map instead of a saved one. Only the
    # navigation half is started: no AMCL and no map_server, because
    # Cartographer already publishes /map AND owns the map->odom transform.
    # Starting localization_launch here would put two nodes in charge of
    # map->odom and the TF tree would fight itself.
    ackermann_bt_xml = os.path.join(
        qcar2_gui_dir, 'behavior_trees', 'navigate_to_pose_ackermann.xml')
    explore_nav_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': ackermann_bt_xml},
        convert_types=True,
    )

    explore_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': explore_nav_params,
        }.items(),
        condition=IfCondition(explore),
    )

    # Same /cmd_vel_nav -> /cmd_vel remap as navigate.launch.py; without it
    # Nav2's velocity commands never reach the motors. See the long note in
    # navigate.launch.py for why this remap exists at all.
    explore_converter_node = Node(
        package='qcar2_nodes',
        executable='nav2_qcar2_converter',
        name='nav2_qcar2_converter',
        output='screen',
        remappings=[('/cmd_vel_nav', '/cmd_vel')],
        condition=IfCondition(explore),
    )

    # Frontier selection -- see qcar2_explorer.py. Delayed so Nav2's lifecycle
    # nodes have activated and /map exists; it waits for both anyway, this just
    # keeps the startup log readable.
    explorer_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_explorer.py',
        name='qcar2_explorer',
        output='screen',
        # ParameterValue(..., value_type=float) is required, not cosmetic:
        # launch infers a parameter's type from the TEXT of the substitution,
        # so "900" arrives as an int and ROS 2 then refuses to override a
        # float-declared parameter with it ("wrong parameter type"), killing
        # the explorer at startup. Forcing the type makes both
        # time_budget_sec:=900 and :=900.0 work.
        parameters=[{'time_budget_sec': ParameterValue(
            LaunchConfiguration('time_budget_sec'), value_type=float)}],
        condition=IfCondition(explore),
    )
    explorer_start = TimerAction(period=12.0, actions=[explorer_node])

    # Spoken commands. The microphone is OFF until switched on in the browser
    # console, so this is safe to run always; in mapping mode the useful ones
    # are "hey car stop"/"cancel" (going to a named object needs Nav2, which
    # only runs in navigate or explore mode).
    # Answers questions about the room ("is there a cooler?", "how far is it?",
    # "how many people are here?") from the landmark map -- see
    # qcar2_assistant.py. Facts come from the map, never from a language model.
    assistant_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_assistant.py',
        name='qcar2_assistant',
        output='screen',
    )

    voice_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_voice_command.py',
        name='qcar2_voice_command',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_voice')),
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
        declare_detect_objects_cmd,
        declare_explore_cmd,
        declare_nav_params_cmd,
        declare_time_budget_cmd,
        declare_use_voice_cmd,
        hardware_launch,
        wheel_imu_odometry_node,
        scan_only_cartographer_node,
        fused_cartographer_start,
        cartographer_occupancy_grid_node,
        drive_gui_node,
        rviz_node,
        web_gui_node,
        announcer_node,
        object_mapper_start,
        voice_node,
        assistant_node,
        explore_navigation_launch,
        explore_converter_node,
        explorer_start,
    ])
