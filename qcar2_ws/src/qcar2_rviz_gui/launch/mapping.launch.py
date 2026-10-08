"""
MAPPING MODE: drive the car around and build a brand new SLAM map with
Cartographer, watching everything live in our custom RViz layout
(360 cameras + LiDAR + growing map).

Usage:
  ros2 launch qcar2_rviz_gui mapping.launch.py
  ros2 launch qcar2_rviz_gui mapping.launch.py use_rviz:=false use_cameras:=false

Pass map_name:=NAME to autosave maps/NAME.* every 10 s (the scripts do).
Otherwise save with the console's Save Map button. Ctrl+C itself never saves.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
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
        description='Serve the QCar2 browser console (map, drive pad, save map, cameras)')
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
    # Name the map + objects are AUTOSAVED under every 10 s (same files
    # overwritten). scripts/start_mapping.sh sets it; empty = no autosave.
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name', default_value='',
        description='Autosave maps/<map_name>.yaml/.pgm/_objects.json every 10 s')
    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('qcar2_nodes'), 'config', 'qcar2_slam_and_nav.yaml'),
        description='Nav2 parameters used by explore mode (costmaps, planner, controller)')
    declare_time_budget_cmd = DeclareLaunchArgument(
        'time_budget_sec', default_value='1800.0',
        description='Hard stop for autonomous exploration, seconds')
    declare_go_next_room_cmd = DeclareLaunchArgument(
        'go_next_room', default_value='false',
        description='Auto-explore: also follow doorways into neighbouring rooms. '
                    'Off by default -- only the room the car starts in is mapped.')
    declare_use_llm_cmd = DeclareLaunchArgument(
        'use_llm', default_value='true',
        description='Auto-explore: run the vision model (Cosmos-Reason2) so the car '
                    'can spot glass, doorways and gaps not worth visiting')
    declare_llm_size_cmd = DeclareLaunchArgument(
        'llm_size', default_value='auto',
        description='Cosmos-Reason2 size for auto-explore: 2B (~1 s per look), 8B '
                    '(~3-4 s, better judgement), or auto = 8B when fully downloaded')
    declare_use_realsense_cmd = DeclareLaunchArgument(
        'use_realsense', default_value='true',
        description='Show the RealSense D435 on top as the console front view, '
                    'with distances and fps drawn on it -- same as navigation. '
                    'Object detection keeps using the four CSI cameras.')

    qcar2_gui_dir = get_package_share_directory('qcar2_rviz_gui')
    qcar2_nodes_dir = get_package_share_directory('qcar2_nodes')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(qcar2_gui_dir, 'launch', 'hardware_base.launch.py')]),
        launch_arguments={
            'use_cameras': cameras_on,
            # The CSI front-preview relay publishes /front/camera/preview,
            # and so does qcar2_depth_view.py when the RealSense is the front
            # view: two publishers on one topic, frames from both cameras
            # interleaved. With the RealSense, the relay stays off (object
            # detection reads the CSI camera's raw stream directly anyway).
            'use_front_camera': PythonExpression([
                "'true' if '", use_front_camera, "' == 'true' and '",
                LaunchConfiguration('use_realsense'), "' != 'true' else 'false'"]),
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
        # Errors only: at info/warn Cartographer prints several lines a second
        # of internal scan-matching chatter ("constraint_builder ... score",
        # "Dropped N earlier points") that buries anything that matters.
        ros_arguments=['--log-level', 'error'],
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
        ros_arguments=['--log-level', 'error'],       # see scan_only_cartographer_node
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
        ros_arguments=['--log-level', 'error'],
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

    # RealSense D435 as the console's front view, exactly as navigate.launch.py
    # runs it (see the notes there): colour + aligned depth at 640x480/15 fps,
    # and qcar2_depth_view.py drawing distances and the real fps on it.
    use_realsense = LaunchConfiguration('use_realsense')
    realsense_node = Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='front', name='realsense', output='screen',
        parameters=[{
            'enable_color': True, 'enable_depth': True,
            'enable_infra1': False, 'enable_infra2': False,
            'enable_gyro': False, 'enable_accel': False,
            'align_depth.enable': True, 'pointcloud.enable': False,
            # Smaller than navigation's 640x480: in mapping the RealSense
            # shares the CPU with four CSI cameras, YOLO-World and
            # Cartographer, and the console showed it at only 2-4 fps.
            # Aligning depth to colour is done on the CPU and scales with
            # the pixel count; 640x360 colour + 480x270 depth is ~45% of
            # the work and still plenty for the view, the vision model
            # (which gets 448 px) and the depth obstacles (every 4th pixel).
            'rgb_camera.profile': '640x360x15', 'depth_module.profile': '480x270x15',
            'publish_tf': False,
        }],
        condition=IfCondition(use_realsense),
    )
    depth_view_node = Node(
        package='qcar2_rviz_gui', executable='qcar2_depth_view.py',
        name='qcar2_depth_view', output='screen',
        condition=IfCondition(use_realsense),
    )
    front_topic = PythonExpression([
        "'/front/camera/depth_view' if '", use_realsense,
        "' == 'true' else '/front/camera/csi_image'"])

    # Browser console -- see scripts/qcar2_web_gui.py and web/index.html.
    # Needs no DISPLAY; open http://<car-ip>:<web_port> from any laptop.
    web_gui_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_web_gui.py',
        name='qcar2_web_gui',
        output='screen',
        # cameras_owned: this launch starts all four cameras itself, so the
        # console must NEVER start its own copies for the 360 view. It used to
        # guess by counting /rear camera publishers at that instant; a main
        # camera mid-restart counted as "not running", duplicates were
        # launched, and the two sets then fought over the devices forever
        # (dozens of csi restarts a minute, CPU starved, controller late).
        # nav2_drives: in explore mode Nav2 + nav2_qcar2_converter drive the
        # motors, so the console must NOT publish its idle zeros straight to
        # /qcar2_motor_speed_cmd -- see drive_tick() in qcar2_web_gui.py.
        parameters=[{'mode': 'mapping', 'port': LaunchConfiguration('web_port'),
                     'front_camera_topic': front_topic,
                     'cameras_owned': ParameterValue(cameras_on, value_type=bool),
                     'nav2_drives': ParameterValue(explore, value_type=bool),
                     'autosave_name': ParameterValue(
                         LaunchConfiguration('map_name'), value_type=str)}],
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
            # Errors only: Nav2 logs every replan ("Passing new path to
            # controller", twice a second) and every slow control cycle.
            # Real failures ("Failed to make progress", planning failures)
            # are errors and still print; the console shows nav state too.
            'log_level': 'error',
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

    # ROOM AWARENESS for auto-explore (see each script's docstring):
    #   qcar2_room_analyzer.py -- outline, coverage %, which gaps matter, map
    #                             quality, and the stop rule (geometry only);
    #   qcar2_explore_vlm.py   -- looks through the cameras with the vision
    #                             model: glass, doorways, gaps not worth it,
    #                             and the final report.
    go_next_room = ParameterValue(LaunchConfiguration('go_next_room'), value_type=bool)
    room_analyzer_node = Node(
        package='qcar2_rviz_gui', executable='qcar2_room_analyzer.py',
        name='qcar2_room_analyzer', output='screen',
        parameters=[{'go_next_room': go_next_room}],
        condition=IfCondition(explore),
    )
    explore_vlm_node = Node(
        package='qcar2_rviz_gui', executable='qcar2_explore_vlm.py',
        name='qcar2_explore_vlm', output='screen',
        parameters=[{'go_next_room': go_next_room}],
        condition=IfCondition(explore),
    )

    # The vision model server, same as navigate.launch.py's (llama.cpp
    # llama-server on 127.0.0.1:8090), started only for auto-explore.
    # llm_size auto = the 8B model when its file is completely downloaded
    # (better judgement on glass/doorways; ~3-4 s a look, which the car can
    # afford because it stands still while looking), else the 2B.
    def start_llm_server(context):
        if LaunchConfiguration('explore').perform(context).lower() != 'true' or \
                LaunchConfiguration('use_llm').perform(context).lower() != 'true':
            return []
        models = os.path.join(os.path.expanduser('~'), 'Desktop', 'Qcar-rviz',
                              'models', 'cosmos-reason2')
        files = {'2B': ('Cosmos-Reason2-2B-Q8_0.gguf', 'mmproj-Cosmos-Reason2-2B-F16.gguf'),
                 '8B': ('Cosmos-Reason2-8B-Q4_K_M.gguf', 'mmproj-Cosmos-Reason2-8B-F16.gguf')}
        size = LaunchConfiguration('llm_size').perform(context).upper()
        if size == 'AUTO':
            model_8b = os.path.join(models, files['8B'][0])
            # A partial download is a smaller file under the final name or a
            # .part beside it; the complete Q4_K_M is ~5.0 GB.
            complete = os.path.isfile(model_8b) and os.path.getsize(model_8b) > 4.9e9
            size = '8B' if complete else '2B'
        binary = os.path.join(os.path.expanduser('~'), 'llama.cpp', 'build', 'bin', 'llama-server')
        model, mmproj = (os.path.join(models, f) for f in files.get(size, files['2B']))
        missing = [p for p in (binary, model, mmproj) if not os.path.isfile(p)]
        if missing:
            return [LogInfo(msg=f'[llm] Not starting the vision model, missing: '
                                f'{", ".join(missing)}. Exploring on geometry alone.')]
        return [LogInfo(msg=f'[llm] Vision model for exploration: Cosmos-Reason2 {size}'),
                ExecuteProcess(
                    cmd=[binary, '-m', model, '--mmproj', mmproj,
                         '-ngl', '99', '-c', '8192', '--host', '127.0.0.1', '--port', '8090'],
                    name='llama_server', output='log')]

    # Obstacle clearance for qcar2_hardware.cpp (the car eases to a stop 8 cm
    # short of anything the LiDAR or depth camera sees, under Nav2 and the
    # drive pad alike), plus the last resort when it touches something
    # neither sensor sees (glass): freezes the odometry's translation so the
    # map is not dragged, tells the explorer to give that goal up, and marks
    # it on both costmaps' bump_layer -- see qcar2_bump_guard.py. Runs in
    # manual mapping too, for the clearance.
    bump_guard_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_bump_guard.py',
        name='qcar2_bump_guard',
        output='screen',
    )
    # Low obstacles under the LiDAR plane, from the RealSense depth -- see
    # qcar2_depth_obstacles.py.
    depth_obstacles_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_depth_obstacles.py',
        name='qcar2_depth_obstacles',
        output='screen',
        condition=IfCondition(use_realsense),
    )

    # Mapping ONLY maps. Voice commands, question answering ("is there a
    # cooler?") and "go to the sofa" live in navigate.launch.py, where a
    # finished, saved map with its objects is loaded and the car can act on
    # it. The one exception is the speaker: spoken callouts of what the car
    # finds ("television on the left, 2.3 metres", "person ahead", nearby
    # obstacles). Speaking only -- no microphone runs here, so no echo cancel.
    announcer_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_announcer.py',
        name='qcar2_announcer',
        output='screen',
        parameters=[{'mode': 'mapping', 'echo_cancel': False}],
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
        declare_map_name_cmd,
        declare_nav_params_cmd,
        declare_time_budget_cmd,
        declare_use_realsense_cmd,
        declare_go_next_room_cmd,
        declare_use_llm_cmd,
        declare_llm_size_cmd,
        OpaqueFunction(function=start_llm_server),
        hardware_launch,
        realsense_node,
        depth_view_node,
        wheel_imu_odometry_node,
        scan_only_cartographer_node,
        fused_cartographer_start,
        cartographer_occupancy_grid_node,
        drive_gui_node,
        rviz_node,
        web_gui_node,
        announcer_node,
        object_mapper_start,
        explore_navigation_launch,
        explore_converter_node,
        explorer_start,
        bump_guard_node,
        depth_obstacles_node,
        room_analyzer_node,
        explore_vlm_node,
    ])
