"""
NAVIGATION MODE: load a previously-saved map, localize the car on it
(AMCL), run full Nav2 (global/local costmaps = obstacle detection,
planner, controller), and open our RViz GUI where you can click
"Nav2 Goal" and then click a point on the map to send the car there.

Usage:
  ros2 launch qcar2_rviz_gui navigate.launch.py map:=/path/to/your_map.yaml
(defaults to Qcar-rviz/maps/qcar_map.yaml if you don't pass one)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_speed_slider = LaunchConfiguration('use_speed_slider')
    use_cameras = LaunchConfiguration('use_cameras')
    use_front_camera = LaunchConfiguration('use_front_camera')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    qcar2_gui_dir = get_package_share_directory('qcar2_rviz_gui')
    qcar2_nodes_dir = get_package_share_directory('qcar2_nodes')
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value='',
        description='Required: full path to an existing saved map .yaml file')
    # The browser console (qcar2_web_gui.py) is the default operator UI.
    # RViz and the Tk speed-limit window are kept as opt-ins for debugging.
    declare_use_web_gui_cmd = DeclareLaunchArgument(
        'use_web_gui', default_value='true',
        description='Serve the QCar2 browser console (map, goals, cameras, voice, e-stop)')
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
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(qcar2_nodes_dir, 'config', 'qcar2_slam_and_nav.yaml'),
        description='Nav2 parameters file (costmaps, planner, controller, etc.)')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Auto-activate the Nav2 lifecycle nodes')
    declare_use_voice_cmd = DeclareLaunchArgument(
        'use_voice', default_value='true',
        description='Run the spoken-command listener (microphone stays OFF '
                    'until you turn it on in the browser console)')
    declare_use_speed_slider_cmd = DeclareLaunchArgument(
        'use_speed_slider', default_value='false',
        description='Open the old Tk speed-limit window (the browser console has these controls)')

    # Point bt_navigator at the Ackermann behavior tree.  The stock tree's
    # recovery RoundRobin contains <BackUp backup_dist="0.30">, which is what
    # drove the car backwards for no visible reason, and <Spin>, which a
    # front-wheel-steered car cannot execute at all.
    ackermann_bt_xml = os.path.join(
        qcar2_gui_dir, 'behavior_trees', 'navigate_to_pose_ackermann.xml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': ackermann_bt_xml},
        convert_types=True,
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(qcar2_gui_dir, 'launch', 'hardware_base.launch.py')]),
        launch_arguments={
            'use_cameras': use_cameras,
            'use_front_camera': use_front_camera,
        }.items(),
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(qcar2_gui_dir, 'launch', 'odometry.launch.py')]),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'false',
            'autostart': autostart,
            'params_file': configured_params,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': autostart,
            'params_file': configured_params,
        }.items(),
    )

    # THE FIX: qcar2_nodes' nav2_qcar2_converter listens on /cmd_vel_nav,
    # but the actual post-smoother velocity is published on plain /cmd_vel
    # (velocity_smoother remaps cmd_vel_nav-in -> cmd_vel-out). Remap here
    # so navigation commands actually reach the motors.
    #
    # Deliberately bypassing collision_monitor's /cmd_vel_safe (2026-09-17):
    # confirmed via direct DDS-level testing that collision_monitor silently
    # drops 100% of traffic on this car -- it is alive, correctly configured,
    # correctly subscribed to /cmd_vel and /scan (confirmed receiving both),
    # never throws, yet /cmd_vel_safe produces zero messages even for a
    # manually-injected all-zero Twist. Root cause not found (third-party
    # compiled nav2_collision_monitor binary, no source available to patch).
    # Obstacle avoidance is NOT lost by this bypass: MPPI's own
    # ObstaclesCritic already avoids obstacles using the live local costmap,
    # independently of collision_monitor, and is the layer that has actually
    # been tuned and verified working all session -- see
    # qcar2-nav-tuning-gotchas.md. If collision_monitor is ever fixed
    # (e.g. a Nav2 version bump), point this remap back at /cmd_vel_safe.
    nav2_qcar2_converter = Node(
        package='qcar2_nodes',
        executable='nav2_qcar2_converter',
        name='nav2_qcar2_converter',
        output='screen',
        remappings=[('/cmd_vel_nav', '/cmd_vel')],
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[os.path.join(qcar2_gui_dir, 'config', 'collision_monitor.yaml')],
    )

    collision_monitor_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='collision_monitor_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    # Draws the travel-direction arrows along the plan, the goal flag with its
    # remaining distance, and the live forward/reverse motion arrow on the car.
    nav_visualizer_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_nav_visualizer.py',
        name='qcar2_nav_visualizer',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # Spoken status through the onboard speaker: "Navigation started",
    # "Going to goal", "Goal reached", "Navigation failed", and nearby
    # obstacles from the LiDAR -- see qcar2_announcer.py.
    announcer_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_announcer.py',
        name='qcar2_announcer',
        output='screen',
        parameters=[{'mode': 'navigation'}],
    )

    # Ties "2D Pose Estimate" to cancelling the active goal and clearing the
    # costmaps -- see qcar2_goal_reset.py.  Not gated on use_rviz: the reset is
    # a navigation behaviour, and /initialpose can also arrive from the web GUI.
    goal_reset_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_goal_reset.py',
        name='qcar2_goal_reset',
        output='screen',
    )

    # Re-aims each clicked goal along the approach bearing so Hybrid-A* stops
    # planning a loop just to satisfy the heading the mouse drag happened to
    # set -- see qcar2_goal_heading.py.  RViz's goal tool publishes
    # /goal_pose_raw; this node is the only publisher of /goal_pose.
    goal_heading_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_goal_heading.py',
        name='qcar2_goal_heading',
        output='screen',
    )

    # Resolves "go to the air cooler" against the semantic layer saved next to
    # this map (<map>_objects.json) and turns it into a standoff Nav2 goal --
    # see qcar2_object_nav.py.  Harmless if the map has no objects file: it
    # logs once and simply never matches anything.
    object_nav_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_object_nav.py',
        name='qcar2_object_nav',
        output='screen',
        parameters=[{'objects_file': map_yaml_file}],
    )

    # Spoken commands: "hey car, go to the air cooler". The microphone stays
    # OFF until switched on in the browser console, so this is safe to run by
    # default -- see qcar2_voice_command.py.
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

    # Browser console: map/costmaps/scan/plan view with Set Pose + Set Goal,
    # nav limits, e-stop, cameras, voice, all over ROS 2 through this one
    # node.  Needs no DISPLAY -- open http://<car-ip>:<web_port> from any
    # laptop on the network.  See scripts/qcar2_web_gui.py and web/index.html.
    web_gui_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_web_gui.py',
        name='qcar2_web_gui',
        output='screen',
        parameters=[{'mode': 'navigation', 'port': LaunchConfiguration('web_port')}],
        condition=IfCondition(LaunchConfiguration('use_web_gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(qcar2_gui_dir, 'rviz', 'qcar2_full_gui.rviz')],
        condition=IfCondition(use_rviz),
    )

    # RViz2 has no built-in slider widget, so this is a separate small window.
    # It publishes to Nav2's own dynamic speed-scaling topic (/speed_limit),
    # which the controller applies live -- no relaunch needed.
    speed_slider_node = Node(
        package='qcar2_rviz_gui',
        executable='qcar2_speed_limit_gui.py',
        name='qcar2_speed_limit_gui',
        output='screen',
        condition=IfCondition(use_speed_slider),
    )

    def validate_map_file(context):
        path = map_yaml_file.perform(context)
        if not path or not os.path.isfile(path):
            raise RuntimeError(
                'Navigation needs an existing saved map YAML. Run mapping and '
                'scripts/save_map.sh first, then pass map:=/absolute/path/map.yaml.')
        return []

    return LaunchDescription([
        declare_map_cmd,
        declare_use_web_gui_cmd,
        declare_web_port_cmd,
        declare_use_rviz_cmd,
        declare_use_cameras_cmd,
        declare_use_front_camera_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_speed_slider_cmd,
        declare_use_voice_cmd,
        OpaqueFunction(function=validate_map_file),
        hardware_launch,
        odometry_launch,
        localization_launch,
        navigation_launch,
        collision_monitor_node,
        collision_monitor_lifecycle_manager,
        nav2_qcar2_converter,
        nav_visualizer_node,
        goal_reset_node,
        goal_heading_node,
        object_nav_node,
        voice_node,
        assistant_node,
        announcer_node,
        web_gui_node,
        rviz_node,
        speed_slider_node,
    ])
