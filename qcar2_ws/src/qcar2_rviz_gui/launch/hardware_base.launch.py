"""
Core QCar2 hardware bring-up: motors/odometry-source hardware node, LiDAR,
the fixed base_link -> base_scan TF, and (optionally) the 4 CSI cameras
for the 360-degree view.

This is included by both mapping.launch.py and navigate.launch.py so the
hardware only has to be described once.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_cameras = LaunchConfiguration('use_cameras')
    use_front_camera = LaunchConfiguration('use_front_camera')
    standalone_front_camera = PythonExpression([
        "'", use_front_camera, "' == 'true' and '", use_cameras, "' != 'true'",
    ])

    declare_use_cameras_cmd = DeclareLaunchArgument(
        'use_cameras', default_value='false',
        description='Whether to start the 4 CSI cameras (360 view; optional)')
    declare_use_front_camera_cmd = DeclareLaunchArgument(
        'use_front_camera', default_value='false',
        description='Start one front CSI camera with a 5 Hz RViz preview')

    qcar2_hardware_node = Node(
        package='qcar2_nodes',
        executable='qcar2_hardware',
        name='qcar2_hardware',
        output='screen',
    )

    lidar_node = Node(
        package='qcar2_nodes',
        executable='lidar',
        name='Lidar',
        output='screen',
    )

    # Publishes the fixed base_link -> base_scan transform (lidar mount
    # offset). This is the one cartographer's qcar2_2d.lua config expects
    # (tracking_frame: base_scan), so we use it everywhere instead of an
    # ad-hoc static_transform_publisher.
    fixed_lidar_frame_node = Node(
        package='qcar2_nodes',
        executable='fixed_lidar_frame',
        name='fixed_lidar_frame',
        output='screen',
    )

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('qcar2_rviz_gui'),
            'launch', 'cameras_360.launch.py')]),
        condition=IfCondition(use_cameras),
    )

    robot_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('qcar2_rviz_gui'),
            'launch', 'robot_model.launch.py')]),
    )

    # ROOT CAUSE FOUND, 2026-09-15: this was never about camera_num, mode, the
    # daemon or a reboot -- every one of those was tried and failed
    # identically, because none of them touch the actual fault.
    #
    # Quanser's video_capture_open() gets its ISP-processed BGR output from
    # the same NVIDIA Argus/EGLStream path as the standard `nvarguscamerasrc`
    # GStreamer element, and that path needs a real, GPU-backed X display to
    # create its EGL context. Launching from an SSH/remote-desktop terminal
    # (this project is normally driven over SSH/VSCode Remote or xrdp) means
    # DISPLAY is whatever forwarded/virtual display that session created
    # (e.g. ":10.0"), not ":0", the actual local display the Tegra GPU is
    # attached to (confirmed via `who`: the "nvidia" user's physical seat0
    # session owns :0). Argus's EGLStream FrameConsumer::initialize() then
    # fails with a plain BadParameter, and Quanser's HIL layer surfaces that
    # as the misleading QERR_UNSUPPORTED_VIDEO_FORMAT ("the video format is
    # not supported... frame rate, frame size or native video formats") --
    # which has nothing to do with format at all.
    #
    # Proven two ways: `gst-launch-1.0 nvarguscamerasrc ...` reproduced the
    # exact same EGLStream failure under the inherited (wrong) DISPLAY, and
    # succeeded immediately -- full ISP demosaic, auto-exposure, real
    # colour -- once forced to DISPLAY=:0. Raw `v4l2-ctl` streaming (no EGL
    # needed) had already shown the sensor, kernel driver and every
    # documented native mode were fine all along.
    #
    # Fix: force DISPLAY to the real local display for JUST this process via
    # additional_env, rather than exporting it for the whole launch -- the Tk
    # GUIs (drive/speed-limit windows) already render correctly over the
    # inherited/forwarded display and must keep doing so for an operator
    # running this over SSH.  ":0" is this car's single physical seat
    # (confirmed above); if that ever changes, this is the line to update.
    front_camera_node = Node(
        package='qcar2_nodes', executable='csi', name='csi_front',
        namespace='front', output='screen', respawn=True, respawn_delay=2.0,
        condition=IfCondition(standalone_front_camera),
        # DISPLAY selects the Tegra-backed seat; XAUTHORITY permits this
        # child (started from SSH/xrdp) to connect to that seat's X server.
        additional_env={'DISPLAY': ':0', 'XAUTHORITY': '/home/nvidia/.Xauthority'},
        parameters=[{
            'camera_num': 2, 'device_type': 'physical',
            'frame_width': 820, 'frame_height': 616, 'frame_rate': 80.0,
        }],
    )
    front_preview_node = Node(
        package='qcar2_rviz_gui', executable='image_preview_throttle.py',
        name='front_camera_preview', output='screen', condition=IfCondition(use_front_camera),
        parameters=[{'preview_hz': 5.0}],
    )

    # CSI capture can fail when it is opened before the QCar hardware has
    # finished initializing.  Start it after the base nodes, and respawn it
    # above if the driver reports an open/start error.
    delayed_front_camera = TimerAction(
        period=3.0, actions=[front_camera_node, front_preview_node])

    return LaunchDescription([
        declare_use_cameras_cmd,
        declare_use_front_camera_cmd,
        qcar2_hardware_node,
        lidar_node,
        fixed_lidar_frame_node,
        robot_model_launch,
        cameras_launch,
        delayed_front_camera,
    ])
