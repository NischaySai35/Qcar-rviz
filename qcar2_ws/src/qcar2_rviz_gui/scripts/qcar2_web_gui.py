#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""QCar2 web GUI: the operator console, served from the car itself.

Replaces RViz + the two Tk windows.  ROS 2 remains the only transport: this
node is the sole thing the browser talks to, and it is a plain rclpy node on
the other side.  One process, two loops:

  * rclpy spins on a background thread and keeps the latest of everything in
    `State` under a lock (map, costmaps, scan in map frame, plan, nav status,
    battery, speed, steering, camera JPEGs).
  * aiohttp runs in the main thread: static page, one WebSocket per browser
    tab carrying JSON (big grids as base64), and MJPEG streams for cameras.

Nothing here needs a DISPLAY, so it works headless over the network.
"""

import asyncio
import base64
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from qcar2_interfaces.msg import MotorCommands
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int32, String, UInt8MultiArray
from tf2_ros import Buffer, TransformListener

PROJECT_DIR = os.path.join(os.path.expanduser('~'), 'Desktop', 'Qcar-rviz')
CAMERAS = ('front', 'rear', 'left', 'right')
NAV_STATUS_TEXT = {
    GoalStatus.STATUS_EXECUTING: 'NAVIGATING',
    GoalStatus.STATUS_SUCCEEDED: 'GOAL REACHED',
    GoalStatus.STATUS_ABORTED: 'NAV FAILED',
    GoalStatus.STATUS_CANCELED: 'GOAL CANCELLED',
    GoalStatus.STATUS_ACCEPTED: 'NAVIGATING',
}
# Keep in step with FollowPath.vx_max in qcar2_slam_and_nav.yaml.
NAV_MAX_SPEED = 0.45
# Mechanical steering stop (URDF hub joint limit); see nav2_qcar_command_convert.cpp.
STEERING_STOP_RAD = 0.5236

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

CAMERA_QOS = QoSProfile(depth=1,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE)


def yaw_to_quaternion(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_payload(msg):
    # Raw bytes, not base64: this dict is an internal representation only
    # (consumed by pack_grid() and sample_grid() below), never json.dumps'd
    # directly. Base64-in-JSON was previously used for the WebSocket wire
    # format too, which bloated every costmap update by ~33% on top of an
    # already-large full-grid resend and made delivery lag unpredictable
    # under any network pressure -- see pack_grid().
    data = np.asarray(msg.data, dtype=np.int8).view(np.uint8)
    return {
        'w': msg.info.width, 'h': msg.info.height, 'res': msg.info.resolution,
        'ox': msg.info.origin.position.x, 'oy': msg.info.origin.position.y,
        # yaw of this grid's origin, in MAP frame. 0 for map/gcost (already
        # published in map frame by Nav2). on_lcost() below overwrites this
        # -- see its comment for why the local costmap needs it at all.
        'yaw': 0.0,
        'data': data.tobytes(),
    }


def sample_grid(grid, x, y):
    """Cost at a world (x, y) in a grid_payload()'d costmap, or None if
    that point falls outside the grid or is still unmarked (255/unknown)."""
    yaw = grid.get('yaw', 0.0)
    dx, dy = x - grid['ox'], y - grid['oy']
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = c * dx + s * dy, -s * dx + c * dy   # world -> grid-local (inverse of the yaw rotation applied in pack_grid's consumers)
    ix, iy = int(lx / grid['res']), int(ly / grid['res'])
    if not (0 <= ix < grid['w'] and 0 <= iy < grid['h']):
        return None
    v = grid['data'][iy * grid['w'] + ix]
    return None if v == 255 else v


def write_map_files(grid, base, walls=()):
    """Save a grid_payload()'d /map as <base>.pgm + <base>.yaml, exactly as
    nav2's map_saver_cli does in its default trinary mode (same thresholds,
    same pixel values, same YAML keys), so navigation loads it unchanged.

    `walls` are extra map-frame (x, y) points written as occupied: the
    obstacles the car bumped into that the LiDAR never saw (glass, see
    qcar2_bump_guard.py). Cartographer maps glass as free floor, so without
    this a saved map sends navigation straight back into the same pane.

    Done here, from the copy this node already holds, because it is instant.
    Running map_saver_cli means starting a fresh ROS node and waiting for DDS
    discovery -- several seconds on this board, which is what made Ctrl+C
    slow when the map was saved on the way out.
    """
    w, h = grid['w'], grid['h']
    cells = np.frombuffer(grid['data'], dtype=np.uint8).reshape(h, w).astype(np.int16)
    cells[cells == 255] = -1                           # unknown
    for x, y in walls:
        ix = int((x - grid['ox']) / grid['res'])
        iy = int((y - grid['oy']) / grid['res'])
        if 0 <= ix < w and 0 <= iy < h:
            cells[iy, ix] = 100
    img = np.full((h, w), 205, dtype=np.uint8)         # map_saver: unknown
    img[(cells >= 0) & (cells <= 25)] = 254            # <= free_thresh 0.25
    img[cells >= 65] = 0                               # >= occupied_thresh 0.65
    img = img[::-1]                                    # grid row 0 is the BOTTOM row
    os.makedirs(os.path.dirname(base) or '.', exist_ok=True)
    # Write to a temp file and rename over the old one: autosave rewrites
    # these every few seconds, and a rename is atomic, so a Ctrl+C or a
    # reader mid-save sees either the old complete map or the new one --
    # never a half-written file.
    with open(base + '.pgm.tmp', 'wb') as fh:
        fh.write(f'P5\n# CREATOR: qcar2_web_gui {grid["res"]:.3f} m/pix\n{w} {h}\n255\n'.encode())
        fh.write(img.tobytes())
    os.replace(base + '.pgm.tmp', base + '.pgm')
    with open(base + '.yaml.tmp', 'w') as fh:
        fh.write(f'image: {os.path.basename(base)}.pgm\n'
                 f'mode: trinary\n'
                 f'resolution: {grid["res"]:.3f}\n'
                 f'origin: [{grid["ox"]:.3f}, {grid["oy"]:.3f}, 0]\n'
                 f'negate: 0\n'
                 f'occupied_thresh: 0.65\n'
                 f'free_thresh: 0.25\n')
    os.replace(base + '.yaml.tmp', base + '.yaml')


def pack_grid(tag, grid):
    """Binary wire format for a grid_payload(): 1-byte tag + w/h/res/ox/oy/yaw
    header, then the raw cost bytes with no base64/JSON wrapping at all.
    Sent via ws.send_bytes() instead of send_json() -- cuts payload size
    (no base64 33% bloat, no JSON string escaping) and, more importantly,
    removes the per-message string-encode/decode cost on both ends that
    made delivery time vary with server/browser load on top of network
    conditions."""
    header = struct.pack('<cHHffff', tag.encode('ascii'), grid['w'], grid['h'],
                         grid['res'], grid['ox'], grid['oy'], grid.get('yaw', 0.0))
    return header + grid['data']


class State:
    """Everything the browser can see, written by ROS callbacks, read by aiohttp."""

    def __init__(self):
        self.lock = threading.Lock()
        self.map = None
        self.map_v = 0
        self.gcost = None
        self.gcost_v = 0
        self.lcost = None
        self.lcost_v = 0
        self.scan_xy = None
        self.scan_v = 0
        self.plan = []
        self.plan_v = 0
        self.battery = float('nan')
        self.speed = 0.0
        self.steer_cmd = 0.0
        self.cmd_throttle = 0.0
        self.steer_wheels = float('nan')
        self.nav = 'IDLE'
        self.goal = None
        # id -> operator-facing text. Sticky: stays until the backend
        # condition that raised it clears, not just until the next tick.
        self.warnings = {}
        self.jpeg = {name: (None, 0) for name in CAMERAS}
        self.camera_stamp = {name: 0.0 for name in CAMERAS}
        # Semantic layer + the new subsystems' status, all published as JSON
        # strings by their owning nodes and forwarded to the browser verbatim.
        self.objects = []
        self.answers = []        # recent question/answer pairs for the console
        self.spoken = []         # recent sentences the speaker said, newest first
        self.detections = {}     # latest live boxes per camera (see on_detections)
        self.object_stats = {}
        self.voice = {'enabled': False, 'source': 'car', 'status': 'off',
                      'partial': '', 'final': '', 'action': '', 'error': ''}
        self.explore = None
        self.room = None         # qcar2_room_analyzer.py's latest verdict
        self.hints = None        # qcar2_explore_vlm.py's notes / closures / report
        self.detection_jpeg = (None, 0)


class WebGuiNode(Node):
    def __init__(self, state):
        super().__init__('qcar2_web_gui')
        self.state = state
        self.mode = self.declare_parameter('mode', 'navigation').value
        self.port = int(self.declare_parameter('port', 8080).value)
        self.open_browser = bool(self.declare_parameter('open_browser', True).value)
        # True when the main launch runs all four cameras (see side_cameras_external).
        self.cameras_owned = bool(self.declare_parameter('cameras_owned', False).value)
        # Image shown as the console's front camera. Mapping keeps the CSI
        # bumper camera; navigation passes the RealSense depth view
        # (/front/camera/depth_view, see qcar2_depth_view.py).
        self.front_topic = self.declare_parameter(
            'front_camera_topic', '/front/camera/csi_image').value
        # AUTOSAVE: rewrite maps/<autosave_name>.yaml/.pgm (+ _objects.json via
        # the object mapper) every autosave_period_sec, overwriting the same
        # files. The mapping scripts pass the name, so Ctrl+C -- which saves
        # nothing and just stops -- loses at most the last few seconds.
        # Empty name = no autosave.
        self.autosave_name = ''.join(
            ch for ch in str(self.declare_parameter('autosave_name', '').value)
            if ch.isalnum() or ch in '-_')
        autosave_period = float(self.declare_parameter('autosave_period_sec', 10.0).value)
        self._autosave_logged = False
        if self.autosave_name:
            self.create_timer(autosave_period, self.autosave)
        # True when Nav2 drives the motors through nav2_qcar2_converter: always
        # in navigation mode, and in mapping mode when auto-explore is on.
        self.nav2_drives = self.mode != 'mapping' or bool(
            self.declare_parameter('nav2_drives', False).value)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # ---- visual layers
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_gcost, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_lcost, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        # ---- telemetry
        self.create_subscription(BatteryState, '/qcar2_battery', self.on_battery, 1)
        # Wheel odometry is /odom in navigation but /wheel_imu_odom in
        # mapping (odometry.launch.py remaps it; mapping.launch.py does not).
        # Listening to /odom alone left speed stuck at 0 while mapping, so
        # any drive over 2.5 s raised a false MOTOR NOT RESPONDING.  Exactly
        # one of the two is published in either mode.
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/wheel_imu_odom', self.on_odom, 10)
        self.create_subscription(MotorCommands, '/qcar2_motor_speed_cmd', self.on_motor_cmd, 10)
        self.create_subscription(Vector3, '/qcar2/steering_report', self.on_steering_report, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_nav_status, LATCHED)
        self.create_subscription(Empty, '/qcar2/nav_reset', self.on_nav_reset, 1)
        # The front CSI stream is deliberately consumed directly.  The old
        # browser path depended on a separate preview-relay process; if that
        # small process failed, the camera was physically working but the UI
        # stayed black.  Sampling JPEG conversion below keeps the browser at
        # 5 Hz without making the relay a single point of failure.  The side
        # cameras remain on their 5 Hz preview topics because all four raw
        # 80 Hz streams would be needless CPU load.
        for name in CAMERAS:
            topic = self.front_topic if name == 'front' else f'/{name}/camera/preview'
            self.create_subscription(
                Image, topic,
                lambda msg, name=name: self.on_image(name, msg), CAMERA_QOS)

        # ---- semantic map, voice and exploration status in
        self.create_subscription(String, '/qcar2/objects', self.on_objects, LATCHED)
        self.create_subscription(String, '/qcar2/answer', self.on_answer, 10)
        self.create_subscription(String, '/qcar2/spoken', self.on_spoken, 10)
        self.create_subscription(String, '/qcar2/detections', self.on_detections, 10)
        self.create_subscription(String, '/qcar2/voice_transcript', self.on_voice_transcript, 10)
        self.create_subscription(String, '/qcar2/explore_status', self.on_explore_status, LATCHED)
        self.create_subscription(String, '/qcar2/nav_object_status', self.on_object_nav_status, 10)
        self.create_subscription(Image, '/qcar2/detection_image',
                                 self.on_detection_image, CAMERA_QOS)
        # The voice node can engage the e-stop on its own ("hey car, stop"),
        # so reflect the real topic rather than only this node's own copy --
        # otherwise the panel button would show OFF while the car is halted.
        self.create_subscription(Bool, '/qcar2_estop', self.on_estop_feedback, LATCHED)
        # Obstacles the car physically hit but the LiDAR cannot see (glass) --
        # see qcar2_bump_guard.py. Kept so a saved map includes them as walls.
        self.bump_points = []
        self.drive_blocked = False
        self.create_subscription(PointCloud2, '/qcar2/bump_obstacles', self.on_bumps, LATCHED)
        self.create_subscription(Bool, '/qcar2/drive_blocked', self.on_drive_blocked, LATCHED)
        # Why the car is holding still -- see check_warnings().
        self.clearance, self.clearance_at = (99.0, 99.0), 0.0
        self.drive_stalled = False
        self.create_subscription(Float32MultiArray, '/qcar2/obstacle_clearance',
                                 self.on_clearance, 10)
        self.create_subscription(Bool, '/qcar2/drive_stalled', self.on_drive_stalled, LATCHED)
        # Room awareness for auto-explore: the analyzer's outline/coverage
        # and the vision model's notes, report and closures (with undo).
        self.create_subscription(String, '/qcar2/room_status', self.on_room_status, LATCHED)
        self.create_subscription(String, '/qcar2/explore_hints', self.on_explore_hints, LATCHED)
        self.hints_clear_pub = self.create_publisher(String, '/qcar2/explore_hints_clear', 10)

        # ---- commands out
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose_raw', 1)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 1)
        self.estop_pub = self.create_publisher(Bool, '/qcar2_estop', LATCHED)
        self.voice_enabled_pub = self.create_publisher(Bool, '/qcar2/voice_enabled', LATCHED)
        self.voice_volume_pub = self.create_publisher(Int32, '/qcar2/voice_volume', LATCHED)
        # Microphone (speech IN) -- entirely separate from voice (speech OUT).
        self.mic_enabled_pub = self.create_publisher(Bool, '/qcar2/mic_enabled', LATCHED)
        self.mic_source_pub = self.create_publisher(String, '/qcar2/mic_source', LATCHED)
        self.mic_audio_pub = self.create_publisher(UInt8MultiArray, '/qcar2/mic_audio', 10)
        self.nav_to_object_pub = self.create_publisher(String, '/qcar2/nav_to_object', 10)
        self.explore_enabled_pub = self.create_publisher(Bool, '/qcar2/explore_enabled', LATCHED)
        self.save_objects_pub = self.create_publisher(String, '/qcar2/save_objects', 10)
        # Typed questions to qcar2_assistant.py. Same path the voice node
        # uses, so typing and speaking behave identically.
        self.ask_pub = self.create_publisher(String, '/qcar2/ask', 10)
        self.focus_pub = self.create_publisher(String, '/qcar2/camera_focus', 10)
        self.view_360 = False
        self.motor_pub = self.create_publisher(MotorCommands, '/qcar2_motor_speed_cmd', 10)
        # Manual arrow-pad override in NAVIGATION mode (mapping mode still
        # uses motor_pub directly above -- nav2_qcar2_converter isn't even
        # running there).  Routed through the converter, the single writer
        # to qcar2_motor_speed_cmd while Nav2 is up, instead of racing it --
        # see nav2_qcar_command_convert.cpp's manual-override handling.
        # Vector3 reused the same way steering_report is: x = steering angle
        # (rad), y = throttle, NOT a Twist.
        self.manual_drive_pub = self.create_publisher(Vector3, '/qcar2/manual_drive_cmd', 10)
        self.manual_drive_active_pub = self.create_publisher(Bool, '/qcar2/manual_drive_active', 10)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.steering_param_client = self.create_client(
            SetParameters, '/nav2_qcar2_command_converter/set_parameters')

        # ---- operator settings (published latched so late starters get them)
        self.voice_on = False   # muted until the operator opts in
        self.voice_volume = 50
        # Microphone starts OFF: an always-listening mic must be opt-in, and
        # the operator should never discover it was hot without asking.
        self.mic_on = False
        self.mic_source = 'car'
        self.speed_pct = 100
        self.steer_limit = STEERING_STOP_RAD
        self.estop = False
        # AMCL starts at (0, 0) only so that the saved map can be rendered.
        # That seed is not localization.  A goal issued before Set Pose can
        # look valid in the UI while referring to the wrong physical place.
        self.initial_pose_set = self.mode != 'navigation'
        self.publish_voice()
        self.publish_mic()
        self.publish_estop()

        # ---- manual drive (plain mapping only).  The hardware latches the
        # last motor command, so zeros are published continuously whenever
        # nothing is held, and a hold without a heartbeat for 0.4 s is dropped.
        #
        # NOT when Nav2 drives (auto-explore). Those idle zeros went to the
        # same /qcar2_motor_speed_cmd the converter was writing Nav2's speed
        # to: 20 zeros a second interleaved with 50 "go"s, and every zero
        # reset qcar2_hardware.cpp's throttle integrator, so it never built
        # enough throttle to break static friction. That is why auto mapping
        # "planned a path but never moved" -- every goal ended in "Failed to
        # make progress" after exactly 5 s. The converter is the single
        # writer then, and the pad uses its manual-override input instead.
        self.drive = {'speed': 0.0, 'steering': 0.0, 'held': False, 'last': 0.0}
        if not self.nav2_drives:
            self.create_timer(0.05, self.drive_tick)

        self.camera_process = None

        # ---- standing operator warnings (persistent banners, not toasts)
        self._stall_since = None
        self.create_timer(0.2, self.check_warnings)

    # ------------------------------------------------------------ callbacks

    def on_map(self, msg):
        with self.state.lock:
            self.state.map = grid_payload(msg)
            self.state.map_v += 1

    def on_gcost(self, msg):
        with self.state.lock:
            self.state.gcost = grid_payload(msg)
            self.state.gcost_v += 1

    def on_lcost(self, msg):
        # The local costmap's global_frame is "odom" (kept separate from
        # map so AMCL corrections don't yank the rolling window around),
        # so msg.info.origin is in ODOM-frame coordinates -- NOT map frame.
        # This was previously sent to the browser as-is and drawn straight
        # onto the map view, which is only correct by coincidence when
        # map->odom happens to be near-identity (e.g. right after boot).
        # The instant AMCL applies a real correction (any meaningful Set
        # Pose), the local costmap patch renders at its stale odom-relative
        # position instead of following the robot -- confirmed live: odom
        # stayed at (0,0,0) the whole time (car never physically moved),
        # while map->odom carried a -3.36 m, ~178 deg correction, and the
        # local costmap patch sat frozen at map (0,0) throughout. This is
        # what showed up as "the old red boundary never goes away".
        payload = grid_payload(msg)
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time())
            yaw_tf = quaternion_to_yaw(tf.transform.rotation)
            origin_yaw = quaternion_to_yaw(msg.info.origin.orientation)
            ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
            c, s = math.cos(yaw_tf), math.sin(yaw_tf)
            payload['ox'] = tf.transform.translation.x + c * ox - s * oy
            payload['oy'] = tf.transform.translation.y + s * ox + c * oy
            payload['yaw'] = yaw_tf + origin_yaw
        except Exception:  # noqa: BLE001 - TF momentarily unavailable; retry next message rather than drop this one entirely
            pass
        with self.state.lock:
            self.state.lcost = payload
            self.state.lcost_v += 1

    def on_scan(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - no TF yet, draw nothing rather than garbage
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        r, a = ranges[ok], angles[ok]
        lx, ly = r * np.cos(a), r * np.sin(a)
        yaw = quaternion_to_yaw(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        mx = tf.transform.translation.x + c * lx - s * ly
        my = tf.transform.translation.y + s * lx + c * ly
        xy = np.empty(len(mx) * 2, dtype=np.float32)
        xy[0::2], xy[1::2] = mx, my
        with self.state.lock:
            self.state.scan_xy = base64.b64encode(xy.tobytes()).decode('ascii')
            self.state.scan_v += 1

    def on_plan(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        step = max(1, len(pts) // 600)
        with self.state.lock:
            self.state.plan = pts[::step]
            self.state.plan_v += 1

    def on_battery(self, msg):
        self.state.battery = msg.voltage

    def on_odom(self, msg):
        self.state.speed = msg.twist.twist.linear.x

    def on_motor_cmd(self, msg):
        try:
            self.state.steer_cmd = msg.values[msg.motor_names.index('steering_angle')]
        except (ValueError, IndexError):
            pass
        try:
            self.state.cmd_throttle = msg.values[msg.motor_names.index('motor_throttle')]
        except (ValueError, IndexError):
            pass

    def on_steering_report(self, msg):
        self.state.steer_wheels = msg.z

    # -------------------------------------------------------- diagnostics

    NEAR_OBSTACLE_COST = 90

    def check_warnings(self):
        """Turn silent Nav2/hardware failure modes into a sticky banner
        instead of a car that just does nothing with no explanation --
        see the near-obstacle-start and collision_monitor-swallowing-
        commands cases that motivated this."""
        s = self.state
        now = time.monotonic()
        warnings = {}

        # Commanded to move but the wheels aren't responding: covers
        # e-stop, a sagging battery, and a broken cmd_vel_safe relay
        # alike without having to diagnose which one from here.
        if self.drive_blocked:
            # Explains the stall below, so it replaces that banner.
            warnings['bumped'] = (
                'BUMPED INTO SOMETHING THE LIDAR CANNOT SEE (glass or a low '
                'object) — throttle cut, marked as an obstacle, backing off.')
            self._stall_since = None
        elif abs(s.cmd_throttle) > 0.05 and abs(s.speed) < 0.02:
            # Before calling it a motor fault, rule out the two DELIBERATE
            # holds -- otherwise the banner cries "fault" every time the car
            # correctly refuses to drive into something (2026-10-06 runs).
            ahead = s.cmd_throttle > 0
            room = None
            if now - self.clearance_at < 0.5:
                room = self.clearance[0 if ahead else 1]
            if self.drive_stalled:
                warnings['stalled'] = (
                    'DRIVE BLOCKED — the wheels could not turn, so the throttle is cut. '
                    'It frees itself by reversing or stopping.')
                self._stall_since = None
            elif room is not None and room < 0.12:
                # Informational, not a fault: the obstacle slow-down at work.
                warnings['holding'] = (
                    f'HOLDING — obstacle {room * 100:.0f} cm '
                    f'{"ahead" if ahead else "behind"}; the car will not drive '
                    'into it. Waiting for a way around.')
                self._stall_since = None
            elif self._stall_since is None:
                self._stall_since = now
            elif now - self._stall_since > 2.5:
                # 2.5 s, not 1.5: the smooth throttle ramp on the slippery
                # tiles legitimately takes a moment to get rolling.
                warnings['motor_stall'] = (
                    'MOTOR NOT RESPONDING — commanded to move but the '
                    'wheels are not turning. Check the e-stop, the battery, '
                    'and that nothing is holding the drive.')
        else:
            self._stall_since = None

        # Start position too close to an obstacle for the planner to
        # accept -- it will silently return an empty path forever.
        if self.mode == 'navigation' and s.lcost is not None:
            pose = self.robot_pose()
            if pose is not None:
                cost = sample_grid(s.lcost, pose['x'], pose['y'])
                if cost is not None and cost >= self.NEAR_OBSTACLE_COST:
                    warnings['near_obstacle'] = (
                        'TOO CLOSE TO AN OBSTACLE — Nav2 cannot plan a '
                        'path starting this close to a wall. Move the car '
                        'away, then set the goal again.')

        with s.lock:
            if warnings != s.warnings:
                s.warnings = warnings

    def on_nav_status(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        text = NAV_STATUS_TEXT.get(status)
        if text is None:
            return
        with self.state.lock:
            self.state.nav = text
            if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                          GoalStatus.STATUS_CANCELED):
                self.state.goal = None
                self.state.plan = []
                self.state.plan_v += 1

    def on_nav_reset(self, _msg):
        with self.state.lock:
            self.state.goal = None
            self.state.plan = []
            self.state.plan_v += 1
            self.state.nav = 'IDLE'

    def on_image(self, name, msg):
        # The Quanser CSI driver publishes bgr8.  Keep rgb8 support as a
        # harmless fallback for a standard ROS camera driver, but reject
        # layouts we cannot safely reshape.
        if msg.encoding.lower() not in ('bgr8', 'rgb8') or msg.step < msg.width * 3:
            return
        now = time.monotonic()
        if now - self.state.camera_stamp[name] < 0.2:
            return
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step // 3, 3)[:, :msg.width]
        except ValueError:
            self.get_logger().warn(
                f'Ignoring malformed {name} image ({msg.width}x{msg.height}, step {msg.step}).')
            return
        if msg.encoding.lower() == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self.state.lock:
                _, version = self.state.jpeg[name]
                self.state.jpeg[name] = (jpg.tobytes(), version + 1)
                self.state.camera_stamp[name] = now

    # ------------------------------------------------------------- queries

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        return {'x': tf.transform.translation.x, 'y': tf.transform.translation.y,
                'yaw': quaternion_to_yaw(tf.transform.rotation)}

    # ------------------------------------------------------------ commands

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = yaw_to_quaternion(yaw)
        # Same covariance RViz's 2D Pose Estimate tool sends.
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initialpose_pub.publish(msg)
        self.initial_pose_set = True
        with self.state.lock:
            self.state.goal = None

    def set_goal(self, x, y, yaw):
        if self.mode == 'navigation' and not self.initial_pose_set:
            return False
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = yaw_to_quaternion(yaw)
        self.goal_pub.publish(msg)
        with self.state.lock:
            self.state.goal = {'x': x, 'y': y, 'yaw': yaw}
            self.state.nav = 'NAVIGATING'
        return True

    def cancel_goal(self):
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())

    def set_speed_limit(self, pct):
        self.speed_pct = max(10, min(int(pct), 150))
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = True
        msg.speed_limit = float(self.speed_pct)
        self.speed_limit_pub.publish(msg)

    def set_steer_limit(self, rad):
        self.steer_limit = max(0.2, min(float(rad), STEERING_STOP_RAD))
        if not self.steering_param_client.service_is_ready():
            return False
        request = SetParameters.Request()
        parameter = ParameterMsg()
        parameter.name = 'max_steering_rad'
        parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                         double_value=self.steer_limit)
        request.parameters = [parameter]
        self.steering_param_client.call_async(request)
        return True

    def publish_estop(self):
        msg = Bool()
        msg.data = self.estop
        self.estop_pub.publish(msg)

    def set_estop(self, engaged):
        self.estop = bool(engaged)
        self.publish_estop()
        if self.estop:
            self.cancel_goal()
            self.drive['held'] = False

    def publish_voice(self):
        enabled = Bool()
        enabled.data = self.voice_on
        self.voice_enabled_pub.publish(enabled)
        volume = Int32()
        volume.data = int(self.voice_volume)
        self.voice_volume_pub.publish(volume)

    def set_voice(self, on, volume):
        self.voice_on = bool(on)
        self.voice_volume = max(0, min(int(volume), 100))
        self.publish_voice()

    # ------------------------------------------------- microphone / objects

    def publish_mic(self):
        enabled = Bool()
        enabled.data = self.mic_on
        self.mic_enabled_pub.publish(enabled)
        source = String()
        source.data = self.mic_source
        self.mic_source_pub.publish(source)

    def set_mic(self, on, source):
        self.mic_on = bool(on)
        if source in ('car', 'browser'):
            self.mic_source = source
        self.publish_mic()
        self.get_logger().info(
            f'microphone {"ON" if self.mic_on else "OFF"} ({self.mic_source})')

    def feed_mic_audio(self, data):
        """Raw 16 kHz mono s16le from the browser -> the voice recogniser.

        Only forwarded when the browser is the selected source, so a tab left
        open cannot inject audio while the car's own mic is in use.
        """
        if not self.mic_on or self.mic_source != 'browser':
            return
        msg = UInt8MultiArray()
        msg.data = list(data) if not isinstance(data, list) else data
        self.mic_audio_pub.publish(msg)

    def go_to_object(self, name):
        name = (name or '').strip()
        if not name:
            return False
        msg = String()
        msg.data = name
        self.nav_to_object_pub.publish(msg)
        return True

    def set_explore(self, on):
        msg = Bool()
        msg.data = bool(on)
        self.explore_enabled_pub.publish(msg)

    def on_objects(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.state.lock:
            self.state.objects = data.get('objects') or []
            self.state.object_stats = data.get('stats') or {}

    def ask(self, question):
        question = (question or '').strip()
        if not question:
            return False
        msg = String()
        msg.data = question
        self.ask_pub.publish(msg)
        return True

    def on_bumps(self, msg):
        pts = [(float(x), float(y)) for x, y, _z in
               point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        with self.state.lock:
            self.bump_points = pts

    def on_drive_blocked(self, msg):
        self.drive_blocked = bool(msg.data)

    def on_clearance(self, msg):
        if len(msg.data) >= 2:
            self.clearance = (float(msg.data[0]), float(msg.data[1]))
            self.clearance_at = time.monotonic()

    def on_drive_stalled(self, msg):
        self.drive_stalled = bool(msg.data)

    def on_room_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.state.lock:
            self.state.room = data

    def on_explore_hints(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.state.lock:
            self.state.hints = data

    def clear_hints(self, what):
        msg = String()
        msg.data = what if what in ('glass', 'doorway', 'skips', 'all') else 'all'
        self.hints_clear_pub.publish(msg)

    def on_detections(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        cameras = data.get('cameras') or {}
        # The detector's 'front' boxes are measured on the CSI bumper camera.
        # When the front pane shows the RealSense instead (a different lens,
        # field of view and mounting), those boxes would land on the wrong
        # things, so they are not drawn there; the RealSense view carries its
        # own distance readouts.
        if self.front_topic != '/front/camera/csi_image':
            cameras = {k: v for k, v in cameras.items() if k != 'front'}
        with self.state.lock:
            self.state.detections = cameras
            self.state.detections_at = time.monotonic()

    def on_answer(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.state.lock:
            # Keep a short history so the panel reads as a conversation
            # rather than a single flashing line.
            self.state.answers = ([{'q': data.get('question', ''),
                                    'a': data.get('answer', '')}]
                                  + self.state.answers)[:6]

    def on_spoken(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.state.lock:
            self.state.spoken = ([data] + self.state.spoken)[:10]

    def on_voice_transcript(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.state.lock:
            self.state.voice = data

    def on_explore_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.state.lock:
            self.state.explore = data

    def on_object_nav_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        # Surface the failure reason the same way other backend problems are
        # surfaced, so "I do not know that object" reaches the operator
        # instead of only the log.
        with self.state.lock:
            if data.get('ok'):
                self.state.warnings.pop('object_nav', None)
            else:
                self.state.warnings['object_nav'] = data.get('reason', 'Object goal failed')

    def on_estop_feedback(self, msg):
        # Mirrors an e-stop engaged by anyone (e.g. "hey car, stop").
        self.estop = bool(msg.data)

    def on_detection_image(self, msg):
        """The object mapper's annotated view, streamed like a camera.

        Worth having its own stream: it is the one place you can see WHY a
        detection did or did not become a landmark (green box = placed on the
        map, amber = seen but no LiDAR return in its sector).
        """
        if msg.encoding.lower() != 'bgr8' or msg.step < msg.width * 3:
            return
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step // 3, 3)[:, :msg.width]
        except ValueError:
            return
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self.state.lock:
                self.state.detection_jpeg = (jpg.tobytes(), self.state.detection_jpeg[1] + 1)

    def set_drive(self, speed, steering):
        if self.estop:
            return
        speed, steering = float(speed), float(steering)
        held = bool(speed or steering)
        if not self.nav2_drives:
            self.drive.update(speed=speed, steering=steering, held=held, last=time.monotonic())
        else:
            # Nav2 is driving: highest-priority manual override.  Published
            # straight through to nav2_qcar2_converter, which substitutes
            # this for whatever Nav2 is asking for as long as 'active' stays
            # true -- Nav2's goal is never cancelled, so releasing the
            # button (held=False below) hands control straight back to
            # wherever the car ended up, not back to the start.
            cmd = Vector3()
            cmd.x, cmd.y = steering, speed
            self.manual_drive_pub.publish(cmd)
            active = Bool()
            active.data = held
            self.manual_drive_active_pub.publish(active)
            self.drive['held'] = held

    def drive_tick(self):
        if self.drive['held'] and time.monotonic() - self.drive['last'] > 0.4:
            self.drive['held'] = False
        msg = MotorCommands()
        msg.motor_names = ['steering_angle', 'motor_throttle']
        if self.drive['held'] and not self.estop:
            msg.values = [self.drive['steering'], self.drive['speed']]
        else:
            msg.values = [0.0, 0.0]
        self.motor_pub.publish(msg)

    def side_cameras_external(self):
        """Are rear/left/right already running from the main launch?

        In mapping mode with object detection on, mapping.launch.py starts all
        four cameras itself. Launching cameras_side.launch.py on top of that
        would start a SECOND csi node for each side camera, and the Quanser
        driver cannot open the same camera twice -- the duplicates fail and
        respawn in a loop behind a view that looks fine.
        """
        # The launch file says so outright when it owns the cameras. Counting
        # publishers is only the fallback: a camera mid-restart counts as 0,
        # and that is exactly how duplicates got started and the two sets of
        # camera nodes ended up fighting over the devices in a restart loop.
        if self.cameras_owned:
            return True
        own = self.camera_process is not None and self.camera_process.poll() is None
        return (not own) and self.count_publishers('/rear/camera/csi_image') > 0

    def set_cameras_360(self, on):
        self.view_360 = bool(on)
        # Tell the object mapper which view is on screen, so the camera being
        # watched gets more detection slots (see its schedule()).
        focus = String()
        focus.data = 'all' if on else 'front'
        self.focus_pub.publish(focus)
        running = self.camera_process is not None and self.camera_process.poll() is None
        if on and not running:
            args = ['ros2', 'launch', 'qcar2_rviz_gui', 'cameras_side.launch.py']
            if self.side_cameras_external():
                # Cameras already streaming (mapping with detection): start
                # only the 5 Hz preview relays this console reads from.
                # Skipping the launch entirely left the 360 view blank;
                # launching it whole started duplicate camera nodes.
                args.append('start_cameras:=false')
            self.camera_process = subprocess.Popen(args)
        elif not on and running:
            # SIGINT so the CSI driver releases the cameras cleanly.
            self.camera_process.send_signal(2)
        return on

    def cameras_360_running(self):
        return self.camera_process is not None and self.camera_process.poll() is None

    def autosave(self):
        with self.state.lock:
            if self.state.map is None:
                return                          # nothing mapped yet
        ok, detail = self.save_map(self.autosave_name, quiet=True)
        if not ok:
            self.get_logger().warn(f'autosave failed: {detail}')
        elif not self._autosave_logged:
            # Once, not every 10 s -- the terminal should stay readable.
            self._autosave_logged = True
            self.get_logger().info(
                f'Autosaving to maps/{self.autosave_name}.yaml (+ objects) every few seconds.')

    def save_map(self, name, quiet=False):
        """Map (.pgm/.yaml) from this node's own copy of /map, plus a request
        to the object mapper for <name>_objects.json. Instant -- see
        write_map_files(). Used by the Save Map button and by autosave."""
        name = ''.join(ch for ch in name if ch.isalnum() or ch in '-_') or 'qcar_map'
        with self.state.lock:
            grid = self.state.map
            walls = list(self.bump_points)
        if grid is None:
            return False, 'No map received yet -- nothing to save.'
        try:
            write_map_files(grid, os.path.join(PROJECT_DIR, 'maps', name), walls)
        except OSError as exc:
            return False, f'Could not write maps/{name}: {exc}'
        msg = String()
        msg.data = name
        self.save_objects_pub.publish(msg)
        if not quiet:
            self.get_logger().info(f'Saved map: maps/{name}.yaml + .pgm')
        return True, f'Saved maps/{name}.yaml'

    def stop_all(self):
        self.set_estop(True)
        subprocess.Popen([os.path.join(PROJECT_DIR, 'scripts', 'stop.sh')])

    def shutdown_cameras(self):
        if self.cameras_360_running():
            self.camera_process.send_signal(2)
            try:
                self.camera_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass


# ==================================================================== web

class WebServer:
    def __init__(self, node, state):
        self.node = node
        self.state = state
        self.clients = set()
        self.web_dir = os.path.join(get_package_share_directory('qcar2_rviz_gui'), 'web')
        self.app = web.Application()
        self.app.router.add_get('/', self.index)
        self.app.router.add_get('/ws', self.websocket)
        self.app.router.add_get('/control', self.control_websocket)
        self.app.router.add_get('/camera/{name}.mjpg', self.mjpeg)
        # colcon's symlink install also links the browser vendor bundle.
        self.app.router.add_static('/static', self.web_dir, follow_symlinks=True)
        # Real QCar2 STL meshes for the 3D view, straight from the qcar2
        # package's installed share dir -- no need to duplicate them into web/.
        try:
            meshes_dir = os.path.join(get_package_share_directory('qcar2'), 'meshes')
            # colcon's symlink install leaves each STL as a symlink into src/.
            # aiohttp otherwise returns 404 for those valid package assets.
            self.app.router.add_static('/mesh', meshes_dir, follow_symlinks=True)
        except Exception as error:  # noqa: BLE001 - 3D view just won't load meshes
            node.get_logger().warn(f'qcar2 meshes not found, 3D view will be body-only: {error}')

    async def index(self, _request):
        return web.FileResponse(os.path.join(self.web_dir, 'index.html'),
                                headers={'Cache-Control': 'no-store'})

    # ---------------------------------------------------------- websocket

    def hello(self):
        n = self.node
        return {'t': 'hello', 'mode': n.mode,
                'voice': {'on': n.voice_on, 'vol': n.voice_volume},
                'cams360': n.cameras_360_running(),
                'speed_pct': n.speed_pct, 'steer_limit': n.steer_limit,
                'steer_stop': STEERING_STOP_RAD, 'nav_max_speed': NAV_MAX_SPEED,
                'estop': n.estop, 'localized': n.initial_pose_set,
                'mic': {'on': n.mic_on, 'source': n.mic_source}}

    def state_message(self):
        s, n = self.state, self.node
        with s.lock:
            return {'t': 'state', 'pose': n.robot_pose(), 'speed': s.speed,
                    'steer_cmd': s.steer_cmd,
                    'steer_wheels': None if math.isnan(s.steer_wheels) else s.steer_wheels,
                    'battery': None if math.isnan(s.battery) else s.battery,
                    'nav': s.nav, 'goal': s.goal, 'estop': n.estop,
                    'cams360': n.cameras_360_running(),
                    'voice': {'on': n.voice_on, 'vol': n.voice_volume},
                    'drive_held': n.drive['held'], 'localized': n.initial_pose_set,
                    'warnings': s.warnings,
                    'mic': {'on': n.mic_on, 'source': n.mic_source},
                    'objects': s.objects, 'object_stats': s.object_stats,
                    'answers': s.answers, 'spoken': s.spoken,
                    # Live boxes, dropped entirely once stale so a stopped
                    # detector cannot leave frozen boxes drawn on live video.
                    'detections': (s.detections if time.monotonic() -
                                   getattr(s, 'detections_at', 0.0) < 3.0 else {}),
                    'voice_in': s.voice, 'explore': s.explore,
                    'room': s.room, 'hints': s.hints}

    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=0)
        await ws.prepare(request)
        self.clients.add(ws)
        versions = {'map': -1, 'gcost': -1, 'lcost': -1, 'scan': -1, 'plan': -1}
        try:
            await ws.send_json(self.hello())
            sender = asyncio.ensure_future(self.push_loop(ws, versions))
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self.handle(ws, json.loads(msg.data))
                elif msg.type == WSMsgType.BINARY:
                    # Browser microphone: 16 kHz mono s16le PCM. Binary rather
                    # than base64-in-JSON because this is a continuous stream,
                    # and the 33% base64 overhead on every chunk would show up
                    # as recognition lag.
                    self.node.feed_mic_audio(msg.data)
                elif msg.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                    break
        finally:
            sender.cancel()
            self.clients.discard(ws)
            # A tab that vanished mid-hold must not leave the car driving.
            self.node.drive['held'] = False
        return ws

    async def control_websocket(self, request):
        """Low-latency command channel, kept separate from telemetry payloads."""
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=0)
        await ws.prepare(request)
        self.clients.add(ws)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self.handle(ws, json.loads(msg.data))
                elif msg.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                    break
        finally:
            self.clients.discard(ws)
            self.node.drive['held'] = False
            self.node.drive['speed'] = 0.0
            self.node.drive['steering'] = 0.0
        return ws

    async def push_loop(self, ws, versions):
        tick = 0
        while not ws.closed:
            s = self.state
            with s.lock:
                # Small, time-critical updates first; the large grids last.
                # All of this shares one ordered TCP stream, so whatever
                # goes out first is what stays fresh when the link can't
                # keep up -- previously the (rarely-changing, ~100KB)
                # global costmap could sit ahead of the live scan/pose in
                # the queue and delay them by however long it took to
                # drain, which is exactly what made the LiDAR/costmap
                # relationship look inconsistent: sometimes in sync,
                # sometimes the costmap trailing behind, sometimes a stale
                # frame still showing, all depending on network timing.
                small = []
                if s.scan_xy is not None and s.scan_v != versions['scan']:
                    versions['scan'] = s.scan_v
                    small.append({'t': 'scan', 'xy': s.scan_xy})
                if s.plan_v != versions['plan']:
                    versions['plan'] = s.plan_v
                    small.append({'t': 'plan', 'pts': s.plan})
                grids = []
                if s.lcost is not None and s.lcost_v != versions['lcost']:
                    versions['lcost'] = s.lcost_v
                    grids.append(pack_grid('L', s.lcost))
                if s.gcost is not None and s.gcost_v != versions['gcost']:
                    versions['gcost'] = s.gcost_v
                    grids.append(pack_grid('G', s.gcost))
                if s.map is not None and s.map_v != versions['map']:
                    versions['map'] = s.map_v
                    grids.append(pack_grid('M', s.map))
            # state_message() takes s.lock itself -- must run after the
            # block above has released it, not inside it (threading.Lock
            # is not reentrant; nesting these deadlocked the whole event
            # loop on tick 0 the first time this was tried).
            if tick % 2 == 0:
                state = self.state_message()
                # The room analysis (outline + every gap) and the vision
                # notes are ~10 KB and change every ~2 s: send them at 2 Hz,
                # not with every 10 Hz state, so they cannot crowd the link.
                if tick % 10 != 0:
                    state.pop('room', None)
                    state.pop('hints', None)
                small.insert(0, state)
            for update in small:
                await ws.send_json(update)
            for payload in grids:
                await ws.send_bytes(payload)
            tick += 1
            await asyncio.sleep(0.05)

    async def toast(self, ws, text, kind='info'):
        await ws.send_json({'t': 'toast', 'msg': text, 'kind': kind})

    async def ack(self, ws, cmd, **extra):
        """Confirm a control command reached the car (the browser shows it),
        so a click that got lost is noticed instead of silently ignored."""
        try:
            await ws.send_json({'t': 'ack', 'cmd': cmd, **extra})
        except Exception:                    # noqa: BLE001 - socket closing
            pass

    async def handle(self, ws, cmd):
        n, t = self.node, cmd.get('t')
        if t == 'initialpose':
            n.set_initial_pose(cmd['x'], cmd['y'], cmd['yaw'])
            await self.toast(ws, 'Initial pose set')
        elif t == 'goal':
            if n.mode != 'navigation':
                await self.toast(ws, 'Goals need navigation mode', 'warn')
            elif not n.set_goal(cmd['x'], cmd['y'], cmd['yaw']):
                await self.toast(ws, 'Set Pose on the car\'s real map position before sending a goal.', 'warn')
            else:
                await self.toast(ws, 'Goal sent')
        elif t == 'cancel':
            n.cancel_goal()
            await self.ack(ws, t)
        elif t == 'estop':
            n.set_estop(cmd.get('on', True))
            await self.ack(ws, t, on=n.estop)
        elif t == 'stopall':
            n.stop_all()
            await self.ack(ws, t)
        elif t == 'speed_limit':
            n.set_speed_limit(cmd['pct'])
        elif t == 'steer_limit':
            if not n.set_steer_limit(cmd['rad']):
                await self.toast(ws, 'Steering limit will apply once the converter is up', 'warn')
        elif t == 'voice':
            n.set_voice(cmd.get('on', n.voice_on), cmd.get('vol', n.voice_volume))
        elif t == 'mic':
            n.set_mic(cmd.get('on', n.mic_on), cmd.get('source', n.mic_source))
            await self.toast(
                ws, f'Microphone {"on" if n.mic_on else "off"} ({n.mic_source})')
        elif t == 'goto_object':
            if n.mode != 'navigation':
                await self.toast(ws, 'Driving to an object needs navigation mode', 'warn')
            elif not n.initial_pose_set:
                await self.toast(ws, "Set Pose on the car's real map position first.", 'warn')
            elif n.go_to_object(cmd.get('name')):
                await self.toast(ws, f'Going to the {cmd.get("name")}')
        elif t == 'explore':
            n.set_explore(cmd.get('on', True))
            await self.ack(ws, t, on=bool(cmd.get('on', True)))
        elif t == 'clear_hints':
            n.clear_hints(str(cmd.get('what', 'all')))
            await self.ack(ws, t)
            await self.toast(
                ws, f'Exploration {"resumed" if cmd.get("on", True) else "paused"}')
        elif t == 'cams360':
            n.set_cameras_360(bool(cmd.get('on')))
        elif t == 'drive':
            n.set_drive(cmd.get('speed', 0.0), cmd.get('steering', 0.0))
        elif t == 'ask':
            if n.ask(cmd.get('q', '')):
                await self.toast(ws, 'Asked')
            else:
                await self.toast(ws, 'Type a question first', 'warn')
        elif t == 'save_map':
            ok, text = await asyncio.get_event_loop().run_in_executor(
                None, n.save_map, cmd.get('name', 'qcar_map'))
            await self.toast(ws, text, 'info' if ok else 'error')

    # -------------------------------------------------------------- mjpeg

    async def mjpeg(self, request):
        name = request.match_info['name']
        # 'detections' is the object mapper's annotated view rather than a
        # physical camera, but it streams identically.
        if name not in CAMERAS and name != 'detections':
            raise web.HTTPNotFound()
        response = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-store'})
        await response.prepare(request)
        last = -1
        try:
            while True:
                with self.state.lock:
                    jpg, version = (self.state.detection_jpeg if name == 'detections'
                                    else self.state.jpeg[name])
                if jpg is not None and version != last:
                    last = version
                    await response.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                        + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                await asyncio.sleep(0.05)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    # ---------------------------------------------------------------- run

    def lan_addresses(self):
        addresses = []
        try:
            out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout
            addresses = [a for a in out.split() if ':' not in a and not a.startswith('172.17.')]
        except (OSError, subprocess.TimeoutExpired):
            pass
        return addresses or [socket.gethostbyname(socket.gethostname())]

    def run(self):
        port = self.node.port
        urls = [f'http://{addr}:{port}' for addr in self.lan_addresses()]
        for url in urls:
            self.node.get_logger().info(f'QCar2 web GUI: {url}')
        if self.node.open_browser and os.environ.get('DISPLAY'):
            url = f'http://localhost:{port}'
            # --kiosk gives a real borderless fullscreen window (no tabs, no
            # URL bar) -- this is the "default full screen" console on the
            # car's own display.  Only Firefox is installed on this image;
            # xdg-open (a normal windowed tab) is the fallback if that binary
            # ever goes missing, e.g. a different disk image.
            import shutil
            browser_cmd = (['firefox', '--kiosk', url] if shutil.which('firefox')
                           else ['xdg-open', url])
            try:
                subprocess.Popen(browser_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                pass
        # shutdown_timeout: how long to wait for open WebSocket/MJPEG streams
        # to drain on Ctrl+C.  The default is 60 s, which read as "hung";
        # 2 s still ate most of the 3 s stop.sh allows, so 0.5 s.
        web.run_app(self.app, host='0.0.0.0', port=port, print=None,
                    handle_signals=True, shutdown_timeout=0.5)


def main():
    # rclpy must NOT install its own SIGINT handler here.  It hooks SIGINT at
    # the C level and swallows it (it only flags its own context for
    # shutdown), so aiohttp's asyncio-level handler never fires and Ctrl+C /
    # scripts/stop.sh leave this process running.  aiohttp owns the signal;
    # rclpy is shut down explicitly once run_app() returns.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    state = State()
    node = WebGuiNode(state)

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    # Not a daemon thread: the interpreter must not tear this down while it
    # is inside rcl.  Shutdown below makes spin() return, then it is joined.
    spinner = threading.Thread(target=spin, name='rclpy-spin')
    spinner.start()
    try:
        WebServer(node, state).run()
    finally:
        node.shutdown_cameras()
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
