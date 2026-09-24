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
from sensor_msgs.msg import BatteryState, Image, LaserScan
from std_msgs.msg import Bool, Empty, Int32
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


class WebGuiNode(Node):
    def __init__(self, state):
        super().__init__('qcar2_web_gui')
        self.state = state
        self.mode = self.declare_parameter('mode', 'navigation').value
        self.port = int(self.declare_parameter('port', 8080).value)
        self.open_browser = bool(self.declare_parameter('open_browser', True).value)

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
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
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
            topic = f'/{name}/camera/csi_image' if name == 'front' else f'/{name}/camera/preview'
            self.create_subscription(
                Image, topic,
                lambda msg, name=name: self.on_image(name, msg), CAMERA_QOS)

        # ---- commands out
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose_raw', 1)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 1)
        self.estop_pub = self.create_publisher(Bool, '/qcar2_estop', LATCHED)
        self.voice_enabled_pub = self.create_publisher(Bool, '/qcar2/voice_enabled', LATCHED)
        self.voice_volume_pub = self.create_publisher(Int32, '/qcar2/voice_volume', LATCHED)
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
        self.speed_pct = 100
        self.steer_limit = STEERING_STOP_RAD
        self.estop = False
        # AMCL starts at (0, 0) only so that the saved map can be rendered.
        # That seed is not localization.  A goal issued before Set Pose can
        # look valid in the UI while referring to the wrong physical place.
        self.initial_pose_set = self.mode != 'navigation'
        self.publish_voice()
        self.publish_estop()

        # ---- manual drive (mapping only).  The hardware latches the last
        # motor command, so zeros are published continuously whenever nothing
        # is held, and a hold without a heartbeat for 0.4 s is dropped.
        self.drive = {'speed': 0.0, 'steering': 0.0, 'held': False, 'last': 0.0}
        if self.mode == 'mapping':
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
        if abs(s.cmd_throttle) > 0.05 and abs(s.speed) < 0.02:
            if self._stall_since is None:
                self._stall_since = now
            elif now - self._stall_since > 1.5:
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

    def set_drive(self, speed, steering):
        if self.estop:
            return
        speed, steering = float(speed), float(steering)
        held = bool(speed or steering)
        if self.mode == 'mapping':
            self.drive.update(speed=speed, steering=steering, held=held, last=time.monotonic())
        else:
            # Navigation mode: highest-priority manual override.  Published
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

    def set_cameras_360(self, on):
        running = self.camera_process is not None and self.camera_process.poll() is None
        if on and not running:
            self.camera_process = subprocess.Popen(
                ['ros2', 'launch', 'qcar2_rviz_gui', 'cameras_side.launch.py'])
        elif not on and running:
            # SIGINT so the CSI driver releases the cameras cleanly.
            self.camera_process.send_signal(2)
        return on

    def cameras_360_running(self):
        return self.camera_process is not None and self.camera_process.poll() is None

    def save_map(self, name):
        name = ''.join(ch for ch in name if ch.isalnum() or ch in '-_') or 'qcar_map'
        script = os.path.join(PROJECT_DIR, 'scripts', 'save_map.sh')
        result = subprocess.run([script, name], capture_output=True, text=True, timeout=30)
        ok = result.returncode == 0
        return ok, (f'Saved maps/{name}.yaml' if ok else result.stderr.strip()[-300:] or 'save failed')

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
                'estop': n.estop, 'localized': n.initial_pose_set}

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
                    'warnings': s.warnings}

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
                small.insert(0, self.state_message())
            for update in small:
                await ws.send_json(update)
            for payload in grids:
                await ws.send_bytes(payload)
            tick += 1
            await asyncio.sleep(0.05)

    async def toast(self, ws, text, kind='info'):
        await ws.send_json({'t': 'toast', 'msg': text, 'kind': kind})

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
        elif t == 'estop':
            n.set_estop(cmd.get('on', True))
        elif t == 'stopall':
            n.stop_all()
            await self.toast(ws, 'Shutting everything down', 'warn')
        elif t == 'speed_limit':
            n.set_speed_limit(cmd['pct'])
        elif t == 'steer_limit':
            if not n.set_steer_limit(cmd['rad']):
                await self.toast(ws, 'Steering limit will apply once the converter is up', 'warn')
        elif t == 'voice':
            n.set_voice(cmd.get('on', n.voice_on), cmd.get('vol', n.voice_volume))
        elif t == 'cams360':
            n.set_cameras_360(bool(cmd.get('on')))
        elif t == 'drive':
            n.set_drive(cmd.get('speed', 0.0), cmd.get('steering', 0.0))
        elif t == 'save_map':
            ok, text = await asyncio.get_event_loop().run_in_executor(
                None, n.save_map, cmd.get('name', 'qcar_map'))
            await self.toast(ws, text, 'info' if ok else 'error')

    # -------------------------------------------------------------- mjpeg

    async def mjpeg(self, request):
        name = request.match_info['name']
        if name not in CAMERAS:
            raise web.HTTPNotFound()
        response = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-store'})
        await response.prepare(request)
        last = -1
        try:
            while True:
                with self.state.lock:
                    jpg, version = self.state.jpeg[name]
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
        # to drain on Ctrl+C.  The default is 60 s, which read as "hung".
        web.run_app(self.app, host='0.0.0.0', port=port, print=None,
                    handle_signals=True, shutdown_timeout=2.0)


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
