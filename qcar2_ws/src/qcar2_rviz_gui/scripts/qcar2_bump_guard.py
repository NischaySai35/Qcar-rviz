#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Detect that the car has physically run into something, and remember it.

The LiDAR is the only obstacle sensor Nav2 has, and it is blind to two things
this car keeps meeting: GLASS (the beam passes straight through, so the
costmap shows free space and the planner aims right at it) and anything below
the scan plane. When the car meets one of those, nothing in the stack knew --
Nav2 kept pushing, the explorer kept re-sending the goal behind the glass,
and the wheel odometry kept counting the spinning wheels as travel, which is
what dragged the SLAM map out of shape.

A hit shows up one of two ways, and this node watches for both:

  STALL  the wheels are commanded but cannot turn. qcar2_hardware.cpp detects
         this itself (throttle pinned at its stall ceiling, wheels still) and
         cuts the throttle; it reports it on /qcar2/drive_stalled.
  SLIP   the wheels ARE turning (spinning on the floor against the obstacle)
         but the car is not moving. The LiDAR settles it: over the last
         `window_sec` the wheels claim at least `min_wheel_travel` metres, yet
         the beams pointing along the direction of travel (front and back
         cones) have not changed range -- a car that really moved 10 cm
         changes every one of those ranges by ~7-10 cm. This works through
         glass too: the beams see whatever is BEHIND the glass, and that does
         not move either.

On a hit it:
  * publishes /qcar2/drive_blocked = true. wheel_imu_odometry.py stops
    integrating translation while it is set (so the spinning wheels can no
    longer push the map), and the explorer abandons the goal behind it;
  * adds a short wall across the bumper, in the map frame, to
    /qcar2/bump_obstacles. Both costmaps have a marking-only `bump_layer` for
    it (see qcar2_slam_and_nav.yaml), so the planner routes around the glass
    from then on and the LiDAR's clearing rays cannot erase it. The console
    stamps these points into the saved map, so navigation knows about the
    glass too.

Blocked clears when the car is commanded the other way (backing off what it
hit), the command stays at zero for `release_zero_sec`, or the LiDAR shows
the car clearly moving again.

Most importantly it keeps the car from touching anything it CAN see:
/qcar2/obstacle_clearance gives, at 20 Hz, how far the front and rear
bumpers can travel along the arc the car is currently steering before they
meet a LiDAR return or a low obstacle remembered from the RealSense depth
(qcar2_depth_obstacles.py). qcar2_hardware.cpp turns that into a speed
limit that eases the car to a stop 8 cm short -- for every drive source,
Nav2 and the console's drive pad alike. A bump is then only possible on
something neither sensor can see.

This node never commands the motors itself.
"""

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from qcar2_interfaces.msg import MotorCommands
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray, Header, String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Footprint from qcar2_slam_and_nav.yaml: front edge +0.22 m, rear -0.21 m.
FRONT_EDGE_M = 0.22
REAR_EDGE_M = -0.21
WHEELBASE_M = 0.25725                # QCar2.urdf


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BumpGuard(Node):

    def __init__(self):
        super().__init__('qcar2_bump_guard')
        g = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.map_frame = g('map_frame', 'map')
        self.base_frame = g('base_frame', 'base_link')
        self.window = float(g('window_sec', 0.8))
        self.min_travel = float(g('min_wheel_travel', 0.08))
        # Median range change below this, along the travel axis, = not moving.
        # Scan noise alone gives ~0.01 m; a real 8 cm move gives >= 0.06 m.
        self.still_change = float(g('still_change_m', 0.025))
        self.cone = math.radians(float(g('cone_deg', 45.0)))
        self.wall_half_width = float(g('wall_half_width', 0.30))
        self.wall_gap = float(g('wall_gap', 0.08))
        self.release_zero = float(g('release_zero_sec', 1.0))
        self.announce = bool(g('announce', True))
        # Clearance lane: car half-width 0.12 m + 5 cm -- see clearance().
        self.lane_half_width = float(g('lane_half_width', 0.17))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.blocked_pub = self.create_publisher(Bool, '/qcar2/drive_blocked', LATCHED)
        self.cloud_pub = self.create_publisher(PointCloud2, '/qcar2/bump_obstacles', LATCHED)
        self.say_pub = self.create_publisher(String, '/qcar2/say', 10)
        self.clearance_pub = self.create_publisher(
            Float32MultiArray, '/qcar2/obstacle_clearance', 10)
        self.create_subscription(PointCloud2, '/front/depth_obstacles', self.on_depth_cloud,
                                 qos_profile_sensor_data)
        # Glass walls the vision model confirmed (qcar2_explore_vlm.py): the
        # same no-go treatment as a bump, but BEFORE the car touches them.
        # Kept separate from self.points so the console's undo removes them.
        self.hint_points = []
        self.create_subscription(String, '/qcar2/explore_hints', self.on_hints, LATCHED)

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        # /wheel_imu_odom in mapping, remapped to /odom in navigation
        # (odometry.launch.py) -- listening to one only left the guard blind
        # in navigation.  Exactly one is published in either mode.
        self.create_subscription(Odometry, '/wheel_imu_odom', self.on_odom, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Bool, '/qcar2/drive_stalled', self.on_stalled, LATCHED)
        self.create_subscription(MotorCommands, '/qcar2_motor_speed_cmd', self.on_command, 20)

        self.scans = deque()                 # (t, angles in base_link, ranges)
        self.wheel = deque()                 # (t, signed wheel speed)
        self.scan_yaw = None                 # base_link -> scan frame yaw
        self.scan_xy = (0.0, 0.0)            # ... and offset
        self.steering = 0.0
        self.lidar_xy, self.lidar_at = None, 0.0
        self.depth_odom, self.depth_frame, self.depth_at = None, 'odom', 0.0
        self.command = 0.0
        self.command_zero_since = time.monotonic()
        self.stalled = False
        self.blocked = False
        self.blocked_dir = 0.0
        self.still_hits = 0
        self.last_hit = 0.0
        self.points = []                     # [(x, y)] in map frame

        self.publish_blocked()
        self.create_timer(0.5, self.publish_cloud)
        self.create_timer(0.05, self.publish_clearance)
        self.get_logger().info('Bump guard up: watching for stalls and wheel slip.')

    # ----------------------------------------------------------------- input

    def on_command(self, msg):
        try:
            speed = msg.values[msg.motor_names.index('motor_throttle')]
            self.steering = msg.values[msg.motor_names.index('steering_angle')]
        except (ValueError, IndexError):
            return
        if speed == 0.0 and self.command != 0.0:
            self.command_zero_since = time.monotonic()
        self.command = speed

    def on_hints(self, msg):
        try:
            closures = json.loads(msg.data).get('closures', [])
        except ValueError:
            return
        pts = []
        for c in closures:
            if c.get('kind') != 'glass':
                continue
            length = math.hypot(c['x2'] - c['x1'], c['y2'] - c['y1'])
            n = max(2, int(length / 0.05) + 1)
            pts += [(c['x1'] + (c['x2'] - c['x1']) * i / (n - 1),
                     c['y1'] + (c['y2'] - c['y1']) * i / (n - 1)) for i in range(n)]
        if pts != self.hint_points:
            self.hint_points = pts
            self.publish_cloud()

    def on_odom(self, msg):
        now = time.monotonic()
        self.wheel.append((now, msg.twist.twist.linear.x))
        while self.wheel and now - self.wheel[0][0] > 3.0:
            self.wheel.popleft()

    def on_stalled(self, msg):
        stalled = bool(msg.data)
        if stalled and not self.stalled:
            self.hit('wheels blocked', self.command)
        self.stalled = stalled

    def on_scan(self, msg):
        now = time.monotonic()
        if self.scan_yaw is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except Exception:                # noqa: BLE001 - TF not up yet
                return
            self.scan_yaw = yaw_of(tf.transform.rotation)
            self.scan_xy = (tf.transform.translation.x, tf.transform.translation.y)
        n = len(msg.ranges)
        if n < 10:
            return
        angles = msg.angle_min + msg.angle_increment * np.arange(n) + self.scan_yaw
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        ranges = np.asarray(msg.ranges, dtype=float)
        self.on_lidar_points(angles, ranges)
        ranges[~np.isfinite(ranges) | (ranges < 0.15) | (ranges > 8.0)] = np.nan
        order = np.argsort(angles)
        self.scans.append((now, angles[order], ranges[order]))
        while self.scans and now - self.scans[0][0] > self.window + 1.0:
            self.scans.popleft()

        self.update_release(now)
        if not self.blocked:
            self.check_slip(now)

    # ------------------------------------------------------------- detection

    def on_lidar_points(self, angles, ranges):
        ok = np.isfinite(ranges) & (ranges > 0.12)
        self.lidar_xy = np.column_stack([
            ranges[ok] * np.cos(angles[ok]) + self.scan_xy[0],
            ranges[ok] * np.sin(angles[ok]) + self.scan_xy[1]])
        self.lidar_at = time.monotonic()

    def on_depth_cloud(self, msg):
        self.depth_odom = np.array(
            [(p[0], p[1]) for p in point_cloud2.read_points(
                msg, field_names=('x', 'y'), skip_nans=True)], dtype=float).reshape(-1, 2)
        self.depth_frame = msg.header.frame_id
        self.depth_at = time.monotonic()

    def obstacle_points(self):
        """Every obstacle point currently known, in base_link: the latest
        LiDAR scan plus the RealSense's remembered low obstacles."""
        now = time.monotonic()
        parts = []
        if self.lidar_xy is not None and now - self.lidar_at < 0.5:
            parts.append(self.lidar_xy)
        if self.depth_odom is not None and len(self.depth_odom) and now - self.depth_at < 1.0:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.depth_frame, Time())
                yaw = yaw_of(tf.transform.rotation)
                c, s = math.cos(yaw), math.sin(yaw)
                t = tf.transform.translation
                d = self.depth_odom
                parts.append(np.column_stack([c * d[:, 0] - s * d[:, 1] + t.x,
                                              s * d[:, 0] + c * d[:, 1] + t.y]))
            except Exception:                # noqa: BLE001 - odom not up yet
                pass
        return np.vstack(parts) if parts else np.zeros((0, 2))

    def clearance(self, pts, direction):
        """Metres the car can travel (forward if direction > 0, else reverse)
        along the arc it is currently steering before its bumper reaches any
        point, within a lane the car's width plus a margin. 99 = clear."""
        if not len(pts):
            return 99.0
        edge = FRONT_EDGE_M if direction > 0 else -REAR_EDGE_M
        x, y = pts[:, 0], pts[:, 1]
        steer = self.steering
        if abs(steer) < 0.03:
            along = direction * x
            lateral = np.abs(y)
        else:
            # Turn about the centre (0, R); R > 0 turning left. Travel sweeps
            # the car around it, counter-clockwise when s > 0.
            R = WHEELBASE_M / math.tan(steer)
            s = math.copysign(1.0, R) * direction
            phi = np.arctan2(y - R, x)
            phi_car = math.atan2(-R, 0.0)
            swept = np.mod(s * (phi - phi_car), 2.0 * np.pi)
            along = abs(R) * swept
            lateral = np.abs(np.hypot(x, y - R) - abs(R))
            along[swept > np.pi] = -1.0      # behind, on the far side of the circle
        ahead = (lateral < self.lane_half_width) & (along > edge - 0.05)
        if not np.any(ahead):
            return 99.0
        return float(max(0.0, np.min(along[ahead]) - edge))

    def publish_clearance(self):
        pts = self.obstacle_points()
        front = self.clearance(pts, 1.0)
        rear = self.clearance(pts, -1.0)
        msg = Float32MultiArray()
        msg.data = [front, rear]
        self.clearance_pub.publish(msg)

    def wheel_travel(self, now):
        """Distance the wheels claim over the window, and its direction.
        Mixed directions (a direction change inside the window) return 0."""
        samples = [(t, v) for t, v in self.wheel if now - t <= self.window]
        if len(samples) < 5 or now - samples[0][0] < 0.7 * self.window:
            return 0.0, 0.0
        travel, signs = 0.0, set()
        for (t0, v0), (t1, _v1) in zip(samples, samples[1:]):
            travel += abs(v0) * (t1 - t0)
            if abs(v0) > 0.03:
                signs.add(1.0 if v0 > 0 else -1.0)
        if len(signs) != 1:
            return 0.0, 0.0
        return travel, signs.pop()

    def scan_change(self, now):
        """Median range change over the window of the beams along the travel
        axis (front and rear cones), or None if there is too little to judge."""
        old = None
        for t, a, r in self.scans:
            if now - t >= self.window:
                old = (a, r)                 # newest scan at least a window old
        if old is None:
            return None
        _t, a_new, r_new = self.scans[-1]
        a_old, r_old = old
        along = (np.abs(a_new) < self.cone) | (np.abs(np.pi - np.abs(a_new)) < self.cone)
        r_then = np.interp(a_new, a_old, r_old, period=2.0 * np.pi)
        ok = along & np.isfinite(r_new) & np.isfinite(r_then)
        if np.count_nonzero(ok) < 15:
            return None
        return float(np.median(np.abs(r_new[ok] - r_then[ok])))

    def check_slip(self, now):
        travel, direction = self.wheel_travel(now)
        if travel < self.min_travel:
            self.still_hits = 0
            return
        change = self.scan_change(now)
        if change is None:
            self.still_hits = 0              # nothing in range to judge by
            return
        if change < self.still_change:
            self.still_hits += 1
            if self.still_hits >= 2:         # two scans in a row, not one glitch
                self.hit(f'wheels turned {travel:.2f} m but the LiDAR moved '
                         f'{change * 100:.1f} cm', direction)
        else:
            self.still_hits = 0

    def update_release(self, now):
        if not self.blocked:
            return
        reversed_ = self.command != 0.0 and (self.command > 0) != (self.blocked_dir > 0)
        paused = self.command == 0.0 and now - self.command_zero_since > self.release_zero
        # The LiDAR shows the car clearly moving: whatever held it has let go
        # (or this was a false alarm). Odometry must not stay frozen.
        change = None if self.stalled else self.scan_change(now)
        moving = change is not None and change > 3.0 * self.still_change
        if reversed_ or paused or moving:
            self.blocked = False
            self.still_hits = 0
            self.publish_blocked()
            self.get_logger().info('Drive no longer blocked.')

    # --------------------------------------------------------------- the hit

    def hit(self, why, direction):
        now = time.monotonic()
        if direction == 0.0:
            direction = self.blocked_dir or 1.0
        self.blocked = True
        self.blocked_dir = 1.0 if direction > 0 else -1.0
        self.still_hits = 0
        self.publish_blocked()
        if now - self.last_hit < 2.0:
            return                           # same collision, already marked
        self.last_hit = now
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:                    # noqa: BLE001 - no map yet
            self.get_logger().warn(f'Bump ({why}) but no map pose to mark it at.')
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)
        edge = (FRONT_EDGE_M if self.blocked_dir > 0 else REAR_EDGE_M) \
            + self.wall_gap * self.blocked_dir
        c, s = math.cos(yaw), math.sin(yaw)
        added = 0
        for lateral in np.arange(-self.wall_half_width, self.wall_half_width + 1e-6, 0.05):
            px = x + c * edge - s * lateral
            py = y + s * edge + c * lateral
            if any(math.hypot(px - qx, py - qy) < 0.04 for qx, qy in self.points[-200:]):
                continue
            self.points.append((px, py))
            added += 1
        self.points = self.points[-3000:]
        side = 'front' if self.blocked_dir > 0 else 'rear'
        self.get_logger().warn(
            f'BUMP at the {side} ({why}) -- marked {added} obstacle points at '
            f'({x:.2f}, {y:.2f}); {len(self.points)} in total.')
        if self.announce:
            msg = String()
            msg.data = 'I bumped into something I cannot see. Marking it and backing off.'
            self.say_pub.publish(msg)
        self.publish_cloud()

    # --------------------------------------------------------------- outputs

    def publish_blocked(self):
        msg = Bool()
        msg.data = self.blocked
        self.blocked_pub.publish(msg)

    def publish_cloud(self):
        header = Header()
        header.frame_id = self.map_frame
        header.stamp = self.get_clock().now().to_msg()
        pts = [(float(px), float(py), 0.10) for px, py in self.points + self.hint_points]
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BumpGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
