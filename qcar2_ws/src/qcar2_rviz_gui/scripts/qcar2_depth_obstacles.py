#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Obstacles below the LiDAR, from the RealSense D435 depth.

The LiDAR scans ONE horizontal plane, 19 cm above the floor. Anything lower
-- a bag, a box, a chair base, a skirting step -- is invisible to it, so the
costmap shows free floor and the car drives into it. The D435 on top looks
forward and down over the same area and measures it.

From the aligned depth image (decimated to every `step`th pixel) each pixel
is placed in base_link using the camera pose from the URDF (realsenseRGB:
9.5 cm ahead, 17.6 cm up, level). Points between `min_height` and
`max_height` above the floor are obstacles; lower ones are floor.

Published:
  /front/depth_scan       LaserScan in base_link, 1 deg bins across the
                          camera's view: the nearest obstacle per bin, inf
                          where the camera sees only floor. Both costmaps have
                          a `depth_layer` for it (qcar2_slam_and_nav.yaml),
                          so the planner and MPPI avoid low objects exactly
                          like walls, and they clear when seen to be gone.
  /front/depth_obstacles  PointCloud2 in odom: REMEMBERED obstacle points for
                          qcar2_bump_guard.py's clearance. The camera cannot
                          see the last ~30 cm in front of the bumper, so a low
                          object would vanish just as the car reaches it.
                          Points are kept (in odom, so they stay put as the
                          car moves) until the camera looks at their spot
                          again and finds it empty, or they are old/far.

Clear glass is still invisible here too: the D435's infrared passes through
it just like the LiDAR does. qcar2_bump_guard.py handles that case.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

# realsenseRGB from QCar2.urdf -- used only until TF provides it.
CAMERA_XYZ = (0.095, 0.034, 0.176)
BIN = math.radians(1.0)


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class DepthObstacles(Node):

    def __init__(self):
        super().__init__('qcar2_depth_obstacles')
        g = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        depth_topic = g('depth_topic', '/front/realsense/aligned_depth_to_color/image_raw')
        info_topic = g('info_topic', '/front/realsense/aligned_depth_to_color/camera_info')
        self.camera_frame = g('camera_frame', 'realsenseRGB')
        self.base_frame = g('base_frame', 'base_link')
        self.odom_frame = g('odom_frame', 'odom')
        self.step = int(g('step', 4))
        self.min_range = float(g('min_range', 0.20))       # D435 minimum
        self.max_range = float(g('max_range', 2.0))        # noisy beyond
        # Above the floor: below min_height is floor (plus depth noise and a
        # little camera tilt); above max_height the car passes underneath.
        self.min_height = float(g('min_height', 0.05))
        self.max_height = float(g('max_height', 0.30))
        self.min_points = int(g('min_points', 3))          # per bin, against speckle
        self.memory_sec = float(g('memory_sec', 60.0))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_pub = self.create_publisher(LaserScan, '/front/depth_scan', 5)
        self.cloud_pub = self.create_publisher(PointCloud2, '/front/depth_obstacles', 5)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)

        self.K = None
        self.rays = None                     # cached per-pixel unit rays
        self.cam_R = None
        self.cam_t = np.array(CAMERA_XYZ)
        self.memory = np.zeros((0, 3))       # odom x, y, time seen
        self.floor = None                    # plane coefficients, see floor_height()
        self.get_logger().info(f'Depth obstacles from {depth_topic}')

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def camera_pose(self):
        if self.cam_R is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time())
                self.cam_R = quat_to_matrix(tf.transform.rotation)
                t = tf.transform.translation
                self.cam_t = np.array([t.x, t.y, t.z])
            except Exception:                # noqa: BLE001 - use the URDF numbers
                # Standard optical frame: z forward, x right, y down.
                return np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float), self.cam_t
        return self.cam_R, self.cam_t

    def floor_height(self, pts):
        """Height of the floor under each point, from a running plane fit.

        The URDF says the camera is level, but a degree or two of real tilt
        (mount tolerance, the car pitching on its springs) puts the floor 2 m
        away 3-7 cm "above" the floor -- enough to read as an obstacle. Fit
        z = a + b*x + c*y to the points that are plausibly floor and measure
        height from that instead; reject fits steeper than ~6 degrees (that is
        a ramp or a wall, not the floor) and smooth the rest.
        """
        cand = np.abs(pts[:, 2]) < 0.08
        if np.count_nonzero(cand) > 200:
            f = pts[cand]
            A = np.column_stack([np.ones(len(f)), f[:, 0], f[:, 1]])
            coef, *_ = np.linalg.lstsq(A, f[:, 2], rcond=None)
            if abs(coef[0]) < 0.06 and abs(coef[1]) < 0.1 and abs(coef[2]) < 0.1:
                self.floor = coef if self.floor is None else 0.9 * self.floor + 0.1 * coef
        if self.floor is None:
            return np.zeros(len(pts))
        return self.floor[0] + self.floor[1] * pts[:, 0] + self.floor[2] * pts[:, 1]

    def on_depth(self, msg):
        if self.K is None or msg.encoding not in ('16UC1', 'mono16'):
            return
        depth = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)
        depth = depth[::self.step, :msg.width:self.step].astype(float) * 0.001
        if self.rays is None or self.rays.shape[:2] != depth.shape:
            v, u = np.mgrid[0:msg.height:self.step, 0:msg.width:self.step]
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            self.rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1)
        R, t = self.camera_pose()
        valid = (depth > self.min_range) & (depth < self.max_range)
        pts = (self.rays[valid] * depth[valid][:, None]) @ R.T + t     # base_link
        height = pts[:, 2] - self.floor_height(pts)
        obstacle = (height > self.min_height) & (height < self.max_height)

        # ---- the LaserScan for the costmaps
        n = int(round(math.radians(90.0) / BIN))
        a_min = -n / 2 * BIN
        bearing = np.arctan2(pts[:, 1], pts[:, 0])
        bins = np.floor((bearing - a_min) / BIN).astype(int)
        inside = (bins >= 0) & (bins < n)
        seen = np.zeros(n, bool)
        seen[bins[inside]] = True
        ranges = np.full(n, np.nan)
        ranges[seen] = np.inf                # floor only: clear this bin
        horiz = np.hypot(pts[:, 0], pts[:, 1])
        ob = obstacle & inside
        counts = np.bincount(bins[ob], minlength=n)
        nearest = np.full(n, np.inf)
        np.minimum.at(nearest, bins[ob], horiz[ob])
        hit = counts >= self.min_points
        ranges[hit] = nearest[hit]
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = a_min + BIN / 2
        scan.angle_max = scan.angle_min + (n - 1) * BIN
        scan.angle_increment = BIN
        scan.range_min = 0.05
        scan.range_max = self.max_range + 0.5
        scan.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(scan)

        # ---- remembered obstacle points for the clearance check
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception:                    # noqa: BLE001 - odom not up yet
            return
        yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
        ox, oy = tf.transform.translation.x, tf.transform.translation.y
        c, s = math.cos(yaw), math.sin(yaw)
        now = time.monotonic()

        mem = self.memory
        if len(mem):
            # Where are the remembered points now, relative to the car?
            dx, dy = mem[:, 0] - ox, mem[:, 1] - oy
            bx, by = c * dx + s * dy, -s * dx + c * dy
            mr = np.hypot(bx, by)
            mb = np.floor((np.arctan2(by, bx) - a_min) / BIN).astype(int)
            in_view = (mb >= 0) & (mb < n) & (mr > self.min_range + CAMERA_XYZ[0] + 0.1) \
                & (mr < self.max_range - 0.2)
            in_view &= seen[np.clip(mb, 0, n - 1)]
            # In view now: still there only if this frame sees something at
            # (or in front of) that range in the same direction.
            still = ~in_view | (nearest[np.clip(mb, 0, n - 1)] <= mr + 0.10)
            keep = still & (now - mem[:, 2] < self.memory_sec) & (mr < 3.0)
            mem = mem[keep]

        new = pts[obstacle][:, :2]
        if len(new):
            new = np.unique(np.round(new / 0.05) * 0.05, axis=0)   # 5 cm cells
            wx = ox + c * new[:, 0] - s * new[:, 1]
            wy = oy + s * new[:, 0] + c * new[:, 1]
            fresh = np.column_stack([wx, wy, np.full(len(new), now)])
            mem = np.vstack([mem, fresh]) if len(mem) else fresh
            # One entry per cell: the newest.
            key = np.round(mem[:, :2] / 0.05).astype(np.int64)
            order = np.argsort(-mem[:, 2])
            _u, first = np.unique(key[order], axis=0, return_index=True)
            mem = mem[order][first]
        self.memory = mem[-4000:]

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.odom_frame
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(
            header, [(float(x), float(y), 0.1) for x, y, _t in self.memory]))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DepthObstacles()
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
