#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""What the car knows about the ROOM it is mapping -- from geometry alone.

The explorer used to see only "frontiers": any edge between mapped and
unmapped space. It had no idea what the room was, how much of it was done,
which gaps mattered, or when to stop. This node answers those questions from
the live map every `period_sec`, deterministically, for any room shape:

ROOM OUTLINE
  Walls (map cells >= 65) plus anything the car has bumped into
  (/qcar2/bump_obstacles) plus closures reported by the vision model
  (/qcar2/explore_hints: glass, and doorways unless go_next_room) are
  "closed" morphologically with a disk of `door_close_m`, which seals
  doorway-sized gaps (< 2 x door_close_m) but leaves straight walls where
  they are. The room is the connected free/unknown space around the car
  inside that. It follows whatever shape the room has -- L-shapes, angled
  walls -- because it is a flood fill, not a rectangle fit.
  The LiDAR "wedges" seen through glass or an open door fall outside it.

COVERAGE
  free cells in the room / (free + unknown cells in the room that MATTER).
  Unknown pockets that do not matter are left out of the denominator:
    * small pockets (< pocket_max_m2) -- under a sofa, behind a bin;
    * pockets enclosed mostly by occupied cells -- the inside of furniture.

GAPS (frontiers) are sorted:
    visit    inside the room, leading to unknown space that matters;
    ignore   leads only into an ignorable pocket, or too small to matter;
    outside  beyond the room outline (a doorway, glass, outside the walls).

go_next_room (default false) -- the operator's switch for following
doorways into neighbouring rooms. When true, doorway-sized gaps are not
sealed and vision-model "doorway" closures are not applied.

MAP QUALITY
  Fraction of the live LiDAR points that land on an already-mapped wall.
  A good map keeps this high; a smeared or drifting one drops, and the
  explorer slows down so Cartographer can re-anchor.

STOP RULE
  `done` = coverage >= done_coverage (0.90) with no worthwhile gap left (an
  unknown area >= worthwhile_m2 behind a visit gap). Below that the explorer
  finishes only when every remaining gap has been tried and is unreachable.
  `stalled` (>= 80 % and the last stall_window_sec added < stall_gain) is
  reported for the console but never ends exploration: in the first live run
  a car stuck behind a bad wall "stalled" and quit with the area behind the
  sofa unseen. A room whose outline still touches the map edge is never done
  here either.

Published:  /qcar2/room_status  std_msgs/String JSON (latched)
"""

import json
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from scipy import ndimage
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)

FREE_MAX = 49           # same thresholds as qcar2_explorer.py
OCCUPIED_MIN = 65


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def disk(radius_cells):
    r = max(1, int(round(radius_cells)))
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return (x * x + y * y) <= r * r


class RoomAnalyzer(Node):

    def __init__(self):
        super().__init__('qcar2_room_analyzer')
        g = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.map_frame = g('map_frame', 'map')
        self.base_frame = g('base_frame', 'base_link')
        self.door_close = float(g('door_close_m', 0.45))
        self.wall_slack = float(g('wall_slack_m', 0.20))
        self.pocket_max = float(g('pocket_max_m2', 0.30))
        self.worthwhile = float(g('worthwhile_m2', 0.50))
        self.min_cells = int(g('min_frontier_cells', 8))
        self.max_room = float(g('max_room_m2', 250.0))
        self.done_coverage = float(g('done_coverage', 0.90))
        self.floor_coverage = float(g('floor_coverage', 0.80))
        self.stall_window = float(g('stall_window_sec', 60.0))
        self.stall_gain = float(g('stall_gain', 0.01))
        self.go_next_room = bool(g('go_next_room', False))
        if self.go_next_room:
            # Following doorways: only seal hairline gaps in walls, so
            # neighbouring rooms join "the room" and get explored too.
            self.door_close = min(self.door_close, 0.15)
        self.poor_quality = float(g('poor_quality', 0.55))
        # CAMERA COVERAGE. The LiDAR maps a whole room from a few spots, but
        # the object detector only names things it sees from fairly close
        # (most landmarks are recorded within ~1-2.5 m). The first live run
        # mapped the room's walls and stopped with only 8 objects named. So
        # floor that has never been within view_radius_m of the car is a
        # target of its own ("not seen up close yet"), and finishing also
        # needs view_done of the floor seen up close.
        self.view_radius = float(g('view_radius_m', 2.0))
        self.view_min = float(g('view_min_m2', 1.0))
        self.view_done = float(g('view_done', 0.85))
        self.track = []                      # car positions in map frame
        self.create_timer(0.5, self.record_track)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(String, '/qcar2/room_status', LATCHED)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/qcar2/bump_obstacles', self.on_bumps, LATCHED)
        self.create_subscription(String, '/qcar2/explore_hints', self.on_hints, LATCHED)

        self.map = None
        self.scan = None
        self.bumps = []
        self.closures = []                   # [(x1, y1, x2, y2, kind)]
        self.history = deque()               # (t, coverage)
        self.quality = None
        self.started = time.monotonic()
        self.create_timer(float(g('period_sec', 2.0)), self.analyze)
        self.get_logger().info(
            f'Room analyzer up (go_next_room={self.go_next_room}, finish at '
            f'{self.done_coverage:.0%} / {self.floor_coverage:.0%} when stalled).')

    # ----------------------------------------------------------------- input

    def on_map(self, msg):
        self.map = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_bumps(self, msg):
        self.bumps = [(float(x), float(y)) for x, y in
                      point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)]

    def on_hints(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self.closures = [(c['x1'], c['y1'], c['x2'], c['y2'], c.get('kind', 'glass'))
                         for c in data.get('closures', [])]

    def record_track(self):
        pose = self.car_pose()
        if pose is None:
            return
        if not self.track or math.hypot(pose[0] - self.track[-1][0],
                                        pose[1] - self.track[-1][1]) > 0.2:
            self.track.append((pose[0], pose[1]))

    def view_targets(self, room_free, ox, oy, res, h, w):
        """Floor never within view_radius of the car: (coverage, targets)."""
        seen = np.zeros((h, w), np.uint8)
        rad = max(1, int(self.view_radius / res))
        for x, y in self.track:
            cv2.circle(seen, (int((x - ox) / res), int((y - oy) / res)), rad, 1, -1)
        seen = seen.astype(bool)
        n_room = int(room_free.sum())
        coverage = float((seen & room_free).sum()) / max(1, n_room)
        unseen = room_free & ~seen
        # Tiles of ~1.25 x view_radius: one target per tile with enough unseen
        # floor. Unseen floor is usually ONE connected patch covering half the
        # room (tested on my_room6/7: 20-50 m2), and a single target in its
        # middle would leave most of it unseen; tiles make the car sweep it.
        tile = max(4, int(1.25 * self.view_radius / res))
        targets = []
        for r0 in range(0, h, tile):
            for c0 in range(0, w, tile):
                rows, cols = np.nonzero(unseen[r0:r0 + tile, c0:c0 + tile])
                area = len(rows) * res * res
                if area < self.view_min:
                    continue
                rows, cols = rows + r0, cols + c0
                # The unseen cell nearest the tile centre (on the floor).
                k = int(np.argmin((rows - (r0 + tile / 2)) ** 2 + (cols - (c0 + tile / 2)) ** 2))
                targets.append({'x': round(float(cols[k] * res + ox + res / 2), 2),
                                'y': round(float(rows[k] * res + oy + res / 2), 2),
                                'size': int(len(rows)), 'area_m2': round(area, 2)})
        return coverage, targets

    def car_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:                    # noqa: BLE001 - TF not ready yet
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    # -------------------------------------------------------------- analysis

    def analyze(self):
        grid, pose = self.map, self.car_pose()
        if grid is None or pose is None or grid.info.width == 0:
            return
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        unknown = data < 0
        free = (data >= 0) & (data <= FREE_MAX)
        occ = data >= OCCUPIED_MIN
        cell_m2 = res * res

        def to_cell(x, y):
            return int((x - ox) / res), int((y - oy) / res)

        # ---- walls: the map, plus bumps, plus vision-model closures
        walls = occ.copy()
        for x, y in self.bumps:
            c, r = to_cell(x, y)
            if 0 <= c < w and 0 <= r < h:
                walls[r, c] = True
        wall_img = walls.astype(np.uint8)
        for x1, y1, x2, y2, kind in self.closures:
            if kind == 'doorway' and self.go_next_room:
                continue
            cv2.line(wall_img, to_cell(x1, y1), to_cell(x2, y2), 1, thickness=2)
        walls = wall_img.astype(bool)
        closed = ndimage.binary_closing(walls, structure=disk(self.door_close / res),
                                        border_value=0) | walls

        # ---- the room: SEEN free space connected to the car, cut at doorway-
        # sized gaps, glass and bumps. Built from free space, not free+unknown:
        # walls seen by one LiDAR plane are never gap-free, and a fill through
        # unknown space leaks out of every gap to the edge of the map.
        open_free = free & ~closed
        labels, _n = ndimage.label(open_free)
        cc, cr = to_cell(pose[0], pose[1])
        rr = int(0.6 / res)
        win = labels[max(0, cr - rr):cr + rr + 1, max(0, cc - rr):cc + rr + 1]
        vals = win[win > 0]
        if not vals.size:
            return
        seed = labels[cr, cc] if 0 <= cc < w and 0 <= cr < h and labels[cr, cc] \
            else np.bincount(vals).argmax()
        room_free = labels == seed
        # A gap between two pieces of furniture can be as narrow as a door,
        # and the closing above seals it the same way -- in the first live
        # run the whole area past two armchairs was cut off as "outside",
        # and the car never went there. A real doorway leads OUT of the
        # room's footprint; a furniture gap leads to space INSIDE it. So any
        # cut-off free area that lies mostly within the convex hull of the
        # room is part of the room after all.
        pts = cv2.findNonZero(room_free.astype(np.uint8))
        if pts is not None and len(pts) > 10:
            hull = np.zeros((h, w), np.uint8)
            cv2.fillConvexPoly(hull, cv2.convexHull(pts), 1)
            hull = hull.astype(bool)
            ids = np.arange(_n + 1)
            inside = ndimage.sum(hull, labels, index=ids)
            total = ndimage.sum(np.ones_like(labels), labels, index=ids)
            merge = [i for i in range(1, _n + 1)
                     if i != seed and total[i] >= 20 and inside[i] / total[i] >= 0.7]
            if merge:
                room_free |= np.isin(labels, merge)
        # The room's footprint: that free space with every enclosed hole
        # (furniture, unseen pockets) filled in.
        room = ndimage.binary_fill_holes(room_free)
        touches_edge = bool(room[0, :].any() or room[-1, :].any()
                            or room[:, 0].any() or room[:, -1].any())

        # ---- unknown pockets: holes inside the room (behind/under things)
        # and the open unknown around it (where the room continues)
        # Unknown cells inside a seal (a doorway gap, a glass closure) belong
        # to the wall, not to the room's unexplored space.
        unknown = unknown & ~closed
        u_labels, n_u = ndimage.label(unknown, structure=np.ones((3, 3)))
        sizes = ndimage.sum(np.ones_like(u_labels), u_labels,
                            index=np.arange(n_u + 1)) * cell_m2 if n_u else np.zeros(1)
        hole_ids = set(np.unique(u_labels[room & unknown])) - {0}
        why = {}
        relevant_ids = set()
        if hole_ids:
            ring = ndimage.binary_dilation(u_labels > 0) & ~(u_labels > 0)
            rim_lab = ndimage.grey_dilation(u_labels, size=(3, 3)) * ring
            rim_occ = ndimage.sum(occ, rim_lab, index=np.arange(n_u + 1))
            rim_all = ndimage.sum(ring, rim_lab, index=np.arange(n_u + 1))
            for i in hole_ids:
                if sizes[i] < self.pocket_max:
                    why[i] = 'small pocket (under or behind furniture)'
                # Only a pocket that is (almost) fully sealed is "inside
                # furniture". Was > 60 % occupied rim, which also swallowed
                # the space BEHIND a sofa (sofa back + wall on two sides) --
                # the operator wants that looked at from the sofa's open
                # ends, which is exactly what its visit gaps lead to.
                elif rim_all[i] and rim_occ[i] / rim_all[i] > 0.9:
                    why[i] = 'inside furniture'
                else:
                    relevant_ids.add(i)      # a real unseen area inside the room
        n_free = int(room_free.sum())
        n_rel = int(sum(sizes[i] for i in relevant_ids) / cell_m2)
        area_cov = n_free / max(1, n_free + n_rel)

        # How much of the room's EDGE is closed. Its outer rim either touches
        # a wall (or a closure / doorway seal) -- finished -- or unknown space
        # the room may continue into.
        # A thin unseen strip right against a wall (the LiDAR grazing it, the
        # wall's own cell edge) is still a closed edge: only unknown more than
        # wall_slack_m from any wall counts as open.
        rim = ndimage.binary_dilation(room) & ~room
        near_wall = ndimage.distance_transform_edt(~closed) * res <= self.wall_slack
        rim_open = rim & unknown & ~near_wall
        n_rim = int(rim.sum())
        closure = 1.0 - int(rim_open.sum()) / max(1, n_rim)
        coverage = min(area_cov, closure)

        # ---- gaps
        u_any = np.zeros_like(unknown)
        u_any[1:, :] |= unknown[:-1, :]
        u_any[:-1, :] |= unknown[1:, :]
        u_any[:, 1:] |= unknown[:, :-1]
        u_any[:, :-1] |= unknown[:, 1:]
        frontier = free & u_any
        f_labels, n_f = ndimage.label(frontier, structure=np.ones((3, 3)))
        frontiers = []
        for i, sl in enumerate(ndimage.find_objects(f_labels), start=1):
            if sl is None:
                continue
            mask = f_labels[sl] == i
            size = int(mask.sum())
            rows, cols = np.nonzero(mask)
            rows, cols = rows + sl[0].start, cols + sl[1].start
            fx = float(cols.mean() * res + ox + res / 2)
            fy = float(rows.mean() * res + oy + res / 2)
            inside = room_free[rows, cols].mean() > 0.5
            # The unknown pockets this gap opens onto.
            r0, r1 = max(0, sl[0].start - 1), min(h, sl[0].stop + 1)
            c0, c1 = max(0, sl[1].start - 1), min(w, sl[1].stop + 1)
            near = ndimage.binary_dilation(f_labels[r0:r1, c0:c1] == i)
            pockets = np.unique(u_labels[r0:r1, c0:c1][near])
            pockets = pockets[pockets > 0]
            area = min(99.0, float(sum(sizes[p] for p in pockets))) if len(pockets) else 0.0
            open_edge = any(p not in why for p in pockets)
            if not inside:
                status, reason = 'outside', ('beyond a doorway or glass'
                                             if not touches_edge else 'outside the room outline')
            elif size < self.min_cells:
                status, reason = 'ignore', 'too small to matter'
            elif not len(pockets):
                # Its unknown side lies only outside the room: a sealed doorway.
                status, reason = 'outside', 'leads out of the room (doorway)'
            elif len(pockets) and not open_edge:
                status, reason = 'ignore', why[int(pockets[0])]
            elif near_wall[rows, cols].mean() > 0.7:
                # The unseen strip at the foot of a wall, not a way onward.
                status, reason = 'ignore', 'along a wall'
            else:
                status, reason = 'visit', ''
            frontiers.append({'id': len(frontiers) + 1, 'x': round(fx, 2), 'y': round(fy, 2),
                              'size': size, 'status': status, 'reason': reason,
                              'area_m2': round(area, 2)})

        # ---- floor the cameras have not seen up close yet
        view_cov, views = self.view_targets(room_free, ox, oy, res, h, w)
        for v in views:
            frontiers.append({'id': len(frontiers) + 1, 'x': v['x'], 'y': v['y'],
                              'size': v['size'], 'status': 'visit',
                              'reason': 'not seen up close by the cameras yet',
                              'area_m2': v['area_m2'], 'view': True})

        # ---- map quality: live scan vs mapped walls
        q = self.scan_quality(occ, ox, oy, res, w, h)
        if q is not None:
            self.quality = q if self.quality is None else 0.8 * self.quality + 0.2 * q

        # ---- progress and the stop rule
        now = time.monotonic()
        self.history.append((now, coverage))
        while self.history and now - self.history[0][0] > self.stall_window + 5.0:
            self.history.popleft()
        old = [c for t, c in self.history if now - t >= self.stall_window]
        gain = coverage - old[-1] if old else None
        room_m2 = float(room.sum() * cell_m2)
        open_room = touches_edge or room_m2 > self.max_room
        worthwhile = [f for f in frontiers if f['status'] == 'visit'
                      and (f['area_m2'] >= self.worthwhile or f['area_m2'] == 0.0)]
        done, done_reason = False, ''
        if not open_room:
            if coverage >= self.done_coverage and not worthwhile and view_cov >= self.view_done:
                done, done_reason = True, (f'room {coverage:.0%} mapped, {view_cov:.0%} of the '
                                           'floor seen up close, nothing worthwhile left')
        # The 80 % "progress has stalled" rule is reported, NOT acted on here.
        # In the first live run (2026-10-06) the car was stuck behind a bad
        # wall, made no progress for a minute, and this rule declared the
        # room done at ~80 % with the area behind the sofa never seen. Stuck
        # is not finished: the explorer ends below 90 % only once every
        # remaining gap has been tried and found unreachable.
        stalled = (coverage >= self.floor_coverage and gain is not None
                   and gain < self.stall_gain and now - self.started > self.stall_window)

        msg = String()
        msg.data = json.dumps({
            'coverage': round(coverage, 3),
            'area_coverage': round(area_cov, 3),
            'edge_closed': round(closure, 3),
            'view_coverage': round(view_cov, 3),
            'open': open_room,
            'room_m2': round(room_m2, 1),
            'mapped_m2': round(n_free * cell_m2, 1),
            'remaining_m2': round(n_rel * cell_m2, 1),
            'ignored_m2': round(float(sum(sizes[i] for i in why)) if why else 0.0, 1),
            'gain': None if gain is None else round(gain, 3),
            'quality': None if self.quality is None else round(self.quality, 2),
            'quality_poor': self.quality is not None and self.quality < self.poor_quality,
            'frontiers': frontiers,
            'outline': self.outline(room, ox, oy, res),
            'done': done, 'done_reason': done_reason, 'stalled': stalled,
            'go_next_room': self.go_next_room,
        })
        self.pub.publish(msg)

    def scan_quality(self, occ, ox, oy, res, w, h):
        scan = self.scan
        if scan is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id, Time())
        except Exception:                    # noqa: BLE001
            return None
        yaw = yaw_of(tf.transform.rotation)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + scan.angle_increment * np.arange(len(r)) + yaw
        ok = np.isfinite(r) & (r > 0.15) & (r < 6.0)
        c = ((tx + r[ok] * np.cos(a[ok]) - ox) / res).astype(int)
        rr = ((ty + r[ok] * np.sin(a[ok]) - oy) / res).astype(int)
        inside = (c >= 0) & (c < w) & (rr >= 0) & (rr < h)
        if np.count_nonzero(inside) < 30:
            return None
        near_wall = ndimage.binary_dilation(occ, iterations=1)
        return float(near_wall[rr[inside], c[inside]].mean())

    @staticmethod
    def outline(room, ox, oy, res):
        img = room.astype(np.uint8)
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        c = max(contours, key=cv2.contourArea)
        c = cv2.approxPolyDP(c, 2.0, True).reshape(-1, 2)
        return [[round(float(x * res + ox + res / 2), 2), round(float(y * res + oy + res / 2), 2)]
                for x, y in c]


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RoomAnalyzer()
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
