#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Autonomous exploration: map a room without anyone driving.

Frontier-based exploration on top of the live Cartographer map. A "frontier"
is the boundary between mapped-free and still-unknown space: driving to one is
by definition the move that reveals the most new map, which is why this beats
any fixed pattern like a lawnmower sweep -- it adapts to whatever shape the
room actually is, and it terminates naturally when no frontiers remain.

Goals go to the SAME Nav2 stack navigation mode uses (MPPI controller,
Hybrid-A* planner, the Ackermann behaviour tree). That reuse is the point:
obstacle avoidance, the speed/steering limits and the recovery behaviours were
already tuned and proven on this car, so exploration inherits all of it rather
than re-implementing collision avoidance badly.

ACKERMANN REALITIES THIS HAS TO RESPECT
---------------------------------------
This car cannot spin in place; its minimum turning radius is ~0.45 m and the
steering stops at 30 degrees. So:
  * frontier goals closer than `min_goal_distance` are skipped -- asking for a
    near-zero-distance pose is asking for an in-place turn it cannot do;
  * a frontier that fails is BLACKLISTED, with a radius, so the planner is not
    asked the same impossible question forever;
  * repeated consecutive failures trigger a bounded nudge (see `recover`)
    rather than letting the run wedge in a corner.

SAFETY
------
Every exit path -- normal completion, no more frontiers, the time budget,
Ctrl+C, or an e-stop -- cancels the active Nav2 goal. The car is never left
with a goal still executing. The node never publishes motor commands itself;
all motion goes through Nav2 and the existing converter, so the e-stop and
cmd_vel watchdog behave exactly as they do when driving manually.

Published:
  /qcar2/explore_status  std_msgs/String  JSON progress for the web console
  /qcar2/frontier_markers visualization_msgs/MarkerArray
  /qcar2/say             std_msgs/String  spoken progress
Subscribed:
  /map                   nav_msgs/OccupancyGrid  (Cartographer)
  /qcar2_estop           std_msgs/Bool
  /qcar2/explore_enabled std_msgs/Bool   pause/resume from the console
"""

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

UNKNOWN = -1
# Free = "more likely empty than occupied", i.e. occupancy < 50.
#
# This was 25 (map_saver's default free_thresh) and it made the car explore
# NOTHING. Cartographer's default miss_probability is 0.49, so a cell seen
# empty once is published as 49, and it takes ~27 LiDAR passes for it to drift
# below 25. The frontier -- the edge of what has been seen -- is by definition
# the least-observed part of the map and sits in that 30..49 band, so with 25
# no frontier was ever detected, exploration "completed" on the first tick,
# and the car drove "home" to the spot it was already on (GOAL REACHED, zero
# movement). 49 matches Nav2's own static layer, which also treats anything
# below its lethal threshold as traversable.
FREE_MAX = 49          # occupancy <= this is free space
OCCUPIED_MIN = 65      # occupancy >= this is a wall


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Explorer(Node):

    def __init__(self):
        super().__init__('qcar2_explorer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('min_goal_distance', 0.70)
        self.declare_parameter('max_goal_distance', 12.0)
        self.declare_parameter('blacklist_radius', 0.60)
        self.declare_parameter('max_failures_per_frontier', 2)
        self.declare_parameter('goal_timeout_sec', 60.0)
        self.declare_parameter('planning_period_sec', 2.0)
        self.declare_parameter('time_budget_sec', 900.0)
        self.declare_parameter('finish_settle_sec', 6.0)
        # Distance/size trade. Higher = greedier about going to the nearest
        # frontier; lower = more willing to cross the room for a big one.
        self.declare_parameter('distance_weight', 0.35)
        self.declare_parameter('return_home', True)
        self.declare_parameter('announce', True)
        # Goals are placed INSIDE observed free space, not on the frontier
        # line itself: a goal on the unknown edge turns into a wall the moment
        # the LiDAR reveals one there, and Nav2 aborts. The LiDAR still sees
        # 12 m into the unknown area from the pulled-back point.
        self.declare_parameter('wall_clearance', 0.40)
        self.declare_parameter('unknown_backoff', 0.35)
        self.declare_parameter('approach_radius', 1.5)
        # Failed frontiers are retried after this long: the map keeps growing,
        # and a frontier that was unreachable a minute ago often is not now.
        self.declare_parameter('blacklist_ttl_sec', 120.0)
        # TWO PHASES. Exploring every frontier equally sent the car poking
        # into every gap under a chair and behind a cabinet. Instead:
        #   main   -- only big openings (>= main_min_cells of frontier, i.e.
        #             1 m at 5 cm cells) whose goal has main_clearance of room:
        #             the open body of the room, driven comfortably.
        #   detail -- afterwards, smaller openings (>= detail_min_cells) that
        #             can be reached with wall_clearance (the 40 cm rule), for
        #             at most detail_budget_sec. Anything smaller is never
        #             chased: it is under or behind furniture.
        self.declare_parameter('main_min_cells', 20)
        self.declare_parameter('main_clearance', 0.55)
        self.declare_parameter('detail_min_cells', 10)
        self.declare_parameter('detail_budget_sec', 180.0)

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.min_cells = int(g('min_frontier_cells'))
        self.min_goal_distance = float(g('min_goal_distance'))
        self.max_goal_distance = float(g('max_goal_distance'))
        self.blacklist_radius = float(g('blacklist_radius'))
        self.max_failures = int(g('max_failures_per_frontier'))
        self.goal_timeout = float(g('goal_timeout_sec'))
        self.time_budget = float(g('time_budget_sec'))
        self.finish_settle = float(g('finish_settle_sec'))
        self.distance_weight = float(g('distance_weight'))
        self.return_home = bool(g('return_home'))
        self.announce = bool(g('announce'))
        self.wall_clearance = float(g('wall_clearance'))
        self.unknown_backoff = float(g('unknown_backoff'))
        self.approach_radius = float(g('approach_radius'))
        self.blacklist_ttl = float(g('blacklist_ttl_sec'))
        self.phases = {
            'main': (int(g('main_min_cells')), float(g('main_clearance'))),
            'detail': (int(g('detail_min_cells')), self.wall_clearance),
        }
        self.detail_budget = float(g('detail_budget_sec'))
        self.phase = 'main'
        self.detail_started = None

        self.map = None
        self.enabled = True
        self.estop = False
        self.goal_handle = None
        # send_goal_async does not hand back a handle immediately. Without a
        # separate "in flight" flag, the next planning tick would see
        # goal_handle still None and send a SECOND goal for the same frontier.
        self.goal_pending = False
        self.goal_sent_at = 0.0
        self.current_goal = None
        self.blacklist = []                  # [(x, y, strikes, last_strike_time)]
        self.retried = False                 # one retry pass before giving up
        self.state = 'starting'              # what it is doing, for the console
        self.reason = ''                     # why, when it is not driving
        self.frontier_count = 0
        self._clearance = None               # (map stamp, ok-mask) cache
        self.home = None
        self.started_at = time.monotonic()
        self.finished = False
        self.empty_since = None
        self.visited = 0
        self.failed = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, '/qcar2/explore_status', LATCHED)
        self.say_pub = self.create_publisher(String, '/qcar2/say', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/qcar2/frontier_markers', 1)

        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(Bool, '/qcar2_estop', self.on_estop, LATCHED)
        self.create_subscription(Bool, '/qcar2/explore_enabled', self.on_enabled, LATCHED)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(float(g('planning_period_sec')), self.tick)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Explorer up; waiting for the map and Nav2.')

    # ----------------------------------------------------------------- input

    def on_map(self, msg):
        self.map = msg

    def on_estop(self, msg):
        engaged = bool(msg.data)
        if engaged and not self.estop:
            self.get_logger().warn('E-stop engaged -- cancelling the exploration goal.')
            self.cancel_goal()
        self.estop = engaged

    def on_enabled(self, msg):
        want = bool(msg.data)
        if want == self.enabled:
            return
        self.enabled = want
        self.get_logger().info(f'Exploration {"resumed" if want else "paused"}')
        if not want:
            self.cancel_goal()

    def car_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:                    # noqa: BLE001 - TF not ready yet
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # ------------------------------------------------------------- frontiers

    def find_frontiers(self):
        """Cluster the free/unknown boundary into candidate goals.

        Returns [(x, y, size)] in map-frame metres.
        """
        grid = self.map
        if grid is None:
            return []
        info = grid.info
        w, h = info.width, info.height
        if w == 0 or h == 0:
            return []
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)

        free = (data >= 0) & (data <= FREE_MAX)
        unknown = (data == UNKNOWN)
        # A frontier cell is free space with unknown space orthogonally
        # adjacent. Shifted slices rather than a Python loop: this runs every
        # planning tick on a grid that can be 2000x2000.
        nb = np.zeros_like(unknown)
        nb[1:, :] |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:, :]
        nb[:, 1:] |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        frontier = free & nb
        if not np.any(frontier):
            return []

        # Connected-component labelling (8-connected) via BFS over the mask.
        labels = np.zeros((h, w), dtype=np.int32)
        clusters = []
        cells = np.argwhere(frontier)
        current = 0
        for r0, c0 in cells:
            if labels[r0, c0]:
                continue
            current += 1
            queue = deque([(r0, c0)])
            labels[r0, c0] = current
            members = []
            while queue:
                r, c = queue.popleft()
                members.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < h and 0 <= cc < w and frontier[rr, cc] \
                                and not labels[rr, cc]:
                            labels[rr, cc] = current
                            queue.append((rr, cc))
            if len(members) < self.min_cells:
                continue
            arr = np.array(members, dtype=float)
            cy = arr[:, 0].mean() * info.resolution + info.origin.position.y
            cx = arr[:, 1].mean() * info.resolution + info.origin.position.x
            clusters.append((cx, cy, len(members)))
        return clusters

    def blacklisted(self, x, y):
        now = time.monotonic()
        ttl = getattr(self, 'blacklist_ttl', 0.0)
        for bx, by, strikes, *rest in self.blacklist:
            if math.hypot(x - bx, y - by) >= self.blacklist_radius:
                continue
            if strikes < self.max_failures:
                continue
            # Expired strikes no longer count: the map has moved on since.
            if ttl and rest and now - rest[0] > ttl:
                continue
            return True
        return False

    def strike(self, x, y):
        now = time.monotonic()
        for i, (bx, by, strikes, *_rest) in enumerate(self.blacklist):
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                self.blacklist[i] = (bx, by, strikes + 1, now)
                return
        self.blacklist.append((x, y, 1, now))

    def rank(self, frontiers, car, min_cells=0):
        """All usable frontiers, best first: big and close beats small and far."""
        scored = []
        for fx, fy, size in frontiers:
            if size < min_cells:
                continue
            d = math.hypot(fx - car[0], fy - car[1])
            # Too close to steer to (Ackermann), or implausibly far.
            if d < self.min_goal_distance or d > self.max_goal_distance:
                continue
            if self.blacklisted(fx, fy):
                continue
            score = math.sqrt(size) - self.distance_weight * d
            scored.append((score, (fx, fy, size, d)))
        scored.sort(key=lambda t: -t[0])
        return [c for _s, c in scored]

    def choose(self, frontiers, car):
        ranked = self.rank(frontiers, car)
        return ranked[0] if ranked else None

    def clearance_mask(self, clearance=None):
        """Cells that are safe to PARK on: observed free, at least `clearance`
        from walls, and pulled back from the unknown edge. The distance
        transforms (the expensive part of a planning tick) are cached per map
        update; each clearance level is then just a threshold."""
        clearance = self.wall_clearance if clearance is None else clearance
        grid = self.map
        stamp = (grid.header.stamp.sec, grid.header.stamp.nanosec,
                 grid.info.width, grid.info.height)
        if self._clearance is None or self._clearance[0] != stamp:
            from scipy.ndimage import distance_transform_edt
            h, w = grid.info.height, grid.info.width
            res = grid.info.resolution
            data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
            free = (data >= 0) & (data <= FREE_MAX)
            d_wall = distance_transform_edt(data < OCCUPIED_MIN) * res
            d_unknown = distance_transform_edt(data != UNKNOWN) * res
            self._clearance = (stamp, free & (d_unknown >= self.unknown_backoff), d_wall)
        _stamp, base, d_wall = self._clearance
        return base & (d_wall >= clearance)

    def approach_goal(self, fx, fy, clearance=None):
        """Nearest safe parking cell to a frontier point, or None.

        The frontier point itself is a bad goal: it sits on the unknown edge
        (and, for a curved frontier, the centroid may not even be on it). The
        car only needs to get CLOSE -- its LiDAR sees far into the unknown
        area from a couple of metres back.
        """
        grid = self.map
        if grid is None:
            return None
        info = grid.info
        res = info.resolution
        ok = self.clearance_mask(clearance)
        h, w = ok.shape
        c0 = int((fx - info.origin.position.x) / res)
        r0 = int((fy - info.origin.position.y) / res)
        rad = max(1, int(self.approach_radius / res))
        r_lo, r_hi = max(0, r0 - rad), min(h, r0 + rad + 1)
        c_lo, c_hi = max(0, c0 - rad), min(w, c0 + rad + 1)
        if r_lo >= r_hi or c_lo >= c_hi:
            return None
        cells = np.argwhere(ok[r_lo:r_hi, c_lo:c_hi])
        if cells.size == 0:
            return None
        cells = cells + np.array([r_lo, c_lo])
        d2 = (cells[:, 0] - r0) ** 2 + (cells[:, 1] - c0) ** 2
        r, c = cells[int(np.argmin(d2))]
        if (r - r0) ** 2 + (c - c0) ** 2 > rad * rad:
            return None
        return (float(c * res + info.origin.position.x + res / 2),
                float(r * res + info.origin.position.y + res / 2))

    # ----------------------------------------------------------------- goals

    def idle(self, state, reason=''):
        """Record WHY the car is not moving, for the console. Every early
        return in tick() goes through here, so "why isn't it driving?" is
        always answered on screen instead of needing the terminal log."""
        if (state, reason) != (self.state, self.reason):
            self.get_logger().info(f'[{state}] {reason}' if reason else f'[{state}]')
        self.state, self.reason = state, reason

    def tick(self):
        if self.finished:
            return
        if time.monotonic() - self.started_at > self.time_budget:
            return self.finish('time budget reached')
        if self.estop:
            return self.idle('paused', 'emergency stop is engaged -- release it to continue')
        if not self.enabled:
            return self.idle('paused', 'paused from the console')
        if self.map is None:
            return self.idle('waiting', 'waiting for the first map from Cartographer')
        if not self.nav.server_is_ready():
            return self.idle('waiting', 'waiting for Nav2 to come up')

        car = self.car_xy()
        if car is None:
            return self.idle('waiting', 'waiting for the car position (map -> base_link)')
        if self.home is None:
            self.home = car

        # A goal in flight (accepted, or still being acknowledged): let it run
        # unless it has overrun its timeout.
        if self.goal_handle is not None or self.goal_pending:
            if time.monotonic() - self.goal_sent_at > self.goal_timeout:
                self.get_logger().warn('Goal timed out; blacklisting and moving on.')
                if self.current_goal:
                    self.strike(*self.current_goal)
                self.failed += 1
                self.cancel_goal()
            return

        frontiers = self.find_frontiers()
        self.frontier_count = len(frontiers)
        self.publish_frontier_markers(frontiers)

        # Best frontier that we can actually park near. One that has nowhere
        # safe to stop (e.g. a gap between two chair legs) gets a strike and we
        # fall through to the next, rather than sending Nav2 a doomed goal.
        if self.phase == 'detail' and \
                time.monotonic() - self.detail_started > self.detail_budget:
            return self.finish('main area mapped; time for the smaller corners used up')
        min_cells, clearance = self.phases[self.phase]
        target = None
        for fx, fy, size, dist in self.rank(frontiers, car, min_cells):
            goal = self.approach_goal(fx, fy, clearance)
            if goal is None:
                self.strike(fx, fy)
                continue
            if math.hypot(goal[0] - car[0], goal[1] - car[1]) < self.min_goal_distance:
                # Already as close as we can usefully get; the LiDAR is
                # looking at it right now. Count it and move on.
                self.strike(fx, fy)
                continue
            target = (fx, fy, size, goal)
            break

        if target is None and self.phase == 'main':
            # The open body of the room is done. Now the smaller, still
            # easy-to-reach openings, with a fresh blacklist (a spot that was
            # too tight for the main phase's clearance may be fine now).
            self.phase = 'detail'
            self.detail_started = time.monotonic()
            self.blacklist = []
            self.get_logger().info('Main area mapped; now the smaller openings that are '
                                   'easy to reach.')
            if self.announce:
                self.say('Main area mapped. Checking the remaining corners.')
            return self.idle('searching', 'main area done; looking at smaller openings')

        if target is None:
            if self.blacklist and not self.retried:
                # Before declaring the room done, give every failed frontier
                # one more go on the now-larger map.
                self.retried = True
                self.blacklist = []
                return self.idle('searching', 'retrying frontiers that failed earlier')
            # Settle briefly before declaring done, because a single tick
            # with no frontier can just be a partially-updated map
            # mid-loop-closure rather than genuine completion.
            if self.empty_since is None:
                self.empty_since = time.monotonic()
            elif time.monotonic() - self.empty_since > self.finish_settle:
                return self.finish('no reachable frontiers left')
            return self.idle('searching', f'{len(frontiers)} frontiers, none reachable yet')
        self.empty_since = None

        fx, fy, size, (gx, gy) = target
        # Face the unexplored area on arrival, so the cameras and LiDAR are
        # looking at what we came to see.
        yaw = math.atan2(fy - gy, fx - gx) if math.hypot(fy - gy, fx - gx) > 0.1 \
            else math.atan2(gy - car[1], gx - car[0])
        self.send_goal(gx, gy, yaw)
        self.current_goal = (fx, fy)
        dist = math.hypot(gx - car[0], gy - car[1])
        self.idle('driving', f'to a frontier {dist:.1f} m away ({size} cells)')
        self.get_logger().info(
            f'-> frontier ({fx:.2f}, {fy:.2f}) via ({gx:.2f}, {gy:.2f}), {size} cells, '
            f'{dist:.1f} m [visited {self.visited}, failed {self.failed}]')

    def send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        z, w = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = z, w
        self.goal_sent_at = time.monotonic()
        self.goal_pending = True
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:             # noqa: BLE001 - keep exploring
            self.get_logger().warn(f'Goal rejected: {exc}')
            self.goal_handle = None
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the frontier goal; blacklisting it.')
            if self.current_goal:
                self.strike(*self.current_goal)
            self.failed += 1
            self.goal_handle = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        status = None
        try:
            status = future.result().status
        except Exception as exc:             # noqa: BLE001 - keep exploring
            self.get_logger().warn(f'Goal result error: {exc}')
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.visited += 1
        else:
            # Aborted/cancelled: mark it so we do not keep retrying a frontier
            # this Ackermann car physically cannot reach.
            if self.current_goal:
                self.strike(*self.current_goal)
            self.failed += 1
        self.current_goal = None

    def cancel_goal(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:                # noqa: BLE001 - already gone
                pass
            self.goal_handle = None
        self.goal_pending = False
        self.current_goal = None

    # ------------------------------------------------------------- finishing

    def finish(self, reason):
        if self.finished:
            return
        self.finished = True
        self.cancel_goal()
        self.get_logger().info(
            f'Exploration complete ({reason}): {self.visited} frontiers reached, '
            f'{self.failed} failed.')
        self.state = 'finished'
        self.reason = reason
        if self.visited == 0:
            # Nothing was ever reached. Driving "home" would just produce a
            # GOAL REACHED on the spot it is already sitting on -- which
            # looks like success and hides the real problem.
            self.reason = f'{reason}; no frontier was reached ({self.failed} failed)'
            self.get_logger().warn(f'Exploration ended without reaching any frontier: {reason}')
            if self.announce:
                self.say('Exploration stopped. I could not reach anywhere new.')
            self.publish_status()
            return
        if self.announce:
            self.say('Exploration complete. You can save the map.')
        if self.return_home and self.home is not None:
            self.get_logger().info('Returning to the start position.')
            car = self.car_xy() or self.home
            yaw = math.atan2(self.home[1] - car[1], self.home[0] - car[0])
            self.send_goal(self.home[0], self.home[1], yaw)
        self.publish_status()

    def say(self, text):
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)

    # --------------------------------------------------------------- outputs

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'running': not self.finished and self.enabled and not self.estop,
            'finished': self.finished,
            'paused': not self.enabled,
            'estop': self.estop,
            'visited': self.visited,
            'failed': self.failed,
            'blacklisted': len(self.blacklist),
            'state': self.state,
            'reason': self.reason,
            'phase': self.phase,
            'frontiers': self.frontier_count,
            'elapsed': round(time.monotonic() - self.started_at, 1),
            'budget': self.time_budget,
            'goal': ({'x': round(self.current_goal[0], 2),
                      'y': round(self.current_goal[1], 2)}
                     if self.current_goal else None),
        })
        self.status_pub.publish(msg)

    def publish_frontier_markers(self, frontiers):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        now = self.get_clock().now().to_msg()
        for i, (fx, fy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(fx), y=float(fy), z=0.05)
            m.pose.orientation.w = 1.0
            scale = min(0.5, 0.10 + 0.01 * size)
            m.scale = Vector3(x=scale, y=scale, z=scale)
            dead = self.blacklisted(fx, fy)
            m.color = (ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.5) if dead
                       else ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.85))
            m.lifetime = DurationMsg(sec=0)
            array.markers.append(m)
        self.marker_pub.publish(array)

    def destroy_node(self):
        # Never leave a goal executing with no node left to manage it.
        self.cancel_goal()
        super().destroy_node()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Explorer()
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
