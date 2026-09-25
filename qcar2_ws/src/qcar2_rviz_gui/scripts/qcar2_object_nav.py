#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Drive to a mapped object by name: "go to the air cooler".

Loads the semantic layer that qcar2_object_mapper.py saved next to the map
(<map>_objects.json), matches a spoken or typed name against it, and turns the
match into a Nav2 goal.

WHY THE GOAL IS NOT THE OBJECT'S COORDINATE
-------------------------------------------
A landmark's (x, y) is a point ON the object -- the LiDAR centroid of the
surface the camera saw. Sending that straight to Nav2 asks the car to drive
*into* the sofa, and the planner either refuses (the cell is lethal in the
costmap) or plans right up against it. So the goal is a STANDOFF pose: a free
cell ~0.7 m short of the object, on the side the car is approaching from, with
the heading turned to face the object. You end up parked in front of the
thing, looking at it, which is what "go to the air cooler" actually means.

If the natural standoff cell is blocked, candidate angles are swept around the
object (and the standoff widened) until a free one is found, so a cooler
shoved into a corner is still reachable from whichever side is open.

NAME MATCHING
-------------
Four passes, most precise first: synonym table from the vocabulary YAML
("fridge" -> "refrigerator"), exact label, substring, then difflib fuzzy match.
This is deliberately more forgiving than the detector's vocabulary, because a
speech recogniser will hand over "air cooler" as "aircooler" often enough to
matter. When several landmarks share the matched label, the one NEAREST the
car wins -- with four chairs in a room, "go to the chair" should mean the one
you are next to, not an arbitrary one.

Subscribed:
  /qcar2/nav_to_object   std_msgs/String   object name to drive to
  /qcar2/objects         std_msgs/String   live landmark list (optional)
Published:
  /goal_pose_raw         geometry_msgs/PoseStamped   (qcar2_goal_heading re-aims it)
  /qcar2/object_markers  visualization_msgs/MarkerArray
  /qcar2/objects         std_msgs/String   the loaded map's landmarks (latched)
  /qcar2/say             std_msgs/String   spoken confirmation
  /qcar2/nav_object_status std_msgs/String JSON result of the last request
"""

import difflib
import json
import math
import os
import re

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Filler words a spoken command carries that never help identify an object.
STOPWORDS = re.compile(
    r'^(please\s+)?(go|drive|move|navigate|head|take\s+me|get)?\s*'
    r'(to|towards|toward|near|at|over\s+to)?\s*(the|a|an|my)?\s+', re.I)


def normalize(text):
    text = (text or '').strip().lower()
    text = re.sub(r'[^a-z0-9\s]', ' ', text)
    text = re.sub(r'\s+', ' ', text).strip()
    prev = None
    while prev != text:
        prev = text
        text = STOPWORDS.sub('', text, count=1).strip()
    return text


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ObjectNav(Node):

    def __init__(self):
        super().__init__('qcar2_object_nav')
        project_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Qcar-rviz')

        self.declare_parameter('objects_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('standoff', 0.70)
        self.declare_parameter('min_standoff', 0.45)
        self.declare_parameter('max_standoff', 1.60)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('lethal_threshold', 60)
        self.declare_parameter('objects_dir', os.path.join(project_dir, 'maps'))
        self.declare_parameter('publish_objects', True)

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.standoff = float(g('standoff'))
        self.min_standoff = float(g('min_standoff'))
        self.max_standoff = float(g('max_standoff'))
        self.lethal = int(g('lethal_threshold'))
        self.publish_list = bool(g('publish_objects'))

        self.objects = []
        self.synonyms = {}
        self.costmap = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose_raw', 1)
        self.say_pub = self.create_publisher(String, '/qcar2/say', 10)
        self.status_pub = self.create_publisher(String, '/qcar2/nav_object_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/qcar2/object_markers', LATCHED)
        self.objects_pub = (self.create_publisher(String, '/qcar2/objects', LATCHED)
                            if self.publish_list else None)

        self.create_subscription(String, '/qcar2/nav_to_object', self.on_request, 10)
        self.create_subscription(OccupancyGrid, g('costmap_topic'), self.on_costmap, 1)
        # If the mapper is running (explore mode), prefer its live list over the
        # file on disk so a just-seen object is immediately addressable.
        self.create_subscription(String, '/qcar2/objects', self.on_live_objects, 1)

        self.load(g('objects_file'))
        self.create_timer(2.0, self.publish_state)

    # ----------------------------------------------------------------- data

    def load(self, path):
        path = (path or '').strip()
        if not path:
            self.get_logger().info('No objects_file given; waiting for a live list.')
            return
        # Accept either the objects JSON itself or the map YAML it sits beside.
        if path.endswith('.yaml'):
            path = path[:-5] + '_objects.json'
        elif not path.endswith('.json'):
            path = path + '_objects.json'
        if not os.path.isabs(path):
            path = os.path.join(self.get_parameter('objects_dir').value, path)
        if not os.path.isfile(path):
            self.get_logger().warn(
                f'No semantic layer at {path}. Navigation by object name will not work '
                f'until you map with detect_objects:=true and save.')
            return
        try:
            with open(path) as fh:
                data = json.load(fh)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Could not read {path}: {exc}')
            return
        self.objects = data.get('objects') or []
        self.synonyms = {k.lower(): v for k, v in (data.get('synonyms') or {}).items()}
        labels = sorted({o['label'] for o in self.objects})
        self.get_logger().info(
            f'Loaded {len(self.objects)} objects from {os.path.basename(path)}: '
            f'{", ".join(labels) if labels else "(none)"}')

    def on_live_objects(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        # Ignore our own republished list: this node both publishes and
        # subscribes to /qcar2/objects, so without this it would feed itself.
        if (data.get('stats') or {}).get('source') == 'saved map':
            return
        objects = data.get('objects')
        # Only take over from the file once the mapper actually has something,
        # so an empty startup publish cannot wipe a good loaded map.
        if objects:
            self.objects = objects

    def on_costmap(self, msg):
        self.costmap = msg

    # -------------------------------------------------------------- matching

    def match(self, query):
        """Spoken/typed name -> label, most precise pass first."""
        q = normalize(query)
        if not q:
            return None
        labels = {o['label'] for o in self.objects}
        if not labels:
            return None
        canonical = self.synonyms.get(q)
        if canonical and canonical in labels:
            return canonical
        for label in labels:
            if q == label.lower():
                return label
        # Substring either way: "cooler" matches "air cooler", and "air cooler
        # thing" matches "air cooler".
        contains = [l for l in labels if q in l.lower() or l.lower() in q]
        if contains:
            return min(contains, key=len)
        alias_hits = [c for a, c in self.synonyms.items()
                      if c in labels and (q in a or a in q)]
        if alias_hits:
            return min(alias_hits, key=len)
        close = difflib.get_close_matches(q, [l.lower() for l in labels], n=1, cutoff=0.7)
        if close:
            for label in labels:
                if label.lower() == close[0]:
                    return label
        return None

    def car_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:                             # noqa: BLE001 - not localized yet
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # ------------------------------------------------------------- planning

    def cell_free(self, x, y):
        """Is this map coordinate a safe place to park? Unknown counts as free.

        Unknown is treated as acceptable on purpose: the global costmap only
        covers what has been observed, and refusing unknown cells would make
        objects at the edge of the mapped area permanently unreachable.
        """
        grid = self.costmap
        if grid is None:
            return True
        info = grid.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return True
        value = grid.data[row * info.width + col]
        return value < self.lethal

    def standoff_goal(self, obj):
        """A free pose ~standoff metres from the object, facing it."""
        ox, oy = float(obj['x']), float(obj['y'])
        car = self.car_xy()
        # Approach from the car's side when we know where the car is, so the
        # route does not loop around the object for no reason.
        base = math.atan2((car[1] - oy), (car[0] - ox)) if car else 0.0

        for dist in (self.standoff,
                     self.standoff + 0.3,
                     self.min_standoff,
                     self.max_standoff):
            # 0 first = straight in from the car; then progressively further
            # around the object, alternating sides.
            for step in range(0, 13):
                offset = (step // 2 + 1) * math.radians(30) * (1 if step % 2 else -1)
                angle = base + (0.0 if step == 0 else offset)
                gx, gy = ox + dist * math.cos(angle), oy + dist * math.sin(angle)
                if self.cell_free(gx, gy):
                    return gx, gy, math.atan2(oy - gy, ox - gx), dist
        return None

    # -------------------------------------------------------------- requests

    def on_request(self, msg):
        query = msg.data or ''
        if not self.objects:
            return self.fail(query, 'No objects are mapped. Map with detect_objects:=true first.')
        label = self.match(query)
        if label is None:
            known = sorted({o['label'] for o in self.objects})
            return self.fail(query, f'I do not know "{normalize(query)}". '
                                    f'Mapped objects: {", ".join(known)}')

        candidates = [o for o in self.objects if o['label'] == label]
        car = self.car_xy()
        if car and len(candidates) > 1:
            # Several of the same thing: mean the closest one.
            candidates.sort(key=lambda o: math.hypot(o['x'] - car[0], o['y'] - car[1]))
        target = candidates[0]

        goal = self.standoff_goal(target)
        if goal is None:
            return self.fail(query, f'I found the {label} but there is no clear space '
                                    f'to stop near it.')
        gx, gy, yaw, dist = goal

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = gx, gy
        z, w = yaw_to_quaternion(yaw)
        pose.pose.orientation.z, pose.pose.orientation.w = z, w
        self.goal_pub.publish(pose)

        self.say(f'Going to the {label}')
        self.get_logger().info(
            f'"{query}" -> {label} at ({target["x"]:.2f}, {target["y"]:.2f}); '
            f'goal ({gx:.2f}, {gy:.2f}) standoff {dist:.2f} m')
        self.publish_status({
            'ok': True, 'query': query, 'label': label,
            'object': {'x': target['x'], 'y': target['y']},
            'goal': {'x': round(gx, 3), 'y': round(gy, 3), 'yaw': round(yaw, 3)},
            'alternatives': len(candidates),
        })

    def fail(self, query, reason):
        self.get_logger().warn(reason)
        self.say(reason)
        self.publish_status({'ok': False, 'query': query, 'reason': reason})

    def say(self, text):
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)

    def publish_status(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    # --------------------------------------------------------------- display

    def publish_state(self):
        if self.objects_pub is not None:
            msg = String()
            msg.data = json.dumps({'objects': self.objects,
                                   'stats': {'confirmed': len(self.objects),
                                             'source': 'saved map'}})
            self.objects_pub.publish(msg)
        self.publish_markers()

    def publish_markers(self):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        now = self.get_clock().now().to_msg()
        for i, obj in enumerate(self.objects):
            body = Marker()
            body.header.frame_id = self.map_frame
            body.header.stamp = now
            body.ns = 'objects'
            body.id = i * 2
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position = Point(x=float(obj['x']), y=float(obj['y']), z=0.15)
            body.pose.orientation.w = 1.0
            body.scale = Vector3(x=0.28, y=0.28, z=0.30)
            body.color = ColorRGBA(r=0.15, g=0.85, b=0.95, a=0.75)
            body.lifetime = DurationMsg(sec=0)
            array.markers.append(body)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = 'object_labels'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=float(obj['x']), y=float(obj['y']), z=0.45)
            text.pose.orientation.w = 1.0
            text.scale = Vector3(x=0.0, y=0.0, z=0.18)
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            text.text = obj['label']
            text.lifetime = DurationMsg(sec=0)
            array.markers.append(text)
        self.marker_pub.publish(array)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ObjectNav()
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
