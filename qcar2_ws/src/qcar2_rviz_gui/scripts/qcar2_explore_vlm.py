#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""The explorer's eyes and judgement: the vision-language model (VLM).

Geometry (qcar2_room_analyzer.py) measures the room -- outline, coverage,
which gaps are left -- and decides when mapping is finished. What geometry
cannot tell apart, this node asks the VLM (Cosmos-Reason2 on llama-server,
127.0.0.1:8090) by LOOKING through the cameras:

  * glass -- a glass wall, glass door or floor-length window. The LiDAR and
    the depth camera both see straight through it, so the map shows open
    floor beyond and the planner drives into the pane;
  * a doorway into another room (followed only with go_next_room:=true);
  * whether the unexplored area the car just drove to is worth it, or is
    under / behind furniture.

WHEN IT LOOKS (event-driven, so it does not starve the object detector of
GPU time):
  arrived   the explorer reached a gap and pauses ~briefly; all four cameras
            (front = RealSense colour) are checked, then /qcar2/look_done
            lets it carry on;
  periodic  every `period_sec` while exploring: ALL FOUR cameras in one
            request, as a labelled 2x2 mosaic;
  trouble   bumped into something / a goal failed: front and rear;
  review    when coverage first passes 50 % and 80 %: the map drawn with the
            remaining gaps numbered, plus a text summary -- room type, which
            gaps matter;
  finished  a final review: the spoken + on-screen report.

HOW ANSWERS BECOME ACTIONS (published on /qcar2/explore_hints, latched):
  closures      glass / doorway seen in some direction -> the gap in the
                mapped wall along that direction is found by ray casting the
                map and recorded as a SUSPECTED line (after `confirmations`
                sightings). Suspected lines never block driving: the model
                knows the direction, not the distance, and the first opening
                along a ray can be a gap between furniture (live run
                2026-10-06: it walled off open floor next to the car). Glass
                becomes a real no-go wall (costmap + saved map, via
                qcar2_bump_guard.py) only when the car bumps within 0.5 m of
                the line; doorways stay suggestions.
  skip_regions  gaps the model judged not worth visiting; the explorer will
                not drive there.
  report        room type, coverage, what was skipped and why.

Undo from the console: /qcar2/explore_hints_clear ('glass', 'doorway',
'skips' or 'all').

Never commands the motors. If the model server is not running, the
exploration simply runs on geometry alone.
"""

import base64
import json
import math
import re
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)

# Camera name -> (topic, heading relative to the car, horizontal FOV).
CAMERAS = {
    'front': ('/front/camera/preview', 0.0, math.radians(70.0)),
    'left': ('/left/camera/preview', math.pi / 2, math.radians(90.0)),
    'rear': ('/rear/camera/preview', math.pi, math.radians(90.0)),
    'right': ('/right/camera/preview', -math.pi / 2, math.radians(90.0)),
}
WHERE = ('none', 'left', 'center', 'right')
AHEAD = ('open_floor', 'under_furniture', 'wall', 'glass', 'doorway', 'cluttered')

LOOK_PROMPT = (
    'This photo is from the {cam} camera of a small robot car, 18 cm above the '
    'floor, that is mapping a room. Answer with JSON only, no explanation:\n'
    '{{"glass": "none|left|center|right", "doorway": "none|left|center|right", '
    '"ahead": "open_floor|under_furniture|wall|glass|doorway|cluttered", '
    '"room_type": "<one or two words>"}}\n'
    'glass = a transparent glass wall, glass door or floor-length window the car '
    'could drive into (NOT a TV, picture frame or mirror on a solid wall). '
    'doorway = a door frame or open passage IN A WALL that leads out of this room '
    'into another room or corridor; a window, glass wall or open floor is NOT a '
    'doorway -- answer none unless you clearly see the door frame or wall opening. '
    'ahead = what fills the middle of the photo at floor level.')

MOSAIC_PROMPT = (
    'These are the four cameras of a small robot car, 18 cm above the floor, '
    'mapping a room: top-left FRONT, top-right LEFT, bottom-left REAR, '
    'bottom-right RIGHT (each labelled). For EACH camera answer with JSON only:\n'
    '{{"front": {{"glass": "none|left|center|right", "doorway": "none|left|center|right"}}, '
    '"left": {{...same...}}, "rear": {{...same...}}, "right": {{...same...}}, '
    '"room_type": "<one or two words>"}}\n'
    'glass = a transparent glass wall, glass door or floor-length window the car could '
    'drive into (NOT a TV, picture or mirror). doorway = a door frame or open passage '
    'IN A WALL leading out of this room; answer none unless you clearly see one.')

REVIEW_PROMPT = (
    'This is the map a robot car has built of a room, seen from above. White = '
    'floor it has seen, black = walls and furniture, grey = not seen yet, green '
    'line = the room outline, numbered orange dots = gaps it could still drive to, '
    'blue = the car.\n{summary}\n'
    'Answer with JSON only: {{"room_type": "<one or two words>", '
    '"skip": [<numbers of gaps NOT worth visiting, e.g. behind furniture or in a '
    'corner too small to matter>], "comment": "<one short sentence about the map>"}}')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_bgr(msg):
    if msg.encoding in ('bgr8', 'rgb8'):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()
    if msg.encoding in ('bgra8', 'rgba8'):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_RGBA2BGR if msg.encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    return None


def seg_distance(px, py, c):
    """Distance from point (px, py) to closure segment c."""
    ax, ay, bx, by = c['x1'], c['y1'], c['x2'], c['y2']
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / max(1e-9, dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def first_json(text):
    """The first {...} object in a model reply, or None."""
    if not text:
        return None
    text = re.sub(r'<think>.*?</think>', '', text, flags=re.S)
    m = re.search(r'\{.*\}', text, flags=re.S)
    if not m:
        return None
    try:
        return json.loads(m.group(0))
    except ValueError:
        return None


class ExploreVlm(Node):

    def __init__(self):
        super().__init__('qcar2_explore_vlm')
        g = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.llm_url = g('llm_url', 'http://127.0.0.1:8090')
        self.timeout = float(g('request_timeout_sec', 45.0))
        self.period = float(g('period_sec', 15.0))
        self.confirmations = int(g('confirmations', 2))
        self.go_next_room = bool(g('go_next_room', False))
        self.map_frame = g('map_frame', 'map')
        self.base_frame = g('base_frame', 'base_link')
        self.announce = bool(g('announce', True))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.hints_pub = self.create_publisher(String, '/qcar2/explore_hints', LATCHED)
        self.look_done_pub = self.create_publisher(String, '/qcar2/look_done', 10)
        self.say_pub = self.create_publisher(String, '/qcar2/say', 10)
        self.images = {}
        for name, (topic, _h, _f) in CAMERAS.items():
            self.create_subscription(Image, topic, lambda m, n=name: self.on_image(n, m),
                                     qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(String, '/qcar2/room_status', self.on_status, LATCHED)
        self.create_subscription(String, '/qcar2/explore_event', self.on_event, 10)
        self.create_subscription(String, '/qcar2/explore_hints_clear', self.on_clear, 10)
        self.create_subscription(PointCloud2, '/qcar2/bump_obstacles', self.on_bumps, LATCHED)

        self.map = None
        self.status = None
        # Re-entrant: note() publishes the hints, and is called from inside
        # propose()'s locked section.
        self.lock = threading.RLock()
        self.jobs = deque()
        self.wake = threading.Event()
        self.llm_ok = False
        self.reviewed = set()
        self.exploring = False
        self.finished = False
        self.last_event = 0.0
        self.candidates = []                 # unconfirmed closures
        self.closures = []
        self.skips = []
        self.notes = deque(maxlen=12)
        self.room_type = ''
        self.report = None
        self.closure_id = 0
        self.bumped_at = 0.0

        self.create_timer(self.period, self.on_period)
        threading.Thread(target=self.worker, daemon=True).start()
        self.publish_hints()
        self.get_logger().info(f'Explore VLM up (model server {self.llm_url}, '
                               f'go_next_room={self.go_next_room}).')

    # ----------------------------------------------------------------- input

    def on_image(self, name, msg):
        self.images[name] = (time.monotonic(), msg)

    def on_map(self, msg):
        self.map = msg

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            return
        # Room status flows only while exploring: start the periodic looks
        # from the very first drive, not from the first reached gap.
        if not self.finished:
            self.exploring = True
        cov = self.status.get('coverage', 0.0)
        for mark in (0.5, 0.8):
            if cov >= mark and mark not in self.reviewed:
                self.reviewed.add(mark)
                self.queue('review', tag=f'{mark:.0%}')

    def on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        kind = ev.get('type')
        if kind == 'finished':
            self.finished = True
            self.exploring = False
        self.last_event = time.monotonic()
        if kind == 'arrived':
            self.queue('look', event=ev, cams=('front', 'left', 'right', 'rear'))
        elif kind == 'blocked':
            self.bumped_at = time.monotonic()
            self.queue('look', event=ev, cams=('front', 'rear'), urgent=True)
        elif kind == 'failed':
            self.queue('look', event=ev, cams=('front',))
        elif kind == 'finished':
            self.queue('final', event=ev)

    def on_period(self):
        # ALL FOUR cameras while driving (operator, 2026-10-06), in one
        # request: a labelled 2x2 mosaic. Four separate looks would hold the
        # GPU ~10 s of every 15 s and starve the object detector.
        if self.exploring and not any(j['kind'] in ('look', 'mosaic') for j in list(self.jobs)):
            self.queue('mosaic')

    def on_clear(self, msg):
        what = msg.data.strip().lower()
        with self.lock:
            if what in ('glass', 'all'):
                self.closures = [c for c in self.closures if c['kind'] != 'glass']
            if what in ('doorway', 'all'):
                self.closures = [c for c in self.closures if c['kind'] != 'doorway']
            if what in ('skips', 'all'):
                self.skips = []
            self.candidates = []
        self.note(f'Operator cleared: {what}')
        self.publish_hints()

    def queue(self, kind, urgent=False, **kw):
        job = dict(kind=kind, **kw)
        with self.lock:
            if urgent:
                self.jobs.appendleft(job)
            else:
                self.jobs.append(job)
            while len(self.jobs) > 6:        # never fall far behind reality
                self.jobs.pop()
        self.wake.set()

    # ---------------------------------------------------------------- worker

    def worker(self):
        while rclpy.ok():
            self.wake.wait(1.0)
            self.wake.clear()
            if not self.llm_ok:
                self.llm_ok = self.server_ready()
                if not self.llm_ok:
                    self.flush_without_model()
                    time.sleep(5.0)
                    continue
                self.get_logger().info('Vision model ready for exploration.')
            while True:
                with self.lock:
                    job = self.jobs.popleft() if self.jobs else None
                if job is None:
                    break
                try:
                    getattr(self, 'do_' + job['kind'])(job)
                except Exception as exc:     # noqa: BLE001 - never kill the worker
                    self.get_logger().warn(f'{job["kind"]} failed: {exc}')
                    if job['kind'] == 'look':
                        self.look_done(job.get('event'))

    def flush_without_model(self):
        """No model: release any waiting explorer immediately, and still
        produce the final report from geometry."""
        with self.lock:
            jobs, self.jobs = list(self.jobs), deque()
        for job in jobs:
            if job['kind'] == 'look':
                self.look_done(job.get('event'))
            elif job['kind'] == 'final':
                self.do_final(job, use_model=False)

    def server_ready(self):
        try:
            with urllib.request.urlopen(self.llm_url + '/health', timeout=2.0) as r:
                return r.status == 200
        except Exception:                    # noqa: BLE001 - not up / still loading
            return False

    def ask(self, prompt, image_bgr=None, max_tokens=160):
        content = [{'type': 'text', 'text': prompt}]
        if image_bgr is not None:
            h, w = image_bgr.shape[:2]
            scale = 448.0 / max(h, w)
            if scale < 1.0:
                image_bgr = cv2.resize(image_bgr, (int(w * scale), int(h * scale)))
            ok, jpg = cv2.imencode('.jpg', image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                content.insert(0, {'type': 'image_url', 'image_url': {
                    'url': 'data:image/jpeg;base64,' + base64.b64encode(jpg.tobytes()).decode()}})
        body = json.dumps({'messages': [{'role': 'user', 'content': content}],
                           'max_tokens': max_tokens, 'temperature': 0.1}).encode()
        req = urllib.request.Request(self.llm_url + '/v1/chat/completions', data=body,
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                out = json.loads(resp.read().decode())
        except Exception as exc:             # noqa: BLE001 - degrade, never die
            self.get_logger().warn(f'Model request failed: {exc}')
            self.llm_ok = self.server_ready()
            return None
        return first_json(out['choices'][0]['message'].get('content') or '')

    # ----------------------------------------------------------------- looks

    def do_look(self, job):
        pose = self.car_pose()
        ev = job.get('event')
        for cam in job['cams']:
            stamp_msg = self.images.get(cam)
            if stamp_msg is None or time.monotonic() - stamp_msg[0] > 2.0 or pose is None:
                continue
            img = to_bgr(stamp_msg[1])
            if img is None:
                continue
            ans = self.ask(LOOK_PROMPT.format(cam=cam), img)
            if not ans:
                continue
            self.apply_look(cam, ans, pose, ev, strong=job['kind'] == 'look'
                            and time.monotonic() - self.bumped_at < 10.0)
        self.look_done(ev)

    def do_mosaic(self, job):
        pose = self.car_pose()
        if pose is None:
            return
        tiles, used = [], []
        for cam in ('front', 'left', 'rear', 'right'):
            got = self.images.get(cam)
            img = to_bgr(got[1]) if got and time.monotonic() - got[0] < 2.0 else None
            if img is None:
                img = np.zeros((240, 320, 3), np.uint8)
            else:
                used.append(cam)
            img = cv2.resize(img, (320, 240))
            cv2.rectangle(img, (0, 0), (92, 26), (0, 0, 0), -1)
            cv2.putText(img, cam.upper(), (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
            tiles.append(img)
        if not used:
            return
        mosaic = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
        ans = self.ask(MOSAIC_PROMPT, mosaic, max_tokens=220)
        if not ans:
            return
        rt = ans.get('room_type')
        for cam in used:
            sub = ans.get(cam)
            if isinstance(sub, dict):
                sub = dict(sub, room_type=rt or '')
                self.apply_look(cam, sub, pose, None, strong=False)

    def apply_look(self, cam, ans, pose, ev, strong):
        _topic, heading, fov = CAMERAS[cam]
        rt = str(ans.get('room_type', '')).strip().lower()[:24]
        if rt and rt not in ('none', 'unknown', '<one or two words>'):
            self.room_type = rt
        offset = {'left': fov / 3.0, 'center': 0.0, 'right': -fov / 3.0}
        for kind in ('glass', 'doorway'):
            where = str(ans.get(kind, 'none')).lower()
            if where not in offset:
                continue
            bearing = pose[2] + heading + offset[where]
            seg = self.opening_along(pose, bearing)
            if seg is None:
                self.note(f'{cam}: {kind} seen {where}, but no wall opening there in the map')
                continue
            self.propose(kind, seg, f'{cam} camera', strong)
        ahead = str(ans.get('ahead', '')).lower()
        if cam == 'front' and ev and ev.get('type') == 'arrived' and \
                ahead in ('under_furniture', 'wall', 'cluttered'):
            fx, fy = ev.get('fx'), ev.get('fy')
            if fx is not None:
                self.skips.append({'x': fx, 'y': fy, 'r': 0.8,
                                   'reason': f'vision: {ahead.replace("_", " ")}'})
                self.note(f'Gap at ({fx:.1f}, {fy:.1f}) skipped: {ahead.replace("_", " ")}')
                self.publish_hints()

    def look_done(self, ev):
        if ev and ev.get('id') is not None:
            msg = String()
            msg.data = str(ev['id'])
            self.look_done_pub.publish(msg)

    # -------------------------------------------------------------- geometry

    def car_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:                    # noqa: BLE001
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def opening_along(self, pose, bearing, half_cone=math.radians(35), max_range=6.0):
        """The gap in the mapped walls in direction `bearing`, as a segment
        between the wall ends either side of it -- where glass or a door would
        sit. None when that direction is plainly a wall (nothing to close)."""
        grid = self.map
        if grid is None:
            return None
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        occ = data >= 65
        steps = np.arange(0.2, max_range, res / 2)

        def cast(a):
            xs = pose[0] + steps * math.cos(a)
            ys = pose[1] + steps * math.sin(a)
            c = ((xs - ox) / res).astype(int)
            r = ((ys - oy) / res).astype(int)
            ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
            hit = np.zeros(len(steps), bool)
            hit[ok] = occ[r[ok], c[ok]]
            idx = np.argmax(hit) if hit.any() else -1
            return (steps[idx], xs[idx], ys[idx]) if idx >= 0 else (math.inf, None, None)

        angles = bearing + np.radians(np.arange(-80, 81, 1.0))
        hits = [cast(a) for a in angles]
        d = np.array([hh[0] for hh in hits])
        mid = len(angles) // 2
        center = d[mid]
        if center < 0.8:
            return None                      # a wall right there, no opening

        # Walk out from the centre to where the room's own wall resumes on
        # each side. Through an opening the rays hit things BEYOND it (furniture
        # in the next room, the far wall); the wall end beside the opening is
        # recognised as a sharp DROP -- a hit much nearer than everything seen
        # through the gap so far. (A fixed distance limit picked objects beyond
        # the glass instead, tested on my_room6.)
        def edge(direction):
            i, nearest = mid, center
            while 0 <= i + direction < len(d):
                i += direction
                if d[i] < 0.65 * nearest:
                    return hits[i]
                nearest = min(nearest, d[i])
            return None
        left, right = edge(1), edge(-1)
        if left is None or right is None:
            return None
        x1, y1, x2, y2 = left[1], left[2], right[1], right[2]
        width = math.hypot(x2 - x1, y2 - y1)
        if not 0.4 <= width <= 6.0:
            return None
        return (float(x1), float(y1), float(x2), float(y2))

    def propose(self, kind, seg, source, strong):
        mx, my = (seg[0] + seg[2]) / 2, (seg[1] + seg[3]) / 2
        with self.lock:
            for c in self.closures:
                if c['kind'] == kind and math.hypot(c['mx'] - mx, c['my'] - my) < 0.8:
                    return                   # already applied
            cand = next((c for c in self.candidates if c['kind'] == kind
                         and math.hypot(c['mx'] - mx, c['my'] - my) < 0.8), None)
            if cand is None:
                cand = {'kind': kind, 'mx': mx, 'my': my, 'seg': seg, 'count': 0}
                self.candidates.append(cand)
            cand['count'] += 2 if strong else 1
            # Doorways need one more sighting than glass: tested on the real
            # 8B model (2026-10-06), it called "doorway" on a glass wall and on
            # a window, while its glass answers were right. A doorway closure
            # only trims the room outline, so being slower to accept it costs
            # little.
            needed = self.confirmations + (1 if kind == 'doorway' else 0)
            if cand['count'] < needed:
                self.note(f'Possible {kind} near ({mx:.1f}, {my:.1f}) -- waiting for a '
                          'second sighting')
                return
            self.candidates.remove(cand)
            self.closure_id += 1
            x1, y1, x2, y2 = cand['seg']
            # SUSPECTED ONLY (2026-10-06, live run). The model reliably says
            # glass is in some DIRECTION, but where along that direction the
            # pane is cannot be read off the map: the ray found the first
            # opening -- the gap between two armchairs next to the car -- and
            # walled off open floor, boxing the car in (NAV FAILED), while the
            # real glass was the far wall. So a vision closure never blocks
            # driving by itself: glass becomes a real no-go wall only when the
            # car physically bumps within 0.5 m of it (see on_bumps), and
            # doorways stay suggestions.
            self.closures.append({'id': self.closure_id, 'kind': kind, 'x1': x1, 'y1': y1,
                                  'x2': x2, 'y2': y2, 'mx': mx, 'my': my,
                                  'apply': False, 'status': 'suspected',
                                  'source': source})
        width = math.hypot(x2 - x1, y2 - y1)
        self.note(f'Possible {kind} near ({mx:.1f}, {my:.1f}), {width:.1f} m wide -- '
                  'marked as suspected, not blocking')
        self.publish_hints()

    def on_bumps(self, msg):
        """A suspected glass line becomes a real no-go wall once the car has
        physically bumped within 0.5 m of it: the camera said glass, and the
        bump says exactly where."""
        pts = [(float(x), float(y)) for x, y in
               point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)]
        changed = False
        with self.lock:
            for c in self.closures:
                if c['kind'] != 'glass' or c['apply'] or not pts:
                    continue
                if min(seg_distance(px, py, c) for px, py in pts) < 0.5:
                    c['apply'], c['status'] = True, 'confirmed by bump'
                    changed = True
        if changed:
            self.note('Glass confirmed by a bump -- now a no-go wall')
            if self.announce:
                self.say('That was glass. I will keep away from it.')
            self.publish_hints()

    # --------------------------------------------------------------- reviews

    def summary_text(self, st, ids):
        visit = [f for f in st.get('frontiers', []) if f['status'] == 'visit']
        lines = [f'Room about {st.get("room_m2", 0):.0f} m2, {st.get("coverage", 0):.0%} mapped.']
        for i, f in zip(ids, visit):
            beyond = ('open unknown space' if f['area_m2'] >= 99
                      else '%.1f m2 not seen yet' % f['area_m2'])
            lines.append(f'Gap {i}: about {f["size"] * 0.05:.1f} m of edge, opens onto {beyond}.')
        return '\n'.join(lines)

    def render_map(self, st):
        grid, pose = self.map, self.car_pose()
        if grid is None:
            return None, []
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w, 3), 160, np.uint8)
        img[(data >= 0) & (data <= 49)] = 255
        img[data >= 65] = 0

        def px(x, y):
            return int((x - ox) / res), int((y - oy) / res)
        outline = np.array([px(x, y) for x, y in st.get('outline', [])], np.int32)
        if len(outline):
            cv2.polylines(img, [outline], True, (0, 170, 0), 1)
        visit = [f for f in st.get('frontiers', []) if f['status'] == 'visit'][:9]
        img = img[::-1].copy()               # grid row 0 is the bottom
        flip = lambda p: (p[0], h - 1 - p[1])   # noqa: E731
        # At least ~600 px on the long side: a 348-cell map at 1:1 left the
        # walls and numbers too small for the model to read.
        scale = max(1, math.ceil(600 / max(w, h)))
        img = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
        ids = []
        for i, f in enumerate(visit, start=1):
            c = flip(px(f['x'], f['y']))
            c = (c[0] * scale, c[1] * scale)
            cv2.circle(img, c, 7, (0, 140, 255), -1)
            cv2.putText(img, str(i), (c[0] + 8, c[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 90, 200), 2)
            ids.append(i)
        if pose:
            c = flip(px(pose[0], pose[1]))
            cv2.circle(img, (c[0] * scale, c[1] * scale), 6, (255, 80, 0), -1)
        return img, ids

    def do_review(self, job):
        st = self.status
        if not st:
            return
        img, ids = self.render_map(st)
        if img is None:
            return
        ans = self.ask(REVIEW_PROMPT.format(summary=self.summary_text(st, ids)), img,
                       max_tokens=200)
        if not ans:
            return
        rt = str(ans.get('room_type', '')).strip().lower()[:24]
        if rt and rt != '<one or two words>':
            self.room_type = rt
        visit = [f for f in st.get('frontiers', []) if f['status'] == 'visit']
        skipped = []
        for n in ans.get('skip', [])[:3] if isinstance(ans.get('skip'), list) else []:
            if isinstance(n, int) and 1 <= n <= len(ids) and len(visit) > 1:
                f = visit[n - 1]
                self.skips.append({'x': f['x'], 'y': f['y'], 'r': 0.6,
                                   'reason': 'vision review: not worth visiting'})
                skipped.append(n)
        comment = str(ans.get('comment', ''))[:160]
        self.note(f'Review at {job.get("tag", "")}: {self.room_type or "room"}'
                  + (f', skipping gaps {skipped}' if skipped else '')
                  + (f' -- {comment}' if comment else ''))
        self.publish_hints()

    def do_final(self, job, use_model=True):
        st = self.status or {}
        comment = ''
        if use_model and self.llm_ok:
            img, ids = self.render_map(st)
            if img is not None:
                ans = self.ask(REVIEW_PROMPT.format(summary=self.summary_text(st, ids)), img,
                               max_tokens=200)
                if ans:
                    rt = str(ans.get('room_type', '')).strip().lower()[:24]
                    if rt and rt != '<one or two words>':
                        self.room_type = rt
                    comment = str(ans.get('comment', ''))[:160]
        ignored = [f for f in st.get('frontiers', []) if f['status'] == 'ignore'
                   and f['reason'] not in ('too small to matter', 'along a wall')]
        glass = sum(1 for c in self.closures if c['kind'] == 'glass')
        doors = sum(1 for c in self.closures if c['kind'] == 'doorway')
        parts = [f'{(self.room_type or "room").capitalize()}: {st.get("coverage", 0):.0%} mapped, '
                 f'{st.get("mapped_m2", 0):.0f} square metres.']
        skipped = []
        if ignored:
            skipped.append(f'{len(ignored)} spots under or behind furniture')
        if self.skips:
            skipped.append(f'{len(self.skips)} gaps the camera judged not worth it')
        if glass:
            skipped.append(f'{glass} glass wall{"s" if glass > 1 else ""}')
        if doors:
            skipped.append(f'{doors} doorway{"s" if doors > 1 else ""} to other rooms')
        if skipped:
            parts.append('Skipped ' + ', '.join(skipped) + '.')
        q = st.get('quality')
        if q is not None:
            parts.append('Map looks consistent.' if q >= 0.55 else
                         'Some walls may be doubled; check the map.')
        if comment:
            parts.append(comment)
        reason = (job.get('event') or {}).get('reason', '')
        self.report = {'text': ' '.join(parts), 'reason': reason,
                       'coverage': st.get('coverage'), 'room_type': self.room_type}
        self.note('Final report: ' + self.report['text'])
        if self.announce:
            self.say(' '.join(parts[:2]))
        self.publish_hints()

    # --------------------------------------------------------------- outputs

    def note(self, text):
        self.get_logger().info(text)
        self.notes.appendleft({'t': time.strftime('%H:%M:%S'), 'text': text})
        self.publish_hints()

    def say(self, text):
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)

    def publish_hints(self):
        with self.lock:
            data = {
                'closures': [c for c in self.closures if c['apply']],
                'all_closures': list(self.closures),
                'skip_regions': list(self.skips),
                'room_type': self.room_type,
                'notes': list(self.notes),
                'report': self.report,
                'model': self.llm_ok,
            }
        msg = String()
        msg.data = json.dumps(data)
        self.hints_pub.publish(msg)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ExploreVlm()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
