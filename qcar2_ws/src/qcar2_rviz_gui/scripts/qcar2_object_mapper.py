#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Semantic object mapping: label the SLAM map with what the cameras see.

Runs during mapping mode alongside Cartographer. Watches the 4 CSI cameras,
detects objects, works out where each one physically is, and maintains a
database of map-frame landmarks that is saved next to the .pgm/.yaml so
navigation mode can later be told "go to the air cooler".

HOW A PIXEL BECOMES A MAP COORDINATE
------------------------------------
1. YOLO-World gives a bounding box in one camera image. The detector is
   open-vocabulary (CLIP-embedded free-text labels from
   config/object_vocabulary.yaml), which is the whole reason "air cooler"
   can be a class at all -- it is not one of the 80 COCO classes.
2. The box's left/right edges become two rays in that camera's optical
   frame. The QCar2 URDF already defines csi_front/back/left/right with
   their exact mount pose, and robot_state_publisher publishes them, so TF
   does this geometry for us -- there are no hand-tuned per-camera yaw
   offsets anywhere in this file, and re-measuring a camera mount means
   editing the URDF, not this node.
3. Those two rays define an angular SECTOR as seen from the camera. Every
   LiDAR return whose bearing (measured from the camera's optical centre,
   not the LiDAR's) falls inside that sector is a candidate.
4. Candidates are sorted by distance and split into clusters. The NEAREST
   cluster is the object: whatever the camera can see must be in front of
   whatever it cannot, so the closest surface inside the box is the thing
   that was detected, and the wall behind it is correctly ignored.
5. That cluster's centroid is transformed into the map frame. Now it is a
   coordinate, not a pixel.

WHY THE SAME TV IS NOT MAPPED TWICE
-----------------------------------
This is the crux of multi-camera semantic mapping, and it is handled by
*never comparing detections in image space*. Every detection is converted to
a map-frame (x, y) at step 5 BEFORE anything is compared. So when the car
turns and the TV that was in the front camera appears in the left camera, it
resolves to the same map coordinate and associates to the existing landmark
instead of creating a new one -- the second camera raises that landmark's
confidence rather than duplicating it. The camera a sighting came from is
recorded as evidence but is deliberately NOT part of the matching key.

Three further guards sit on top of that:
  * association requires the same label AND a distance gate (per-class, since
    a sofa's LiDAR hit moves much further between viewpoints than a laptop's);
  * a periodic merge pass collapses same-label landmarks that have drifted
    together, which is what heals a SLAM loop-closure nudging old landmarks;
  * `min_hits` sightings are required before a landmark is published or saved,
    so a single-frame false positive never reaches the map.

KNOWN LIMITATION
----------------
The LiDAR is a 2D scanner in a horizontal plane ~0.19 m off the floor. An
object that never intersects that plane -- a wall-mounted TV, a rug, a
picture frame -- yields a bearing but no range, so it is counted as
unresolved and skipped rather than guessed at. Furniture that stands on the
floor (sofa, cooler, fridge, chair, table legs) intersects the plane and maps
fine. `unresolved` in the stats topic tells you how often this is happening.

Published:
  /qcar2/object_markers   visualization_msgs/MarkerArray  (RViz)
  /qcar2/objects          std_msgs/String  JSON, latched  (web console, voice)
  /qcar2/detection_image  sensor_msgs/Image  annotated view (optional)
Subscribed:
  /qcar2/save_objects     std_msgs/String  map name or path -> writes JSON
"""

import json
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point, Vector3
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Camera topic namespace -> the URDF link that camera is mounted on. The URDF
# calls the rear one "csi_back" while the ROS topic namespace is "rear"; this
# table is the only place that mismatch is resolved.
CAMERA_FRAMES = {
    'front': 'csi_front',
    'rear': 'csi_back',
    'left': 'csi_left',
    'right': 'csi_right',
}


def ang_norm(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def number_names(objects):
    """Give same-label objects distinct spoken names: sofa, sofa 2, sofa 3.

    Ordered by landmark id (creation order), so the numbers stay the same
    every time a saved map is loaded. Adds a 'name' key; 'label' stays the
    plain class, which is what counting and matching use. Same rule as
    qcar2_object_nav.py's copy -- keep them identical.
    """
    seen = {}
    for o in sorted(objects, key=lambda o: (o['label'], o.get('id', 0))):
        n = seen[o['label']] = seen.get(o['label'], 0) + 1
        o['name'] = o['label'] if n == 1 else f"{o['label']} {n}"
    return objects


def quat_to_matrix(x, y, z, w):
    """Rotation matrix from a quaternion (no tf_transformations dependency)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def image_to_numpy(msg):
    """sensor_msgs/Image -> HxWx3 BGR uint8, without cv_bridge.

    cv_bridge is avoided on purpose: it is an extra binary dependency that
    has to match both the ROS distro and the numpy ABI, and all we need is a
    reshape plus a channel swap.
    """
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        channels = 3
    elif enc in ('bgra8', 'rgba8'):
        channels = 4
    elif enc in ('mono8', '8uc1'):
        channels = 1
    else:
        return None
    expected = msg.height * msg.width * channels
    # `step` can include row padding, so honour it rather than assuming the
    # buffer is tightly packed.
    if msg.step and msg.step != msg.width * channels:
        if buf.size < msg.height * msg.step:
            return None
        img = buf[:msg.height * msg.step].reshape(msg.height, msg.step)
        img = img[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    else:
        if buf.size < expected:
            return None
        img = buf[:expected].reshape(msg.height, msg.width, channels)
    if channels == 1:
        return np.repeat(img, 3, axis=2)
    if channels == 4:
        img = img[:, :, :3]
    if enc.startswith('rgb'):
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


class Landmark:
    """One physical object, accumulated over many sightings from any camera.

    SELF-CORRECTING. A landmark is a running belief, not a fixed record:

    * WHAT it is -- `votes` accumulates evidence per label instead of freezing
      the first guess. A chair first seen from across the room as "bed" is
      relabelled once closer, better-lit sightings outvote that. `label` is
      always the current winner, so a mistake is outgrown rather than stuck.
    * WHERE it is -- the position is an exponentially-weighted mean whose
      accumulated weight is CAPPED. Without the cap, weight grows without
      bound and after a hundred distant sightings a close-up correction moves
      the estimate by ~1%; with it, a good close pass can still pull the
      landmark to where the object actually is.
    * WHETHER it exists -- `misses` counts the times the car looked straight
      at where this landmark claims to be, unoccluded and in range, and saw
      nothing. Enough of those and it is deleted as a false positive or an
      object that has since been moved.
    """

    __slots__ = ('id', 'votes', 'x', 'y', 'weight', 'hits', 'misses',
                 'conf_best', 'cameras', 'first_seen', 'last_seen',
                 'transient', 'best_range', 'max_weight')

    def __init__(self, lid, label, x, y, weight, conf, camera, stamp,
                 transient, rng, max_weight):
        self.id = lid
        self.votes = {label: weight}
        self.x = x
        self.y = y
        self.weight = weight
        self.hits = 1
        self.misses = 0.0
        self.conf_best = conf
        self.cameras = {camera}
        self.first_seen = stamp
        self.last_seen = stamp
        self.transient = transient
        self.best_range = rng
        self.max_weight = max_weight

    @property
    def label(self):
        """Whichever label currently has the most evidence behind it."""
        return max(self.votes.items(), key=lambda kv: kv[1])[0]

    @property
    def label_confidence(self):
        total = sum(self.votes.values())
        return (max(self.votes.values()) / total) if total > 0 else 0.0

    def update(self, label, x, y, weight, conf, camera, stamp, rng):
        self.votes[label] = self.votes.get(label, 0.0) + weight
        # Cap the weight the history carries so the estimate never becomes too
        # rigid to correct. This is what turns the running mean into an EMA:
        # once saturated, each new sighting keeps a real say in the position.
        held = min(self.weight, self.max_weight)
        total = held + weight
        self.x = (self.x * held + x * weight) / total
        self.y = (self.y * held + y * weight) / total
        self.weight = min(self.max_weight, self.weight + weight)
        self.hits += 1
        # Seeing it again is evidence it is real; forgive an earlier miss.
        self.misses = max(0.0, self.misses - 1.0)
        self.conf_best = max(self.conf_best, conf)
        self.cameras.add(camera)
        self.last_seen = stamp
        self.best_range = min(self.best_range, rng)

    def miss(self):
        self.misses += 1.0

    def absorb(self, other):
        held, oth = min(self.weight, self.max_weight), min(other.weight, other.max_weight)
        total = held + oth
        self.x = (self.x * held + other.x * oth) / total
        self.y = (self.y * held + other.y * oth) / total
        self.weight = min(self.max_weight, self.weight + other.weight)
        for label, w in other.votes.items():
            self.votes[label] = self.votes.get(label, 0.0) + w
        self.hits += other.hits
        self.misses += other.misses
        self.conf_best = max(self.conf_best, other.conf_best)
        self.cameras |= other.cameras
        self.first_seen = min(self.first_seen, other.first_seen)
        self.last_seen = max(self.last_seen, other.last_seen)
        self.best_range = min(self.best_range, other.best_range)

    def as_dict(self):
        ranked = sorted(self.votes.items(), key=lambda kv: -kv[1])
        return {
            'id': self.id,
            'label': self.label,
            'x': round(float(self.x), 3),
            'y': round(float(self.y), 3),
            'hits': int(self.hits),
            'misses': round(float(self.misses), 1),
            'confidence': round(float(self.conf_best), 3),
            # How sure we are of the NAME (not the detection): 1.0 means every
            # sighting agreed, 0.55 means it was nearly a coin flip with the
            # runner-up. Lets the assistant say "probably a sofa" honestly.
            'label_confidence': round(float(self.label_confidence), 3),
            'alternatives': [{'label': l, 'score': round(float(w), 2)}
                             for l, w in ranked[1:4]],
            'closest_seen_m': round(float(self.best_range), 2),
            'cameras': sorted(self.cameras),
        }


class ClipVerifier:
    """A second, independent opinion on what is inside a detection box.

    YOLO-World proposes boxes and a name; this crops each box and asks CLIP
    (a vision-language model: it scores how well an image matches each text
    prompt) which of the vocabulary labels -- or a bare wall/floor -- it
    shows. The two models fail differently: the detector is judging a region
    in the context of a whole wide-angle frame, CLIP only the object itself,
    so a crop that is really a dark window or a cabinet front stops being
    mapped as a "television" just because it is rectangular and dark.

    Why CLIP and not a chat-style VLM (LLaVA, moondream...): those take
    seconds per image on this GPU while it is also running the detector and
    the car; CLIP scores a whole batch of crops in a few milliseconds, so it
    can check EVERY sighting, from every angle, continuously.
    """

    def __init__(self, labels, background, device='cuda'):
        import clip
        import torch
        self.torch = torch
        self.device = device
        self.model, self.preprocess = clip.load('ViT-B/32', device=device)
        self.model.eval()
        self.labels = list(labels) + list(background)
        self.background = set(background)
        prompts = [f'a photo of a {name}' for name in self.labels]
        with torch.no_grad():
            text = self.model.encode_text(clip.tokenize(prompts).to(device))
            self.text = text / text.norm(dim=-1, keepdim=True)

    def classify(self, crops):
        """BGR uint8 crops -> [(label, probability), ...], one per crop."""
        if not crops:
            return []
        from PIL import Image as PILImage
        torch = self.torch
        batch = torch.stack([
            self.preprocess(PILImage.fromarray(np.ascontiguousarray(c[:, :, ::-1])))
            for c in crops]).to(self.device)
        with torch.no_grad():
            feats = self.model.encode_image(batch)
            feats = feats / feats.norm(dim=-1, keepdim=True)
            probs = (100.0 * feats @ self.text.T).softmax(dim=-1).float().cpu().numpy()
        best = probs.argmax(axis=1)
        return [(self.labels[i], float(probs[k, i])) for k, i in enumerate(best)]


def crop_box(img, x1, y1, x2, y2, pad=0.08):
    """The detection box plus a little context, clipped to the image."""
    h, w = img.shape[:2]
    px, py = (x2 - x1) * pad, (y2 - y1) * pad
    a, b = max(0, int(x1 - px)), max(0, int(y1 - py))
    c, d = min(w, int(x2 + px)), min(h, int(y2 + py))
    return img[b:d, a:c] if c - a >= 8 and d - b >= 8 else None


class ObjectMapper(Node):

    def __init__(self):
        super().__init__('qcar2_object_mapper')

        share = get_package_share_directory('qcar2_rviz_gui')
        default_vocab = os.path.join(share, 'config', 'object_vocabulary.yaml')
        project_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Qcar-rviz')

        self.declare_parameter('vocabulary_file', default_vocab)
        # YOLO-World v2 LARGE (was the small model). Measured 2026-09-25 on 150
        # COCO indoor photos, prompted with this car's real vocabulary:
        #   yolov8s-worldv2  precision 0.69  recall 0.55  28 ms/frame on the Orin
        #   yolov8l-worldv2  precision 0.76  recall 0.64  50 ms/frame
        #   yoloe-26s/m      precision 0.66  recall 0.64  (newer family: far
        #                    better at people, but worse at TVs and sofas --
        #                    the furniture this map is for -- and it needs an
        #                    ultralytics upgrade; not worth it)
        # 50 ms fits easily in the 167 ms per-frame budget of detect_rate_hz 6.
        self.declare_parameter('model_path', os.path.join(project_dir, 'models', 'yolov8l-worldv2.pt'))
        self.declare_parameter('cameras', ['front', 'rear', 'left', 'right'])
        self.declare_parameter('detect_rate_hz', 6.0)
        # 0.25 -> 0.35 with the move to the Large model (far fewer confident
        # false positives to lose): weak guesses were what filled the map
        # with a row of "televisions" along one wall.
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('half', False)
        # Horizontal field of view of the CSI lens, degrees. Quanser do not
        # publish an intrinsics file and the driver emits no CameraInfo, so
        # this is a tunable rather than a calibration. 120 deg matches the
        # QCar2's wide CSI lens closely enough that a bbox sector lands on the
        # right LiDAR returns; see README for how to check it on your car.
        self.declare_parameter('hfov_deg', 120.0)
        # Shrink each bbox's angular sector toward its centre before matching
        # LiDAR returns. Boxes tend to be slightly generous, and a sector that
        # overhangs the object picks up the wall beside it.
        self.declare_parameter('sector_shrink', 0.15)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('min_range', 0.25)
        self.declare_parameter('cluster_gap', 0.35)
        self.declare_parameter('min_cluster_points', 2)
        # 0.60 -> 0.85 m. The LiDAR hits a DIFFERENT part of a big object
        # (a TV, a sofa) from each viewpoint, so one object's sightings spread
        # well over half a metre and split into several landmarks -- the
        # "many TVs" on the map. Per-class overrides widen it further.
        self.declare_parameter('assoc_radius', 0.85)
        # 3 -> 5: three glimpses while creeping past something was enough to
        # make a false detection permanent.
        self.declare_parameter('min_hits', 5)
        # --- self-correction -------------------------------------------------
        # How close a detection of a DIFFERENT class must be to an existing
        # landmark before it is treated as "the same object, named better"
        # rather than a new object. Tighter than assoc_radius on purpose: a
        # desk and the chair under it must stay two landmarks.
        # 0.35 -> 0.45 m, in step with the wider assoc_radius: also used by
        # merge_pass() to fuse one object that got two names.
        self.declare_parameter('revision_radius', 0.45)
        # Ceiling on accumulated position weight. This is what keeps a
        # landmark correctable: without it, old distant sightings outvote a
        # close-up correction forever.
        # 6.0 gives a correction time-constant of roughly 7 close sightings --
        # a second or two of driving past an object. Higher makes landmarks
        # rigid and slow to fix; much lower makes them chase single frames.
        self.declare_parameter('max_weight', 6.0)
        # Evidence of absence: penalise landmarks the car looked at and did
        # not see, and eventually delete them.
        self.declare_parameter('verify_absent', True)
        self.declare_parameter('max_misses', 6.0)
        self.declare_parameter('verify_max_range', 3.5)
        # Only trust the middle of the frame for absence: detections get
        # unreliable at the very edge of a wide lens, and a false miss is
        # worse than a slow one.
        self.declare_parameter('verify_fov_fraction', 0.7)
        # --- auto-calibration ------------------------------------------------
        # Refine hfov_deg from agreement between each bbox's width and the
        # angular width of the LiDAR cluster it resolved to. See calibrate().
        self.declare_parameter('auto_calibrate_hfov', True)
        self.declare_parameter('calibration_min_samples', 60)
        # --- detection sanity -------------------------------------------------
        # Largest box, as a fraction of the frame, accepted as an object. The
        # camera sits 11 cm off the ground, so the floor fills the bottom half
        # of every frame; a box that big is the floor or a wall, not furniture.
        self.declare_parameter('max_box_area', 0.40)
        # --- display hysteresis (anti-flicker) ------------------------------
        # A box appears at `confidence` but, once shown, stays while the
        # detector still sees it at >= display_confidence, and survives brief
        # misses for box_hold_sec. Mapping still uses `confidence` only.
        # 0.12 -> 0.25: a shown box may coast on weaker frames, but not on
        # near-noise ones.
        self.declare_parameter('display_confidence', 0.25)
        # A box only APPEARS on screen at this confidence -- or when it lands
        # on a CONFIRMED landmark. Was: any detection that touched any
        # landmark, even one seen once, which is how so many wrong boxes
        # showed up.
        self.declare_parameter('box_confidence', 0.45)
        self.declare_parameter('box_hold_sec', 2.0)
        # --- second opinion ---------------------------------------------------
        # Every box about to be MAPPED is cropped and classified independently
        # by CLIP (a vision-language model; see ClipVerifier). It overrules
        # the detector's name when clearly more confident, and vetoes crops
        # it sees as bare wall/floor. Continuous, from every viewpoint, so a
        # landmark's name converges on what the object really is.
        self.declare_parameter('clip_verify', True)
        self.declare_parameter('clip_min_prob', 0.60)
        # How long a transient sighting (a person) still counts as "now".
        self.declare_parameter('transient_ttl_sec', 20.0)
        self.declare_parameter('merge_period_sec', 5.0)
        # Server-side annotated image. Off by default: the console now draws
        # boxes client-side from /qcar2/detections, which is cheaper and works
        # on all four cameras at once. Turn on for RViz debugging.
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('objects_dir', os.path.join(project_dir, 'maps'))

        g = lambda n: self.get_parameter(n).value
        self.cameras = [c for c in g('cameras') if c in CAMERA_FRAMES]
        self.detect_period = 1.0 / max(0.5, float(g('detect_rate_hz')))
        self.conf_thresh = float(g('confidence'))
        self.imgsz = int(g('imgsz'))
        self.half = bool(g('half'))
        self.hfov = math.radians(float(g('hfov_deg')))
        self.sector_shrink = min(0.45, max(0.0, float(g('sector_shrink'))))
        self.max_range = float(g('max_range'))
        self.min_range = float(g('min_range'))
        self.cluster_gap = float(g('cluster_gap'))
        self.min_cluster_points = int(g('min_cluster_points'))
        self.assoc_radius = float(g('assoc_radius'))
        self.min_hits = int(g('min_hits'))
        self.merge_period = float(g('merge_period_sec'))
        self.revision_radius = float(g('revision_radius'))
        self.max_weight = float(g('max_weight'))
        self.verify_absent = bool(g('verify_absent'))
        self.max_misses = float(g('max_misses'))
        self.verify_max_range = float(g('verify_max_range'))
        self.verify_fov_fraction = float(g('verify_fov_fraction'))
        self.auto_calibrate = bool(g('auto_calibrate_hfov'))
        self.calib_min_samples = int(g('calibration_min_samples'))
        self.transient_ttl = float(g('transient_ttl_sec'))
        self.max_box_area = float(g('max_box_area'))
        self.display_conf = min(self.conf_thresh, float(g('display_confidence')))
        self.box_hold = float(g('box_hold_sec'))
        self.box_conf = float(g('box_confidence'))
        self.clip_enabled = bool(g('clip_verify'))
        self.clip_min_prob = float(g('clip_min_prob'))
        self.clip = None                # ClipVerifier, loaded with the detector
        self.publish_debug = bool(g('publish_debug_image'))
        self.map_frame = g('map_frame')
        self.objects_dir = g('objects_dir')
        self._last_saved_count = None

        self.vocab, self.synonyms, self.overrides = self.load_vocabulary(g('vocabulary_file'))
        if not self.vocab:
            self.get_logger().error('Vocabulary is empty; nothing can be detected. Check the YAML.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.frames = {}                     # camera -> (numpy image, stamp)
        self.scans = deque(maxlen=40)        # (stamp_sec, LaserScan)
        self.landmarks = {}
        self.next_id = 1
        self.stats = {'detections': 0, 'unresolved': 0, 'frames': 0, 'infer_ms': 0.0,
                      'relabels': 0, 'pruned': 0, 'clip_renamed': 0, 'clip_vetoed': 0}
        # Focal-length samples for auto-calibration (see calibrate()).
        self.fx_samples = deque(maxlen=400)
        self.hfov_source = 'default'
        self.width = 0          # image width, learned from the first frame
        self.model = None
        self.model_error = None

        for cam in self.cameras:
            self.create_subscription(
                Image, f'/{cam}/camera/csi_image',
                lambda msg, c=cam: self.on_image(c, msg), qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/qcar2/save_objects', self.on_save_request, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/qcar2/object_markers', LATCHED)
        self.objects_pub = self.create_publisher(String, '/qcar2/objects', LATCHED)
        # Live boxes for the browser to draw over each camera feed. Data, not
        # pixels: the console overlays them itself, so all four 360 views get
        # boxes at once and the car never re-encodes an annotated image.
        self.boxes_pub = self.create_publisher(String, '/qcar2/detections', 10)
        self.latest_boxes = {}         # camera -> {'t', 'boxes'}
        self.box_tracks = {}           # camera -> [on-screen box tracks]
        self.box_seq = 0
        self.person_numbers = {}       # transient landmark id -> display number
        self.focus = 'all'             # 'front' when the console shows one camera
        self.create_subscription(String, '/qcar2/camera_focus', self.on_focus, 10)
        self.debug_pub = (self.create_publisher(Image, '/qcar2/detection_image', 1)
                          if self.publish_debug else None)

        self.create_timer(1.0, self.publish_objects)
        self.create_timer(self.merge_period, self.merge_pass)
        self.create_timer(self.merge_period, self.prune_pass)
        self.create_timer(10.0, self.calibrate)

        # Inference runs on its own thread so a 30 ms GPU call can never stall
        # a ROS callback and make us drop scans.
        self.running = True
        self.worker = threading.Thread(target=self.detect_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'Object mapper up: {len(self.vocab)} labels, cameras={self.cameras}, '
            f'{1.0 / self.detect_period:.1f} Hz total')

    # ------------------------------------------------------------ vocabulary

    def load_vocabulary(self, path):
        self.ignore, self.ignore_set = [], set()
        try:
            with open(path) as fh:
                data = yaml.safe_load(fh) or {}
        except OSError as exc:
            self.get_logger().error(f'Cannot read vocabulary {path}: {exc}')
            return [], {}, {}
        classes = [str(c).strip() for c in (data.get('classes') or []) if str(c).strip()]
        overrides = data.get('overrides') or {}
        # Background surfaces the detector may recognise but we discard. See
        # the `ignore:` block in object_vocabulary.yaml for why naming the
        # floor is what stops it being boxed as a desk.
        self.ignore = [str(c).strip() for c in (data.get('ignore') or [])
                       if str(c).strip() and str(c).strip() not in classes]
        self.ignore_set = {c.lower() for c in self.ignore}
        # Flatten synonyms into alias -> canonical label for command matching.
        synonyms = {}
        for canonical, aliases in (data.get('synonyms') or {}).items():
            synonyms[canonical.lower()] = canonical
            for alias in aliases or []:
                synonyms[str(alias).lower()] = canonical
        for c in classes:
            synonyms.setdefault(c.lower(), c)
        return classes, synonyms, overrides

    def override(self, label, key, default):
        entry = self.overrides.get(label) or {}
        return entry.get(key, default)

    # --------------------------------------------------------------- inputs

    def on_image(self, camera, msg):
        img = image_to_numpy(msg)
        if img is None:
            self.get_logger().warn(
                f'Unsupported image encoding "{msg.encoding}" on {camera}', once=True)
            return
        with self.lock:
            self.frames[camera] = (img, msg.header.stamp)

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.scans.append((stamp, msg))

    def nearest_scan(self, stamp_msg):
        """The scan closest in time to this image, so a turning car does not
        match a bbox against LiDAR returns from a different heading."""
        target = stamp_msg.sec + stamp_msg.nanosec * 1e-9
        with self.lock:
            if not self.scans:
                return None
            return min(self.scans, key=lambda s: abs(s[0] - target))[1]

    # ------------------------------------------------------------ detection

    def ensure_model(self):
        if self.model is not None or self.model_error is not None:
            return self.model
        path = self.get_parameter('model_path').value
        try:
            import torch
            from ultralytics import YOLOWorld
            if not torch.cuda.is_available():
                self.get_logger().warn(
                    'CUDA is not available -- detection will run on CPU and will be '
                    'far too slow to keep up with driving.')
            self.get_logger().info(f'Loading detector {path} ...')
            model = YOLOWorld(path)
            # This is what makes free-text labels work: CLIP embeds each phrase
            # and the detector matches regions against those embeddings.
            model.set_classes(self.vocab + self.ignore)
            self.model = model
            self.get_logger().info('Detector ready.')
            if self.clip_enabled:
                try:
                    self.clip = ClipVerifier(self.vocab, self.ignore)
                    self.get_logger().info('CLIP second-opinion check ready.')
                except Exception as exc:              # noqa: BLE001 - optional
                    self.get_logger().warn(f'CLIP check unavailable ({exc}); '
                                           'mapping on the detector alone.')
        except Exception as exc:                      # noqa: BLE001 - report and disable
            self.model_error = str(exc)
            self.get_logger().error(
                f'Could not start the detector ({exc}). '
                f'See "Setup" in README.md. Mapping continues without object labels.')
        return self.model

    def detect_loop(self):
        model = None
        idx = 0
        while self.running and rclpy.ok():
            if model is None:
                model = self.ensure_model()
                if model is None:
                    time.sleep(5.0)
                    continue
            start = time.monotonic()
            if not self.cameras:
                time.sleep(1.0)
                continue
            # Round-robin: each pass handles one camera, so the configured rate
            # is the TOTAL GPU load regardless of how many cameras are enabled.
            schedule = self.schedule()
            camera = schedule[idx % len(schedule)]
            idx += 1
            with self.lock:
                entry = self.frames.get(camera)
            if entry is not None:
                try:
                    self.process(model, camera, entry[0], entry[1])
                except Exception as exc:              # noqa: BLE001 - never kill the thread
                    self.get_logger().warn(f'Detection failed on {camera}: {exc}')
            elapsed = time.monotonic() - start
            time.sleep(max(0.0, self.detect_period - elapsed))

    def process(self, model, camera, img, stamp):
        t0 = time.monotonic()
        results = model.predict(img, device=0, verbose=False, imgsz=self.imgsz,
                                conf=self.display_conf, half=self.half)
        infer_ms = (time.monotonic() - t0) * 1000.0
        boxes = results[0].boxes
        names = results[0].names
        self.stats['frames'] += 1
        self.stats['infer_ms'] = infer_ms
        if boxes is None or len(boxes) == 0:
            self.publish_boxes(camera, img.shape[1], img.shape[0], [])
            if self.debug_pub is not None:
                self.publish_debug_image(img, [], camera)
            return

        scan = self.nearest_scan(stamp)
        if scan is None:
            self.publish_raw_boxes(camera, img, boxes, names)
            return
        pose = self.camera_pose(camera, stamp, scan.header.frame_id)
        if pose is None:
            self.publish_raw_boxes(camera, img, boxes, names)
            return
        origin_2d, rot = pose

        h, w = img.shape[:2]
        self.width = w
        fx = (w / 2.0) / math.tan(self.hfov / 2.0)
        cx, cy = w / 2.0, h / 2.0
        points = self.scan_points(scan)

        verdicts = self.second_opinion(img, boxes, names, w, h)

        drawn = []
        seen_ids = set()
        weak_labels = set()     # seen, but below the mapping threshold
        for i in range(len(boxes)):
            conf = float(boxes.conf[i])
            label = names[int(boxes.cls[i])] if names else None
            if label is None:
                continue
            x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i])
            if not self.plausible(label, x1, y1, x2, y2, w, h):
                continue
            if conf < float(self.override(label, 'min_confidence', self.conf_thresh)):
                # Too weak to MAP, but kept for display so a box already on
                # screen does not blink out on a slightly-worse frame. The
                # display tracker decides whether it may appear on its own.
                drawn.append((x1, y1, x2, y2, label, conf, None, None))
                weak_labels.add(label)
                continue
            verdict = verdicts.get(i)
            if verdict is not None:
                clip_label, prob = verdict
                if clip_label in self.ignore_set:
                    # CLIP sees bare wall/floor in this crop: not an object.
                    self.stats['clip_vetoed'] += 1
                    continue
                if clip_label != label:
                    self.stats['clip_renamed'] += 1
                    label = clip_label
            self.stats['detections'] += 1

            # Bbox edges -> two rays -> an angular sector in the scan frame.
            v_mid = (y1 + y2) / 2.0
            shrink = (x2 - x1) * self.sector_shrink / 2.0
            bearings = []
            for u in (x1 + shrink, x2 - shrink):
                d_opt = np.array([(u - cx) / fx, (v_mid - cy) / fx, 1.0])
                d = rot @ d_opt
                if abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9:
                    bearings = []
                    break
                bearings.append(math.atan2(d[1], d[0]))
            if len(bearings) != 2:
                continue

            hit = self.range_in_sector(points, origin_2d, bearings[0], bearings[1])
            if hit is None:
                self.stats['unresolved'] += 1
                drawn.append((x1, y1, x2, y2, label, conf, None, None))
                continue
            px, py, rng, cluster = hit
            world = self.to_map(px, py, scan.header.frame_id, stamp)
            if world is None:
                continue
            lid = self.add_observation(label, world[0], world[1], rng, conf, camera, stamp)
            if lid is not None:
                seen_ids.add(lid)
            self.collect_calibration(cluster, origin_2d, rot, x1, x2, cx, fx)
            drawn.append((x1, y1, x2, y2, label, conf, rng, lid))

        # Everything the car should have seen here but did not is evidence
        # against those landmarks -- see verify_unseen().
        self.verify_unseen(origin_2d, rot, points, seen_ids,
                           scan.header.frame_id, stamp, weak_labels)

        self.publish_boxes(camera, w, h, drawn)
        if self.debug_pub is not None:
            self.publish_debug_image(img, drawn, camera)

    def second_opinion(self, img, boxes, names, w, h):
        """CLIP's verdict for every box that is about to be MAPPED.

        {box index: (label, probability)}, only where CLIP is confident
        (>= clip_min_prob) -- an unsure second opinion is no opinion, and the
        detector's own label stands. People are left alone: they move, are
        never saved, and CLIP on a partly-visible person is unreliable.
        One batched GPU call per frame.
        """
        if self.clip is None:
            return {}
        idxs, crops = [], []
        for i in range(len(boxes)):
            label = names[int(boxes.cls[i])] if names else None
            if label is None or self.override(label, 'transient', False):
                continue
            conf = float(boxes.conf[i])
            if conf < float(self.override(label, 'min_confidence', self.conf_thresh)):
                continue
            x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i])
            if not self.plausible(label, x1, y1, x2, y2, w, h):
                continue
            crop = crop_box(img, x1, y1, x2, y2)
            if crop is not None:
                idxs.append(i)
                crops.append(crop)
        try:
            results = self.clip.classify(crops)
        except Exception as exc:                      # noqa: BLE001 - disable, keep mapping
            self.get_logger().warn(f'CLIP check failed ({exc}); disabling it.')
            self.clip = None
            return {}
        return {i: (lab, p) for i, (lab, p) in zip(idxs, results)
                if p >= self.clip_min_prob and not self.override(lab, 'transient', False)}

    # ----------------------------------------------------------- live boxes

    def schedule(self):
        """Which camera each detection slot goes to.

        Plain round-robin normally. When the console is showing only the front
        view, the front camera gets every other slot: the live boxes you are
        actually watching refresh twice as often, at no extra GPU cost, while
        the other three still keep mapping in the background.
        """
        if self.focus == 'front' and 'front' in self.cameras and len(self.cameras) > 1:
            others = [c for c in self.cameras if c != 'front']
            order = []
            for c in others:
                order += ['front', c]
            return order
        return self.cameras

    def on_focus(self, msg):
        focus = (msg.data or '').strip().lower()
        if focus in ('front', 'all'):
            self.focus = focus

    def plausible(self, label, x1, y1, x2, y2, w, h):
        """Reject detections that cannot be a piece of furniture.

        Two ways the floor was getting boxed as "desk", both closed here:
          * it was labelled as one of the background surfaces we asked the
            detector to recognise (floor, wall, ceiling) -- those exist only
            to soak up regions that would otherwise be forced onto the
            nearest furniture word, and are never shown or mapped;
          * the box is floor-SHAPED: huge, or spanning the full width along
            the bottom edge. The camera is 11 cm off the ground, so floor
            fills the lower half of every frame; nothing we want to map looks
            like that until it is closer than the LiDAR's minimum range.
        This matters beyond the label: a floor box's LiDAR sector hits some
        wall at 2-3 m, and that used to be WRITTEN INTO THE MAP as a desk.
        """
        if label.lower() in self.ignore_set:
            return False
        if w <= 0 or h <= 0:
            return False
        bw, bh = max(0.0, x2 - x1) / w, max(0.0, y2 - y1) / h
        if bw * bh > float(self.override(label, 'max_area', self.max_box_area)):
            return False
        if y2 >= 0.97 * h and bw >= 0.85:
            return False
        return True

    def publish_raw_boxes(self, camera, img, boxes, names):
        """Boxes straight from the detector, when LiDAR/TF are not ready.

        The overlay should still show what the camera sees even before the
        car is localised -- it just cannot say how far away anything is.
        """
        drawn = []
        for i in range(len(boxes)):
            conf = float(boxes.conf[i])
            label = names[int(boxes.cls[i])] if names else None
            if label is None:
                continue
            x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i])
            if not self.plausible(label, x1, y1, x2, y2, img.shape[1], img.shape[0]):
                continue
            drawn.append((x1, y1, x2, y2, label, conf, None, None))
        self.publish_boxes(camera, img.shape[1], img.shape[0], drawn)

    def person_number(self, lid):
        """Stable 'person N' for a tracked person.

        The number is the lowest one not held by anyone currently present, so
        with two people in the room they are person 1 and person 2, and when
        person 1 walks out, "1" is free for whoever comes in next -- rather
        than the count climbing to person 37 over a long session.
        """
        if lid in self.person_numbers:
            return self.person_numbers[lid]
        active = {lm.id for lm in self.transient_now()}
        # Release numbers held by people who are no longer around.
        for old in [k for k in self.person_numbers if k not in active and k != lid]:
            self.person_numbers.pop(old, None)
        used = set(self.person_numbers.values())
        n = 1
        while n in used:
            n += 1
        self.person_numbers[lid] = n
        return n

    @staticmethod
    def box_iou(a, b):
        ix = max(0.0, min(a['x2'], b['x2']) - max(a['x1'], b['x1']))
        iy = max(0.0, min(a['y2'], b['y2']) - max(a['y1'], b['y1']))
        inter = ix * iy
        union = ((a['x2'] - a['x1']) * (a['y2'] - a['y1']) +
                 (b['x2'] - b['x1']) * (b['y2'] - b['y1']) - inter)
        return inter / union if union > 0 else 0.0

    def publish_boxes(self, camera, w, h, drawn):
        """Update this camera's on-screen boxes and publish every camera's.

        ANTI-FLICKER. Each camera is only re-detected every ~0.7 s, and a far
        object hovering near the confidence threshold scores above it on one
        frame and below on the next -- so a box that simply mirrored the last
        frame blinked on and off every couple of seconds. Boxes are therefore
        TRACKS with hysteresis:
          * to APPEAR a box needs the normal confidence (or to have resolved
            to a mapped landmark);
          * once shown it STAYS while the detector still sees it at the lower
            display_confidence, matched frame-to-frame by overlap;
          * it survives complete misses for box_hold_sec, drawn faded
            ("stale") so it is clear it is not freshly confirmed.
        Mapping is unaffected: only detections at full confidence are mapped.
        """
        now = time.time()
        dets = []
        for x1, y1, x2, y2, raw_label, conf, rng, lid in drawn:
            lm = self.landmarks.get(lid) if lid is not None else None
            if lm is not None and lm.transient:
                kind = 'person'
                label = f'{raw_label} {self.person_number(lid)}'
            else:
                kind = 'object'
                # Show the landmark's CORRECTED name when this box resolved to
                # one: if it was relabelled "bed" -> "sofa", the box says sofa
                # even on a frame where the detector wobbled back to "bed".
                label = lm.label if lm is not None else raw_label
            # Appear only when genuinely confident, or when the box sits on a
            # CONFIRMED landmark (seen min_hits times). Not merely "touched
            # some landmark once" -- that let every stray guess on screen.
            min_entry = max(self.box_conf,
                            float(self.override(raw_label, 'min_confidence', self.conf_thresh)))
            entry = (conf >= min_entry
                     or (lm is not None and lm.hits >= self.min_hits))
            dets.append({
                # Normalised 0..1 so the browser can scale to any view size.
                'x1': round(max(0.0, x1 / w), 4), 'y1': round(max(0.0, y1 / h), 4),
                'x2': round(min(1.0, x2 / w), 4), 'y2': round(min(1.0, y2 / h), 4),
                'label': label, 'raw': raw_label, 'conf': round(conf, 2),
                'range': round(rng, 2) if rng is not None else None,
                'kind': kind, 'id': lid, 'mapped': lm is not None, 'entry': entry,
            })

        tracks = self.box_tracks.setdefault(camera, [])
        matched = set()
        for d in sorted(dets, key=lambda d: -d['conf']):
            best, best_iou = None, 0.0
            for t in tracks:
                if id(t) in matched:
                    continue
                same = (d['id'] is not None and d['id'] == t['id']) or d['raw'] == t['raw']
                v = self.box_iou(d, t)
                if ((same and v > 0.2) or v > 0.6) and v > best_iou:
                    best, best_iou = t, v
            if best is not None:
                if d['id'] is None and best['id'] is not None:
                    # A weak frame of an already-identified box: keep its
                    # identity ("person 2", the corrected name, its distance)
                    # rather than regressing to the raw detector label.
                    for k in ('id', 'label', 'kind', 'mapped', 'range'):
                        d[k] = best[k]
                key, hits = best['key'], best['hits']
                best.clear()
                best.update(d, key=key, hits=hits + 1, last=now)
                matched.add(id(best))
            elif d['entry']:
                self.box_seq += 1
                d.update(key=f'{camera}-{self.box_seq}', hits=1, last=now)
                tracks.append(d)
                matched.add(id(d))
        tracks[:] = [t for t in tracks if now - t['last'] <= self.box_hold]

        out = []
        for t in tracks:
            o = {k: t[k] for k in ('x1', 'y1', 'x2', 'y2', 'label', 'raw', 'conf',
                                   'range', 'kind', 'id', 'mapped', 'key')}
            o['stale'] = t['last'] < now
            out.append(o)
        self.latest_boxes[camera] = {'t': now, 'boxes': out}
        msg = String()
        msg.data = json.dumps({
            'time': now,
            'cameras': {cam: {'age': round(now - v['t'], 2), 'boxes': v['boxes']}
                        for cam, v in self.latest_boxes.items()},
        })
        self.boxes_pub.publish(msg)

    def collect_calibration(self, cluster, origin, rot, x1, x2, cx, fx):
        """One focal-length sample from a bbox/LiDAR-cluster pair.

        Only samples where the cluster is comfortably narrower than the search
        sector are kept. A cluster that fills the sector was probably clipped
        by it, and its angular width would then just re-derive the fx we
        started with instead of measuring anything.
        """
        if not self.auto_calibrate or cluster is None or cluster.shape[0] < 3:
            return
        rel = cluster - origin
        # Into the optical frame, where u = fx * (x/z) + cx.
        inv = rot.T
        ts = []
        for px, py in rel:
            d = inv @ np.array([px, py, 0.0])
            if d[2] <= 1e-6:
                return
            ts.append(d[0] / d[2])
        t_lo, t_hi = min(ts), max(ts)
        if t_hi - t_lo < 1e-6:
            return
        # Pixel width the detector reported, against angular width the LiDAR
        # measured. Their ratio is the focal length, independent of the fx
        # currently in use -- which is what makes this a real calibration.
        px_width = abs(x2 - x1)
        sector = abs((x2 - cx) / fx - (x1 - cx) / fx)
        if sector <= 1e-6 or (t_hi - t_lo) > 0.9 * sector:
            return                       # cluster clipped by the sector
        fx_est = px_width / (t_hi - t_lo)
        if 0.2 * fx < fx_est < 5.0 * fx:
            self.fx_samples.append(fx_est)

    def scan_points(self, scan):
        """LaserScan -> Nx2 array of valid (x, y) in the scan's own frame."""
        n = len(scan.ranges)
        if n == 0:
            return np.empty((0, 2))
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > max(scan.range_min, 1e-3)) & \
            (ranges < min(scan.range_max, self.max_range * 2.0))
        if not np.any(valid):
            return np.empty((0, 2))
        r, a = ranges[valid], angles[valid]
        return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)

    def range_in_sector(self, points, origin, b1, b2):
        """Nearest LiDAR cluster inside the bbox's angular sector.

        Bearings are measured FROM THE CAMERA, not from the LiDAR: the two are
        up to ~20 cm apart on this chassis, which is a several-degree error at
        close range -- enough to pick the wrong object on a cluttered wall.
        """
        if points.shape[0] == 0:
            return None
        rel = points - origin
        dist = np.hypot(rel[:, 0], rel[:, 1])
        bearing = np.arctan2(rel[:, 1], rel[:, 0])

        width = ang_norm(b2 - b1)
        lo = b1 if width >= 0 else b2
        width = abs(width)
        if width < 1e-4:
            return None
        delta = np.mod(bearing - lo + math.pi, 2.0 * math.pi) - math.pi
        inside = (delta >= 0.0) & (delta <= width) & \
            (dist >= self.min_range) & (dist <= self.max_range)
        if not np.any(inside):
            return None

        sel = points[inside]
        d = dist[inside]
        order = np.argsort(d)
        sel, d = sel[order], d[order]
        # Nearest cluster wins: the detected object occludes whatever is behind
        # it, so the closest surface in the sector is the object and the wall
        # further back is correctly discarded.
        end = 1
        while end < len(d) and (d[end] - d[end - 1]) <= self.cluster_gap:
            end += 1
        if end < self.min_cluster_points:
            return None
        cluster = sel[:end]
        cx, cy = float(np.mean(cluster[:, 0])), float(np.mean(cluster[:, 1]))
        # The cluster itself is returned as well: calibrate() needs the
        # object's true angular extent, not just where its middle is.
        return cx, cy, float(np.mean(d[:end])), cluster

    # ----------------------------------------------------------------- frames

    def camera_pose(self, camera, stamp, scan_frame):
        """(camera origin xy, rotation matrix) of a camera in the scan frame."""
        frame = CAMERA_FRAMES[camera]
        tf = self.lookup(scan_frame, frame, stamp)
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.array([t.x, t.y]), quat_to_matrix(q.x, q.y, q.z, q.w)

    def to_map(self, x, y, src_frame, stamp):
        tf = self.lookup(self.map_frame, src_frame, stamp)
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        p = quat_to_matrix(q.x, q.y, q.z, q.w) @ np.array([x, y, 0.0])
        return float(p[0] + t.x), float(p[1] + t.y)

    def to_frame(self, x, y, dst_frame, stamp):
        """Map coordinate -> some other frame. The inverse of to_map, used to
        ask "where would this landmark appear from where the car is now?"."""
        tf = self.lookup(dst_frame, self.map_frame, stamp)
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        p = quat_to_matrix(q.x, q.y, q.z, q.w) @ np.array([x, y, 0.0])
        return float(p[0] + t.x), float(p[1] + t.y)

    def lookup(self, target, source, stamp):
        """TF at the image's own timestamp, falling back to the latest.

        Using the image stamp matters while the car is turning. The fallback
        exists because Cartographer can briefly outrun the TF buffer during a
        loop closure, and dropping every detection for that window would lose
        exactly the sightings a loop closure is meant to improve.
        """
        try:
            return self.tf_buffer.lookup_transform(
                target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.05))
        except Exception:                             # noqa: BLE001 - fall through
            try:
                return self.tf_buffer.lookup_transform(target, source, Time())
            except Exception:                         # noqa: BLE001 - not ready yet
                return None

    # -------------------------------------------------------------- landmarks

    def add_observation(self, label, x, y, rng, conf, camera, stamp):
        """Fuse one sighting into the landmark database, in MAP coordinates.

        Because x/y are already map-frame here, a sighting from any camera --
        and from any car pose -- lands in the same space, which is what makes
        the front/left double-mapping problem disappear rather than needing to
        be detected and undone.
        """
        radius = float(self.override(label, 'assoc_radius', self.assoc_radius))
        transient = bool(self.override(label, 'transient', False))
        # Closer sightings are geometrically AND semantically more reliable --
        # more pixels on the object, less motion blur, a tighter LiDAR
        # cluster -- so they must outweigh distant ones enough to actually
        # correct an earlier mistake, not merely nudge it.
        # Steep on purpose: a sighting at 0.8 m carries ~7x the weight of one
        # at 5 m. Distant guesses should barely move the estimate, so that a
        # close pass can actually overturn them rather than merely nudge them.
        weight = conf / (0.3 + 0.7 * max(0.0, rng))
        t = stamp.sec + stamp.nanosec * 1e-9

        with self.lock:
            # Pass 1: same label, normal (generous) gate.
            best, best_d = None, radius
            for lm in self.landmarks.values():
                if lm.label != label:
                    continue
                d = math.hypot(lm.x - x, lm.y - y)
                if d < best_d:
                    best, best_d = lm, d

            # Pass 2: RELABELLING. Nothing of this class here, but if some
            # other landmark sits almost exactly where this detection landed,
            # it is far more likely to be the same physical object seen better
            # than a second object hiding inside the first. Feed it in as a
            # vote instead of creating a rival landmark, and let the votes
            # decide the name. The gate is deliberately much tighter than the
            # same-label one so a desk and the chair tucked under it stay two
            # things.
            if best is None:
                tight = min(radius, self.revision_radius)
                for lm in self.landmarks.values():
                    if lm.transient != transient:
                        continue
                    d = math.hypot(lm.x - x, lm.y - y)
                    if d < tight:
                        was = lm.label
                        lm.update(label, x, y, weight, conf, camera, t, rng)
                        if lm.label != was:
                            self.stats['relabels'] += 1
                            self.get_logger().info(
                                f'Landmark {lm.id} relabelled "{was}" -> "{lm.label}" '
                                f'(seen from {rng:.1f} m)')
                        return lm.id

            if best is not None:
                best.update(label, x, y, weight, conf, camera, t, rng)
                return best.id
            lid = self.next_id
            self.next_id += 1
            self.landmarks[lid] = Landmark(
                lid, label, x, y, weight, conf, camera, t, transient,
                rng, self.max_weight)
            return lid

    def calibrate(self):
        """Learn the real camera FOV from the LiDAR, while simply driving.

        Quanser publish no intrinsics and the CSI driver emits no CameraInfo,
        so hfov_deg starts as an educated guess -- and every bearing, and
        therefore every landmark position, is only as good as that guess.

        But each resolved detection is a free calibration sample: the bounding
        box gives the object's width in PIXELS, and the LiDAR cluster it
        matched gives the same object's width in ANGLE. One divided by the
        other is the focal length. Collect those while the car drives around
        and the median converges on the truth, which tightens every bearing
        from then on -- and with it the accuracy of "go to the air cooler".

        The median (not the mean) matters: a box that clipped the object, or a
        cluster that caught the wall behind it, produces a wild sample, and a
        mean would chase it.
        """
        if not self.auto_calibrate or len(self.fx_samples) < self.calib_min_samples:
            return
        samples = np.array(self.fx_samples, dtype=float)
        fx = float(np.median(samples))
        if not np.isfinite(fx) or fx <= 1.0:
            return
        hfov = 2.0 * math.atan((self.width / 2.0) / fx) if self.width else None
        if hfov is None:
            return
        # Clamp to physically plausible lenses. If the estimate lands outside
        # this, the samples are wrong, not the lens -- better to keep the
        # default than to silently adopt nonsense geometry.
        hfov = max(math.radians(60.0), min(math.radians(165.0), hfov))
        if abs(hfov - self.hfov) < math.radians(1.5):
            return
        self.get_logger().info(
            f'Camera FOV auto-calibrated: {math.degrees(self.hfov):.1f} deg -> '
            f'{math.degrees(hfov):.1f} deg (median of {len(samples)} LiDAR-matched '
            f'detections)')
        self.hfov = hfov
        self.hfov_source = f'auto ({len(samples)} samples)'

    def verify_unseen(self, origin, rot, points, seen_ids, scan_frame, stamp,
                      weak_labels=()):
        """Penalise landmarks the car looked straight at and did not see.

        Association alone can only ever ADD things: a false positive, or an
        object that has since been carried out of the room, would otherwise
        sit on the map forever. This is the other half -- evidence of absence.

        A miss is only counted when absence is actually meaningful:
          * the landmark is inside this camera's field of view,
          * it is within detection range,
          * and the LiDAR shows clear space up to it, so nothing is standing
            in front of it. Without that last check the car would "disprove"
            the sofa every time a person walked between it and the camera.
        """
        if not self.verify_absent:
            return
        half_fov = self.hfov / 2.0 * self.verify_fov_fraction
        with self.lock:
            # A weak sighting of the same kind of object is not evidence of
            # absence -- it is usually that very object, far away or badly
            # lit. Penalising it would slowly delete real furniture.
            candidates = [lm for lm in self.landmarks.values()
                          if lm.id not in seen_ids and lm.hits >= self.min_hits
                          and lm.label not in weak_labels]
        for lm in candidates:
            local = self.to_frame(lm.x, lm.y, scan_frame, stamp)
            if local is None:
                continue
            dx, dy = local[0] - origin[0], local[1] - origin[1]
            dist = math.hypot(dx, dy)
            if dist < self.min_range or dist > self.verify_max_range:
                continue
            # Bearing of the landmark relative to where this camera points.
            axis = rot @ np.array([0.0, 0.0, 1.0])
            if abs(axis[0]) < 1e-9 and abs(axis[1]) < 1e-9:
                continue
            off = ang_norm(math.atan2(dy, dx) - math.atan2(axis[1], axis[0]))
            if abs(off) > half_fov:
                continue                      # not in this camera's view
            if points.shape[0]:
                rel = points - origin
                bearing = np.arctan2(rel[:, 1], rel[:, 0])
                delta = np.abs(np.mod(bearing - math.atan2(dy, dx) + math.pi,
                                      2 * math.pi) - math.pi)
                near = np.hypot(rel[:, 0], rel[:, 1])[delta < math.radians(4.0)]
                if near.size and float(np.min(near)) < dist - 0.40:
                    continue                  # something is in the way
            lm.miss()

    def prune_pass(self):
        """Delete landmarks that the evidence no longer supports."""
        removed = []
        with self.lock:
            for lid, lm in list(self.landmarks.items()):
                # Consistently looked for and not found -> it is not there.
                if lm.misses >= self.max_misses and lm.misses > lm.hits:
                    removed.append((lm.label, lm.id))
                    self.landmarks.pop(lid, None)
        for label, lid in removed:
            self.stats['pruned'] += 1
            self.get_logger().info(
                f'Landmark {lid} ("{label}") removed: repeatedly not seen where it was mapped.')

    def merge_pass(self):
        """Collapse same-label landmarks that have drifted into each other.

        Needed because SLAM is not static: a Cartographer loop closure shifts
        the map under landmarks that were placed earlier, so two entries that
        were legitimately distinct when created can end up on the same object.
        """
        with self.lock:
            items = sorted(self.landmarks.values(), key=lambda l: -l.weight)
            dead = set()
            for i, a in enumerate(items):
                if a.id in dead:
                    continue
                radius = float(self.override(a.label, 'assoc_radius', self.assoc_radius))
                for b in items[i + 1:]:
                    if b.id in dead or b.transient != a.transient:
                        continue
                    d = math.hypot(a.x - b.x, a.y - b.y)
                    # Same name: merge within the class's association radius.
                    # DIFFERENT names but practically the same spot (a "desk"
                    # and a "coffee table" 20 cm apart): one object seen two
                    # ways -- merge, and the pooled votes pick the name. The
                    # tight revision radius keeps a chair beside a desk apart.
                    if (b.label == a.label and d < radius) or \
                            (not a.transient and d < self.revision_radius):
                        a.absorb(b)
                        dead.add(b.id)
            for lid in dead:
                self.landmarks.pop(lid, None)
        if dead:
            self.get_logger().debug(f'Merged {len(dead)} duplicate landmark(s)')

    def confirmed(self):
        with self.lock:
            return [lm for lm in self.landmarks.values()
                    if lm.hits >= self.min_hits and not lm.transient]

    def transient_now(self):
        """Transient things (people) seen recently enough to still count.

        Kept separate from confirmed(): a person is real and worth reporting
        live, but writing them into a saved map would be wrong -- they move.
        The recency window is what makes "how many people are here?" mean NOW
        rather than "at any point during the whole mapping run".
        """
        now = time.time()
        with self.lock:
            return [lm for lm in self.landmarks.values()
                    if lm.transient and lm.hits >= 2
                    and (now - lm.last_seen) < self.transient_ttl]

    # --------------------------------------------------------------- outputs

    def publish_objects(self):
        objects = sorted(self.confirmed(), key=lambda l: (l.label, l.id))
        dicts = number_names([lm.as_dict() for lm in objects])
        payload = {
            'objects': dicts,
            'stats': {
                'tracked': len(self.landmarks),
                'confirmed': len(objects),
                'detections': self.stats['detections'],
                'unresolved': self.stats['unresolved'],
                'frames': self.stats['frames'],
                'infer_ms': round(self.stats['infer_ms'], 1),
                'relabels': self.stats['relabels'],
                'pruned': self.stats['pruned'],
                'clip_renamed': self.stats['clip_renamed'],
                'clip_vetoed': self.stats['clip_vetoed'],
                'clip': 'on' if self.clip is not None else 'off',
                'hfov_deg': round(math.degrees(self.hfov), 1),
                'hfov_source': self.hfov_source,
                'calib_samples': len(self.fx_samples),
                'detector': 'error' if self.model_error else
                            ('ready' if self.model is not None else 'loading'),
            },
            # Live (unsaved) transient detections, e.g. people. Counted for
            # questions like "how many people are here?" but deliberately kept
            # out of the saved map, since they walk away.
            'transient': [lm.as_dict() for lm in self.transient_now()],
        }
        if self.model_error:
            payload['stats']['error'] = self.model_error
        msg = String()
        msg.data = json.dumps(payload)
        self.objects_pub.publish(msg)
        self.publish_markers(objects, {d['id']: d['name'] for d in dicts})

    def publish_markers(self, objects, names=None):
        names = names or {}
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        now = self.get_clock().now().to_msg()
        for lm in objects:
            body = Marker()
            body.header.frame_id = self.map_frame
            body.header.stamp = now
            body.ns = 'objects'
            body.id = lm.id * 2
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position = Point(x=lm.x, y=lm.y, z=0.15)
            body.pose.orientation.w = 1.0
            body.scale = Vector3(x=0.28, y=0.28, z=0.30)
            body.color = ColorRGBA(r=0.15, g=0.85, b=0.95, a=0.75)
            body.lifetime = DurationMsg(sec=0)
            array.markers.append(body)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = 'object_labels'
            text.id = lm.id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=lm.x, y=lm.y, z=0.45)
            text.pose.orientation.w = 1.0
            text.scale = Vector3(x=0.0, y=0.0, z=0.18)
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            text.text = names.get(lm.id, lm.label)
            text.lifetime = DurationMsg(sec=0)
            array.markers.append(text)
        self.marker_pub.publish(array)

    def publish_debug_image(self, img, drawn, camera):
        try:
            import cv2
        except ImportError:
            return
        canvas = img.copy()
        for x1, y1, x2, y2, label, conf, rng, _lid in drawn:
            # Green = placed on the map, amber = detected but no LiDAR return
            # in its sector (see the 2D-plane limitation in the module docstring).
            colour = (0, 220, 0) if rng is not None else (0, 170, 255)
            cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), colour, 2)
            tag = f'{label} {conf:.2f}' + (f' {rng:.1f}m' if rng is not None else ' no range')
            cv2.putText(canvas, tag, (int(x1), max(14, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
        cv2.putText(canvas, camera, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = CAMERA_FRAMES[camera]
        msg.height, msg.width = canvas.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = canvas.tobytes()
        self.debug_pub.publish(msg)

    # ------------------------------------------------------------------ save

    def on_save_request(self, msg):
        target = (msg.data or '').strip() or 'qcar_map'
        path = self.save(target)
        # Autosave asks every 10 s; only say so when the object count changed,
        # so the terminal is not a wall of identical "Saved ..." lines.
        count = len(self.confirmed())
        if path and count != self._last_saved_count:
            self._last_saved_count = count
            self.get_logger().info(f'Saved {count} objects to {path}')

    def save(self, target):
        """Write <map>_objects.json beside the .pgm/.yaml the map saver wrote."""
        if os.path.isabs(target):
            base = target[:-5] if target.endswith('.yaml') else target
        else:
            base = os.path.join(self.objects_dir, target)
        path = base + '_objects.json'
        objects = sorted(self.confirmed(), key=lambda l: (l.label, l.id))
        payload = {
            'version': 1,
            'map': os.path.basename(base),
            'frame_id': self.map_frame,
            'created': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'vocabulary': self.vocab,
            'synonyms': self.synonyms,
            'objects': number_names([lm.as_dict() for lm in objects]),
        }
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            # temp + rename: autosave rewrites this every 10 s, and a reader
            # (or a Ctrl+C) must never catch a half-written file.
            with open(path + '.tmp', 'w') as fh:
                json.dump(payload, fh, indent=2)
            os.replace(path + '.tmp', path)
        except OSError as exc:
            self.get_logger().error(f'Could not write {path}: {exc}')
            return None
        return path

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ObjectMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
