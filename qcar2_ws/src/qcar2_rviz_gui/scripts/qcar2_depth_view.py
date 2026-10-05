#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Front view from the RealSense D435, with distances drawn on it.

The D435 on top of the car (USB 8086:0b07) gives colour plus depth. In
navigation it replaces the small CSI bumper camera as THE front camera
(operator decision, 2026-10-05): this node turns the realsense2_camera
driver's output into the two images everything else uses.

  /front/camera/depth_view  console view: colour + a centre crosshair with its
                            distance, a marker on the nearest thing in view
                            with its distance, and the camera's real fps.
  /front/camera/preview     the same colour frame with NOTHING drawn on it --
                            qcar2_assistant.py hands this to the vision model,
                            which would otherwise "see" the overlay text.

Depth comes from the driver's aligned_depth_to_color stream, so the distance
at a pixel is the distance of exactly what is visible at that pixel (the raw
depth image is from a different lens with a wider field of view).

Distances are along the camera's viewing axis, in cm, from the front of the
camera. The D435 cannot measure closer than ~20 cm (shown as "<20 cm" when
the centre is that close and the sensor returns nothing) and gets noisy past
~4 m. Each reading is a median over a small patch, so a single bad pixel
cannot produce a wild number.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

MIN_VALID_MM = 150          # below the D435's minimum range: treat as no data
MAX_SHOWN_MM = 10000


def to_bgr(msg):
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()


def to_depth_mm(msg):
    if msg.encoding not in ('16UC1', 'mono16'):
        return None
    d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)
    return d[:, :msg.width]


def patch_median(depth, cx, cy, half):
    """Median of valid depth (mm) in a square patch, or None."""
    h, w = depth.shape
    p = depth[max(0, cy - half):min(h, cy + half + 1), max(0, cx - half):min(w, cx + half + 1)]
    v = p[(p >= MIN_VALID_MM) & (p <= MAX_SHOWN_MM)]
    return float(np.median(v)) if v.size >= max(4, p.size // 4) else None


def nearest_block(depth, block=16):
    """Closest region as (x, y, mm) at block resolution, or None.

    Per-block MEDIAN, not the minimum pixel: depth has speckle at edges, and
    one stray close pixel would otherwise be reported as "nearest" every frame.
    """
    h, w = depth.shape
    bh, bw = h // block, w // block
    d = depth[:bh * block, :bw * block].astype(np.float32)
    d[(d < MIN_VALID_MM) | (d > MAX_SHOWN_MM)] = np.nan
    blocks = d.reshape(bh, block, bw, block).transpose(0, 2, 1, 3).reshape(bh, bw, -1)
    valid = np.sum(~np.isnan(blocks), axis=2)
    with np.errstate(all='ignore'):
        med = np.nanmedian(blocks, axis=2)
    med[valid < block * block // 3] = np.nan       # mostly-empty block: no reading
    if np.all(np.isnan(med)):
        return None
    by, bx = np.unravel_index(np.nanargmin(med), med.shape)
    return int(bx * block + block // 2), int(by * block + block // 2), float(med[by, bx])


def label(img, text, x, y, colour):
    """Readable text on any background: dark box behind it."""
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
    (tw, th), base = cv2.getTextSize(text, font, scale, thick)
    x = int(min(max(4, x), img.shape[1] - tw - 4))
    y = int(min(max(th + 4, y), img.shape[0] - 4))
    cv2.rectangle(img, (x - 3, y - th - 4), (x + tw + 3, y + base), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), font, scale, colour, thick, cv2.LINE_AA)


def cm(mm):
    return f'{int(round(mm / 10.0))} cm'


class DepthView(Node):

    def __init__(self):
        super().__init__('qcar2_depth_view')
        self.declare_parameter('color_topic', '/front/realsense/color/image_raw')
        self.declare_parameter('depth_topic', '/front/realsense/aligned_depth_to_color/image_raw')
        self.declare_parameter('view_topic', '/front/camera/depth_view')
        self.declare_parameter('clean_topic', '/front/camera/preview')
        self.declare_parameter('rate_hz', 10.0)
        g = lambda n: self.get_parameter(n).value

        self.color = None
        self.depth = None
        self.color_times = []
        self.view_pub = self.create_publisher(Image, g('view_topic'), 1)
        self.clean_pub = self.create_publisher(Image, g('clean_topic'), 1)
        self.subs = []
        self.subscribe(g('color_topic'), g('depth_topic'))
        self.create_timer(1.0 / float(g('rate_hz')), self.publish)
        self.started = time.monotonic()
        self.rediscovered = False
        self.get_logger().info(f'RealSense depth view: {g("color_topic")} + {g("depth_topic")}')

    def subscribe(self, color_topic, depth_topic):
        for s in self.subs:
            self.destroy_subscription(s)
        self.subs = [
            self.create_subscription(Image, color_topic, self.on_color, qos_profile_sensor_data),
            self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data),
        ]

    def rediscover(self):
        """Driver versions name topics differently (/camera/color/... vs
        /camera/camera/color/...). If nothing arrived on the configured names,
        look for the right ones once instead of showing a black view."""
        names = [n for n, _ in self.get_topic_names_and_types()]
        color = next((n for n in names if n.endswith('/color/image_raw')
                      and 'aligned' not in n), None)
        depth = next((n for n in names if n.endswith('/aligned_depth_to_color/image_raw')), None)
        if color and depth:
            self.get_logger().warn(f'No frames on the configured topics; using {color} + {depth}')
            self.subscribe(color, depth)

    def on_color(self, msg):
        self.color = msg
        now = time.monotonic()
        self.color_times.append(now)
        while self.color_times and now - self.color_times[0] > 2.0:
            self.color_times.pop(0)

    def on_depth(self, msg):
        self.depth = msg

    def publish(self):
        if self.color is None:
            if not self.rediscovered and time.monotonic() - self.started > 8.0:
                self.rediscovered = True
                self.rediscover()
            return
        msg = self.color
        self.clean_pub.publish(msg)               # untouched, for the vision model
        img = to_bgr(msg)
        h, w = img.shape[:2]
        fps = len(self.color_times) / 2.0
        depth = to_depth_mm(self.depth) if self.depth is not None else None
        if depth is not None and depth.shape[:2] != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        cx, cy = w // 2, h // 2
        white, yellow = (255, 255, 255), (0, 220, 255)
        cv2.drawMarker(img, (cx, cy), white, cv2.MARKER_CROSS, 22, 2)
        if depth is None:
            label(img, 'no depth', cx + 14, cy - 10, white)
        else:
            c = patch_median(depth, cx, cy, 5)
            label(img, cm(c) if c is not None else '<20 cm / --', cx + 14, cy - 10, white)
            near = nearest_block(depth)
            if near is not None:
                nx, ny, nmm = near
                cv2.circle(img, (nx, ny), 12, yellow, 2)
                text = f'nearest {cm(nmm)}'
                tw = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0][0]
                # Label on whichever side has room, so it never hides the marker.
                lx = nx + 18 if nx + 18 + tw < w - 4 else nx - 18 - tw
                label(img, text, lx, ny + 6, yellow)
        label(img, f'RealSense {fps:.0f} fps', 8, 22, white)

        out = Image()
        out.header = msg.header
        out.height, out.width = h, w
        out.encoding = 'bgr8'
        out.step = w * 3
        out.data = img.tobytes()
        self.view_pub.publish(out)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DepthView()
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
