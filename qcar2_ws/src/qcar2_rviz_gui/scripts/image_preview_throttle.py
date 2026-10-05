#!/usr/bin/python3
# ROOT CAUSE FOUND, 2026-09-15: this node crashed instantly on every launch
# with "ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'".
# `#!/usr/bin/env python3` was resolving PATH to pyenv's Python 3.7
# (~/.pyenv/versions/3.7.17/...), not the system Python 3.8 ROS Humble's
# rclpy C extension is actually compiled for -- proven from the captured
# traceback, which showed the import failing inside pyenv's own
# importlib, not ROS's. Hardcoding the system interpreter here bypasses
# PATH/pyenv entirely. Applied to every script in this package, since
# they all carried the same shebang and are equally exposed even though
# this is the only one that had actually hit it in practice.
"""Republish the newest camera image at a display-friendly fixed rate."""

import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

# This node has crashed silently on startup (exit code 1) with no traceback
# visible in the launch log, which only records process lifecycle events, not
# stdout/stderr content. Writing the full traceback here -- in addition to
# the normal stderr print Python already does -- means the actual error
# survives past a fast-scrolling multi-node launch and can be read back after
# the fact instead of guessed at.
_CRASH_LOG = '/tmp/front_camera_preview_crash.log'


class ImagePreviewThrottle(Node):
    def __init__(self):
        super().__init__('front_camera_preview')
        self.declare_parameter('input_topic', '/front/camera/csi_image')
        self.declare_parameter('output_topic', '/front/camera/preview')
        self.declare_parameter('preview_hz', 5.0)
        self.latest_image = None
        self.warned_no_image = False
        self.publisher = self.create_publisher(
            Image, self.get_parameter('output_topic').value, 1)
        # Accept either reliability mode used by CSI/image_transport drivers.
        camera_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, self.get_parameter('input_topic').value, self._on_image,
            camera_qos)
        preview_hz = max(float(self.get_parameter('preview_hz').value), 0.5)
        self.create_timer(1.0 / preview_hz, self._publish_latest)

    def _on_image(self, image):
        self.latest_image = image
        self.warned_no_image = False

    def _publish_latest(self):
        if self.latest_image is not None:
            self.publisher.publish(self.latest_image)
        elif not self.warned_no_image:
            # A SECOND bug found and fixed the same day as the DISPLAY/shebang
            # ones: rclpy's Logger.warning() is not Python stdlib logging --
            # it does not accept a separate %s substitution argument, only
            # the message string itself. This threw TypeError on every first
            # tick before any frame arrived, which is exactly why it was
            # never seen before: the node always crashed at import time
            # first (see the shebang fix above), so this line never actually
            # ran until that bug was fixed and let execution reach it.
            self.get_logger().warning(
                'No front-camera frames received yet on '
                f'{self.get_parameter("input_topic").value}. Check the '
                'csi_front launch output and CSI cable/camera power.')
            self.warned_no_image = True


def main():
    try:
        rclpy.init()
        node = ImagePreviewThrottle()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            if rclpy.ok():  # ROS's own signal handler may already have shut down
                rclpy.shutdown()
    except KeyboardInterrupt:
        pass                # Ctrl+C is a normal stop, not a crash to report
    except SystemExit:
        raise
    except Exception:
        with open(_CRASH_LOG, 'a') as f:
            f.write(traceback.format_exc() + '\n')
        raise


if __name__ == '__main__':
    main()
