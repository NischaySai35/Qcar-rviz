#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Draw a readable QCar2 body and heading indicator in RViz."""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class QCarPoseMarker(Node):
    def __init__(self):
        super().__init__('qcar2_pose_marker')
        self.publisher = self.create_publisher(MarkerArray, '/qcar2/visual', 1)
        self.create_timer(0.2, self.publish)

    @staticmethod
    def marker(marker_id, marker_type, x, y, z, sx, sy, sz, color):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.ns, marker.id, marker.type, marker.action = 'qcar2', marker_id, marker_type, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
        return marker

    def publish(self):
        markers = MarkerArray()
        # x is the QCar's forward direction. The body and bright nose make
        # position and heading easy to read from the map view.
        markers.markers.append(self.marker(0, Marker.CUBE, 0.0, 0.0, 0.14, 0.52, 0.31, 0.18, (0.08, 0.52, 0.78)))
        markers.markers.append(self.marker(1, Marker.CUBE, 0.08, 0.0, 0.28, 0.30, 0.26, 0.12, (0.72, 0.88, 1.0)))
        markers.markers.append(self.marker(2, Marker.CUBE, 0.30, 0.0, 0.15, 0.07, 0.31, 0.14, (1.0, 0.62, 0.08)))
        for marker_id, (x, y) in enumerate(((0.14, 0.20), (0.14, -0.20), (-0.16, 0.20), (-0.16, -0.20)), start=3):
            markers.markers.append(self.marker(marker_id, Marker.CUBE, x, y, 0.07, 0.13, 0.07, 0.13, (0.05, 0.07, 0.10)))
        arrow = self.marker(7, Marker.ARROW, 0.24, 0.0, 0.33, 0.38, 0.12, 0.12, (0.15, 1.0, 0.65))
        markers.markers.append(arrow)
        label = self.marker(8, Marker.TEXT_VIEW_FACING, 0.0, 0.0, 0.52, 0.0, 0.0, 0.16, (0.85, 0.95, 1.0))
        label.text = 'QCAR2  →'
        markers.markers.append(label)
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = QCarPoseMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass            # Ctrl+C is a normal stop, not a crash to report
    finally:
        node.destroy_node()
        if rclpy.ok():  # ROS's own signal handler may already have shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
