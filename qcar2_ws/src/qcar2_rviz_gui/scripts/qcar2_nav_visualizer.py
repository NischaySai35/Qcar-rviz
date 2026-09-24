#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Rich navigation overlay for the QCar2 RViz GUI.

RViz's stock Path display can only draw a bare polyline, or an arrow at every
single pose (Nav2 emits one roughly every 5 cm, which is unreadable and slow).
This node adds the parts that make the plan actually readable while driving:

  /qcar2/path_markers   - evenly spaced travel-direction arrows along the
                          global plan, plus a goal flag and a distance label
  /qcar2/motion_markers - a live velocity arrow above the car, green when
                          driving forward and red when reversing, with a
                          speed readout

Both are plain MarkerArrays, so they are just two more displays in RViz.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry, Path
from qcar2_interfaces.msg import MotorCommands
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import ColorRGBA
from action_msgs.msg import GoalStatus, GoalStatusArray
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray

# Spacing between direction arrows along the plan, in metres. Large enough to
# stay legible on a small indoor map, small enough to read the curve.
ARROW_SPACING_M = 0.45
ARROW_LENGTH_M = 0.26
# Below this speed the car is treated as stopped, so the motion arrow does not
# flicker on sensor noise while it is holding position.
MOVING_SPEED_MPS = 0.02

GREEN = ColorRGBA(r=0.0, g=1.0, b=0.53, a=0.95)
PALE_GREEN = ColorRGBA(r=0.70, g=1.0, b=0.86, a=1.0)
RED = ColorRGBA(r=1.0, g=0.25, b=0.25, a=0.95)
AMBER = ColorRGBA(r=1.0, g=0.75, b=0.0, a=1.0)


def yaw_to_quaternion(yaw):
    """Marker orientation for an arrow lying flat, pointing along `yaw`."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class QCar2NavVisualizer(Node):
    def __init__(self):
        super().__init__('qcar2_nav_visualizer')

        self.path_publisher = self.create_publisher(MarkerArray, '/qcar2/path_markers', 1)
        self.motion_publisher = self.create_publisher(MarkerArray, '/qcar2/motion_markers', 1)

        # Nav2 publishes /plan with a transient-local-ish volatile profile;
        # match the reliable/keep-last defaults it actually uses.
        plan_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Path, '/plan', self.on_path, plan_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # qcar2_goal_reset fires this when the operator sets a new initial
        # pose.  planner_server stops publishing /plan when its goal is
        # cancelled, so without an explicit signal the old arrows and goal
        # flag would stay frozen on screen against the corrected map.
        self.create_subscription(Empty, '/qcar2/nav_reset', self.on_reset, 1)
        # Nav2 never says out loud that it finished, so the operator could not
        # tell "arrived" from "still trying" -- and an Ackermann car nudging at
        # the last few centimetres looks exactly like circling.  Watch the
        # action result and put it on screen.
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.on_nav_status, 10)
        self.banner = None
        self.banner_until = 0
        self.last_status = None

        self.latest_odom = None
        self.create_timer(0.1, self.publish_motion)

        # Steering actually being applied, for the on-screen readout. This is
        # the converter's output, i.e. after the learned bias correction, so
        # it shows what the wheels are really being asked to do.
        self.steering_cmd = 0.0
        self.create_subscription(
            MotorCommands, '/qcar2_motor_speed_cmd', self.on_motor_cmd, 10)
        # ...and the angle the wheels evidently REACHED, inferred by the
        # converter from gyro yaw rate and wheel speed (there is no steering
        # sensor on this car).  NaN when the car is too slow to infer it.
        self.steering_actual = math.nan
        self.create_subscription(
            Vector3, '/qcar2/steering_report', self.on_steering_report, 10)

        # /plan arrives at 1 Hz; redrawing on a faster timer against the car's
        # live pose keeps the drawn line starting at the car between plans.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_path = None
        self.latest_path_frame = 'map'
        self.create_timer(0.2, self.redraw_path)

    # ---------------------------------------------------------------- status

    def on_nav_status(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        if status == self.last_status:
            return
        self.last_status = status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.banner = ('GOAL REACHED', GREEN)
            self.get_logger().info('GOAL REACHED -- ready for a new goal.')
            # Drop the plan arrows: that route is finished, and leaving them up
            # is what made a completed run look like it was still going.
            self.on_reset(None)
        elif status == GoalStatus.STATUS_ABORTED:
            self.banner = ('NAV FAILED', RED)
            self.get_logger().warn('Navigation ABORTED -- set a new goal.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.banner = ('GOAL CANCELLED', AMBER)
            self.get_logger().info('Goal cancelled.')
        elif status == GoalStatus.STATUS_EXECUTING:
            self.banner = None
        if self.banner is not None:
            self.banner_until = self.get_clock().now().nanoseconds + 15_000_000_000

    def on_reset(self, _message):
        """Drop every path marker: the plan they described is cancelled."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self.path_publisher.publish(markers)

    # ------------------------------------------------------------------ path

    @staticmethod
    def _angle_text(radians):
        degrees = math.degrees(radians)
        if abs(degrees) < 1.5:
            return 'CENTRE'
        # Positive steering is a left turn: the converter derives it from
        # atan(L * wz / v) and a positive wz is counter-clockwise.
        return f'{abs(degrees):.0f} deg {"LEFT" if degrees > 0 else "RIGHT"}'

    def steering_text(self):
        text = f'STEER {self._angle_text(self.steering_cmd)}'
        if not math.isnan(self.steering_actual):
            text += f'  |  WHEELS {self._angle_text(self.steering_actual)}'
        return text

    def on_steering_report(self, msg):
        self.steering_actual = msg.z

    def on_motor_cmd(self, msg):
        try:
            self.steering_cmd = msg.values[msg.motor_names.index('steering_angle')]
        except (ValueError, IndexError):
            pass

    def on_path(self, msg):
        self.latest_path = [p.pose.position for p in msg.poses]
        self.latest_path_frame = msg.header.frame_id or 'map'
        self.draw_path(self.latest_path, self.latest_path_frame)

    def redraw_path(self):
        """Redraw the stored plan, trimmed to what is still ahead of the car.

        The controller deliberately keeps steering back to the ORIGINAL line
        rather than re-routing from wherever it has drifted to -- that is the
        point of replanning only when the path is blocked. But drawing the
        whole original plan, including the part already driven, made it look
        like the car was ignoring a stale line. Trimming at the nearest point
        shows the remaining route, so what is on screen matches where the car
        is actually still trying to go.
        """
        if not self.latest_path:
            return
        self.draw_path(self.latest_path, self.latest_path_frame)

    def car_position(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return None
        return tf.transform.translation

    def trim_to_ahead(self, points, frame):
        """Drop the portion of the plan the car has already driven past."""
        car = self.car_position(frame)
        if car is None or len(points) < 2:
            return points
        nearest = min(
            range(len(points)),
            key=lambda i: (points[i].x - car.x) ** 2 + (points[i].y - car.y) ** 2)
        # Always leave at least a short stub so the line still reads as a path.
        remaining = points[nearest:]
        return remaining if len(remaining) >= 2 else points[-2:]

    def draw_path(self, all_points, frame):
        markers = MarkerArray()
        # Clear the previous set first: a new plan is usually shorter or longer
        # than the last one, and leftover arrows would linger forever.
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        points = self.trim_to_ahead(all_points, frame)

        if len(points) >= 2:
            markers.markers.extend(self.direction_arrows(points, frame))
            markers.markers.append(self.goal_flag(points[-1], frame))
            markers.markers.append(
                self.distance_label(points, frame))

        self.path_publisher.publish(markers)

    def direction_arrows(self, points, frame):
        """One arrow every ARROW_SPACING_M along the polyline."""
        arrows = []
        travelled = 0.0
        next_arrow_at = ARROW_SPACING_M

        for previous, current in zip(points, points[1:]):
            segment = math.hypot(current.x - previous.x, current.y - previous.y)
            if segment <= 1e-6:
                continue
            heading = math.atan2(current.y - previous.y, current.x - previous.x)

            # A single segment can span several arrow slots on a straight run.
            while travelled + segment >= next_arrow_at:
                ratio = (next_arrow_at - travelled) / segment
                arrows.append(self.arrow_marker(
                    marker_id=len(arrows),
                    frame=frame,
                    x=previous.x + (current.x - previous.x) * ratio,
                    y=previous.y + (current.y - previous.y) * ratio,
                    yaw=heading,
                ))
                next_arrow_at += ARROW_SPACING_M

            travelled += segment
        return arrows

    def arrow_marker(self, marker_id, frame, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = 'plan_direction'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        # Float just above the path ribbon so the arrows are never z-fought
        # into the costmap or the map image underneath.
        marker.pose.position.z = 0.05
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = yaw_to_quaternion(yaw)
        marker.scale.x = ARROW_LENGTH_M   # length
        marker.scale.y = 0.055            # shaft width
        marker.scale.z = 0.055            # head width
        marker.color = PALE_GREEN
        return marker

    def goal_flag(self, goal, frame):
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = goal.x
        marker.pose.position.y = goal.y
        marker.pose.position.z = 0.02
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.34
        marker.scale.z = 0.04
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.53, a=0.55)
        return marker

    def distance_label(self, points, frame):
        total = sum(
            math.hypot(b.x - a.x, b.y - a.y)
            for a, b in zip(points, points[1:]))
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = 'goal'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = points[-1].x
        marker.pose.position.y = points[-1].y
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.17
        marker.color = PALE_GREEN
        marker.text = f'GOAL  {total:.1f} m'
        return marker

    # ---------------------------------------------------------------- motion

    def on_odom(self, msg):
        self.latest_odom = msg

    def publish_motion(self):
        if self.latest_odom is None:
            return

        twist = self.latest_odom.twist.twist
        speed = twist.linear.x
        markers = MarkerArray()

        # Drawn in base_link, so the arrow rides with the car and always points
        # along its own forward axis regardless of where it is on the map.
        arrow = Marker()
        arrow.header.frame_id = 'base_link'
        arrow.ns = 'motion'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.z = 0.42
        moving_backward = speed < -MOVING_SPEED_MPS
        # Flip the arrow instead of using a negative scale, which RViz rejects.
        yaw = math.pi if moving_backward else 0.0
        (arrow.pose.orientation.x, arrow.pose.orientation.y,
         arrow.pose.orientation.z, arrow.pose.orientation.w) = yaw_to_quaternion(yaw)
        # Grow the arrow with speed so it reads as a speedometer at a glance.
        arrow.scale.x = 0.24 + min(abs(speed), 0.6) * 0.9
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color = RED if moving_backward else GREEN
        arrow.color.a = 0.95 if abs(speed) > MOVING_SPEED_MPS else 0.25
        markers.markers.append(arrow)

        label = Marker()
        label.header.frame_id = 'base_link'
        label.ns = 'motion'
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = 0.64
        label.pose.orientation.w = 1.0
        # RViz text markers have no bold weight, so size is the only way to
        # make this readable at a glance while driving.
        label.scale.z = 0.20
        if moving_backward:
            label.text = f'REVERSE  {abs(speed):.2f} m/s'
            label.color = RED
        elif speed > MOVING_SPEED_MPS:
            label.text = f'{speed:.2f} m/s'
            label.color = GREEN
        else:
            label.text = 'STOPPED'
            label.color = AMBER
        # Second line: what the wheels are actually being told to do, AFTER the
        # learned bias/gain correction. On this car the commanded angle and the
        # angle the wheels reach are not the same thing, so seeing the real
        # command is the only way to tell "it chose not to steer" apart from
        # "it steered and the car did not respond".
        label.text += '\n' + self.steering_text()
        markers.markers.append(label)

        banner = Marker()
        banner.header.frame_id = 'base_link'
        banner.ns = 'motion'
        banner.id = 2
        banner.type = Marker.TEXT_VIEW_FACING
        banner.pose.position.z = 1.02
        banner.pose.orientation.w = 1.0
        banner.scale.z = 0.26
        if (self.banner is not None and
                self.get_clock().now().nanoseconds < self.banner_until):
            text, colour = self.banner
            banner.action = Marker.ADD
            banner.text = text
            banner.color.r, banner.color.g = colour.r, colour.g
            banner.color.b, banner.color.a = colour.b, 1.0
        else:
            banner.action = Marker.DELETE
        markers.markers.append(banner)

        self.motion_publisher.publish(markers)


def main():
    rclpy.init()
    node = QCar2NavVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
