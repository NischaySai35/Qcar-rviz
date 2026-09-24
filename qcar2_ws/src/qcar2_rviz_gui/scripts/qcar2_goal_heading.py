#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Aim each clicked goal along the direction the car will actually approach it.

Smac Hybrid-A* plans in (x, y, heading): it does not just reach the goal
POSITION, it arrives at the goal ORIENTATION.  RViz's "2D Goal Pose" tool
sets that orientation from however far the operator happened to drag the
mouse, and with forward-only (Dubin) primitives the only way to arrive at a
heading that disagrees with the approach direction is to swing out and loop
back.  That is why a goal sitting on a plainly straight run still produced a
circle at the end.  The previous NavFn planner never showed this because it
is holonomic and ignores goal orientation entirely -- but it also returned
paths this Ackermann chassis could not drive, which is what made goals
behind the car fail in the first place.

So rather than give up feasible planning, this drops the part of the click
that carries no real intent: the goal keeps its position, and its heading is
chosen to match how the car should actually arrive.

  * Goal ahead (within 90 deg of the car's heading) -> aim the goal along the
    bearing to it.  Reeds-Shepp then returns a straight run or a gentle arc.
  * Goal behind -> keep the car's CURRENT heading as the goal heading.  The
    car does not need to face the goal to reach it: holding its heading and
    planning to a pose behind it is precisely what makes Reeds-Shepp choose a
    straight reverse instead of swinging around nose-first.

Wired in by remapping RViz's goal tool to /goal_pose_raw; this node is then
the only publisher of the /goal_pose that bt_navigator consumes.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class QCar2GoalHeading(Node):
    def __init__(self):
        super().__init__('qcar2_goal_heading')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.create_subscription(PoseStamped, '/goal_pose_raw', self.on_goal, 1)
        self.get_logger().info(
            'Goal heading rewriter active: clicked goals are re-aimed along '
            'the approach bearing (drag direction is ignored).')

    def on_goal(self, goal):
        frame = goal.header.frame_id or 'map'
        try:
            transform = self.tf_buffer.lookup_transform(
                frame, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except Exception as error:
            # Without the car's pose there is no bearing to compute. Passing
            # the goal through unchanged is the safe fallback: it is exactly
            # the behaviour that existed before this node.
            self.get_logger().warn(
                f'No {frame}->base_link transform ({error}); '
                'forwarding goal with its original heading.')
            self.publisher.publish(goal)
            return

        dx = goal.pose.position.x - transform.transform.translation.x
        dy = goal.pose.position.y - transform.transform.translation.y
        # A goal clicked essentially on top of the car has no meaningful
        # bearing; keep whatever heading was drawn.
        if math.hypot(dx, dy) < 0.05:
            self.publisher.publish(goal)
            return

        bearing = math.atan2(dy, dx)
        car_yaw = self._yaw_of(transform.transform.rotation)
        # Signed smallest angle between where the car points and where the
        # goal lies.
        offset = math.atan2(math.sin(bearing - car_yaw),
                            math.cos(bearing - car_yaw))

        if abs(offset) <= math.pi / 2.0:
            yaw = bearing          # ahead-ish: drive forward at it
            manoeuvre = 'forward'
        else:
            yaw = car_yaw          # behind: hold heading and reverse to it
            manoeuvre = 'reverse'

        aimed = PoseStamped()
        aimed.header = goal.header
        aimed.pose.position = goal.pose.position
        aimed.pose.orientation.z = math.sin(yaw * 0.5)
        aimed.pose.orientation.w = math.cos(yaw * 0.5)
        self.publisher.publish(aimed)
        self.get_logger().info(
            f'Goal is {math.degrees(offset):+.0f} deg off the nose -> '
            f'{manoeuvre}; goal heading set to {math.degrees(yaw):.0f} deg.')

    @staticmethod
    def _yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = QCar2GoalHeading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
