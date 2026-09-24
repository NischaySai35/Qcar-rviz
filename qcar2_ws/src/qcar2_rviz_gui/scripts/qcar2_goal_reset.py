#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Make "2D Pose Estimate" actually reset navigation.

Nav2 treats re-localising and navigating as unrelated: publishing a new
initial pose on /initialpose corrects AMCL, but it does NOT touch an
in-flight NavigateToPose goal.  The practical effect on this car was that
after fixing a bad pose estimate:

  * bt_navigator kept driving toward the OLD goal, now re-projected onto
    the corrected map -- i.e. at a different physical place than intended;
  * the previous green plan stayed frozen on screen, because planner_server
    had stopped publishing /plan and RViz simply keeps the last message; and
  * both costmaps still held obstacles that had been marked while the pose
    was wrong.  Those marks sit at map coordinates the LiDAR now cannot
    raytrace through, so nothing ever clears them -- they are the "old
    boundaries" that survived re-localising.

This node ties the three together: a new initial pose cancels the active
goal, clears both costmaps, and tells the visualiser to drop its markers.
"""
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Empty


class QCar2GoalReset(Node):
    # AMCL does not apply a new pose the instant /initialpose arrives: it
    # resamples its particle filter and only then republishes map->odom.
    # Clearing the costmaps straight from the /initialpose callback therefore
    # wipes them a beat BEFORE the transform actually jumps, and the obstacle
    # layer re-marks whatever scans land in that gap using the OLD transform.
    # That is why a stale boundary reappeared at the pre-reset position even
    # though this node had already cleared once.  Re-clearing after the jump
    # has settled removes those.  Clearing is cheap and safe to repeat: the
    # static layer repopulates instantly and real obstacles are re-marked
    # from the very next scan.
    RECLEAR_DELAYS = (0.6, 1.5)

    def __init__(self):
        super().__init__('qcar2_goal_reset')
        self._reclear_timers = []

        # RViz publishes /initialpose with plain volatile/reliable defaults.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, qos)

        # Cancelling through the action server's own CancelGoal service rather
        # than an ActionClient: this node never sent the goal, so it holds no
        # goal handle to cancel.  An empty goal_id means "cancel everything".
        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # The visualiser owns /qcar2/path_markers; publishing a reset flag is
        # cleaner than adding a second publisher on /plan, which would leave
        # two nodes writing the same topic.
        self.reset_publisher = self.create_publisher(Empty, '/qcar2/nav_reset', 1)

        self.get_logger().info(
            'Goal reset armed: a new 2D Pose Estimate will cancel the active '
            'goal and clear both costmaps.')

    def on_initial_pose(self, _message):
        self.get_logger().info('New initial pose received -- resetting navigation.')

        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())
        else:
            self.get_logger().warn(
                'navigate_to_pose cancel service not available; goal left running.')

        self.clear_costmaps()
        self.reset_publisher.publish(Empty())
        self.schedule_reclears()

    def clear_costmaps(self):
        for client, label in ((self.clear_global, 'global'), (self.clear_local, 'local')):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warn(f'{label} costmap clear service not available.')

    def schedule_reclears(self):
        # A second Set Pose while re-clears are still pending restarts the
        # sequence rather than stacking two of them on top of each other.
        for timer in self._reclear_timers:
            timer.cancel()
            self.destroy_timer(timer)
        self._reclear_timers = [self.one_shot(delay, self.clear_costmaps)
                                for delay in self.RECLEAR_DELAYS]

    def one_shot(self, delay, action):
        """rclpy timers repeat; cancel on first fire to get a one-shot.  The
        timer is destroyed by the next schedule_reclears(), not from inside
        its own callback."""
        holder = {}

        def fire():
            holder['timer'].cancel()
            action()

        holder['timer'] = self.create_timer(delay, fire)
        return holder['timer']


def main():
    rclpy.init()
    node = QCar2GoalReset()
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
