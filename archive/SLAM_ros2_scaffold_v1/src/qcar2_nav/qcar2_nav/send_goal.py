#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set initial pose if needed (not strictly required for SLAM Toolbox usually)
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    # navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to be fully active
    navigator.waitUntilNav2Active(localizer='slam_toolbox')

    # Define target goal
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0  # Set your target X coordinate
    goal_pose.pose.position.y = 0.5  # Set your target Y coordinate
    goal_pose.pose.orientation.w = 1.0

    print('Navigating to goal: ', goal_pose.pose.position.x, goal_pose.pose.position.y)
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')

            # Some basic error handling / timeout
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
