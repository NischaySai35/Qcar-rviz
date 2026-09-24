#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Wait for Nav2 to be fully active
    navigator.waitUntilNav2Active(localizer='slam_toolbox')

    # Define waypoints
    waypoints = []
    
    # Waypoint 1
    wp1 = PoseStamped()
    wp1.header.frame_id = 'map'
    wp1.header.stamp = navigator.get_clock().now().to_msg()
    wp1.pose.position.x = 1.0
    wp1.pose.position.y = 0.0
    wp1.pose.orientation.w = 1.0
    waypoints.append(wp1)

    # Waypoint 2
    wp2 = PoseStamped()
    wp2.header.frame_id = 'map'
    wp2.header.stamp = navigator.get_clock().now().to_msg()
    wp2.pose.position.x = 2.0
    wp2.pose.position.y = 1.0
    wp2.pose.orientation.w = 1.0
    waypoints.append(wp2)

    # Waypoint 3 (Back to near start)
    wp3 = PoseStamped()
    wp3.header.frame_id = 'map'
    wp3.header.stamp = navigator.get_clock().now().to_msg()
    wp3.pose.position.x = 0.5
    wp3.pose.position.y = 0.5
    wp3.pose.orientation.w = 1.0
    waypoints.append(wp3)

    print('Starting waypoint following...')
    navigator.followWaypoints(waypoints)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing waypoint: ' + str(feedback.current_waypoint + 1) + '/' + str(len(waypoints)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Waypoints reached successfully!')
    elif result == TaskResult.CANCELED:
        print('Waypoints were canceled!')
    elif result == TaskResult.FAILED:
        print('Waypoints failed!')
    else:
        print('Waypoints returned invalid status!')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
