"""
QCar2's hardware driver does not itself publish nav_msgs/Odometry, so Nav2
(AMCL + controller) has nothing to fuse in navigation mode. This launches
wheel_imu_odometry.py -- the same encoder+gyro dead-reckoning node mapping
mode already uses as Cartographer's sensor-fusion prior -- with TF
publishing turned on, so it also supplies:
  - the /odom topic (nav_msgs/Odometry)
  - the odom -> base_link TF

Previously this used rf2o_laser_odometry (pure LiDAR scan-matching odometry
instead of wheel encoders). Dropped it: it has a known upstream bug where
the estimated direction of travel can come out inverted while the map->odom
transform itself is fine (MAPIRlab/rf2o_laser_odometry#20) -- the car would
visibly drive backward in RViz while the real car (driven directly off
/cmd_vel, independent of this odometry) moved forward correctly, and Nav2's
goal checker would never see the goal as reached since its distance
estimate was diverging in the wrong direction.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    wheel_imu_odometry_node = Node(
        package='qcar2_rviz_gui',
        executable='wheel_imu_odometry.py',
        name='wheel_imu_odometry',
        output='screen',
        parameters=[{'publish_tf': True}],
        remappings=[('/wheel_imu_odom', '/odom')],
    )
    return LaunchDescription([wheel_imu_odometry_node])
