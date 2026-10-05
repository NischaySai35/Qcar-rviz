#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Publish independent QCar2 wheel-encoder + gyro odometry.

The hardware node already exposes motor tachometer counts on /qcar2_joint
and raw angular velocity on /qcar2_imu, but does not publish nav_msgs/Odometry.
This node turns those measurements into a short-term Ackermann dead-reckoning
prior.

By default it does not publish TF: during mapping, Cartographer owns
map -> odom (and odom -> base_link) itself. Navigation mode has no other
odom -> base_link source, so it starts this node with publish_tf:=true --
see odometry.launch.py. This replaced rf2o_laser_odometry (pure LiDAR
scan-matching), which has a known upstream bug where the estimated
direction of travel can come out inverted (map->odom fine, but the
odom delta itself backwards) -- see MAPIRlab/rf2o_laser_odometry#20.
That made the car look like it was driving backward in RViz while the
real car (driven directly off /cmd_vel, independent of this odometry)
moved forward correctly, and made Nav2's goal checker never see the
goal as reached since its distance estimate was diverging.
"""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from qcar2_interfaces.msg import MotorCommands
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster


WHEEL_RADIUS_M = 0.033
WHEELBASE_M = 0.256
# QCar2 motor encoder -> wheel linear-speed conversion. Keep this identical
# to qcar2_hardware.cpp's speed controller.
COUNTS_TO_MPS = ((13.0 * 19.0) / (70.0 * 30.0)) * (2.0 * math.pi * WHEEL_RADIUS_M) / (720.0 * 4.0)


class WheelImuOdometry(Node):
    def __init__(self):
        super().__init__('wheel_imu_odometry')
        self.publish_tf = self.declare_parameter('publish_tf', False).value
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.publisher = self.create_publisher(Odometry, '/wheel_imu_odom', 20)
        self.create_subscription(JointState, '/qcar2_joint', self._joint, 20)
        self.create_subscription(Imu, '/qcar2_imu', self._imu, 50)
        self.create_subscription(MotorCommands, '/qcar2_motor_speed_cmd', self._command, 20)

        self.counts_per_second = None
        self.encoder_count = None
        self.previous_encoder_count = None
        self.speed_sign = 1.0
        self.gyro_z = None
        self.steering = 0.0
        self.gyro_bias_sum = 0.0
        self.gyro_bias_samples = 0
        self.gyro_bias = 0.0
        self.bias_ready = False
        self.x = self.y = self.yaw = 0.0
        self.last_tick = time.monotonic()
        self.create_timer(0.02, self._publish)

    def _joint(self, message):
        if message.velocity:
            self.counts_per_second = message.velocity[0]
        # Quadrature encoder count. Unlike the motor-speed channel this is
        # unambiguously SIGNED, so it is the reliable source of travel
        # direction -- see _signed_speed().
        if message.position:
            self.encoder_count = message.position[0]

    def _imu(self, message):
        self.gyro_z = message.angular_velocity.z

    def _command(self, message):
        try:
            self.steering = message.values[message.motor_names.index('steering_angle')]
        except (ValueError, IndexError):
            pass

    def _signed_speed(self, dt):
        """Wheel speed in m/s, with the sign taken from the encoder.

        qcar2_hardware publishes velocity[0] from Quanser channel 14000
        ("Motor Speed"), whose magnitude is well filtered but which cannot be
        relied on to carry the direction of travel.  That never mattered while
        the car was forward-only, but reverse is now a normal manoeuvre: an
        unsigned speed integrates the pose FORWARD while the car actually
        backs up, which corrupts odometry, the controller's feedback, and the
        steering-bias learner in nav2_qcar_command_convert.cpp all at once.

        position[0] is the quadrature encoder count and is properly signed, so
        its delta gives direction unambiguously.  Magnitude still comes from
        the filtered speed channel; only the sign is taken from the encoder,
        and the last known sign is held through the noise around standstill.
        """
        magnitude = abs(self.counts_per_second * COUNTS_TO_MPS)

        if self.encoder_count is not None and dt > 0.0:
            if self.previous_encoder_count is not None:
                delta = self.encoder_count - self.previous_encoder_count
                # Ignore encoder dither while stationary; hold the last sign.
                if abs(delta * COUNTS_TO_MPS / dt) > 0.01:
                    self.speed_sign = 1.0 if delta >= 0.0 else -1.0
            self.previous_encoder_count = self.encoder_count

        return magnitude * self.speed_sign

    def _publish(self):
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now
        if self.counts_per_second is None:
            return

        linear_speed = self._signed_speed(dt)
        # Estimate gyro zero bias only while the car is stationary during
        # startup. Cartographer starts after this node, giving this a window
        # to settle before fused mapping begins.
        if (not self.bias_ready and self.gyro_z is not None and
                abs(linear_speed) < 0.01):
            self.gyro_bias_sum += self.gyro_z
            self.gyro_bias_samples += 1
            if self.gyro_bias_samples >= 50:
                self.gyro_bias = self.gyro_bias_sum / self.gyro_bias_samples
                self.bias_ready = True
                self.get_logger().info('Gyro bias calibrated for wheel/IMU odometry')

        # Gyro supplies heading independently of the LiDAR. If it is absent,
        # retain a physically plausible Ackermann fallback rather than stop
        # publishing and block Cartographer's sensor collator.
        if self.gyro_z is not None and self.bias_ready:
            yaw_rate = self.gyro_z - self.gyro_bias
        else:
            yaw_rate = linear_speed * math.tan(self.steering) / WHEELBASE_M

        mid_yaw = self.yaw + 0.5 * yaw_rate * dt
        self.x += linear_speed * math.cos(mid_yaw) * dt
        self.y += linear_speed * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + yaw_rate * dt),
                              math.cos(self.yaw + yaw_rate * dt))

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        message.twist.twist.linear.x = linear_speed
        message.twist.twist.angular.z = yaw_rate
        # This is a local prior, not global truth. Cartographer remains free
        # to correct it when scan geometry is good.
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.08
        message.twist.covariance[0] = 0.02
        message.twist.covariance[35] = 0.04
        self.publisher.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = message.header.stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = WheelImuOdometry()
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
