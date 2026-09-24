#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Supply the QCar2 URDF joints, including live front-wheel steering."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from qcar2_interfaces.msg import MotorCommands


class QCar2ModelJointState(Node):
    def __init__(self):
        super().__init__('qcar2_model_joint_state')
        self.steering = 0.0
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(MotorCommands, '/qcar2_motor_speed_cmd', self.command, 10)
        self.create_timer(0.05, self.publish)

    def command(self, message):
        for name, value in zip(message.motor_names, message.values):
            if name == 'steering_angle':
                self.steering = max(-0.5236, min(0.5236, value))

    def publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [
            'hub_frontLeft_joint', 'hub_frontRight_joint',
            'wheel_frontLeft_joint', 'wheel_frontRight_joint',
            'wheel_rearLeft_joint', 'wheel_rearRight_joint',
        ]
        message.position = [self.steering, self.steering, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = QCar2ModelJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
