#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Publish the car's physical sensor data as CAN frames on vcan0.

The OTA / cloud integration reads vehicle data the way it would on a real
car: from a SocketCAN interface. The QCar2 has no CAN bus of its own (all
I/O goes through the Quanser HIL board), so this node is that bus: it
subscribes to the sensor topics below and writes fixed-layout classic CAN
frames to `can_interface` (default vcan0). The interface itself is created
by scripts/setup_vcan.sh.

Only physical sensor data is sent -- no navigation, safety or status
information.

All four frames are sent together once per `rate_hz` (default 1 Hz), each
carrying the latest value of its sensor.

Frame layout (all standard 11-bit IDs, little-endian/Intel, signed int16
unless noted). The authoritative copy for decoding is config/qcar2_vcan.dbc
-- keep the two in step.

  ID     name          signals (x scale)
  0x100  QCAR_Motion   speed m/s x0.001 | yaw rate rad/s x0.001 |
                       steering angle rad x0.0001
  0x110  QCAR_Odom     x, y m x0.01 | yaw rad x0.0001
  0x120  QCAR_IMU      accel x/y/z m/s^2 x0.01 | gyro z rad/s x0.001
  0x130  QCAR_Battery  uint16 voltage V x0.01

Speed, yaw rate and odometry come from wheel encoder + IMU odometry:
/wheel_imu_odom while mapping, the same data renamed /odom in navigation.
The car has no steering sensor, so the steering angle is the one sent to
the servo (/qcar2_motor_speed_cmd); -32768 raw = no recent command.

A frame is only sent while its source is fresh (< `stale_sec`), so a dead
sensor shows up as missing frames instead of a frozen value.

If vcan0 does not exist (setup_vcan.sh not run yet) the node warns and
keeps retrying; it never affects driving -- it only subscribes.
"""

import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from qcar2_interfaces.msg import MotorCommands

ID_MOTION = 0x100
ID_ODOM = 0x110
ID_IMU = 0x120
ID_BATTERY = 0x130

CAN_FRAME = struct.Struct('=IB3x8s')   # struct can_frame: id, dlc, pad, data
INT16_NA = -32768                      # "not available" for int16 signals


def i16(value, scale):
    """Scale to a saturated int16; None/NaN/inf -> INT16_NA."""
    if value is None or not math.isfinite(value):
        return INT16_NA
    return max(-32767, min(32767, int(round(value / scale))))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CanBridge(Node):
    def __init__(self):
        super().__init__('qcar2_can_bridge')
        self.iface = self.declare_parameter('can_interface', 'vcan0').value
        self.stale_sec = float(self.declare_parameter('stale_sec', 1.0).value)
        rate_hz = float(self.declare_parameter('rate_hz', 1.0).value)

        self.sock = None
        self.last_open_try = 0.0
        self.warned_missing = False

        self.odom = None
        self.odom_t = 0.0
        self.steering = None
        self.steering_t = 0.0
        self.imu = None
        self.imu_t = 0.0
        self.battery_v = None
        self.battery_t = 0.0

        # /wheel_imu_odom in mapping, remapped to /odom in navigation
        self.create_subscription(Odometry, '/wheel_imu_odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(MotorCommands, '/qcar2_motor_speed_cmd', self.on_cmd, 10)
        self.create_subscription(Imu, '/qcar2_imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/qcar2_battery', self.on_battery,
                                 qos_profile_sensor_data)

        self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(f'CAN sensor bridge -> {self.iface} at {rate_hz:g} Hz')

    # ---- subscriptions -------------------------------------------------
    def on_odom(self, msg):
        self.odom, self.odom_t = msg, time.monotonic()

    def on_cmd(self, msg):
        for name, value in zip(msg.motor_names, msg.values):
            if name == 'steering_angle':
                self.steering, self.steering_t = value, time.monotonic()

    def on_imu(self, msg):
        self.imu, self.imu_t = msg, time.monotonic()

    def on_battery(self, msg):
        self.battery_v, self.battery_t = msg.voltage, time.monotonic()

    # ---- CAN output ----------------------------------------------------
    def open_socket(self):
        now = time.monotonic()
        if now - self.last_open_try < 2.0:
            return False
        self.last_open_try = now
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.setblocking(False)
            sock.bind((self.iface,))
        except OSError as exc:
            if not self.warned_missing:
                self.get_logger().warn(
                    f'{self.iface} unavailable ({exc}); run `sudo scripts/setup_vcan.sh`. '
                    'Retrying every 2 s.')
                self.warned_missing = True
            return False
        self.sock = sock
        self.warned_missing = False
        self.get_logger().info(f'Writing sensor data to {self.iface}')
        return True

    def send(self, can_id, data):
        if self.sock is None:
            return
        try:
            self.sock.send(CAN_FRAME.pack(can_id, len(data), data.ljust(8, b'\0')))
        except BlockingIOError:
            pass   # TX queue full (nobody is draining a real bus); drop this frame.
        except OSError as exc:
            # Interface deleted or down: drop the socket and reopen later.
            self.get_logger().warn(f'{self.iface} send failed ({exc}); reopening')
            self.sock.close()
            self.sock = None

    def fresh(self, stamp):
        return time.monotonic() - stamp < self.stale_sec

    def on_timer(self):
        if self.sock is None and not self.open_socket():
            return
        if self.odom is not None and self.fresh(self.odom_t):
            tw = self.odom.twist.twist
            steering = self.steering if self.fresh(self.steering_t) else None
            self.send(ID_MOTION, struct.pack(
                '<hhh', i16(tw.linear.x, 0.001), i16(tw.angular.z, 0.001),
                i16(steering, 0.0001)))
            p = self.odom.pose.pose
            self.send(ID_ODOM, struct.pack(
                '<hhh', i16(p.position.x, 0.01), i16(p.position.y, 0.01),
                i16(yaw_of(p.orientation), 0.0001)))

        if self.imu is not None and self.fresh(self.imu_t):
            a, g = self.imu.linear_acceleration, self.imu.angular_velocity
            self.send(ID_IMU, struct.pack(
                '<hhhh', i16(a.x, 0.01), i16(a.y, 0.01), i16(a.z, 0.01), i16(g.z, 0.001)))

        if self.battery_v is not None and self.fresh(self.battery_t):
            v = self.battery_v
            self.send(ID_BATTERY, struct.pack('<H', max(0, min(65535, int(round(v / 0.01))))))

    def destroy_node(self):
        if self.sock is not None:
            self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CanBridge()
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
