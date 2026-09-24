#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Spoken status announcements through the QCar2's onboard speaker.

The car has a 32 mm speaker on the Quanser carrier board, wired to the
Orin's I2S1 and exposed as the PulseAudio sink
`alsa_output.platform-sound.analog-stereo`.  It shipped muted (sink volume
0%), which is why it looked like there was no speaker at all.  Speech is
synthesised by speech-dispatcher's espeak-ng backend via `spd-say`, both
already installed on the car.

What it says:
  * on start: "Mapping started" / "Navigation started" (per `mode`)
  * any LiDAR return closer than `obstacle_distance` (default 2 m):
    "Obstacle ahead, 1.4 metres" -- sector is front / left / right / behind,
    computed in base_link so it does not depend on how the LiDAR is mounted.
    Rate-limited so a person walking alongside does not produce a stream.
  * navigation only: "Going to goal", "Goal reached", "Navigation failed",
    "Goal cancelled", from the navigate_to_pose action status.

Speech runs on its own worker thread through a queue so a slow `spd-say`
can never stall a ROS callback, and consecutive identical messages collapse.
"""

import math
import queue
import subprocess
import threading

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32
from tf2_ros import Buffer, TransformListener

PULSE_SINK = 'alsa_output.platform-sound.analog-stereo'


class Speaker:
    """Serialised, non-blocking text-to-speech."""

    def __init__(self, logger, rate, voice, pitch, volume_percent, enabled):
        self.logger = logger
        self.rate = rate
        # speech-dispatcher voice type: male1..3 / female1..3 / child_*.
        # espeak-ng renders these as pitch/timbre variants of its voice, and
        # on its own the female3 variant read as ambiguous/male-ish on this
        # hardware -- a +50 pitch boost (spd-say -p, range -100..100) on top
        # of it is what actually reads as clearly female; confirmed by ear.
        self.voice = voice
        self.pitch = pitch
        # Starts muted by default (the panel's VOICE button matches this):
        # the operator wants to opt in, not be greeted before they are ready.
        # The PulseAudio sink itself is still unmuted/volumed below so the
        # hardware is immediately ready the moment they do opt in.
        self.enabled = enabled
        self.queue = queue.Queue()
        self.last_text = None
        threading.Thread(target=self._worker, daemon=True).start()
        self._pactl(['set-sink-mute', PULSE_SINK, '0'])
        self._pactl(['set-default-sink', PULSE_SINK])
        self.set_volume(volume_percent)

    @staticmethod
    def _pactl(args):
        # Best effort: if PulseAudio is not up yet the announcements simply
        # go nowhere, the node keeps running, and the next call retries.
        try:
            subprocess.run(['pactl', *args], check=False, timeout=3,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def set_volume(self, percent):
        percent = max(0, min(int(percent), 100))
        # The sink ships muted at 0%, so the volume is owned here, not left to
        # whatever the desktop last set.
        self._pactl(['set-sink-volume', PULSE_SINK, f'{percent}%'])

    def say(self, text):
        if not self.enabled:
            return
        # Collapse duplicates already waiting, but always let a new sentence
        # through even if it matches the last SPOKEN one (e.g. a second goal
        # reached).
        if not self.queue.empty() and self.last_text == text:
            return
        self.last_text = text
        self.queue.put(text)

    def _worker(self):
        while True:
            text = self.queue.get()
            if not self.enabled:
                continue
            self.logger.info(f'SAY: {text}')
            try:
                subprocess.run(
                    ['spd-say', '-w', '-r', str(self.rate), '-p', str(self.pitch),
                     '-t', self.voice, text],
                    check=False, timeout=20,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except (OSError, subprocess.TimeoutExpired) as error:
                self.logger.warn(f'speech failed: {error}')


class Announcer(Node):
    def __init__(self):
        super().__init__('qcar2_announcer')
        mode = self.declare_parameter('mode', 'navigation').value
        self.obstacle_distance = float(
            self.declare_parameter('obstacle_distance', 2.0).value)
        self.obstacle_cooldown = float(
            self.declare_parameter('obstacle_cooldown', 4.0).value)
        rate = int(self.declare_parameter('speech_rate', -20).value)
        voice = self.declare_parameter('voice', 'female3').value
        pitch = int(self.declare_parameter('voice_pitch', 50).value)
        volume = int(self.declare_parameter('volume_percent', 85).value)
        default_enabled = bool(self.declare_parameter('voice_enabled_default', False).value)
        self.speaker = Speaker(self.get_logger(), rate, voice, pitch, volume, default_enabled)

        # Voice on/off and volume from the in-RViz control panel
        # (qcar2_rviz_panels).  transient_local so a panel that published
        # before this node started still gets its setting applied.
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/qcar2/voice_enabled', self.on_voice_enabled, latched)
        self.create_subscription(Int32, '/qcar2/voice_volume', self.on_voice_volume, latched)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.last_obstacle_time = 0.0
        self.last_sector = None
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        if mode == 'navigation':
            self.last_status = None
            # bt_navigator publishes status transient_local; matching it means
            # a status already in flight when we start is still received.
            self.create_subscription(
                GoalStatusArray, '/navigate_to_pose/_action/status',
                self.on_nav_status,
                QoSProfile(depth=1,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # Give the rest of the launch a moment to come up so the greeting is
        # not spoken over a still-initialising audio server.  Voice starts
        # muted by default, so this rarely fires while anyone can hear it --
        # instead the FIRST time the operator clicks VOICE ON, that click is
        # what triggers "Mapping started"/"Navigation started", however long
        # after launch that turns out to be.  See _try_announce_start().
        self.startup_text = 'Mapping started' if mode == 'mapping' else 'Navigation started'
        self.startup_ready = False
        self.startup_spoken = False
        self.startup_timer = self.create_timer(3.0, self.announce_start)

    def announce_start(self):
        self.startup_timer.cancel()
        self.startup_ready = True
        self._try_announce_start()

    def _try_announce_start(self):
        if self.startup_spoken or not self.startup_ready or not self.speaker.enabled:
            return
        self.speaker.say(self.startup_text)
        self.startup_spoken = True

    def on_voice_enabled(self, msg):
        if msg.data != self.speaker.enabled:
            self.speaker.enabled = msg.data
            self.get_logger().info(f'voice {"ON" if msg.data else "OFF"}')
            if msg.data:
                if self.startup_spoken:
                    self.speaker.say('Voice on')
                else:
                    self._try_announce_start()

    def on_voice_volume(self, msg):
        self.speaker.set_volume(msg.data)

    # ------------------------------------------------------------- obstacles

    def on_scan(self, scan):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_obstacle_time < self.obstacle_cooldown:
            return

        nearest = None
        for index, distance in enumerate(scan.ranges):
            if (math.isfinite(distance) and scan.range_min < distance < scan.range_max
                    and distance < self.obstacle_distance
                    and (nearest is None or distance < nearest[0])):
                nearest = (distance, scan.angle_min + index * scan.angle_increment)
        if nearest is None:
            self.last_sector = None
            return

        distance, angle = nearest
        # Bearing in base_link, so "ahead" means the car's nose regardless of
        # the LiDAR's own mounting yaw.  Fall back to the raw scan angle if TF
        # is not available yet.
        bearing = angle
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', scan.header.frame_id, rclpy.time.Time())
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            bearing = math.atan2(math.sin(angle + yaw), math.cos(angle + yaw))
        except Exception:  # noqa: BLE001 - any TF failure just means "use raw angle"
            pass

        degrees = math.degrees(bearing)
        if abs(degrees) <= 45:
            sector = 'ahead'
        elif abs(degrees) >= 135:
            sector = 'behind'
        elif degrees > 0:
            sector = 'on the left'
        else:
            sector = 'on the right'

        self.last_obstacle_time = now
        self.last_sector = sector
        self.speaker.say(f'Obstacle {sector}, {distance:.1f} metres')

    # ------------------------------------------------------------ navigation

    def on_nav_status(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        if status == self.last_status:
            return
        self.last_status = status
        text = {
            GoalStatus.STATUS_EXECUTING: 'Going to goal',
            GoalStatus.STATUS_SUCCEEDED: 'Goal reached',
            GoalStatus.STATUS_ABORTED: 'Navigation failed',
            GoalStatus.STATUS_CANCELED: 'Goal cancelled',
        }.get(status)
        if text:
            self.speaker.say(text)


def main():
    rclpy.init()
    node = Announcer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
