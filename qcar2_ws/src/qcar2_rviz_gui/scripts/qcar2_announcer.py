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

import json
import math
import os
import queue
import time
import subprocess
import threading

# THE SPEAKER-SILENT BUG (2026-10-05). PulseAudio and speech-dispatcher both
# find their sockets through XDG_RUNTIME_DIR (/run/user/<uid>). A launch
# started from an xrdp remote-desktop or SSH/VS Code session does not have
# it set, and then `spd-say` fails ("Can't connect to unix socket
# ~/.cache/speech-dispatcher/speechd.sock ... Autospawn failed") and every
# `pactl` call is refused -- so nothing is ever spoken, the 85 % volume and
# echo cancelling are never applied, and with output discarded it all looked
# like a dead speaker. Fill it in before anything below spawns a child.
if not os.environ.get('XDG_RUNTIME_DIR') and os.path.isdir(f'/run/user/{os.getuid()}'):
    os.environ['XDG_RUNTIME_DIR'] = f'/run/user/{os.getuid()}'

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformListener

PULSE_SINK = 'alsa_output.platform-sound.analog-stereo'

# The car's two microphones: a stereo PDM pair on the Orin's DMIC2 port,
# loaded by /etc/pulse/default.pa as this source. NOT PulseAudio's default
# source -- that is the I2S1 capture side, which has nothing wired to it and
# records pure silence (why the car mic "did not work"; see
# qcar2_voice_command.py's MicFrontEnd for the measurements).
MIC_SOURCE = 'alsa_input.hw_1_1'
# Echo-cancelled pair created by module-echo-cancel. Speech played into
# EC_SINK reaches the speaker AND is handed to the canceller as the
# reference, so it can subtract the car's own voice from EC_SOURCE.
EC_SINK = 'qcar2_spk_ec'
EC_SOURCE = 'qcar2_mic_ec'
# Measured on this car, speaker at 85 %, a recorded human voice as "the
# person" and the announcer's own speech as the echo, through the voice
# node's real capture path (MicFrontEnd):
#   raw mic:        person "one zero zero zero one now know to went on ..."
#                   car    "first i can't see up the television"   <- heard itself
#   echo-cancelled: person "one zero zero zero one yeah no to went on ..."
#                   car    ""                                       <- nothing
# (espeak's "goal reached" even came out as "resume" -- a real command -- in
# the grammar, so hearing itself is not harmless.)
#
# analog_gain_control MUST be off: WebRTC's analog AGC works by turning the
# master source's volume down, and in testing it silently left the mic at
# 33 % (-28.6 dB) even after the module was unloaded.
# noise_suppression MUST be off: with it on, the person came out as "zero
# zero zero one to water zero zero three" -- it treats this mic's quiet
# speech as noise. MicFrontEnd already removes the hum that matters.
EC_ARGS = 'analog_gain_control=0 digital_gain_control=0 noise_suppression=0'


PIPER_DIR = os.path.join(os.path.expanduser('~'), 'Desktop', 'Qcar-rviz', 'models', 'piper')


class Speaker:
    """Serialised, non-blocking text-to-speech."""

    def __init__(self, logger, rate, voice, pitch, volume_percent, enabled,
                 on_speaking=None, echo_cancel=True, on_spoken=None,
                 engine='espeak', piper_voice='', piper_length_scale=1.0):
        self.logger = logger
        # 'espeak' = speech-dispatcher's espeak-ng (robotic, always present);
        # 'piper' = Piper neural TTS (models/piper), much clearer. Piper is
        # used only if its binary and the chosen voice exist, else espeak.
        self.engine = engine
        self.piper_model = os.path.join(PIPER_DIR, 'voices', f'{piper_voice}.onnx')
        # >1 speaks slower; a touch slower is easier to follow on a 32 mm speaker.
        self.piper_length_scale = piper_length_scale
        self.piper_rate = 22050
        if engine == 'piper':
            binary = os.path.join(PIPER_DIR, 'piper', 'piper')
            if os.path.isfile(binary) and os.path.isfile(self.piper_model):
                try:
                    with open(self.piper_model + '.json') as fh:
                        self.piper_rate = int(json.load(fh)['audio']['sample_rate'])
                except (OSError, ValueError, KeyError):
                    pass
                logger.info(f'TTS: Piper voice {piper_voice}')
            else:
                logger.warn(f'TTS: Piper or voice "{piper_voice}" not found in {PIPER_DIR}; '
                            f'using espeak-ng.')
                self.engine = 'espeak'
        # Called with True just before each utterance and False once it has
        # finished, so the microphone can go deaf while the car is talking.
        self.on_speaking = on_speaking or (lambda _on: None)
        # Called with (text, ok) after each utterance, for the console's
        # "Said" log -- so a sentence that was hard to make out can be read.
        self.on_spoken = on_spoken or (lambda _text, _ok: None)
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
        self.ec_module = None
        if not (echo_cancel and self.setup_echo_cancel()):
            self._pactl(['set-default-sink', PULSE_SINK])
        self.set_volume(volume_percent)

    @staticmethod
    def _pactl(args):
        # Best effort: if PulseAudio is not up yet the announcements simply
        # go nowhere, the node keeps running, and the next call retries.
        try:
            return subprocess.run(['pactl', *args], check=False, timeout=3,
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            return ''

    def setup_echo_cancel(self):
        """Route speech through WebRTC echo cancellation. True on success.

        Idempotent: a module left over from an earlier run is reused. The
        default sink is pointed at the cancelling sink because spd-say has no
        output-device option -- speech-dispatcher plays wherever the default
        is -- and any stream already open is moved there too.
        """
        if EC_SINK not in self._pactl(['list', 'short', 'sinks']):
            if MIC_SOURCE not in self._pactl(['list', 'short', 'sources']):
                self.logger.warn(f'echo cancel: mic source {MIC_SOURCE} not found; '
                                 'speaking without it (half-duplex muting still applies)')
                return False
            out = self._pactl(['load-module', 'module-echo-cancel',
                               f'source_master={MIC_SOURCE}', f'sink_master={PULSE_SINK}',
                               f'source_name={EC_SOURCE}', f'sink_name={EC_SINK}',
                               'aec_method=webrtc', 'rate=48000', 'channels=1',
                               f'aec_args="{EC_ARGS}"'])
            if not out.isdigit():
                self.logger.warn('echo cancel: module-echo-cancel failed to load; '
                                 'speaking without it (half-duplex muting still applies)')
                return False
            self.ec_module = out
        # Undo any damage from a run that had WebRTC's analog AGC enabled.
        self._pactl(['set-source-volume', MIC_SOURCE, '100%'])
        self._pactl(['set-default-sink', EC_SINK])
        for line in self._pactl(['list', 'short', 'sink-inputs']).splitlines():
            idx = line.split('\t', 1)[0]
            if idx.isdigit():
                self._pactl(['move-sink-input', idx, EC_SINK])
        self.logger.info(f'echo cancellation on: speech -> {EC_SINK}, clean mic = {EC_SOURCE}')
        return True

    def close(self):
        """Put the audio routing back the way the desktop had it."""
        if self.ec_module:
            self._pactl(['set-default-sink', PULSE_SINK])
            self._pactl(['unload-module', self.ec_module])
            self._pactl(['set-source-volume', MIC_SOURCE, '100%'])
            self.ec_module = None

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
            # Bracket the utterance so qcar2_voice_command can mute the mic
            # for exactly as long as the speaker is talking. `spd-say -w`
            # blocks until playback has finished, which is what makes the
            # False edge accurate rather than a guess at speech duration.
            self.on_speaking(True)
            ok = False
            try:
                ok = self._piper(text) if self.engine == 'piper' else self._espeak(text)
            except (OSError, subprocess.TimeoutExpired) as error:
                self.logger.warn(f'speech failed: {error}')
            finally:
                self.on_speaking(False)
                self.on_spoken(text, ok)

    def _espeak(self, text):
        result = subprocess.run(
            ['spd-say', '-w', '-r', str(self.rate), '-p', str(self.pitch),
             '-t', self.voice, text],
            check=False, timeout=20,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        # Was stderr=DEVNULL, which is how a speaker that could not speak at
        # all stayed invisible -- say why.
        if result.returncode != 0:
            self.logger.warn(f'speech failed (spd-say exit {result.returncode}): '
                             f'{result.stderr.strip()[:200]}')
        return result.returncode == 0

    def _piper(self, text):
        """Piper -> raw 16-bit mono PCM -> paplay (the same PulseAudio default
        sink spd-say plays to, so volume and echo cancelling still apply).
        paplay exits when playback has finished, which keeps the
        on_speaking(False) edge accurate for the microphone mute."""
        synth = subprocess.Popen(
            [os.path.join(PIPER_DIR, 'piper', 'piper'), '--model', self.piper_model,
             '--output_raw', '--length_scale', str(self.piper_length_scale)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        play = subprocess.Popen(
            ['paplay', '--raw', f'--rate={self.piper_rate}', '--format=s16le', '--channels=1'],
            stdin=synth.stdout, stderr=subprocess.PIPE)
        synth.stdout.close()          # paplay owns the pipe now
        synth.stdin.write((text.replace('\n', ' ') + '\n').encode())
        synth.stdin.close()
        try:
            play.wait(timeout=30)
            synth.wait(timeout=5)
        except subprocess.TimeoutExpired:
            synth.kill()
            play.kill()
            raise
        if synth.returncode != 0 or play.returncode != 0:
            self.logger.warn(f'speech failed (piper {synth.returncode}, paplay {play.returncode}): '
                             f'{(synth.stderr.read() + play.stderr.read()).decode()[-200:]}')
            return False
        return True


class Announcer(Node):
    def __init__(self):
        super().__init__('qcar2_announcer')
        mode = self.declare_parameter('mode', 'navigation').value
        self.obstacle_distance = float(
            self.declare_parameter('obstacle_distance', 1.5).value)
        self.obstacle_cooldown = float(
            self.declare_parameter('obstacle_cooldown', 4.0).value)
        rate = int(self.declare_parameter('speech_rate', -20).value)
        voice = self.declare_parameter('voice', 'female3').value
        pitch = int(self.declare_parameter('voice_pitch', 50).value)
        volume = int(self.declare_parameter('volume_percent', 85).value)
        default_enabled = bool(self.declare_parameter('voice_enabled_default', False).value)
        # WebRTC echo cancellation between the speaker and the car mic -- see
        # EC_ARGS above. Half-duplex muting below stays on as well; this is
        # the second layer, for the echo tail and anything the mute misses.
        echo_cancel = bool(self.declare_parameter('echo_cancel', True).value)
        # Half-duplex audio: /qcar2/speaking is True while the car is talking,
        # and qcar2_voice_command drops microphone audio for that window.
        # Without it the mic hears "Going to the air cooler" and may act on
        # its own voice.
        self.speaking_pub = self.create_publisher(Bool, '/qcar2/speaking', 10)
        # Every sentence actually spoken, as JSON {text, ok, t} -- shown in the
        # console's Voice card ("Said") so it can be read if not understood.
        self.spoken_pub = self.create_publisher(String, '/qcar2/spoken', 10)
        # Voice engine. Piper en_US-ryan (US male) was picked by ear from four
        # samples played on the car's speaker (2026-10-05) over espeak-ng,
        # which was hard to understand. Falls back to espeak-ng if Piper or
        # the voice file is missing (tts_engine:=espeak forces the old one).
        engine = self.declare_parameter('tts_engine', 'piper').value
        piper_voice = self.declare_parameter('piper_voice', 'en_US-ryan-medium').value
        length_scale = float(self.declare_parameter('piper_length_scale', 1.1).value)
        self.speaker = Speaker(self.get_logger(), rate, voice, pitch, volume,
                               default_enabled, on_speaking=self.publish_speaking,
                               echo_cancel=echo_cancel, on_spoken=self.publish_spoken,
                               engine=engine, piper_voice=piper_voice,
                               piper_length_scale=length_scale)
        self.mic_on = False

        # Voice on/off and volume from the in-RViz control panel
        # (qcar2_rviz_panels).  transient_local so a panel that published
        # before this node started still gets its setting applied.
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/qcar2/voice_enabled', self.on_voice_enabled, latched)
        self.create_subscription(Int32, '/qcar2/voice_volume', self.on_voice_volume, latched)

        # Arbitrary text from other nodes -- voice-command confirmations
        # ("Going to the air cooler") and explorer progress. Routed through the
        # same Speaker so it honours the VOICE on/off button and volume, and so
        # it queues behind obstacle callouts instead of talking over them.
        self.create_subscription(String, '/qcar2/say', self.on_say, 10)
        # While the microphone is listening, obstacle callouts are suppressed
        # entirely: they are frequent, and every one of them is speech the
        # mic would have to be deaf through. Important messages (started,
        # goal reached, answers to questions) are still spoken.
        self.create_subscription(Bool, '/qcar2/mic_enabled', self.on_mic_enabled, latched)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.last_obstacle_time = 0.0
        self.last_sector = None
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        # Object callouts: "television on the left, 2.3 metres". Each newly
        # CONFIRMED landmark is announced once (the mapper only lists an
        # object after min_hits sightings, so these are not guesses); people
        # are live, so they are re-announced if still present after
        # person_repeat_sec. At most one callout per /qcar2/objects update
        # (1 Hz), closest first, so a room full of new objects does not
        # become a wall of speech.
        self.announce_objects = bool(self.declare_parameter('announce_objects', True).value)
        self.person_repeat = float(self.declare_parameter('person_repeat_sec', 15.0).value)
        self.announced = set()
        self.person_said = {}
        if self.announce_objects:
            self.create_subscription(String, '/qcar2/objects', self.on_objects, latched)

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

    def on_say(self, msg):
        text = (msg.data or '').strip()
        if text:
            self.speaker.say(text)

    # ------------------------------------------------------------- obstacles

    def publish_speaking(self, on):
        msg = Bool()
        msg.data = bool(on)
        self.speaking_pub.publish(msg)

    def publish_spoken(self, text, ok):
        msg = String()
        msg.data = json.dumps({'text': text, 'ok': bool(ok), 't': round(time.time(), 1)})
        self.spoken_pub.publish(msg)

    def on_mic_enabled(self, msg):
        self.mic_on = bool(msg.data)

    def on_scan(self, scan):
        if self.mic_on:
            return          # quiet while listening -- see on_mic_enabled
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

        sector = self.sector(bearing)
        self.last_obstacle_time = now
        self.last_sector = sector
        self.speaker.say(f'Obstacle {sector}, {distance:.1f} metres')

    @staticmethod
    def sector(bearing):
        degrees = math.degrees(bearing)
        if abs(degrees) <= 45:
            return 'ahead'
        if abs(degrees) >= 135:
            return 'behind'
        return 'on the left' if degrees > 0 else 'on the right'

    # --------------------------------------------------------------- objects

    def on_objects(self, msg):
        if self.mic_on or not self.speaker.enabled:
            return
        try:
            data = json.loads(msg.data)
            tf = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
        except Exception:  # noqa: BLE001 - bad JSON or not localised yet
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        now = time.monotonic()

        def relative(obj):
            # map -> base_link: where the object is relative to the car's nose.
            x, y = float(obj['x']), float(obj['y'])
            bx, by = c * x - s * y + t.x, s * x + c * y + t.y
            return math.hypot(bx, by), math.atan2(by, bx)

        candidates = []
        for obj in data.get('objects', []):
            if obj.get('id') not in self.announced:
                candidates.append((obj, False))
        for obj in data.get('transient', []):
            if now - self.person_said.get(obj.get('id'), -1e9) > self.person_repeat:
                candidates.append((obj, True))
        best = None
        for obj, is_person in candidates:
            dist, bearing = relative(obj)
            if best is None or dist < best[1]:
                best = (obj, dist, bearing, is_person)
        if best is None:
            return
        obj, dist, bearing, is_person = best
        if is_person:
            self.person_said[obj.get('id')] = now
            self.speaker.say(f'Person {self.sector(bearing)}, {dist:.1f} metres')
        else:
            self.announced.add(obj.get('id'))
            self.speaker.say(f"{obj.get('label', 'object')} {self.sector(bearing)}, "
                             f'{dist:.1f} metres')

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
    except KeyboardInterrupt:
        pass
    finally:
        node.speaker.close()
        node.destroy_node()
        if rclpy.ok():  # ROS's own signal handler may already have shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
