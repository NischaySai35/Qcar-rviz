#!/usr/bin/python3
"""Spoken low-battery warnings for the QCar2 -- always on, no ROS needed.

Runs as a boot-time systemd service (qcar2-battery-voice.service, installed by
install_battery_voice.sh). Independent of mapping/navigation: it works with
the ROS stack running or not.

WHAT IT SAYS
  Nothing while the battery is at or above 10.8 V. The first time it drops
  below 10.8 V: "Battery low, 10.7 volts." Then once at each further 0.1 V
  step (10.6, 10.5, ...). Never repeats a step. Fitting a charged battery
  (above 11.5 V) re-arms it. Spoken at 100 % speaker volume; the previous
  volume is restored afterwards.

WHERE THE VOLTAGE COMES FROM
  The carrier board's INA3221 monitor, the same reading Quanser's own
  quarc_power_monitor service uses (/sys/bus/i2c/devices/1-0040/hwmon/...).
  Not the HIL card, so it never competes with qcar2_hardware for the board.
  Those sysfs files are root-only, which is why this runs as root; speech is
  then played as the desktop user, through their PulseAudio.

SMOOTHING
  Read every 2 s, decided on the median of the last 10 s. Motor current makes
  the voltage dip for a moment under load; a raw reading would announce steps
  the battery has not really reached.
"""

import glob
import os
import re
import statistics
import subprocess
import sys
import threading
import time

START_BELOW_MV = 10800      # first warning when the battery drops below this
REARM_ABOVE_MV = 11500      # a charged battery re-arms the warnings
READ_EVERY_SEC = 2.0
WINDOW = 5                  # readings in the median (= 10 s)
USER = 'nvidia'
PULSE_SINK = 'alsa_output.platform-sound.analog-stereo'
PIPER = f'/home/{USER}/Desktop/Qcar-rviz/models/piper/piper/piper'
VOICE = f'/home/{USER}/Desktop/Qcar-rviz/models/piper/voices/en_US-ryan-medium.onnx'
CLIENT_NAME = 'qcar2-battery-voice'   # PulseAudio application.name of the warning


def log(text):
    print(text, flush=True)          # -> journalctl -u qcar2-battery-voice


def find_battery_input():
    """The INA3221 channel that reads a 3S battery (9-13.5 V)."""
    for path in sorted(glob.glob('/sys/bus/i2c/devices/1-0040/hwmon/hwmon*/in*_input')):
        try:
            mv = int(open(path).read().strip())
        except (OSError, ValueError):
            continue
        if 9000 <= mv <= 13500:
            log(f'battery input: {path} ({mv} mV)')
            return path
    return None


def user_env():
    uid = int(subprocess.run(['id', '-u', USER], capture_output=True, text=True).stdout)
    return uid, {'XDG_RUNTIME_DIR': f'/run/user/{uid}', 'HOME': f'/home/{USER}',
                 'PATH': '/usr/bin:/bin'}


def as_user(args, **kw):
    """Run a command as the desktop user, in their PulseAudio session."""
    _, env = user_env()
    return subprocess.run(['runuser', '-u', USER, '--', 'env',
                           *[f'{k}={v}' for k, v in env.items()], *args], **kw)


class Ducker:
    """Battery warnings have top priority (operator, 2026-10-09): while one
    is spoken, every OTHER speech stream on the car (the ROS announcer's
    answers, safety-shield lines, callouts) is MUTED -- not stopped: it keeps
    playing silently and is audible again the moment the warning ends.

    A PulseAudio sink-input is muted when it belongs to a client (paplay,
    speech-dispatcher) other than this script's own player.  Module-owned
    inputs are left alone: the echo-cancel module's feed to the speaker is
    one of them, and the warning itself travels through it.  Polled every
    0.1 s while speaking, so a sentence that STARTS during the warning is
    muted too."""

    def __init__(self, own_names):
        self.own_names = own_names
        self.muted = set()
        self.stop = threading.Event()
        self.thread = None

    def others(self):
        out = as_user(['pactl', 'list', 'sink-inputs'], capture_output=True, text=True,
                      timeout=3).stdout
        found = []
        for block in out.split('Sink Input #')[1:]:
            idx = block.split('\n', 1)[0].strip()
            client = re.search(r'^\s*Client:\s*(\S+)', block, re.M)
            app = re.search(r'application\.name = "([^"]*)"', block)
            if not idx.isdigit() or client is None or client.group(1) == 'n/a':
                continue                         # module-owned (echo cancel): keep
            if app and any(app.group(1).startswith(n) for n in self.own_names):
                continue                         # the warning itself
            found.append(idx)
        return found

    def _run(self):
        while not self.stop.is_set():
            try:
                for idx in self.others():
                    if idx not in self.muted:
                        as_user(['pactl', 'set-sink-input-mute', idx, '1'], timeout=3)
                        self.muted.add(idx)
            except (OSError, subprocess.SubprocessError):
                pass
            self.stop.wait(0.1)

    def __enter__(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        time.sleep(0.15)                         # first sweep done before we speak
        return self

    def __exit__(self, *_exc):
        self.stop.set()
        self.thread.join(timeout=2)
        for idx in self.muted:                   # gone already = harmless error
            as_user(['pactl', 'set-sink-input-mute', idx, '0'], timeout=3,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return False


def speak(text):
    """Say `text` at 100 % volume, then put the volume back. True if played."""
    uid, _ = user_env()
    if not os.path.exists(f'/run/user/{uid}/pulse/native'):
        log('no PulseAudio session (nobody logged in?); will retry at the next reading')
        return False
    out = as_user(['pactl', 'get-sink-volume', PULSE_SINK], capture_output=True, text=True,
                  timeout=5).stdout
    m = re.search(r'(\d+)%', out)
    before = m.group(1) if m else None
    as_user(['pactl', 'set-sink-mute', PULSE_SINK, '0'], timeout=5)
    as_user(['pactl', 'set-sink-volume', PULSE_SINK, '100%'], timeout=5)
    use_piper = os.path.isfile(PIPER) and os.path.isfile(VOICE)
    # Own stream = what the ducker must NOT mute.
    own = [CLIENT_NAME] if use_piper else ['speech-dispatcher']
    try:
        with Ducker(own):
            if use_piper:
                played = as_user(['sh', '-c',
                                  f'echo "$0" | "{PIPER}" --model "{VOICE}" --output_raw '
                                  f'--length_scale 1.1 2>/dev/null | '
                                  f'paplay --client-name={CLIENT_NAME} --raw --rate=22050 '
                                  f'--format=s16le --channels=1', text],
                                 timeout=30).returncode == 0
            else:
                played = as_user(['spd-say', '-w', text], timeout=30).returncode == 0
    except subprocess.TimeoutExpired:
        played = False
    finally:
        if before is not None:
            as_user(['pactl', 'set-sink-volume', PULSE_SINK, f'{before}%'], timeout=5)
    return played


def main():
    path = None
    readings = []
    # Lowest 0.1 V step already announced, in tenths of a volt. Starting at
    # 108 means the first announcement is the 10.7 step (below 10.8 V).
    announced = START_BELOW_MV // 100
    log('QCar2 battery voice started')
    while True:
        if path is None:
            path = find_battery_input()
            if path is None:
                time.sleep(10.0)
                continue
        try:
            readings.append(int(open(path).read().strip()))
        except (OSError, ValueError) as exc:
            log(f'read failed ({exc}); rescanning')
            path = None
            continue
        readings = readings[-WINDOW:]
        mv = statistics.median(readings)
        if len(readings) == WINDOW:
            if mv > REARM_ABOVE_MV and announced < START_BELOW_MV // 100:
                log(f'{mv / 1000:.2f} V: charged battery, warnings re-armed')
                announced = START_BELOW_MV // 100
            step = int(mv // 100)                       # 10.73 V -> 107
            if mv < START_BELOW_MV and step < announced:
                text = f'Battery low, {step // 10} point {step % 10} volts.'
                log(f'{mv / 1000:.2f} V -> "{text}"')
                if speak(text):
                    announced = step
        time.sleep(READ_EVERY_SEC)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
