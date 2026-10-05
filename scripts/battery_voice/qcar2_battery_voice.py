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
import time

START_BELOW_MV = 10800      # first warning when the battery drops below this
REARM_ABOVE_MV = 11500      # a charged battery re-arms the warnings
READ_EVERY_SEC = 2.0
WINDOW = 5                  # readings in the median (= 10 s)
USER = 'nvidia'
PULSE_SINK = 'alsa_output.platform-sound.analog-stereo'
PIPER = f'/home/{USER}/Desktop/Qcar-rviz/models/piper/piper/piper'
VOICE = f'/home/{USER}/Desktop/Qcar-rviz/models/piper/voices/en_US-ryan-medium.onnx'


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
    try:
        if os.path.isfile(PIPER) and os.path.isfile(VOICE):
            played = as_user(['sh', '-c',
                              f'echo "$0" | "{PIPER}" --model "{VOICE}" --output_raw '
                              f'--length_scale 1.1 2>/dev/null | '
                              f'paplay --raw --rate=22050 --format=s16le --channels=1', text],
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
