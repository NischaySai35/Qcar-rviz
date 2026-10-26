#!/usr/bin/python3
"""Hold-to-drive QCar whose YOLO class list is updated over the air by eSync.

Same OTA flow as yolo_ota/, plus the car now moves:

1. Detector (main thread): reads the RealSense RGB + depth streams, runs
   YOLOv8n and keeps only the classes in active_classes.json. Each detection
   gets a distance from the depth image.

2. Drive: the camera window has a MOVE button. While the mouse is
   held down on it the car drives straight forward. If a DETECTED object is
   in the car's lane it slows down from --slow-dist and stops at --stop-dist.
   An object YOLO is not told to look for is invisible to this rule, so with
   the starting list (person only) the car drives into a bottle; after the
   OTA update (person + bottle) it stops for the bottle too.
   A 50 Hz drive thread writes the motor; if the camera loop stops refreshing
   the command for 0.3 s (stall, OTA restart, crash) it writes 0.

3. Watcher (background thread): every few seconds looks in esync-campaigns/
   for a package eSync downloaded (*.x or *.zip), unzips it, swaps the .json
   inside in as active_classes.json, restarts YOLO, then deletes the package.

The esync-campaigns/ folder is mounted into the eSync container at
/downloads (see run_esync.sh), so packages from a campaign land here.

The ROS stack must be stopped first: this opens the car's HIL board
directly, and only one program can own it.

Run:  /usr/bin/python3 yolo_ota2_demo.py            (--reset: start from person only)
Keys: q quits the window.
"""
import argparse
import json
import os
import shutil
import signal
import sys
import threading
import time
import zipfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# Quanser's camera and car wrappers live in the SDV folder, not in site-packages.
sys.path.insert(0, '/home/nvidia/Desktop/SDV')

PACKAGE_EXTS = ('.x', '.zip')
UNPACK_DIR = '_unpacked'
WINDOW = 'QCar YOLO + eSync OTA - hold to move'

# RealSense D435: RGB 640x480, depth 848x480 (depth at 640x480 delivers no
# frames through Quanser's Video3D). Nominal intrinsics; both images are
# centred on the same axis and depth has the wider field of view. Good to a
# few cm, which is plenty for a stop distance.
W, H = 640, 480
DW, DH = 848, 480
FX_RGB = 615.0
FX_DEPTH = 425.0
CAM_HEIGHT = 0.176      # m above the floor (URDF)
CAM_LEFT = 0.034        # m left of the car's centre line (URDF)
# Rough real heights, only used when the depth image has no reading.
KNOWN_HEIGHT = {'person': 1.7, 'bottle': 0.24}

# Index layout PAL's QCar.write() expects for a QCar 2:
# 0-3 indicators, 4 brake, 5 reverse, 6-7 headlamps.
LED_DRIVE = np.array([0, 0, 0, 0, 0, 0, 1, 1], dtype=np.int8)
LED_BRAKE = np.array([0, 0, 0, 0, 1, 0, 1, 1], dtype=np.int8)


def log(msg):
    line = f'[{time.strftime("%H:%M:%S")}] {msg}'
    print(line, flush=True)
    with open(os.path.join(HERE, 'ota_log.txt'), 'a') as fh:
        fh.write(line + '\n')


def read_class_file(path, model_names):
    """Return (version, [class ids]) from a class JSON.

    Accepts {"version": "...", "classes": ["person", ...]} or a bare list.
    Names are matched to the model's COCO names ignoring case; an unknown
    name is an error, so a typo can't silently detect nothing.
    """
    with open(path) as fh:
        data = json.load(fh)
    version = '?'
    if isinstance(data, dict):
        version = str(data.get('version', '?'))
        data = data.get('classes')
    if not isinstance(data, list) or not data:
        raise ValueError(f'{os.path.basename(path)} has no "classes" list')
    by_name = {name.lower(): idx for idx, name in model_names.items()}
    ids = []
    for c in data:
        key = str(c).strip().lower()
        if key not in by_name:
            raise ValueError(f'"{c}" is not one of the 80 COCO classes')
        ids.append(by_name[key])
    return version, ids


class Detector:
    def __init__(self, model_path, class_file, device):
        self.model_path = model_path
        self.class_file = class_file
        self.device = device
        self.lock = threading.Lock()
        self.status = 'starting'
        self.banner = None          # (text, expiry time) shown after an update
        self.start()

    def start(self):
        """Load YOLO and the active class list (also used to restart)."""
        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        self.version, self.class_ids = read_class_file(self.class_file, self.model.names)
        self.status = 'running'
        log(f'YOLO running, classes v{self.version}: {self.class_names()}')

    def stop(self):
        self.status = 'updating'
        self.model = None
        log('YOLO stopped')

    def class_names(self):
        return ', '.join(self.model.names[i] for i in self.class_ids)

    def detect(self, frame):
        """Return [(class name, confidence, (x1, y1, x2, y2))]."""
        with self.lock:
            if self.model is None:
                return []
            result = self.model.predict(frame, classes=self.class_ids, conf=0.4,
                                        device=self.device, verbose=False)[0]
            return [(self.model.names[int(c)], float(p), tuple(int(v) for v in b))
                    for c, p, b in zip(result.boxes.cls, result.boxes.conf,
                                       result.boxes.xyxy)]

    def apply_update(self, new_json, package):
        """Stop YOLO, swap the class file, restart. Rolls back on failure."""
        with self.lock:
            old_names = self.class_names()
            backup = self.class_file + '.bak'
            shutil.copyfile(self.class_file, backup)
            self.stop()
            shutil.copyfile(new_json, self.class_file)
            try:
                self.start()
            except Exception as exc:
                log(f'New class file rejected ({exc}); restoring the previous one')
                shutil.copyfile(backup, self.class_file)
                self.start()
                return False
            log(f'OTA update from {package}: [{old_names}] -> [{self.class_names()}]')
            self.banner = (f'OTA UPDATE v{self.version}: now detecting {self.class_names()}',
                           time.time() + 8.0)
            return True


def find_packages(folder):
    found = []
    for root, dirs, files in os.walk(folder, followlinks=True):
        dirs[:] = [d for d in dirs if d != UNPACK_DIR]
        found += [os.path.join(root, f) for f in files if f.lower().endswith(PACKAGE_EXTS)]
    return sorted(found)


def is_settled(path, quiet=3.0):
    """True once the file stopped growing, so a half-downloaded package
    is left for the next check instead of being opened."""
    try:
        size = os.path.getsize(path)
        age = time.time() - os.path.getmtime(path)
        time.sleep(1.0)
        return age >= quiet and os.path.getsize(path) == size
    except OSError:
        return False


def handle_package(detector, folder, package):
    name = os.path.basename(package)
    work = os.path.join(folder, UNPACK_DIR, os.path.splitext(name)[0])
    log(f'Found package {name}')
    try:
        shutil.rmtree(work, ignore_errors=True)
        with zipfile.ZipFile(package) as zf:
            zf.extractall(work)
        log(f'Unpacked: {", ".join(sorted(os.listdir(work)))}')
        jsons = sorted(os.path.join(r, f) for r, _, fs in os.walk(work)
                       for f in fs if f.lower().endswith('.json'))
        if not jsons:
            log(f'{name} has no .json inside; ignored')
        else:
            if len(jsons) > 1:
                log(f'Several .json files; using {os.path.basename(jsons[0])}')
            detector.apply_update(jsons[0], name)
    except (zipfile.BadZipFile, OSError, ValueError) as exc:
        log(f'Could not use {name}: {exc}')
    finally:
        # Always clean up, even a bad package, or it is retried forever.
        for path in (package, work):
            if os.path.isdir(path):
                shutil.rmtree(path, ignore_errors=True)
            elif os.path.exists(path):
                os.remove(path)
        try:
            os.rmdir(os.path.join(folder, UNPACK_DIR))
        except OSError:
            pass                # not empty: another package is mid-unpack
        log(f'Deleted {name} and its unpacked files')


def watch(detector, folder, period, stop_event):
    log(f'Watching {folder} for *.x / *.zip packages')
    while not stop_event.is_set():
        for package in find_packages(folder):
            if is_settled(package):
                handle_package(detector, folder, package)
        stop_event.wait(period)


# ---------------------------------------------------------------- distance

def box_distance(depth_m, name, box):
    """Metres to the object in box, from the depth image.

    Depth has a wider view than RGB, so the box is scaled about the image
    centre into depth pixels. The inner part of the box is used (edges hit
    the background) and the 30th percentile of it, which leans to the near
    object over whatever is behind a thin bottle. Falls back to the box
    height when there is no depth reading.
    """
    x1, y1, x2, y2 = box
    if depth_m is not None:
        k = FX_DEPTH / FX_RGB
        bx, by = (x2 - x1) * 0.25, (y2 - y1) * 0.2
        dx1, dx2 = [int(DW / 2 + (x - W / 2) * k) for x in (x1 + bx, x2 - bx)]
        dy1, dy2 = [int(DH / 2 + (y - H / 2) * k) for y in (y1 + by, y2 - by)]
        patch = depth_m[max(dy1, 0):max(dy2, dy1 + 1), max(dx1, 0):max(dx2, dx1 + 1)]
        valid = patch[(patch > 0.1) & (patch < 8.0)]
        if valid.size >= 15:
            return float(np.percentile(valid, 30))
    height = KNOWN_HEIGHT.get(name)
    if height is None:
        return None
    if y1 <= 2 or y2 >= H - 2:
        return 0.0              # cut off by the frame edge: treat as close
    return FX_RGB * height / max(y2 - y1, 1)


def in_lane(box, dist, lane_half):
    """True if the object's sideways extent overlaps the car's lane."""
    if dist is None:
        return False
    d = max(dist, 0.2)
    # Image right = car right; the camera sits CAM_LEFT left of centre.
    left = (box[0] - W / 2) * d / FX_RGB - CAM_LEFT
    right = (box[2] - W / 2) * d / FX_RGB - CAM_LEFT
    return right > -lane_half and left < lane_half


def floor_point(d, side):
    """Pixel where a floor point d m ahead and side m right of centre is seen."""
    u = W / 2 + (side + CAM_LEFT) * FX_RGB / d
    v = H / 2 + CAM_HEIGHT * FX_RGB / d
    return int(u), int(v)


# ------------------------------------------------------------------- drive

class Drive:
    """Owns the car. A 50 Hz thread writes the latest command, or 0 when the
    command is older than WATCHDOG (the camera loop stopped refreshing it)."""
    WATCHDOG = 0.3

    def __init__(self, max_throttle, steering):
        # PAL decides "physical car" with os.getlogin(), which raises in a
        # terminal with no login tty (VS Code, remote desktop). Answer it
        # with the user name instead.
        try:
            os.getlogin()
        except OSError:
            import pwd
            os.getlogin = lambda: pwd.getpwuid(os.getuid()).pw_name
        from pal.products.qcar import QCar
        self.car = QCar(readMode=0, pwmLimit=max_throttle)
        if not self.car.card.is_valid():
            raise RuntimeError('Could not open the QCar HIL board (is the ROS stack running?)')
        self.steering = steering
        self.lock = threading.Lock()
        self.throttle = 0.0
        self.leds = LED_DRIVE
        self.stamp = 0.0
        self.speed = 0.0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        log(f'QCar open (throttle cap {max_throttle:.2f} PWM)')

    def command(self, throttle, leds):
        with self.lock:
            self.throttle, self.leds, self.stamp = throttle, leds, time.time()

    def _run(self):
        while not self.stop_event.is_set():
            with self.lock:
                fresh = time.time() - self.stamp < self.WATCHDOG
                throttle = self.throttle if fresh else 0.0
                leds = self.leds if fresh else LED_BRAKE
            self.car.read_write_std(throttle=throttle, steering=self.steering, LEDs=leds)
            self.speed = abs(float(self.car.motorTach))
            self.stop_event.wait(0.02)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.car.terminate()       # writes 0 throttle / 0 steering first
        log('QCar stopped and released')


class HoldButton:
    """The on-screen MOVE button (hold to drive), driven by the window's mouse events.
    rect is in window pixels (the camera image scaled by scale)."""

    def __init__(self, scale):
        s = scale
        self.rect = (int((W - 230) * s), int((H - 150) * s), int((W - 14) * s), int((H - 56) * s))
        self.pressed = False

    def on_mouse(self, event, x, y, flags, _param):
        x1, y1, x2, y2 = self.rect
        if event == cv2.EVENT_LBUTTONDOWN and x1 <= x <= x2 and y1 <= y <= y2:
            self.pressed = True
        elif event == cv2.EVENT_LBUTTONUP:
            self.pressed = False    # released anywhere = stop
        elif event == cv2.EVENT_MOUSEMOVE and self.pressed and not flags & cv2.EVENT_FLAG_LBUTTON:
            self.pressed = False    # missed the release (e.g. outside the window)


def ros_stack_running():
    """True if qcar2_hardware (the ROS driver) already owns the HIL board.
    Reads /proc/*/comm, so it matches the process name only."""
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/comm') as fh:
                if fh.read().strip() == 'qcar2_hardware':
                    return True
        except OSError:
            pass
    return False


# ----------------------------------------------------------------- display

def draw(frame, detector, dets, state, button, args, speed):
    """Scale the camera image up by args.scale and draw the overlay on the
    big image, so text and lines stay sharp instead of being stretched."""
    s = args.scale
    frame = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_LINEAR)
    h, w = frame.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    def p(pt):
        return int(pt[0] * s), int(pt[1] * s)

    def text(txt, org, size, color, thick=2, bg=None):
        size, thick = size * s, max(1, int(round(thick * s)))
        if bg is not None:
            (tw, th), base = cv2.getTextSize(txt, font, size, thick)
            cv2.rectangle(frame, (org[0] - 4, org[1] - th - 6), (org[0] + tw + 4, org[1] + base),
                          bg, -1)
        cv2.putText(frame, txt, org, font, size, color, thick, cv2.LINE_AA)

    # Lane on the floor (car width + margin), with the stop line in red.
    lane = args.lane_half
    near, far = max(args.stop_dist, 0.45), 3.0
    pts = np.array([p(floor_point(near, -lane)), p(floor_point(far, -lane)),
                    p(floor_point(far, lane)), p(floor_point(near, lane))], np.int32)
    cv2.polylines(frame, [pts], True, (255, 200, 0), 2, cv2.LINE_AA)
    if args.stop_dist >= 0.45:
        cv2.line(frame, p(floor_point(args.stop_dist, -lane)),
                 p(floor_point(args.stop_dist, lane)), (0, 0, 255), 3, cv2.LINE_AA)

    for name, conf, box, dist, blocking in dets:
        color = (0, 0, 255) if blocking else (0, 200, 0)
        a, b = p(box[:2]), p(box[2:])
        cv2.rectangle(frame, a, b, color, 3)
        label = f'{name} {dist:.2f} m' if dist is not None else f'{name} ?'
        # Above the box, or just inside it when the box reaches the top bar.
        text(label, (a[0] + 6, max(a[1] - 8, int(84 * s))), 0.7, (255, 255, 255), 2, bg=color)

    # Top bar: class list (what the OTA changes) and what is seen.
    cv2.rectangle(frame, (0, 0), (w, int(56 * s)), (0, 0, 0), -1)
    if detector.status == 'running':
        top = f'Classes v{detector.version}: {detector.class_names()}'
    else:
        top = 'UPDATING CLASS LIST...'
    text(top, (int(10 * s), int(22 * s)), 0.6, (255, 255, 255))
    seen = sorted({d[0] for d in dets})
    text('Seeing: ' + (', '.join(seen) or 'nothing'), (int(10 * s), int(47 * s)), 0.6,
         (0, 255, 255))
    text(f'{speed:.2f} m/s', (w - int(120 * s), int(47 * s)), 0.6, (255, 255, 255))

    # The button: grey idle, green driving, red held but blocked.
    x1, y1, x2, y2 = button.rect
    if not args.drive:
        color, label = (90, 90, 90), 'NO DRIVE'
    elif not button.pressed:
        color, label = (110, 110, 110), 'MOVE'
    elif state['blocked']:
        color, label = (0, 0, 220), 'STOPPED'
    else:
        color, label = (0, 170, 0), 'MOVING'
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, -1)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), max(2, int(2 * s)))
    (tw, th), _ = cv2.getTextSize(label, font, 0.8 * s, int(2 * s))
    text(label, ((x1 + x2 - tw) // 2, (y1 + y2 + th) // 2), 0.8, (255, 255, 255))
    if state['blocked']:
        text(state['blocked'], (int(10 * s), int((H - 62) * s)), 0.75, (255, 255, 255),
             bg=(0, 0, 200))

    if detector.banner and time.time() < detector.banner[1]:
        cv2.rectangle(frame, (0, h - int(42 * s)), (w, h), (0, 140, 0), -1)
        text(detector.banner[0], (int(10 * s), h - int(14 * s)), 0.6, (255, 255, 255))
    return frame


# -------------------------------------------------------------------- main

def decide(dets, args, state, now):
    """Throttle for this frame from the nearest detected object in the lane.

    Full --throttle when clear, a linear slow-down from --slow-dist to
    --stop-dist, 0 inside it. Once stopped it stays stopped until nothing
    has blocked for 0.5 s, so a flickering detection can't jerk it forward.
    """
    blocking = [(d[3], d[0]) for d in dets if d[4]]
    nearest = min(blocking) if blocking else None
    if nearest and nearest[0] <= args.stop_dist:
        state['block_until'] = now + 0.5
        state['blocked'] = f'STOP: {nearest[1]} {nearest[0]:.2f} m ahead'
    elif now >= state['block_until']:
        state['blocked'] = None
    if state['blocked']:
        return 0.0
    if nearest and nearest[0] < args.slow_dist:
        frac = (nearest[0] - args.stop_dist) / (args.slow_dist - args.stop_dist)
        return args.min_throttle + (args.throttle - args.min_throttle) * frac
    return args.throttle


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', default=os.path.join(HERE, 'models', 'yolov8n.pt'))
    ap.add_argument('--classes', default=os.path.join(HERE, 'active_classes.json'))
    ap.add_argument('--watch', default=os.path.join(HERE, 'esync-campaigns'))
    ap.add_argument('--period', type=float, default=2.0, help='seconds between folder checks')
    ap.add_argument('--device', default='0', help='0 = GPU, cpu = CPU')
    ap.add_argument('--reset', action='store_true',
                    help='start from classes/old.json (person only) for a fresh demo')
    ap.add_argument('--no-drive', dest='drive', action='store_false',
                    help='detection only; never opens the motor')
    ap.add_argument('--throttle', type=float, default=0.08, help='PWM while clear')
    ap.add_argument('--min-throttle', type=float, default=0.05,
                    help='PWM at the end of the slow-down (motor deadband is ~0.03)')
    ap.add_argument('--max-throttle', type=float, default=0.12, help='hard PWM cap')
    ap.add_argument('--steering', type=float, default=0.0, help='rad trim if it drifts')
    ap.add_argument('--stop-dist', type=float, default=0.5, help='m: stop for a detected object')
    ap.add_argument('--slow-dist', type=float, default=1.2, help='m: start slowing down')
    ap.add_argument('--lane-half', type=float, default=0.15,
                    help='m each side of centre that counts as in the way')
    ap.add_argument('--scale', type=float, default=2.0,
                    help='window size as a multiple of the 640x480 camera image')
    ap.add_argument('--window-x', type=int, default=320, help='window position on screen')
    ap.add_argument('--window-y', type=int, default=60)
    args = ap.parse_args()
    args.throttle = min(args.throttle, args.max_throttle)
    args.min_throttle = min(args.min_throttle, args.throttle)
    args.slow_dist = max(args.slow_dist, args.stop_dist + 0.05)

    if not os.environ.get('DISPLAY'):
        os.environ['DISPLAY'] = ':0'     # the car's own screen
    if args.drive and ros_stack_running():
        sys.exit('qcar2_hardware is running and owns the car. Stop the ROS stack first, '
                 'or use --no-drive.')
    if args.reset:
        shutil.copyfile(os.path.join(HERE, 'classes', 'old.json'), args.classes)
        log('Reset active_classes.json to classes/old.json')

    os.makedirs(args.watch, exist_ok=True)
    detector = Detector(args.model, args.classes, args.device)
    stop_event = threading.Event()
    watcher = threading.Thread(target=watch, daemon=True,
                               args=(detector, args.watch, args.period, stop_event))
    watcher.start()

    # Quanser's read_RGB() catches KeyboardInterrupt and carries on, so
    # Ctrl+C would never reach the loop. Turn it into a flag instead.
    quit_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: quit_event.set())
    signal.signal(signal.SIGTERM, lambda *_: quit_event.set())

    from pal.utilities.vision import Camera3D
    # readMode=0: reads return -1 at once when no frame is ready, instead of
    # waiting forever for a stream that never delivers.
    camera = Camera3D(mode='RGB, Depth', frameWidthRGB=W, frameHeightRGB=H, frameRateRGB=30.0,
                      frameWidthDepth=DW, frameHeightDepth=DH, frameRateDepth=30.0,
                      readMode=0)
    log(f'RealSense open ({W}x{H} RGB + {DW}x{DH} depth)')

    drive = None
    try:
        drive = Drive(args.max_throttle, args.steering) if args.drive else None
        button = HoldButton(args.scale)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
        cv2.setMouseCallback(WINDOW, button.on_mouse)

        state = {'blocked': None, 'block_until': 0.0}
        depth, depth_time = None, 0.0
        last_seen, last_blocked, was_moving = None, None, False
        placed = False
        while not quit_event.is_set():
            if camera.read_RGB() == -1:
                cv2.waitKey(5)          # keeps the window responsive meanwhile
                continue
            frame = camera.imageBufferRGB.copy()
            now = time.time()
            if camera.read_depth(dataMode='M') != -1:
                depth, depth_time = camera.imageBufferDepthM[:, :, 0].copy(), now
            fresh_depth = depth if now - depth_time < 0.5 else None

            dets = []
            for name, conf, box in detector.detect(frame):
                dist = box_distance(fresh_depth, name, box)
                dets.append((name, conf, box, dist, in_lane(box, dist, args.lane_half)))
            throttle = decide(dets, args, state, now)

            moving = button.pressed and drive is not None and throttle > 0.0
            if drive is not None:
                drive.command(throttle if button.pressed else 0.0,
                              LED_DRIVE if moving else LED_BRAKE)

            seen = sorted({d[0] for d in dets})
            if seen != last_seen:
                print(f'  seeing: {", ".join(seen) or "nothing"}', flush=True)
                last_seen = seen
            # Log when it starts blocking, not every centimetre of change.
            if bool(state['blocked']) != bool(last_blocked):
                if state['blocked']:
                    log(state['blocked'])
                last_blocked = state['blocked']
            if moving != was_moving:
                log('Moving forward' if moving else 'Stopped')
                was_moving = moving

            speed = drive.speed if drive is not None else 0.0
            cv2.imshow(WINDOW, draw(frame, detector, dets, state, button, args, speed))
            if not placed:
                # Otherwise the window manager may drop it partly off-screen.
                cv2.moveWindow(WINDOW, args.window_x, args.window_y)
                placed = True
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break                   # window closed with its X button
    except KeyboardInterrupt:
        pass
    finally:
        if drive is not None:
            drive.close()
        stop_event.set()
        camera.terminate()
        cv2.destroyAllWindows()
        log('Stopped')


if __name__ == '__main__':
    main()
