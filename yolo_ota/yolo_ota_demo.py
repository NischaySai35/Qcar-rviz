#!/usr/bin/python3
"""YOLO object detection whose class list is updated over the air by eSync.

Two parts run in one process:

1. Detector (main thread): reads the top RealSense camera, runs YOLOv8n
   (the 80 COCO classes) and keeps only the classes listed in
   active_classes.json. Boxes are drawn in a window.

2. Watcher (background thread): every few seconds looks in esync-campaigns/
   for a package eSync has downloaded (*.x, or *.zip -- an .x is just a
   renamed zip). For each one it unzips it, takes the .json inside, stops
   YOLO, writes that JSON over active_classes.json, restarts YOLO with the
   new classes, then deletes the package and the unpacked files. Deleting is
   what stops the same package being applied again on every check.

The esync-campaigns/ folder is mounted into the eSync container at
/downloads (see run_esync.sh), so packages from a campaign land here.

Run:  /usr/bin/python3 yolo_ota_demo.py
Keys: q quits the window.
"""
import argparse
import json
import os
import shutil
import sys
import threading
import time
import zipfile

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
# Quanser's camera wrapper lives in the SDV folder, not in site-packages.
sys.path.insert(0, '/home/nvidia/Desktop/SDV')

PACKAGE_EXTS = ('.x', '.zip')
UNPACK_DIR = '_unpacked'


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
        with self.lock:
            if self.model is None:
                return frame, []
            result = self.model.predict(frame, classes=self.class_ids, conf=0.4,
                                        device=self.device, verbose=False)[0]
            seen = sorted({self.model.names[int(c)] for c in result.boxes.cls})
            return result.plot(), seen

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


def draw_overlay(frame, detector, seen):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 52), (0, 0, 0), -1)
    if detector.status == 'running':
        top = f'Classes v{detector.version}: {detector.class_names()}'
    else:
        top = 'UPDATING CLASS LIST...'
    cv2.putText(frame, top, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, 'Seeing: ' + (', '.join(seen) or 'nothing'), (8, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    if detector.banner and time.time() < detector.banner[1]:
        cv2.rectangle(frame, (0, h - 40), (w, h), (0, 140, 0), -1)
        cv2.putText(frame, detector.banner[0], (8, h - 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return frame


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', default=os.path.join(HERE, 'models', 'yolov8n.pt'))
    ap.add_argument('--classes', default=os.path.join(HERE, 'active_classes.json'))
    ap.add_argument('--watch', default=os.path.join(HERE, 'esync-campaigns'))
    ap.add_argument('--period', type=float, default=2.0, help='seconds between folder checks')
    ap.add_argument('--device', default='0', help='0 = GPU, cpu = CPU')
    ap.add_argument('--no-window', action='store_true', help='print detections only')
    args = ap.parse_args()

    if not args.no_window and not os.environ.get('DISPLAY'):
        os.environ['DISPLAY'] = ':0'     # the car's own screen

    os.makedirs(args.watch, exist_ok=True)
    detector = Detector(args.model, args.classes, args.device)
    stop_event = threading.Event()
    watcher = threading.Thread(target=watch, daemon=True,
                               args=(detector, args.watch, args.period, stop_event))
    watcher.start()

    from pal.utilities.vision import Camera3D
    camera = Camera3D(mode='RGB', frameWidthRGB=640, frameHeightRGB=480, frameRateRGB=30.0)
    log('RealSense open (640x480 RGB)')
    last_seen = None
    try:
        while True:
            if camera.read_RGB() == -1:
                time.sleep(0.01)
                continue
            frame, seen = detector.detect(camera.imageBufferRGB.copy())
            if seen != last_seen:
                print(f'  seeing: {", ".join(seen) or "nothing"}', flush=True)
                last_seen = seen
            if not args.no_window:
                cv2.imshow('QCar YOLO + eSync OTA', draw_overlay(frame, detector, seen))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        camera.terminate()
        cv2.destroyAllWindows()
        log('Stopped')


if __name__ == '__main__':
    main()
