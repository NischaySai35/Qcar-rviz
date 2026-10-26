# yolo_ota2: hold-to-drive OTA demo

This demo builds on `yolo_ota/`. The car drives forward while you hold down the **MOVE** button in the camera window. It stops for objects YOLO detects in its lane, using RealSense depth for the distance. An eSync campaign changes which objects YOLO detects.

| Stage | Classes | Bottle in the lane | Person in the lane |
|---|---|---|---|
| Before the update (`classes/old.json`, v0.0.1) | person | not seen, so the car drives into it | slows from 1.2 m, stops at 0.5 m |
| After the update (`classes/new.json`, v0.1.0) | person, bottle | slows and stops | slows and stops |

## Run

1. Stop the ROS stack. This demo opens the car's HIL board directly, and only one program can own it. The demo refuses to start while `qcar2_hardware` is running.
2. Start the detector and drive window from a fresh state with `/usr/bin/python3 yolo_ota2_demo.py --reset`.
3. In a second terminal, run `./run_esync.sh` to start eSync with VIN `qcar123`. The vendor pushes the campaign, and the package lands in `esync-campaigns/`. The demo then swaps the class list, restarts YOLO and shows a green banner.
4. To test without the vendor, run `/usr/bin/python3 make_test_package.py classes/new.json`.

Press `q` in the window to quit. Releasing the button stops the car. If the camera or YOLO loop stalls for 0.3 s, a watchdog sets the throttle to 0.

## Tuning flags

| Flag | Default | Meaning |
|---|---|---|
| `--throttle` | 0.08 | PWM while the lane is clear |
| `--max-throttle` | 0.12 | Hard PWM cap |
| `--stop-dist` | 0.5 m | Distance at which the car stops for a detected object |
| `--slow-dist` | 1.2 m | Distance at which the car starts slowing down |
| `--lane-half` | 0.15 m | Width counted as "in the way" on each side of the centre line |
| `--steering` | 0 rad | Trim if the car drifts sideways |
| `--no-drive` | off | Detection only; the motor is never opened |
| `--scale` | 2.0 | Window size as a multiple of the 640×480 camera image (2.0 gives 1280×960) |
| `--window-x`, `--window-y` | 320, 60 | Where the window opens on screen |

The car has no brake. At 0 throttle it coasts a few centimetres, so raise `--stop-dist` if you raise `--throttle`.
