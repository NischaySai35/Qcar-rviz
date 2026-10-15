# QCar2 Autonomous Indoor Robot — Project Overview

A Quanser QCar2 (1/10-scale self-driving car) turned into an indoor robot that:

- **maps a room** with LiDAR SLAM, either driven by hand or **fully on its own** (room-aware frontier exploration),
- **labels the map with objects** it sees through its cameras (open-vocabulary detection, e.g. "air cooler", "sofa"),
- **navigates** on a saved map by click, by typed command, or by voice ("hey car, go to the air cooler"),
- **answers questions** about the room and what it sees, using an on-board vision-language model,
- **keeps itself and people safe**: speed limits, watchdogs, a bump/glass detector, and a LiDAR "safety shield" that backs away from people,
- is operated from a **browser console** served by the car itself (no app install, works from any laptop/phone).

Everything runs **on the car** — no cloud, no audio or images leave the machine.

For day-to-day commands see [commands.md](commands.md); for the long-form manual and troubleshooting see [README.md](README.md).

---

## Contents

1. [Hardware](#1-hardware)
2. [How each piece of hardware is used](#2-how-each-piece-of-hardware-is-used)
3. [Software stack](#3-software-stack)
4. [AI models](#4-ai-models)
5. [System architecture](#5-system-architecture)
6. [Operating modes](#6-operating-modes)
7. [SLAM, odometry and localization](#7-slam-odometry-and-localization)
8. [Navigation (Nav2) configuration](#8-navigation-nav2-configuration)
9. [Motor control and the safety layers](#9-motor-control-and-the-safety-layers)
10. [Perception: objects, depth, glass](#10-perception-objects-depth-glass)
11. [Autonomous room mapping](#11-autonomous-room-mapping)
12. [Voice, speech and the assistant](#12-voice-speech-and-the-assistant)
13. [Operator interfaces](#13-operator-interfaces)
14. [Every node, at a glance](#14-every-node-at-a-glance)
15. [Key ROS topics](#15-key-ros-topics)
16. [Network ports and background servers](#16-network-ports-and-background-servers)
17. [Repository layout](#17-repository-layout)
18. [Scripts and operations](#18-scripts-and-operations)
19. [Engineering rules learned the hard way](#19-engineering-rules-learned-the-hard-way)
20. [Known limitations and open items](#20-known-limitations-and-open-items)

---

## 1. Hardware

### 1.1 Compute

| Item | Detail |
|---|---|
| Board | **NVIDIA Jetson AGX Orin Developer Kit** (ARM64 CPU + Ampere GPU, CUDA arch sm_87) |
| OS | Ubuntu 20.04, kernel 5.10.216-tegra (JetPack 5.x), system Python 3.8 |
| Hardware I/O | Quanser HIL SDK (`libhil`, `libquanser-media`) talking to the car's own microcontroller via `hil_open("qcar2","0")` |
| Networking | dual-antenna 802.11ac Wi-Fi + Bluetooth, Gigabit Ethernet |
| Display/IO | HDMI, USB-C, 2.7" 400×240 LCD diagnostic screen (Quanser firmware) |

GPU usage today: YOLO-World + CLIP (PyTorch/CUDA), Cosmos-Reason2 (llama.cpp CUDA), Whisper (whisper.cpp CUDA). Nav2/SLAM run on the ARM CPU cores.

### 1.2 Chassis and drivetrain (from the URDF)

| Property | Value |
|---|---|
| Layout | 1/10-scale, rear-wheel drive, **Ackermann steering** (cannot turn in place) |
| Wheelbase | **0.25725 m** |
| Track | ~0.112 m |
| Wheel radius | 0.033 m |
| Steering mechanical stop | **±0.5236 rad (±30°)** |
| Minimum turning radius | 0.25725 / tan(30°) = **0.446 m** |
| Footprint used by Nav2 | 0.43 × 0.24 m (`[[0.22,0.12],[0.22,-0.12],[-0.21,-0.12],[-0.21,0.12]]`) |
| Motor | single brushed DC with quadrature encoder, 720×4 counts/rev, gear ratio (13·19)/(70·30); ≈118,000 counts/s per 1 m/s |
| Battery | 3S 11.1 V 3300 mAh LiPo (XT60), ~30 min of driving |

### 1.3 Sensors

| Sensor | Model / spec | Mount (base_link) |
|---|---|---|
| **2D LiDAR** | RPLIDAR A2M12, serial 256000 baud, 20 Hz, up to 1680 pts/rev, 12 m range | (-0.014, 0, 0.194) — one horizontal plane ~19 cm above the floor |
| **IMU** | 6-axis gyro (±250 dps, 500 Hz) + accelerometer (±16 g, 1 kHz) | (0.012, 0, 0.090) |
| **Motor encoder** | quadrature, on the single drive motor | — |
| **4 × CSI cameras** | IMX219-class, 160° fisheye; front / back / left / right | front (0.198, 0, 0.110), back (-0.165, 0, 0.110), right (0.016, -0.072, 0.110), left (0.016, 0.051, 0.110) |
| **RGB-D camera** | Intel RealSense D435 (USB 8086:0b07), colour + depth | RGB (0.095, 0.034, 0.176), depth (0.095, -0.003, 0.176) |
| **Battery monitor** | HIL analog 2, plus the carrier board's INA3221 (I²C, sysfs) | — |
| **Current sensing** | HIL analog 3/4: electronics current, motor current | — |
| **Microphones** | 2 × PDM mics (stereo pair) on the Orin's **DMIC2** → ALSA `hw:1,1` → Pulse source `alsa_input.hw_1_1` | — |

### 1.4 Actuators and outputs

| Output | Channel | Notes |
|---|---|---|
| Steering servo | HIL "other" 1000 (radians) | **write-only — there is no steering-angle sensor** |
| Motor throttle | HIL "other" 11000 (PWM) | clipped in the driver; motor deadband ≈ 0.03 PWM |
| Body LEDs | digital 11–26 | headlights, brake, reverse, turn signals (16 LEDs) |
| Underbody RGB strip | SPI | 33 LEDs |
| Speaker | PUI AST-03208MR-R, 32 mm, 8 Ω, 0.1 W on the Orin's **I2S1** | Pulse sink `alsa_output.platform-sound.analog-stereo`; **ships muted at 0 %** |

### 1.5 Hardware that does NOT exist (don't design around it)

- No steering encoder / servo feedback — the real wheel angle can only be *inferred*: δ = atan(L·ω_gyro / v).
- No per-wheel speed sensors — only one motor encoder.
- **The encoder does not reliably give direction** (reads positive while reversing; proven from speed logs). Direction is taken from the sign of the applied throttle, updated only when the wheels are ~stopped.
- No GPS, no bumper switches, no ultrasonic sensors.
- No brake — "braking" is reverse throttle, which must be cut the moment wheel speed stops falling.

### 1.6 Car-specific quirks

- The front wheels sit mechanically **off-centre (pull left)**. A steering centre-bias (~+0.06 rad on top of Quanser's 0.05 rad) is learned online and **persisted to `~/.qcar2_steering_bias`** so every launch starts straight.
- The LiDAR is at base_link x≈0.10 m, the front bumper at x=0.22 m, and the RPLIDAR returns nothing under ~0.15 m — so **objects within ~5–8 cm of the front bumper are invisible** to the LiDAR.
- The RealSense cannot measure the first ~20–30 cm.
- The LiDAR motor only spins down on a **graceful** exit (`rplidar_close()`); a SIGKILL leaves it spinning.

---

## 2. How each piece of hardware is used

| Hardware | Used for | Software that owns it |
|---|---|---|
| **RPLIDAR A2M12** | SLAM mapping (Cartographer), localization (AMCL), Nav2 costmaps, object range for labelling, bump/slip detection, the safety shield, obstacle announcements | `lidar` (C++, `qcar2_nodes`) → `/scan` |
| **Motor encoder** | wheel odometry (speed magnitude), closed-loop speed control, stall/slip detection, steering-bias learning | `qcar2_hardware` → `/qcar2_joint`; `wheel_imu_odometry.py` |
| **IMU gyro** | heading for wheel+IMU odometry (bias calibrated at startup), Cartographer IMU input, inferring real steering angle | `qcar2_hardware` → `/qcar2_imu` |
| **IMU accel** | published, fed to Cartographer in fused mode | `/qcar2_imu` |
| **Steering servo** | Ackermann steering from Nav2, drive pad, shield | `qcar2_hardware` (single final writer) |
| **Drive motor** | throttle from the feed-forward + trim speed loop | `qcar2_hardware::drive_throttle()` |
| **4 × CSI cameras** | 360° object detection during mapping (YOLO-World), VLM "looks" (2×2 mosaic) while auto-exploring, console previews | `csi` (C++) per camera, `cameras_360.launch.py` |
| **RealSense D435** | the **front camera** in navigation/mapping (console view with distances), low-obstacle detection below the LiDAR plane (`/front/depth_scan` → costmap `depth_layer`), front image for the VLM | `realsense2_camera_node` (640×480 @ 15 fps, depth aligned to colour), `qcar2_depth_view.py`, `qcar2_depth_obstacles.py` |
| **Speaker** | spoken status, replies to questions, safety-shield lines, battery warnings | `qcar2_announcer.py` (Piper TTS), battery-voice systemd service |
| **Microphones (DMIC2)** | always-on wake word + voice commands, with WebRTC echo cancellation so the car does not hear itself | `qcar2_voice_command.py`, Pulse `module-echo-cancel` set up by the announcer |
| **Battery voltage** | live voltage on the console; spoken low-battery warnings | `/qcar2_battery`; `qcar2_battery_voice.py` (reads INA3221 via sysfs, root service) |
| **Body LEDs / RGB strip** | driven by Quanser's hardware node (lights, brake, reverse) | `qcar2_hardware` |
| **Wi-Fi** | serving the browser console, SSH, VS Code Remote, VNC/xrdp | `qcar2_web_gui.py` (aiohttp :8080) |
| **LCD, user buttons, CAN-FD, expansion header** | not used by this project (LCD is drivable via `quanser_lcd_display.h`) | — |

---

## 3. Software stack

### 3.1 Platform

| Layer | What |
|---|---|
| OS | Ubuntu 20.04 (L4T / JetPack 5.x) |
| Middleware | **ROS 2 Humble** (sourced by `scripts/env.sh`; the default shell has ROS 1 Noetic, which `env.sh` strips out) |
| Build | `colcon` with `--symlink-install`, run in a clean `env -i` shell by `scripts/build.sh` (mixed ROS1/ROS2 CMake paths break `find_package(PCL)`) |
| Languages | C++17 (drivers, converter, safety shield), Python 3.8 (all higher-level nodes), HTML/JS (console) |
| Vendor SDK | Quanser HIL SDK 1.25.x (`libhil`, `libquanser-media`, `quanser_lcd_display`) |
| Camera stack | NVIDIA Argus / EGLStream for CSI (needs `DISPLAY=:0`), `librealsense2` + `realsense2_camera` ROS wrapper |
| Audio | PulseAudio, speech-dispatcher/espeak-ng (fallback), Piper, `paplay`, `sounddevice`/PortAudio |

### 3.2 ROS 2 packages in `qcar2_ws/src`

| Package | Origin | Purpose |
|---|---|---|
| `qcar2_nodes` | Quanser, **heavily modified** | C++ hardware driver, LiDAR, CSI, RealSense, Nav2→QCar command converter, static LiDAR TF, **safety shield** (new) |
| `qcar2_interfaces` | Quanser | `MotorCommands.msg`, `BooleanLeds.msg` |
| `qcar2` | Quanser | URDF + meshes of the car |
| `cartographer_ros` | upstream (vendored) | SLAM; patched gflag clash (`collect_metrics` → `offline_collect_metrics`) |
| `pcl_conversions` | upstream (vendored) | build dependency not in apt for Humble on this machine |
| `rf2o_laser_odometry` | upstream (vendored) | LiDAR-only odometry — **superseded** by `wheel_imu_odometry.py` (upstream direction-inversion bug) |
| `qcar2_rviz_gui` | **new** | launch files, all Python nodes, web console, Nav2 behaviour tree, object vocabulary, RViz layout |
| `qcar2_rviz_panels` | **new** | compiled C++ RViz panel (2×2 cameras + voice controls); only used with `use_rviz:=true` |

### 3.3 Third-party ROS / system components

- **Nav2** (Humble): `nav2_bringup`, AMCL, map_server, planner_server (**SmacPlannerHybrid**), controller_server (**MPPI**), velocity_smoother, behavior_server, bt_navigator, lifecycle_manager, map_saver.
- **Cartographer** (2D, with `cartographer_occupancy_grid_node`).
- **robot_state_publisher** (URDF → TF for every sensor mount).
- **realsense2_camera**.
- `nav2_collision_monitor` is still launched but **not in the command path** — it silently dropped every command on this build, so the converter reads `/cmd_vel` directly.

### 3.4 Python AI / perception libraries

Installed with `--no-deps` into `/usr/bin/python3` so pip never replaces NVIDIA's Jetson CUDA `torch`:

- `torch` (NVIDIA Jetson build, CUDA), `ultralytics==8.2.101`, Ultralytics `CLIP`, `ftfy`, `regex`
- `vosk`, `sounddevice`, `cffi`, `srt`, `libportaudio2`
- `aiohttp` (web server), `numpy`, `opencv`, `PyYAML`

### 3.5 Native AI servers (outside ROS, plain HTTP on localhost)

- **llama.cpp** built with CUDA at `~/llama.cpp` → `llama-server` on `127.0.0.1:8090`
- **whisper.cpp** built for sm_87 at `~/whisper.cpp/build-sm87` → `whisper-server` on `127.0.0.1:8091`
- **Piper** binary + ONNX Runtime 1.14 in `models/piper/`

### 3.6 Front-end

Single-file console `web/index.html` (canvas 2D map + **three.js** 3D view with OrbitControls and the car's STL mesh), WebSocket JSON telemetry, MJPEG camera streams, browser microphone via WebSocket PCM.

---

## 4. AI models

| Model | Type | Runtime / where | Size | Used for | Measured |
|---|---|---|---|---|---|
| **YOLO-World v2 Large** (`yolov8l-worldv2.pt`) | open-vocabulary object detector (CLIP-text-embedded classes) | Ultralytics + PyTorch on the Orin GPU, `qcar2_object_mapper.py` | ~90 MB | labelling the SLAM map with objects from all 4 CSI cameras | P/R 0.76/0.64, **50 ms/frame** (vs small: 0.69/0.55, 28 ms) |
| YOLO-World v2 Small (`yolov8s-worldv2.pt`) | same | kept as a fallback | ~25 MB | — | 28 ms/frame |
| **CLIP ViT-B/32** | vision-language embedding | PyTorch GPU (`ClipVerifier`) | ~350 MB | second opinion on every detection crop: relabels when clearly more confident, vetoes bare wall/floor crops (`clip_min_prob 0.60`) | — |
| **Cosmos-Reason2-8B** (Q4_K_M GGUF + F16 mmproj) | vision-language model (Qwen-VL based, NVIDIA) | llama.cpp `llama-server`, all layers on GPU (`-ngl 99`, ctx 8192) | ~6 GB | **default** in navigation and auto-mapping: "what do you see", free-form chat, intent classification, glass/doorway judgement, map reviews | ~2.4 s per camera look, ~5 s per map review |
| **Cosmos-Reason2-2B** (Q8_0 GGUF + F16 mmproj) | same, smaller | same | ~3 GB | `llm_size:=2B` for faster replies | scene ~0.9 s, people count ~0.15 s, intent JSON ~0.5 s, load ~3.6 s |
| **Whisper small.en** (`ggml-small.en.bin`) | speech-to-text | whisper.cpp server, GPU | ~0.5 GB GPU | transcribing the sentence after "hey car" (any phrasing) | ~0.26–0.3 s per sentence warm |
| **Vosk small en-IN 0.4** | streaming offline ASR (Kaldi) | CPU, always on | ~36 MB | wake phrase "hey car", bare "stop", sentence boundaries; fallback transcript if Whisper is down | real-time |
| Vosk small en-US 0.15 | same | — | — | previous model, kept on disk | — |
| **Piper `en_US-ryan-medium`** | neural text-to-speech (ONNX) | CPU, ONNX Runtime | ~60 MB | all spoken output (picked by ear over 3 other voices and espeak-ng) | — |
| espeak-ng | formant TTS | speech-dispatcher | — | fallback if Piper is missing; battery voice | — |

How the models are layered (decision rules):

- **Map facts always win over the model.** Counts, distances and directions are computed from the landmark database and live TF; the VLM answers only what the map cannot.
- **Geometry decides, the VLM advises** in auto-mapping. The VLM can only flag "suspected" glass/doorways or skip gaps; it never declares the room finished and never puts a no-go wall on the costmap by itself (a physical bump must confirm it).
- **The LLM never controls motion.**
- Prompts all say "Always reply in English" — Cosmos-Reason2 replied in Chinese once at temperature 0.7; non-English replies are retried once, then dropped.

---

## 5. System architecture

```
                       ┌──────────────────── Browser (laptop / phone) ─────────────────────┐
                       │  Map 2D/3D · costmaps · LiDAR · path · cameras · drive pad · E-STOP │
                       │  Set Pose / Set Goal · Ask box · MIC · shield toggle · Save Map     │
                       └──────────────▲───────────────────────────────┬─────────────────────┘
                                      │ WebSocket JSON + MJPEG         │ commands / mic PCM
                                      │                     (:8080)    ▼
                               ┌──────┴──────────────────────────────────────┐
                               │            qcar2_web_gui.py (rclpy+aiohttp) │
                               └──────┬──────────────────────────────────────┘
                                      │ ROS 2 topics only
  ┌──────────────────── PERCEPTION ───┼─────────────────────┐   ┌──────── AI SERVERS (HTTP, localhost) ────────┐
  │ lidar ──/scan──────────────────────┼──┬──────────────┐   │   │ llama-server :8090  Cosmos-Reason2 2B/8B VLM │
  │ csi ×4 ──/<cam>/camera/csi_image ──┼──┤ object_mapper│   │   │ whisper-server :8091 Whisper small.en         │
  │ realsense ──color/aligned depth ───┼──┤ (YOLO-World  │   │   └────▲───────────────▲─────────────────────────┘
  │   ├─ depth_view (console + VLM img)│  │  + CLIP)     │   │        │               │
  │   └─ depth_obstacles → /front/depth_scan             │   │   assistant /     voice_command
  │ bump_guard (stall / slip / glass) → /qcar2/bump_obstacles│   explore_vlm     (Vosk wake + Whisper)
  └────────────────────────────────────┼───────────────────┘
                                       ▼
  ┌──── SLAM / LOCALIZATION ────┐   ┌──────────────── NAV2 ─────────────────┐
  │ mapping: Cartographer       │   │ bt_navigator (Ackermann BT, 2-min     │
  │   (+ wheel_imu_odometry)    │──▶│   timeout) → SmacHybrid (Reeds-Shepp) │
  │ navigation: map_server+AMCL │   │ → MPPI (20 Hz) → velocity_smoother    │
  │   + wheel_imu_odometry (TF) │   │ costmaps: obstacle + depth + bump +   │
  └─────────────────────────────┘   │   static + inflation                   │
                                    └───────────────┬────────────────────────┘
       explorer / room_analyzer ──goals──▶          │ /cmd_vel (Twist)
       object_nav / goal_heading ──goals──▶         ▼
                                    ┌──────────────────────────────────────┐
         drive pad ─manual_drive──▶ │ nav2_qcar2_converter (C++, 50 Hz,    │
                                    │  300 ms watchdog, Twist→steer angle) │
                                    └───────────────┬──────────────────────┘
                                                    │ /qcar2_motor_speed_cmd (single writer)
   safety_shield ──/qcar2/shield_cmd──┐             ▼
   bump_guard ──/qcar2/obstacle_clearance──▶ ┌──────────────────────────────┐
   E-STOP ──/qcar2_estop──────────────────▶  │ qcar2_hardware (C++, HIL)    │
                                             │ 1 kHz sensor read, 15 ms     │
                                             │ speed loop, steering bias,   │
                                             │ throttle limits, clearance   │
                                             └───────────────┬──────────────┘
                                                             ▼
                                              steering servo · drive motor · LEDs
```

Design principles behind the architecture:

1. **ROS 2 is the only transport.** The browser talks only to `qcar2_web_gui.py`, which is a normal rclpy node.
2. **Exactly one writer per actuator topic.** When Nav2 is up, every drive source (drive pad, explorer) goes through the converter's manual-override input; the hardware node is the single place every steering/throttle command funnels through.
3. **Every command relay has a watchdog** (converter 300 ms, shield command 150 ms, clearance > 0.5 s stale = ignored).
4. **Safety lives as low as possible** — the obstacle-clearance speed cap and steering bias are in the hardware node, so they also apply to manual driving that bypasses Nav2.

---

## 6. Operating modes

| Mode | Start | What runs |
|---|---|---|
| **Manual mapping** | `scripts/start_mapping.sh my_room` | hardware, LiDAR, 4 CSI + RealSense, Cartographer (fused), object mapper, bump guard, depth obstacles, web console with drive pad; autosaves map + objects every 10 s |
| **Autonomous mapping** | `scripts/start_mapping_auto.sh my_room` | everything above + Nav2 (navigation half only, on the live map) + explorer + room analyzer + VLM explorer + llama-server; finishes, saves and shuts down by itself (budget 1800 s) |
| **Navigation** | `scripts/start_navigate.sh map:=…/maps/my_room.yaml` | hardware, LiDAR, RealSense, map_server + AMCL, wheel/IMU odometry (owns odom→base_link), full Nav2, safety shield, bump guard, depth obstacles, object nav, assistant, voice (Vosk + Whisper), announcer, llama-server, whisper-server, web console |

Mapping deliberately runs no voice/assistant: they need the finished map and would only steal CPU from SLAM.

Common launch flags: `use_rviz`, `use_web_gui`, `web_port`, `use_cameras`, `use_realsense`, `detect_objects`, `sensor_fusion`, `use_llm`, `llm_size` (`8B` / `2B` / `auto`), `use_voice`, `safety_shield`, `go_next_room`, `time_budget_sec`.

---

## 7. SLAM, odometry and localization

### 7.1 Mapping — Cartographer 2D

Two profiles in `qcar2_nodes/config/`:

| | `qcar2_2d.lua` (LiDAR-only) | `qcar2_2d_fused.lua` (**default**, `sensor_fusion:=true`) |
|---|---|---|
| Odometry input | none | `wheel_imu_odometry.py` |
| IMU | off | on |
| Range | 0.12–3.5 m | 0.12–10 m (open room centres get mapped as free) |
| Real-time CSM window | — | 0.20 m / 8° |
| Pose-graph odometry weights | — | translation 1e3 (wheels slip), rotation 1e5 (gyro is good) |

Map resolution 0.05 m/cell. Saved as `maps/<name>.pgm` + `.yaml` + `<name>_objects.json`.

### 7.2 Wheel + IMU odometry (`wheel_imu_odometry.py`)

- Speed magnitude from the motor encoder; heading from the gyro (bias calibrated from stationary samples at start).
- Direction from the hardware node's signed velocity (throttle-sign logic, see §1.5).
- Freezes translation while `/qcar2/drive_blocked` (wheels spinning against an obstacle), and caps wheel-speed rise at 1.0 m/s² / fall at 2.0 m/s² — wheel slip was what smeared maps.
- Replaced `rf2o_laser_odometry`, which had an upstream bug that inverted direction of travel.
- Mapping: Cartographer owns `map→odom→base_link`. Navigation: this node publishes `odom→base_link` (remapped to `/odom`).

### 7.3 Localization in navigation

- `map_server` loads the saved map; **AMCL** (1500 particles) localizes against it.
- The operator sets the initial pose with **Set Pose** (console) / "2D Pose Estimate" (RViz). `always_reset_initial_pose: false` so it is never thrown away.
- `qcar2_goal_reset.py`: a new pose cancels the active goal, clears both costmaps and clears the overlay markers.

### 7.4 TF tree

`map → odom → base_link → {base_scan (static), csi_front/back/left/right, realsenseRGB, imu, wheels…}`.
`base_link → base_scan` is a **static** transform (`fixed_lidar_frame.cpp`) — publishing it dynamically made Nav2 drop scans and emptied the local costmap.

---

## 8. Navigation (Nav2) configuration

Main file: `qcar2_ws/src/qcar2_nodes/config/qcar2_slam_and_nav.yaml`.

### 8.1 Planner — SmacPlannerHybrid (Hybrid-A*)

- `motion_model_for_search: REEDS_SHEPP` (forward **and** reverse; forward preferred via `reverse_penalty`).
- `minimum_turning_radius: 0.55` m — deliberately looser than the controller can execute.
- A holonomic planner (NavFn) was tried first and broke every goal behind or beside the car.

### 8.2 Controller — MPPI (20 Hz)

| Parameter | Value | Why |
|---|---|---|
| `vx_max` / `vx_min` | 0.45 / −0.20 m/s | reverse enabled end to end |
| `wz_max` | 1.0 rad/s | tightest arc = vx_max / wz_max = 0.45 m ≥ chassis 0.446 m |
| `AckermannConstraints.min_turning_r` | 0.45 m | traces back to the 30° stop |
| `time_steps` × `model_dt` | 56 × 0.05 = 2.8 s (~1.26 m lookahead) | long enough to see a swerve pay off |
| `batch_size` | 1000 | 2000×75 overran the ARM CPU and starved the costmap |
| `consider_footprint` | false | footprint rasterisation was 150k polygon checks/cycle |
| `PathAngleCritic.mode` | 1 (bidirectional) | mode 2 inverts steering on reverse segments |
| `PreferForwardCritic`, `GoalAngleCritic` | disabled | they stall reverse / cause end-of-goal steering twitch |

Turning-radius ordering that must always hold: **planner 0.55 > controller 0.45 ≥ chassis 0.446**.

### 8.3 Goal handling

- `xy_goal_tolerance: 0.30` m, `yaw_goal_tolerance: 3.2` (any final heading — a car cannot rotate in place).
- `qcar2_goal_heading.py` re-aims every clicked goal: ahead → face along the bearing; behind → keep current heading (so Reeds-Shepp picks a straight reverse instead of a loop).
- Console/RViz publish `/goal_pose_raw`; `qcar2_goal_heading.py` is the sole publisher of `/goal_pose`.

### 8.4 Costmaps

| | Local (odom frame) | Global (map frame) |
|---|---|---|
| Update rate | 10 Hz | 5 Hz |
| Layers | obstacle (`/scan`), `depth_layer` (`/front/depth_scan`), `bump_layer` (marking-only), inflation | static, obstacle, depth, bump, inflation |
| Inflation | 0.35 m, scaling 3.0 | 0.90 m, scaling 3.0 (keeps paths centred in gaps) |
| Notes | ObstacleLayer (not VoxelLayer — a 2D scan can't clear voxels); raytrace range > mark range | `lethal_cost_threshold: 65` — Cartographer's live map tops out at ~90, so walls read as free at the default 100 |

### 8.5 Behaviour tree (`behavior_trees/navigate_to_pose_ackermann.xml`)

- Stock "replanning + recovery" tree with **Spin removed** (impossible on Ackermann) and **Wait removed** (operator: never idle).
- Recoveries: **BackUp (0.25 m at 0.12 m/s) first**, then clear both costmaps.
- Whole recovery node wrapped in a BT.CPP `Timeout` of **120 s** → a real "NAV FAILED" after 2 minutes of genuine effort.
- Continuous replanning at 1 Hz (the green path is always current).
- `SimpleProgressChecker`: stuck after 3 s without progress.

### 8.6 Velocity smoother

`max_velocity: [0.45, 0, 1.0]` — the angular limit **must equal** MPPI's `wz_max` (it silently clipped steering once). Accel 0.5 / decel −0.7 m/s², angular deadband 0.01.

### 8.7 Command converter (`nav2_qcar_command_convert.cpp`)

- `/cmd_vel` (Twist) → `MotorCommands` (steering angle + speed) via the bicycle model δ = atan(L·ω / v), with a speed floor so slow crawling doesn't swing the wheels lock-to-lock.
- 50 Hz republish, **300 ms watchdog** (zeroes everything if Nav2 goes quiet — fixed the "drives backwards forever" bug).
- Steering slew limit, `max_steering_rad 0.52`, E-STOP gate, manual-drive override with dead-man timing.
- Publishes `/qcar2/steering_report` (asked / sent / inferred-from-gyro) — the only way to see what the wheels really did.

---

## 9. Motor control and the safety layers

Safety is layered from "plan around it" down to "cut the throttle":

| Layer | Where | What it does |
|---|---|---|
| 1. Costmaps + MPPI ObstaclesCritic | Nav2 | plans and steers around LiDAR / depth / bump obstacles |
| 2. Speed limits | explorer (0.20 m/s, 0.12 on a poor map), Nav2 (0.45), drive pad (0.25 default, 0.35 max) | limits the *request* |
| 3. **Safety shield** | `safety_shield.cpp` + `shield_core.hpp` (navigation, on by default) | tracks objects in the live LiDAR scan, detects movers (people), keeps a car-shaped buffer (people 0.50 m front/back, 0.30 m side; still objects 0.35 m), smoothly backs away / escapes (≤0.6 m/s), resumes the goal afterwards; blind-zone memory for the last ~8 cm; speaks "Making room." |
| 4. **Obstacle clearance cap** | `qcar2_bump_guard.py` → `qcar2_hardware.cpp::target_speed()` | distance to the nearest LiDAR/depth point **along the current steering arc**; speed ≤ 1.2·(d − 0.08) → eases to rest ~8 cm short; applies to manual driving too |
| 5. **Bump guard** | `qcar2_bump_guard.py` | detects STALL (hardware) or SLIP (wheels turn, LiDAR ranges don't change) — i.e. glass or low objects; publishes a no-go wall on `/qcar2/bump_obstacles` (marking-only costmap layer, stamped into saved maps) and `/qcar2/drive_blocked` |
| 6. **Hardware speed loop** | `qcar2_hardware.cpp` | learned feed-forward `cruise_pwm(setpoint)` + small trim + P; throttle hard-clamped (≤ cruise+0.03, ≤ 0.12 absolute, ≤ 0.10 while breaking away); ramped setpoint (0.40 up / 0.70 down m/s²); traction control; stall cut-off (`/qcar2/drive_stalled`); overspeed cut as backup only |
| 7. Watchdogs | converter 300 ms, shield 150 ms, clearance 0.5 s | stale command = stop |
| 8. **E-STOP** | console button, voice "stop" (no wake word needed), `/qcar2_estop` | latched zero throttle, bypasses everything |
| 9. Shutdown hooks | `rclcpp::on_shutdown` in the hardware node | zeroes the latched motor output, saves the steering bias |
| 10. Battery voice | root systemd service | spoken warnings below 10.8 V, each 0.1 V step, mutes all other speech while talking |

Operator rule behind layer 6: **limit the command, never "check speed after commanding"** — the old pure-integrator loop wound up and surged into walls.

Per-tick speed log for diagnosis: `~/.ros/qcar2_speed_logs/speed_*.csv` (desired, target, setpoint, measured, throttle, feed-forward, trim, clearances…).

The shield has an offline test harness: `qcar2_nodes/test/test_shield_core.cpp` (scenario + noise-seed tests, smoothness metrics).

---

## 10. Perception: objects, depth, glass

### 10.1 Semantic object mapping (`qcar2_object_mapper.py`, mapping mode)

1. **Detect** — YOLO-World Large runs round-robin over the 4 CSI cameras at 6 Hz total (GPU load doesn't scale with camera count). Vocabulary is free text in `config/object_vocabulary.yaml` (air cooler, sofa, TV, fridge, door, person, …) plus "background" labels (floor, wall) that are detected and thrown away so floor isn't forced into "desk".
2. **Verify** — CLIP ViT-B/32 re-scores each crop; can relabel or veto.
3. **Locate** — the box's left/right edges become two rays from the camera's URDF pose; the **nearest LiDAR cluster** inside that sector is the object; its centre is transformed into the map frame.
4. **Fuse** — every sighting becomes a map coordinate *before* comparison, so the same TV seen by the front and then the left camera merges into one landmark (per-class association radius, default 0.85 m, `min_hits 5`).
5. **Revise** — label by weighted vote (close sightings count more), position as a capped-weight average, and **deletion** if the car repeatedly looks at the spot with clear LiDAR and sees nothing.
6. **Self-calibrate the lens FOV** — the CSI driver publishes no intrinsics; each LiDAR-matched detection gives pixel width vs angular width → median focal length over ~60 samples.

Limitation: objects that never cross the LiDAR plane (~19 cm high) — wall TVs, rugs — get a bearing but no range and are skipped ("amber" boxes in the console).

### 10.2 Driving to objects (`qcar2_object_nav.py`)

Name matching: synonyms → exact → substring → fuzzy (difflib); nearest instance wins. The goal is a **standoff pose** ~0.7 m short of the object, facing it, swept around the object until a free cell is found.

### 10.3 RealSense depth (`qcar2_depth_obstacles.py`, `qcar2_depth_view.py`)

- Depth → points in base_link (pose from URDF) → floor-plane fit → anything between min/max height is an obstacle → `/front/depth_scan` (LaserScan, 1° bins) for the costmaps; remembered points in odom (`/front/depth_obstacles`) for the clearance cap because the camera can't see the last 30 cm.
- Console view `/front/camera/depth_view` with crosshair distance and nearest-object marker; a clean `/front/camera/preview` for the VLM.

### 10.4 Glass and low obstacles

LiDAR and depth both see through glass. Detection is physical (bump guard stall/slip) plus visual (VLM "suspected glass", needs 2 sightings); a VLM claim only becomes a no-go wall when a bump lands within 0.9 m of the line. Each bump also creates a permanent ±35° no-go cone for the explorer and a 3 m cut line in the room analyzer.

---

## 11. Autonomous room mapping

Three nodes cooperate (mapping launch with `explore:=true`):

| Node | Role |
|---|---|
| `qcar2_room_analyzer.py` | **Geometry, decides.** Room outline (walls + bumps + confirmed closures, morphologically closed to seal doorways, flood-filled from seen free space), coverage %, gap classification (visit / ignore / outside), map quality, and the **STOP rule**: coverage ≥ 90 % with no worthwhile gap (or ≥ 80 % and nothing added in 60 s — reported as "stalled", not done), plus ≥ 85 % of floor seen up close by cameras |
| `qcar2_explorer.py` | **Drives.** Picks frontier / unseen-floor targets in tiers, sends Nav2 goals (speed 0.20 m/s), skips goals too close for an Ackermann car, blacklists failures (permanent after 4 strikes), nudges out of corners, yields to the drive pad (resumes 2 s after release), always stops safely and saves on exit. Operator rule: **try first, decide after** — a place is only written off after the car has gone near it |
| `qcar2_explore_vlm.py` | **Eyes, advises.** Asks Cosmos-Reason2 about glass, doorways and whether a gap is worth it — event-driven (on arrival at a gap, every 15 s as one 2×2 mosaic of all four cameras, on bump/failure, map reviews at 50 % and 80 %, final spoken report) to leave GPU time for YOLO |

`go_next_room` defaults to **false** — it never goes through doorways unless asked.

---

## 12. Voice, speech and the assistant

### 12.1 Listening (`qcar2_voice_command.py`, navigation only, mic OFF by default)

- **Two microphones**: the car's DMIC2 pair (echo-cancelled Pulse source, ≥100 Hz high-pass for hum) or the browser's mic streamed over the WebSocket (needs `localhost` → SSH tunnel to port 18080).
- **Two recognisers**: Vosk (en-IN, restricted grammar) is always on and decides *when* — wake phrase "hey car", bare "stop", sentence end; Whisper small.en then transcribes *what* was said. Falls back to Vosk text if Whisper is down.
- Three filters against normal conversation: wake phrase, restricted grammar, intent match against the map. Bare "stop" works without the wake phrase.

### 12.2 Answering (`qcar2_assistant.py`)

1. **Rules (instant):** go-to commands, time/date, "is there / how many / where / how far" from the landmark DB and live pose.
2. **Camera + VLM:** "what do you see", "can you see a laptop", colours, people count (front RealSense frame).
3. **LLM on its own:** classifies unusual phrasings and answers anything else.
4. **Safety-shield follow-ups:** one playful line after the shield acts, from the shield's facts.

### 12.3 Speaking (`qcar2_announcer.py`)

- Piper `en_US-ryan-medium` (length scale 1.1) → `paplay`; falls back to espeak-ng via `spd-say`.
- Unmutes the speaker at start and sets up **WebRTC echo cancellation** (`analog_gain_control=0`, `noise_suppression=0`).
- Two queues: `/qcar2/say_now` (urgent: replies, shield lines — never dropped) before `/qcar2/say` (normal; dropped after 12 s); `/qcar2/say_brief` expires after ~2 s. Nothing interrupts a sentence in progress.
- Says mode start, obstacle warnings, "Going to goal / Goal reached / Navigation failed / Goal cancelled".

### 12.4 Battery voice (separate from ROS)

`qcar2-battery-voice.service` (root) runs `scripts/battery_voice/qcar2_battery_voice.py`: reads the INA3221 every 2 s, median over 10 s, warns below 10.8 V and at each further 0.1 V, re-arms above 11.5 V, speaks at 100 % and **mutes** all other speech while it talks. Reinstall after edits: `sudo scripts/battery_voice/install_battery_voice.sh`.

---

## 13. Operator interfaces

### 13.1 Browser console (default) — `qcar2_web_gui.py` + `web/index.html`

- URL: `http://<car-ip>:8080` (on the car: `http://localhost:8080`; with mic: SSH tunnel `ssh -N -L 18080:127.0.0.1:8080 nvidia@<car-ip>` → `http://localhost:18080`).
- Map (2D and 3D tilt with the car's mesh), global/local costmaps (local transformed from odom into map frame), LiDAR, green plan, car pose, goal, follow mode.
- **Set Pose / Set Goal** by click-drag, nav speed and steering-limit sliders, **E-STOP**, STOP ALL, safety-shield toggle.
- Cameras: front (RealSense with distances) + "SHOW 360°" for the CSI set; **SHOW DETECTIONS** (green = placed on map, amber = no LiDAR range).
- Mapping: hold-to-drive pad (buttons or W/A/S/D, diagonals, Space = latched E-STOP, absolute authority over auto-explore), Save Map, auto-explore panel (coverage, gaps, VLM hints, PAUSE, UNDO glass).
- Navigation: Objects panel (click to go), Ask box, MIC on/off + source, voice transcript, battery voltage, MOTOR NOT RESPONDING warning.
- All control commands go over a dedicated `/control` WebSocket and are acknowledged with a toast.

### 13.2 RViz (debug, `use_rviz:=true`)

`rviz/qcar2_full_gui.rviz`: Map, LaserScan, global/local costmaps (`Draw Behind` set to avoid z-fighting), global/local plan, path-direction arrows and motion arrow from `qcar2_nav_visualizer.py`, footprint, robot model. Camera displays removed — this machine's OpenGL crashes RViz with extra render windows.

### 13.3 Legacy Tk windows

`qcar2_drive_gui.py` (hold-to-drive) and `qcar2_speed_limit_gui.py` (speed/steer sliders, E-STOP, STOP ALL) — opt-in only.

### 13.4 Remote access

SSH, VS Code Remote-SSH, xrdp (separate XFCE session), VNC to `:5900` (the live GNOME screen). The car's Wi-Fi IP changes with network/reboot: `ip -4 -o addr show wlan0`.

---

## 14. Every node, at a glance

### C++ (`qcar2_nodes`)

| Executable | Role |
|---|---|
| `qcar2_hardware` | HIL I/O at 1 kHz; publishes `/qcar2_joint`, `/qcar2_imu`, `/qcar2_battery`; 15 ms speed loop; steering bias learn + persist; gyro calibration; clearance cap; shield override; E-STOP; LEDs; shutdown hooks; speed CSV |
| `lidar` | RPLIDAR driver → `/scan`; spins the motor down on graceful exit |
| `csi` | one per CSI camera (needs `DISPLAY=:0`) |
| `rgbd` | Quanser RealSense driver (unused; the official `realsense2_camera` is used instead) |
| `fixed_lidar_frame` | static `base_link → base_scan` |
| `nav2_qcar2_converter` | Twist → Ackermann `MotorCommands`, watchdog, manual override, steering report |
| `qcar2_safety_shield` | people/obstacle shield (navigation) |
| `command` | Quanser's manual command helper |

### Python (`qcar2_rviz_gui/scripts`)

| Script | Role |
|---|---|
| `qcar2_web_gui.py` | browser console server |
| `wheel_imu_odometry.py` | encoder + gyro odometry |
| `qcar2_object_mapper.py` | YOLO-World + CLIP + LiDAR semantic landmarks |
| `qcar2_object_nav.py` | name → standoff Nav2 goal |
| `qcar2_explorer.py` | autonomous exploration driver |
| `qcar2_room_analyzer.py` | room outline, coverage, stop rule |
| `qcar2_explore_vlm.py` | VLM glass/doorway/gap judgement |
| `qcar2_bump_guard.py` | stall/slip detection, bump walls, obstacle clearance |
| `qcar2_depth_obstacles.py` | RealSense depth → low-obstacle scan + memory |
| `qcar2_depth_view.py` | front view with distances, clean frame for the VLM |
| `qcar2_assistant.py` | question answering (rules → VLM → LLM) |
| `qcar2_voice_command.py` | wake word + Whisper commands |
| `qcar2_announcer.py` | TTS, speaker unmute, echo cancel, speech queues |
| `qcar2_goal_heading.py` | re-aims clicked goals for Hybrid-A* |
| `qcar2_goal_reset.py` | new pose → cancel goal, clear costmaps |
| `qcar2_nav_visualizer.py` | path arrows, motion arrow, GOAL REACHED / NAV FAILED |
| `qcar2_model_joint_state.py` | URDF joints incl. live steering |
| `qcar2_pose_marker.py` | RViz car body/heading marker |
| `image_preview_throttle.py` | rate-limited camera previews |
| `qcar2_drive_gui.py`, `qcar2_speed_limit_gui.py` | legacy Tk windows |

---

## 15. Key ROS topics

| Topic | Type | Producer → consumer |
|---|---|---|
| `/scan` | LaserScan | lidar → SLAM, AMCL, costmaps, mapper, bump guard, shield, console |
| `/qcar2_joint`, `/qcar2_imu`, `/qcar2_battery` | JointState / Imu / … (1 kHz) | hardware → odometry, console |
| `/odom`, `/wheel_imu_odom` | Odometry | wheel_imu_odometry (navigation remaps to `/odom`) |
| `/map` | OccupancyGrid | Cartographer (mapping) / map_server (navigation) |
| `/cmd_vel` | Twist | velocity_smoother → converter |
| `/qcar2_motor_speed_cmd` | MotorCommands | converter (single writer) → hardware |
| `/qcar2/manual_drive_cmd`, `/qcar2/manual_drive_active` | | console drive pad → converter / explorer |
| `/qcar2_estop` | Bool (latched) | console, voice → converter, hardware |
| `/goal_pose_raw` → `/goal_pose` | PoseStamped | console/RViz → goal_heading → bt_navigator |
| `/initialpose` | PoseWithCovarianceStamped | console → AMCL, goal_reset |
| `/speed_limit` | nav2 SpeedLimit | console, explorer → controller |
| `/front/depth_scan`, `/front/depth_obstacles` | LaserScan / PointCloud2 | depth_obstacles → costmaps, bump guard |
| `/qcar2/bump_obstacles`, `/qcar2/drive_blocked`, `/qcar2/drive_stalled` | | bump guard / hardware |
| `/qcar2/obstacle_clearance` | [front, rear] m | bump guard → hardware |
| `/qcar2/shield_cmd`, `/qcar2/shield_status`, `/qcar2/shield_enable` | | shield ↔ hardware / console |
| `/qcar2/steering_report` | Vector3 (asked, sent, inferred) | converter → console |
| `/qcar2/explore_status`, `/qcar2/explore_hints`, `/qcar2/look_done` | | room analyzer / VLM ↔ explorer, console |
| `/qcar2/nav_to_object` | String | voice / console → object_nav |
| `/qcar2/say`, `/qcar2/say_now`, `/qcar2/say_brief` | String | anyone → announcer |
| `/qcar2/mic_enabled`, `/qcar2/mic_source`, `/qcar2/mic_audio` | | console → voice |
| `/<front,rear,left,right>/camera/csi_image` | Image | csi → object mapper, console |
| `/front/camera/depth_view`, `/front/camera/preview` | Image | depth_view → console, assistant |

---

## 16. Network ports and background servers

| Port | Server | Started by |
|---|---|---|
| 8080 | Web console (aiohttp: page, `/ws`, `/control`, MJPEG) | mapping & navigation launches |
| 8090 (localhost) | `llama-server` — Cosmos-Reason2 VLM | navigation (`use_llm`), auto-mapping |
| 8091 (localhost) | `whisper-server` — Whisper small.en | navigation (`use_voice`) |
| 5900 | VNC (live desktop) | system |
| 22 | SSH | system |

Systemd: `qcar2-battery-voice.service` (root, always on), `nvargus-daemon` (CSI cameras; `sudo systemctl restart nvargus-daemon` if every camera says "video format not supported").

---

## 17. Repository layout

```
Qcar-rviz/
├── overview.md              ← this file
├── README.md                ← full manual, setup, troubleshooting, bug history
├── commands.md              ← copy-paste cheat sheet
├── qcar2_ws/src/            ← ROS 2 Humble workspace (§3.2)
│   ├── qcar2_nodes/         ← C++ drivers, converter, safety shield, Nav2 + Cartographer configs, shield tests
│   └── qcar2_rviz_gui/      ← launch/, scripts/, web/, behavior_trees/, config/, rviz/
├── scripts/                 ← build.sh, env.sh, start_mapping(.sh|_auto.sh), start_navigate.sh, stop.sh, battery_voice/
├── maps/                    ← <name>.pgm/.yaml/_objects.json
├── models/                  ← YOLO-World, Vosk, Piper, Cosmos-Reason2 GGUFs (gitignored)
├── recordings/              ← run videos
├── camera_test/             ← CSI camera debugging captures
└── archive/                 ← two abandoned earlier attempts (OpenCV-only nav, empty ROS 2 scaffold)
```

Outside the repo: `~/llama.cpp`, `~/whisper.cpp`, `~/.qcar2_steering_bias`, `~/.ros/qcar2_speed_logs/`, `/usr/local/lib/qcar2/` (installed battery voice).

---

## 18. Scripts and operations

| Script | Does |
|---|---|
| `scripts/build.sh` | clean-environment `colcon build --symlink-install` (Python, launch and config edits need no rebuild; C++ does) |
| `source scripts/env.sh` | ROS 2 Humble + workspace, strips ROS 1 Noetic from the shell |
| `scripts/start_mapping.sh [name] [flags]` | manual mapping, autosave every 10 s |
| `scripts/start_mapping_auto.sh [name] [flags]` | autonomous mapping, self-finishing |
| `scripts/start_navigate.sh map:=… [flags]` | navigation on a saved map |
| `scripts/stop.sh` / `--hard` / `--lidar` | graceful SIGINT → SIGKILL after 3 s; spins the LiDAR down if it was killed |
| `qcar_save_map <name>` | save the current map by hand while mapping |

Stopping: **Ctrl+C once** and wait ~2 s. Never close the terminal (leaves the LiDAR spinning).

---

## 19. Engineering rules learned the hard way

1. Every turning parameter must trace back to **30° steering and 0.25725 m wheelbase**; never raise `wz_max` above `vx_max / 0.446`; mirror it into the velocity smoother.
2. Planner turning radius > controller radius ≥ chassis radius.
3. Any command relay needs a **watchdog**; exactly **one writer** on the motor topic.
4. Never trust the encoder for direction; never infer direction from a small speed rise.
5. This car has no brake: reverse throttle while rolling only while speed is falling.
6. Limit the command, don't correct after — throttle is clamped at the source.
7. Rigid sensor mounts must be **static** TF.
8. Never enforce a goal heading on an Ackermann car.
9. The VLM may advise, never decide alone or touch the costmap; map facts beat model answers.
10. CSI cameras need `DISPLAY=:0`; a wedged `nvargus-daemon` needs a restart.
11. Kill ROS processes by name with `pgrep -x` / anchored paths — `pgrep -f` matches compiler jobs.
12. Don't run test ROS nodes on the car while the stack is up — they join the live graph and corrupt TF/costmaps.
13. Per-run learned state persists via `rclcpp::on_shutdown`, not destructors.
14. Wheel slip, not SLAM, was the cause of smeared maps — distrust wheel translation in Cartographer.

---

## 20. Known limitations and open items

- **2D LiDAR plane:** anything not crossing ~19 cm height is invisible to the LiDAR (partly covered by RealSense depth for the front only).
- **Glass** still needs one physical bump before it becomes a firm no-go wall.
- **Front blind zone** (~5–8 cm off the bumper) is covered only by short-lived scan memory.
- **Safety shield** physics limit: a person jumping in ~5 cm ahead at 0.45 m/s still makes contact (~0.11 s latency); a brisk walker into the side from < 1 m is reported, not always avoided.
- Several recent changes (speed-loop rewrite, shield v2, glass no-go cones) were verified offline and in short live tests, not in long runs.
- The VLM can be wrong (hallucinated a bottle, over-calls doorways); camera answers are best guesses.
- "Go around the room and …" multi-step tasks are not built yet.
- CSI cameras have no published intrinsics — FOV is self-calibrated, not factory-calibrated.
- `nav2_collision_monitor` is bypassed (unexplained command drop on this build).
- Unused hardware: LCD screen, user buttons, CAN-FD, expansion header, Quanser `rgbd` driver.
