# QCar2 — Quick Commands

Copy-paste cheat sheet. Full explanations are in `README.md`.
All commands assume you are in the project folder (works the same in
`Qcar-rviz-copy` — just `cd` there instead).

---

## 0. Every new terminal (do this first)

```bash
cd ~/Desktop/Qcar-rviz
scripts/build.sh
source scripts/env.sh
```
`build.sh` takes a few seconds when nothing changed, ~1 min after code changes.
Always `build.sh` **before** `source scripts/env.sh`.

---

## 1. Mapping

### Manual mapping (you drive from the browser console)
```bash
scripts/start_mapping.sh my_room
```
- Autosaves every 10 s to `maps/my_room.yaml/.pgm/_objects.json`.
- **Same name = overwrites the old map.** Use a new name to keep the old one.
- Stop: **Ctrl+C once** and wait ~2 s. The last autosave is your map.

### Autonomous mapping (car explores by itself)
```bash
scripts/start_mapping_auto.sh my_room
```
Finishes, saves and shuts down on its own (time budget 1800 s).

Options:
```bash
scripts/start_mapping_auto.sh my_room go_next_room:=true   # also go through doorways
scripts/start_mapping_auto.sh my_room use_llm:=false       # geometry only, no vision model
scripts/start_mapping_auto.sh my_room llm_size:=2B         # force the small model (default auto: 8B if present)
```

### Mapping flags (work with both)
```bash
detect_objects:=false     # no object labels (no cameras, less GPU)
sensor_fusion:=false      # LiDAR-only odometry (if wheel/IMU misbehaves)
use_realsense:=false      # no RealSense depth camera
use_cameras:=true         # all 4 CSI cameras
use_rviz:=true            # also open RViz
use_drive_gui:=true       # also open the desktop drive window
```

### Save the current map by hand (while mapping runs, from another terminal)
```bash
qcar_save_map my_room
```

---

## 2. Navigation (on a saved map)

```bash
scripts/start_navigate.sh map:=/home/nvidia/Desktop/Qcar-rviz/maps/my_room.yaml
```
Then in the console: **Set Pose** (where the car really is) → **Set Goal**.

Options:
```bash
llm_size:=2B          # smaller, faster vision model (default is 8B: better answers)
use_llm:=false        # no vision model
use_voice:=false      # no microphone / voice commands
use_rviz:=true        # also open RViz
safety_shield:=false  # start with the safety shield OFF (console button switches it anytime)
```

Saved maps:
```bash
ls maps/*.yaml
```

---

## 3. Browser console

| Where | URL |
|---|---|
| On the car | `http://localhost:8080` |
| From a laptop | `http://10.164.58.236:8080` |
| Laptop with mic (tunnel) | run the ssh line below, then `http://localhost:18080` |

> ⚠️ `10.164.58.236` = car's current IP; it changes after reboots / network switch. Check the QCar's display (or the command below) and replace it.

Car IP (run on the car):
```bash
ip -4 -o addr show wlan0 | awk '{print $4}'
```
Tunnel (run on the **laptop**, leave it open):
```bash
ssh -N -L 18080:127.0.0.1:8080 nvidia@10.164.58.236
```
> ⚠️ Replace `10.164.58.236` with the car's current IP if it changed.

Open this project in VS Code from the laptop (Remote-SSH):
```bash
code --remote ssh-remote+nvidia@10.164.58.236 /home/nvidia/Desktop/Qcar-rviz
```
> ⚠️ Replace `10.164.58.236` with the car's current IP if it changed.

Full desktop instead of VS Code:
- **Remote Desktop (RDC app)** → enter `10.164.58.236`, log in as `nvidia`. Opens a **separate XFCE desktop** (not GNOME — `~/.xsession` starts XFCE for xrdp), not the car's live screen.
- **Live screen** (the GNOME desktop on the car's monitor) → a VNC viewer to `10.164.58.236:5900`.

> ⚠️ Replace `10.164.58.236` with the car's current IP if it changed.

---

## 4. Stopping

```bash
# Normal: Ctrl+C ONCE in the launch terminal. Never close the window instead.
scripts/stop.sh            # stuck / Ctrl+C didn't work (graceful, then force)
scripts/stop.sh --hard     # something truly wedged
scripts/stop.sh --lidar    # nothing running but the LiDAR still spins
```
Still spinning after that → power-cycle the car.

---

## 5. Build

```bash
scripts/build.sh
```
Needed after C++ changes or adding new nodes. Python scripts, launch files and
configs are symlinked — edits work without a rebuild.
Section 0 runs it every time anyway, so code changes are never missed.

---

## 6. Health checks (after `source scripts/env.sh`)

```bash
ros2 node list                              # what's running
ros2 topic hz /scan                         # LiDAR streaming?
ros2 topic hz /front/camera/csi_image       # a CSI camera streaming?
ros2 topic echo --once /qcar2_battery       # battery
ros2 topic echo --once /qcar2/explore_status   # auto-mapping progress
ros2 topic info /map                        # Publisher count should be 1
```

---

## 7. Quick fixes

```bash
# Car thinks it's boxed in by "ghost" obstacles
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}

# Object labels sit left/right of the real object
ros2 param set /qcar2_object_mapper hfov_deg 110.0

# Detection makes the car stutter
ros2 param set /qcar2_object_mapper detect_rate_hz 3.0

# Every camera says "video format is not supported"
sudo systemctl restart nvargus-daemon      # then relaunch

# Car mic present?
pactl list short sources | grep hw_1_1
```

---

## 8. Files you'll edit most

| What | File |
|---|---|
| Object names / synonyms | `qcar2_ws/src/qcar2_rviz_gui/config/object_vocabulary.yaml` |
| Auto-mapping time budget | `TIME_BUDGET` at top of `scripts/start_mapping_auto.sh` |
| Nav2 speed / tuning | `qcar2_ws/src/qcar2_nodes/config/qcar2_slam_and_nav.yaml` |

---

> **Note:** `10.164.58.236` is the car's IP right now. It changes with the network
> and after reboots — check the QCar's display (or the `ip` command in section 3)
> and replace it everywhere on this page.
