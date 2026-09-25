# QCar2 SLAM + Navigation RViz GUI

Everything in this folder is self-contained. Nothing outside `Qcar-rviz/` was
changed — the ROS2 driver code was **copied** in from Quanser's workspace
(`~/Documents/Quanser/5_research/sdcs/qcar2/ros2`), then wired up with a new
custom RViz layout, launch files and helper scripts.

If you just want to run it: jump to **[Quick Start](#quick-start)**.

---

## 1. Plain-language explanation

- **SLAM** = the car uses its LiDAR (spinning laser distance sensor) to draw
  a 2D floor-plan map of the room while simultaneously figuring out where it
  is on that map — like mapping a building blindfolded using a tape measure.
- **RViz** = a window that shows live ROS data. It doesn't drive or map
  anything itself — it just draws whatever topics (data streams) it's told
  to subscribe to. "Nice GUI" = a well-chosen set of these displays.
- **Nav2** = the navigation brain. Give it a map + a goal point, it plans a
  path avoiding walls and drives the car there, replanning if something
  blocks the way.
- **Costmap** = a "danger map" built from the LiDAR: cells near walls are
  marked unsafe so the path planner naturally avoids them. This is your
  **obstacle detection** — shown in RViz as a colored overlay.
- **Topic** = a named data channel, e.g. `/scan` (LiDAR), `/map` (the SLAM
  map), `/front/camera/csi_image` (front camera). RViz displays = "watch
  this topic, draw it."
- **TF** = the tree of coordinate frames (`map` → `odom` → `base_link` →
  `base_scan`) that lets ROS convert "a laser point 2m ahead" into "a point
  on the map."

---

## 2. What's in this folder

```
Qcar-rviz/
├── README.md                 <- you are here
├── qcar2_ws/                 <- a self-contained ROS2 (Humble) workspace
│   └── src/
│       ├── qcar2_nodes/       (copied) real QCar2 hardware driver: motors,
│       │                       LiDAR, CSI cameras, RealSense, Nav2<->QCar
│       │                       command converter
│       ├── qcar2_interfaces/  (copied) custom message types the driver uses
│       ├── cartographer_ros/  (copied) the SLAM engine (builds the map)
│       ├── pcl_conversions/   (copied) small build-time dependency of
│       │                       cartographer_ros that wasn't available as a
│       │                       system package for ROS2 Humble on this
│       │                       machine (see "Bugs I fixed" below)
│       ├── rf2o_laser_odometry/ (copied) estimates the car's motion from
│       │                       the LiDAR alone (the hardware driver doesn't
│       │                       publish wheel odometry on its own — see
│       │                       "Bugs I fixed" below)
│       └── qcar2_rviz_gui/    <-- NEW, written for this project:
│           ├── rviz/qcar2_full_gui.rviz   the GUI layout
│           ├── web/index.html             the browser console
│           ├── config/object_vocabulary.yaml  <- EDIT THIS to change which
│           │                               objects can be detected
│           ├── launch/                    launch files (see below)
│           └── scripts/
│               ├── qcar2_object_mapper.py   cameras + LiDAR -> named
│               │                            landmarks on the map
│               ├── qcar2_object_nav.py      "go to the air cooler" -> a goal
│               ├── qcar2_voice_command.py   wake-word speech commands
│               ├── qcar2_explorer.py        frontier auto-exploration
│               └── ...                      (the rest, see §4)
├── maps/                      <- saved maps land here (.yaml + .pgm, plus
│                                 <name>_objects.json for the object labels)
├── models/                    <- downloaded YOLO-World + Vosk weights
│                                 (gitignored; scripts/install_deps.sh fetches)
├── scripts/                   <- shell scripts, see Quick Start
└── archive/                   <- your two earlier attempts, kept for
    ├── QCar_Navigation_standalone_opencv/   reference, not used anymore
    └── SLAM_ros2_scaffold_v1/
```

Nothing under `qcar2_ws/src` was hand-edited except the new `qcar2_rviz_gui`
package — the driver/SLAM code is the original Quanser code, just copied.

### Why build on the Quanser workspace instead of your two old attempts?
- `archive/QCar_Navigation_standalone_opencv/` never used ROS or RViz at
  all — different technology, dead end for this goal.
- `archive/SLAM_ros2_scaffold_v1/` was an empty shell (no hardware driver
  code, empty `rviz/` folder) — never actually wired up.
- `~/Documents/Quanser/.../qcar2_nodes` is Quanser's real, working ROS2
  driver for this exact car (confirmed by months of run logs on this
  machine), so it's the only sound foundation to build the GUI on.

---

## 3. The RViz GUI (`qcar2_full_gui.rviz`)

| Display | Topic | What it shows |
|---|---|---|
| Map | `/map` | the SLAM map (grows live while mapping) |
| LaserScan | `/scan` | raw LiDAR points, cyan (range 12 m) |
| Global Costmap | `/global_costmap/costmap` | obstacle/inflation overlay for the whole map |
| Local Costmap | `/local_costmap/costmap` | obstacle overlay right around the car |
| Global Plan | `/plan` | the planned route: thick green ribbon |
| Local Plan | `/local_plan` | the short-term path the controller is following, amber |
| Path Direction | `/qcar2/path_markers` | arrows every 0.45 m along the route showing travel direction, plus a goal disc labelled with the remaining distance |
| Motion Indicator | `/qcar2/motion_markers` | arrow riding on the car: **green forward**, **red REVERSE**, length scales with speed, reads `STOPPED` when idle |
| Robot Footprint | `/local_costmap/published_footprint` | the car's outline |
| QCar2 Robot Model | URDF | the real QCar2 3D model |

**Point-and-click navigation**: in RViz's toolbar, click **"2D Goal Pose"**
(a.k.a. Nav2 Goal), then click-and-drag on the map where you want the car to
go (the drag direction sets the facing angle). There's also **"2D Pose
Estimate"** to tell the car where it currently is on the map if localization
gets confused, and **"Publish Point"** for marking points.

You can rearrange/resize panels however you like and RViz remembers your
layout — just re-save over `qcar2_ws/src/qcar2_rviz_gui/rviz/qcar2_full_gui.rviz`
from RViz's File > Save Config As, then re-run `scripts/build.sh` (or just
overwrite it, since it's a symlink-installed file after a `--symlink-install`
build, edits apply immediately without rebuilding).

---

## 4. Three modes

**Mapping mode** — drive around, build a brand-new map with Cartographer SLAM.
While mapping, the four cameras also **label the map with the objects they
see**, so the saved map knows where the sofa and the air cooler are.
**Autonomous mapping** — the same thing, except the car explores and drives
itself until the room is covered (`scripts/start_mapping_auto.sh`).
**Navigation mode** — load a map you already saved, and click-to-drive with
Nav2 — or say *“hey car, go to the air cooler”*.

---

## 5. Bugs found & fixed while wiring this up

While reading Quanser's original launch files, two real gaps turned up that
would have silently broken navigation:

1. **No odometry.** `qcar2_hardware` (the motor/sensor driver) never
   publishes `/odom`, but Nav2 (AMCL localization + the controller) needs
   it. Fixed by adding `rf2o_laser_odometry`, which estimates motion purely
   from consecutive LiDAR scans — no wheel encoders required.
2. **Nav2's drive commands never reached the motors.** Nav2 publishes
   velocity commands on `/cmd_vel`, but the QCar2's `nav2_qcar2_converter`
   node (which turns that into a motor command) only listens on
   `/cmd_vel_nav`. Quanser's original "official" nav launch file never
   remapped between the two. Fixed with a topic remap in our launch files.
3. **`cartographer_ros` failed to build** with a `PCL can not be found` /
   then a missing `pcl_conversions` error. Root cause: this machine's
   default terminal has ROS1 Noetic sourced, and building with both ROS1 +
   ROS2 CMake paths mixed together breaks `find_package(PCL)`. Fixed by (a)
   making `scripts/build.sh` build in a totally clean environment
   (`env -i`) instead of the ambient mixed shell, and (b) copying in the
   small `pcl_conversions` package, which isn't published for ROS2 Humble
   on this system's apt repos.
4. **`cartographer_node` crashed instantly on first real run** with
   `flag 'collect_metrics' was defined more than once`. Both `node_main.cpp`
   (compiled into `cartographer_node`) and `offline_node.cpp` (compiled into
   the shared `cartographer_ros` library that `cartographer_node` links
   against) declared a gflag with the same name — this build's gflags
   treats that as fatal. Fixed by renaming the unused one (in
   `offline_node.cpp`, only used by `cartographer_offline_node`, which we
   don't run) to `offline_collect_metrics`.
5. **All 4 CSI cameras failed** with `"video format is not supported"`.
   The driver's own default (820x410 @ 30fps) is not actually a valid
   native mode — 820x410 is only valid at 120fps per the driver's own
   documented native formats. Fixed by using 820x616 @ 80fps in
   `cameras_360.launch.py`, one of the documented valid combinations.
6. **RViz segfaulted** (`exit code -11`) a few seconds after opening, right
   after logging "Stereo is NOT SUPPORTED" six times (once per render
   window it creates: the main 3D view + one per Image display). This
   machine reports a very old/likely-software OpenGL (`3.1, GLSL 1.4`),
   and creating several camera render windows simultaneously at startup
   is a known trigger for this kind of crash on weak/remote GL setups.
   Fixed by removing the 4 camera image displays from the default RViz
   configuration. RViz now opens with just the Map/LiDAR/pose displays.
   Viewing CSI cameras inside this RViz instance is not supported on this
   computer; use a separate image viewer instead, e.g.
   `ros2 run rqt_image_view rqt_image_view /front/camera/csi_image`.
7. **Wheels kept spinning after `Ctrl+C`**, even though the LiDAR spun down
   and RViz closed cleanly. The motor throttle is a *latched* hardware
   output on the QCar2's HIL card — closing the card with `hil_close()`
   doesn't zero it, and the code never wrote a zero before exiting, so
   whatever speed was last commanded stayed applied with no process left
   alive to stop it. Fixed in `qcar2_hardware.cpp`: a `stop_motors()` call
   now runs from an `rclcpp::on_shutdown()` hook the instant `Ctrl+C` is
   received (not just in the destructor), and `speed_controller()` refuses
   to write to the card again once a stop has been issued.
8. **The car drove backward for no visible reason during autonomous
   navigation.** Nav2's default recovery behavior tree includes
   `<BackUp backup_dist="0.30">`, triggered automatically whenever the
   planner or controller fails a few times in a row — completely normal
   Nav2 behavior, but surprising if you don't expect it, and doubly so here
   because `behavior_server` publishes straight to `/cmd_vel`, bypassing
   `velocity_smoother`'s reverse-preventing `min_velocity: [0.0, ...]`
   clamp. The same stock tree also includes `<Spin>`, which an
   Ackermann-steered car can't execute at all. Fixed with a custom
   behavior tree (`qcar2_rviz_gui/behavior_trees/navigate_to_pose_ackermann.xml`)
   that drops both, keeping only the recoveries that make sense here
   (costmap clearing + wait). `navigate.launch.py` rewrites
   `bt_navigator`'s `default_nav_to_pose_bt_xml` to point at it.
9. **The local costmap's red obstacle boundary never followed the LiDAR
   correctly** — sometimes offset, sometimes rotated ~180°, and old marks
   that should have cleared stayed frozen on screen after every "Set Pose."
   The mistake: the local costmap is published in the **`odom`** frame (on
   purpose, so it rolls smoothly under the robot instead of jumping every
   time AMCL corrects itself), but `qcar2_web_gui.py` was taking that grid's
   origin and drawing it straight onto the **map**-frame view with no
   conversion. That's only correct by coincidence when `map→odom` happens to
   be near-zero (e.g. right after boot) — the instant AMCL applies a real
   correction, the local costmap patch stays exactly where it was in `odom`,
   which now points at the wrong spot in `map`. Confirmed live: the car's
   `odom→base_link` transform sat at `(0,0,0)` the entire time (it hadn't
   physically moved), while `map→odom` carried a real `-3.36 m, ~178°`
   correction from a single Set Pose — and the local costmap was frozen at
   exactly the un-corrected spot the whole time. Fixed by transforming the
   local costmap's origin (position **and** rotation) into map frame via TF
   before sending it to the browser, and teaching both the 2D canvas and 3D
   renderer to actually rotate the grid image instead of assuming it's
   always axis-aligned with the map. Also separately fixed along the way:
   `base_scan`'s TF was being published as *dynamic* at 10 Hz even though a
   bolted-down sensor mount never moves — Nav2 looks up transforms at each
   scan's exact timestamp, so a scan landing between two 10 Hz broadcasts
   got rejected as "extrapolation into the future" and silently dropped,
   which independently broke obstacle marking/clearing. Fixed by publishing
   it as a proper static transform instead (`fixed_lidar_frame.cpp`).
10. **The car received valid goals, planned valid paths, and never moved.**
    Traced the full command chain live: `controller_server` was correctly
    outputting real velocity commands, `velocity_smoother` passed them
    through fine — then `nav2_collision_monitor` (a stock Nav2 safety node,
    not our code) silently dropped every single one, including a manually
    injected all-zero test command, with no error, crash, or log line
    anywhere. Root cause not found (third-party compiled binary, no source
    to patch). Obstacle avoidance is not lost by removing it: MPPI's own
    `ObstaclesCritic` already avoids obstacles using the live local
    costmap, independently of `collision_monitor`, and is the layer that's
    actually been tuned (see items above). Fixed by pointing
    `nav2_qcar2_converter`'s remap in `navigate.launch.py` at `/cmd_vel`
    (the smoother's real output) instead of `/cmd_vel_safe`
    (`collision_monitor`'s dead output topic). If `collision_monitor` is
    ever fixed (e.g. a Nav2 version bump), point it back at `/cmd_vel_safe`.

---

## 6. Quick Start

### One-time setup: build the workspace
```bash
cd ~/Desktop/Qcar-rviz
scripts/build.sh
```
This compiles the driver, cartographer, rf2o and the new GUI package. Takes
a few minutes the first time (rebuilds are much faster).

### One-time setup: object detection + voice commands
```bash
scripts/install_deps.sh
```
Installs the CLIP text encoder, `vosk` + `sounddevice`, and downloads the
YOLO-World and speech models into `models/`. Safe to re-run — every step
checks first and skips what is already done. It deliberately installs with
`--no-deps` into `/usr/bin/python3`, because this car's `torch` is NVIDIA's
Jetson build and a dependency resolver would happily replace it with a
generic wheel that has no CUDA.

### Every terminal you open for this project, first run:
```bash
cd ~/Desktop/Qcar-rviz
source scripts/env.sh
```
(Your default terminal loads ROS1 Noetic via `~/.bashrc` — this sources
ROS2 Humble + this workspace on top, for that terminal only. It does not
change your `.bashrc`.)

### Open the console from another laptop

Use the Orin's LAN address from the launcher output:
```text
http://10.206.234.236:8080
```
The separator before `8080` is a **colon**, not a dot. If you specifically
want to use a localhost URL in the laptop browser, run this command
in a terminal on the laptop and leave it running:
```bash
ssh -N -L 18080:127.0.0.1:8080 nvidia@10.206.234.236
```
Then open `http://localhost:18080` on the laptop. `18080` is the laptop-side
port; the Orin-side console remains on port `8080`. Replace the IP if the
Orin's Wi-Fi address changes.

### A) Build a new map (Mapping mode)
```bash
scripts/start_mapping.sh
```
- `scripts/startmapping.sh` is also supported as a compatibility alias.
- The **QCar2 Map Drive Console** opens automatically. Click and hold its
  direction buttons, or hold **W/A/S/D** (or the arrow keys), to move.
  The four diagonal buttons move and turn together; the keyboard does too,
  so holding **W + D** drives forward-right and releasing only D keeps the
  car driving forward. The middle left/right buttons deliberately steer
  without throttle for checking the wheel angle.
  car stops immediately when you release. **Space** is a latched emergency
  stop; click **Reset & arm** before moving again. Start at the default low
  speed, keep the LiDAR clear of people/objects, and only drive where you
  can safely see the car. The console shows the live battery voltage and
  has separate speed/turn limits. The manual speed slider permits up to
  **0.35 m/s**, but use **0.15–0.25 m/s while mapping**; rapid motion reduces
  LiDAR scan overlap and makes SLAM more likely to lose its place. It sends native QCar motor commands
  directly while mapping, so it does not depend on Nav2 being active.
- Drive slowly around the room so the LiDAR sees all the walls you want on
  the map. Go around loops twice if you can — it helps Cartographer close the
  loop and fix drift.
- Watch the **Map** display fill in live in RViz, together with the red
  LiDAR dots and pose/TF visualization. The mapping RViz layout intentionally
  does not create camera image panels: this QCar computer's OpenGL driver
  crashes RViz when those extra render windows are present. The drive console
  is a separate desktop window, not an RViz panel.
- When you're happy with the map, **in a second terminal** (don't close the
  mapping one yet):
  ```bash
  cd ~/Desktop/Qcar-rviz && source scripts/env.sh
  scripts/save_map.sh my_room      # saves maps/my_room.yaml + .pgm
  ```
- Then Ctrl+C the mapping terminal.

Useful flags:
```bash
scripts/start_mapping.sh use_cameras:=true      # opt in to the 4 CSI cameras
scripts/start_mapping.sh use_rviz:=false        # headless, no GUI
scripts/start_mapping.sh use_drive_gui:=false   # no desktop drive console
scripts/start_mapping.sh sensor_fusion:=false   # LiDAR-only mapping (no wheel-encoder/IMU odometry)
scripts/start_mapping.sh detect_objects:=false  # skip object labelling (no cameras, no GPU load)
scripts/start_mapping.sh use_voice:=false       # do not start the voice listener at all
```

`sensor_fusion` is **on by default**: it starts Cartographer four seconds
later so the motor encoder and IMU streams are already publishing. The map
is still LiDAR-based; encoder speed plus gyro heading provide an independent
short-term motion estimate, and Cartographer is tuned not to accept large
corrections from an ambiguous scan. If an auxiliary sensor has a hardware
fault, fall back to the known-good scan-only mode with
`sensor_fusion:=false`.

The fused profile records free space out to 10 m, so an open room centre is
shown as mapped free area rather than left unknown when the closest wall is
distant.

### A2) Object labels on the map (on by default while mapping)

While you map, all four CSI cameras run an **open-vocabulary detector**
(YOLO-World) and the map gets labelled with what is in the room. Saving the
map writes a third file next to the `.pgm`/`.yaml`:

```text
maps/my_room.yaml          the usual occupancy grid metadata
maps/my_room.pgm           the usual occupancy grid
maps/my_room_objects.json  <- names + map coordinates of what was seen
```

Turn it off with `scripts/start_mapping.sh detect_objects:=false`.

**Why "open-vocabulary" matters.** A normal YOLOv8 only knows the 80 COCO
classes, and *air cooler is not one of them* — it could never label yours.
YOLO-World takes free text instead: every label is embedded with CLIP and
matched against the image. So the detectable set is just a list you edit:

```text
qcar2_ws/src/qcar2_rviz_gui/config/object_vocabulary.yaml
```
Add `"water purifier"`, `"shoe rack"`, anything. No retraining, no rebuild
(the file is symlink-installed). `synonyms:` in the same file is what lets
you *say* "fridge" and have it find the `refrigerator`.

**How a camera pixel becomes a map coordinate.** The bounding box's left and
right edges become two rays in that camera's frame — taken from the URDF
mounts (`csi_front/back/left/right`), so there are no hand-tuned per-camera
angles anywhere. Those rays define an angular sector; every LiDAR return
inside it is a candidate, and the **nearest cluster** is the object (whatever
the camera sees must be in front of whatever it cannot, so the wall behind is
correctly ignored). That cluster's centre is transformed into the map frame.

**Why the same TV is never mapped twice.** This is the part that usually goes
wrong with multiple cameras, and it is solved by *never comparing detections
as images*. Every sighting becomes a map coordinate **before** anything is
compared. So when the car turns and the TV that was in the front camera
appears in the left camera, it resolves to the same coordinate and merges
into the existing landmark — the second camera *raises the confidence*
instead of creating a duplicate. Which camera saw it is recorded as evidence
but is deliberately not part of the matching. On top of that: a per-class
distance gate (a sofa needs a wider one than a laptop), a periodic merge pass
that heals SLAM loop-closure drift, and `min_hits` sightings required before
anything is published — so a one-frame false positive never reaches the map.

**Known limitation — the 2D LiDAR plane.** The LiDAR sweeps one horizontal
plane about 19 cm off the floor. An object that never crosses that plane — a
wall-mounted TV, a rug, a picture — gets a bearing but no range, so it is
counted as *unresolved* and skipped rather than guessed at. Anything standing
on the floor maps fine. In the browser console press **SHOW DETECTIONS**:
green boxes were placed on the map, amber ones had no LiDAR range.

Tuning worth knowing (all `ros2 param` on `/qcar2_object_mapper`):

| Parameter | Default | What it does |
|---|---|---|
| `hfov_deg` | `120.0` | CSI lens horizontal FOV. Quanser publish no intrinsics and the driver emits no `CameraInfo`, so this is a **tunable, not a calibration**. If labels land consistently left/right of the real object, this is the knob. |
| `detect_rate_hz` | `6.0` | Total detections/sec across all 4 cameras (round-robin), so GPU load does not scale with camera count. |
| `min_hits` | `3` | Sightings before a landmark is published/saved. |
| `assoc_radius` | `0.60` m | How close two sightings must be to count as one object. Per-class overrides live in the vocabulary YAML. |
| `max_range` | `6.0` m | Ignore detections further away than this. |

### A3) Autonomous mapping — the car maps the room by itself

```bash
scripts/start_mapping_auto.sh              # saves maps/auto_map.*
scripts/start_mapping_auto.sh my_room      # or pick the name
```

It brings up mapping (with object detection), runs **Nav2 on the live SLAM
map**, and repeatedly drives to the best *frontier* — the boundary between
mapped and unknown space. Going to a frontier is by definition the move that
reveals the most new map, which is why this beats a fixed lawnmower pattern:
it adapts to the room's actual shape and stops on its own when there is
nothing unknown left to reach. When it finishes it **saves the map and the
objects automatically**, then shuts down cleanly (LiDAR spun down).

It reuses the *same* tuned Nav2 you drive with in navigation mode — MPPI
controller, Hybrid-A\* planner, your Ackermann behaviour tree — so all the
obstacle-avoidance tuning applies. Only the navigation half starts: no AMCL
and no map_server, because Cartographer already publishes `/map` **and** owns
`map -> odom`; starting Nav2's localization too would put two nodes in charge
of the same transform.

Ackermann realities it respects: this car cannot turn in place (min radius
~0.45 m), so frontiers closer than `min_goal_distance` are skipped, and a
frontier that fails twice is **blacklisted** so the planner is not asked the
same impossible question forever.

You stay in control the whole time at `http://<car-ip>:8080`: **E-STOP**
pauses it, the drive pad overrides it, **PAUSE EXPLORING** holds it, and
`Ctrl+C` saves the map built so far before shutting down. Default time budget
is 900 s (`TIME_BUDGET` at the top of the script).

### A4) Voice commands — "hey car, go to the air cooler"

There is a **MIC** button in the browser console. It is **off by default** —
an always-listening microphone should be something you opt into. Turn it on
and say:

```text
"hey car, go to the air cooler"      drive to a mapped object
"hey car, go to the fridge"          synonyms work
"hey car"  →  "Yes?"  →  "go to the sofa"      two-step also works
"stop"                               emergency stop (no wake phrase needed)
"hey car, cancel"                    cancel the current goal
"hey car, resume"                    release the e-stop
```

**Why it ignores your normal conversation.** Three independent filters,
because for something always listening any one alone is not enough:

1. **Wake phrase.** Nothing acts unless the utterance contains *"hey car"*.
   Saying *"I should go to the air cooler later"* to a person does nothing.
2. **Restricted grammar.** Vosk is given an explicit word list — the wake
   phrase, the command verbs and your object names — instead of open
   vocabulary, so unrelated speech cannot be force-fitted onto the nearest
   command. This also makes it noticeably faster and more accurate on the
   words that do matter.
3. **Intent match.** What survives must still parse as a command *and* resolve
   to an object on the map.

The one deliberate exception is a bare **"stop"**, which works without the
wake phrase: a spurious stop costs nothing, needing a wake phrase during an
emergency is a genuinely bad trade. Turn that off with
`emergency_stop_without_wake:=false`.

**Two microphones**, switchable in the same panel:
- **Car microphone** (default) — the QCar2's onboard mic. Talk to the car in
  the room; no browser tab needed.
- **This device's mic** — your laptop/phone streams audio to the car over the
  existing WebSocket. Better range and quality. Browsers only grant mic
  access over HTTPS or localhost, so over plain HTTP use the SSH tunnel in
  §"Open the console from another laptop" and open `http://localhost:18080`.

Both feed the same recogniser, so behaviour is identical either way.
Everything runs **offline on the car** — no audio leaves the machine.

Going to a named object computes a **standoff goal**: a free cell ~0.7 m
short of the object, on the side you are approaching from, turned to face it.
Sending the object's own coordinate would ask Nav2 to drive *into* the sofa.
If that cell is blocked, it sweeps around the object until it finds a clear
one, so a cooler wedged in a corner is still reachable. You can also just
click any object in the **Objects** panel.

### A5) Asking questions — "is there a cooler?", "how many chairs?"

There is an **Ask** box in the console (type it), and the same questions work
by voice once the mic is on. Questions are recognised as questions, so
*"how far is the sofa"* is answered rather than obeyed as an order to drive
there.

```text
"is there a cooler in the room"     -> Yes. The air cooler is 4.0 metres away, ahead and to your left.
"how many chairs are there"         -> There are three chairs.
"how far is the air cooler"         -> The air cooler is 4.0 metres away.
"where is the desk"                 -> The desk is 3.0 metres away, to your right.
"what can you see"                  -> I have mapped one air cooler, three chairs, and one desk.
"how many people are there"         -> I can see two people right now.
"how many people are there, go and check"  -> drives a sweep, then answers
```

**Answers come from the map, never from a language model.** Counts are
counted from the landmark database; distances and directions are measured
from the car's live pose. This matters: a small LLM asked *"how many chairs
are in here"* will cheerfully invent a number, and a robot that makes up
facts about its surroundings is worse than one that says it does not know.

So the assistant is honest about its limits by design:
- an object it has never seen gets *"I do not know what a helicopter is"*,
  followed by what it **has** mapped — not a guess;
- when the label is uncertain it says so: *"...though I am not certain; it
  might be a bed"*;
- **people are answered as "right now"**, from sightings in the last ~20 s,
  never from the saved map — people walk away. If the cameras have not looked
  recently it tells you the number is stale instead of passing it off as
  current, and offers to go and check.
- if it is not localised yet it says it cannot measure, rather than returning
  a meaningless distance.

**Optional local LLM.** `scripts/install_llm.sh` installs Ollama plus a small
instruct model (~2 GB, runs on the Orin, nothing leaves the car). It is
**not required** — without it the rule parser already handles the phrasings
above. Its only job is to turn unusual wording into one of the known
question types, and whatever it returns is validated against the real
intents and object names before use. It can choose the *question*; it can
never supply the *answer*.

> **Live "go and check" needs a mode where the car may drive itself** — that
> is autonomous mapping (`start_mapping_auto.sh`). In plain mapping or
> navigation mode it will answer from what it can currently see instead of
> setting off.

### A6) How the map corrects itself

Detections are not always right, so a landmark is treated as a running belief
rather than a fixed record. Three things can be revised:

| What | How |
|---|---|
| **The name** | Every sighting votes, weighted by how close it was. A chair first seen across the room as *"bed"* gets relabelled once nearer, clearer sightings outvote that. Watch for `relabelled "bed" -> "chair"` in the log. |
| **The position** | Position is an average whose accumulated weight is **capped**, so it never becomes too rigid to fix. Without that cap, a hundred distant sightings would drown out a close-up correction; with it, one good close pass pulls the landmark to where the object really is. |
| **Whether it exists at all** | If the car looks straight at where a landmark claims to be — in range, in view, and with the LiDAR showing clear space up to it — and repeatedly sees nothing, the landmark is deleted. That removes false positives and objects that have since been carried out of the room. |

The occlusion check matters: without it the car would "disprove" the sofa
every time someone walked in front of it.

Objects also carry a `label_confidence` and the runner-up labels in the saved
JSON, so you can see which ones were close calls.

**Camera FOV calibrates itself.** Quanser publish no camera intrinsics and
the CSI driver emits no `CameraInfo`, so `hfov_deg` starts as an educated
guess — and every bearing, hence every object position, is only as good as
that guess. But each detection that matched a LiDAR cluster is a free
calibration sample: the bounding box gives the object's width in *pixels*,
the LiDAR gives the same object's width in *angle*, and one divided by the
other is the focal length. The median over ~60 such samples converges on the
truth while you simply drive around, and the console shows the current value
and its source. Samples where the cluster filled the search sector are
discarded, since a clipped cluster would only re-derive the value already in
use. Disable with `auto_calibrate_hfov:=false`.

### B) Navigate on a saved map (Navigation mode)
```bash
scripts/start_navigate.sh map:=/home/nvidia/Desktop/Qcar-rviz/maps/my_room.yaml
```
(With no `map:=...` argument, the launcher uses the first `.yaml` map in
`maps/` alphabetically. A supplied `map:=...` always takes priority. If the
folder is empty, it prints a clear error; first complete a mapping run and
save one with `scripts/save_map.sh`.)

- RViz opens with the saved map loaded.
- RViz initially seeds AMCL at `(0, 0)` only so the `map` frame and saved map
  can be drawn; **do not treat that first marker location as correct**.
  First localize the car: click **"2D Pose Estimate"**, then click-and-drag
  on the saved map from the car's real location in its real heading. AMCL
  compares the live LiDAR scan against the saved walls and thereafter
  maintains the `map -> odom -> base_link` transform while the LiDAR odometry
  tracks short-term motion. Repeat the estimate if the marker is wrong.
- Click **"2D Goal Pose"**, then click+drag where you want the car to
  drive. Nav2 plans through free space, then continuously checks the live
  LiDAR local/global costmaps. A new person, chair or other object is marked
  as an obstacle, causes the controller to steer around it or stop, and
  triggers replanning when the global path is blocked.
- Top navigation speed is 0.36 m/s (3x the original 0.12 m/s tuning) —
  `vx_max` in the MPPI controller and `max_velocity`/`max_accel`/`max_decel`
  in the velocity smoother, both in `qcar2_slam_and_nav.yaml`. Max steering
  angle is 0.60 rad (slightly up from 0.50 rad), matching the range the
  manual drive console already uses safely.
- A separate small **"QCar2 • Nav Control"** window opens automatically next
  to RViz. RViz2 has no built-in slider/button widgets, so this is a
  standalone window instead — same pattern as the manual drive console.
  Launch with `use_speed_slider:=false` to skip it. It has:
  - **Speed limit slider** (10%–150%, no relaunch) — publishes to Nav2's own
    `/speed_limit` topic, which the controller applies immediately.
  - **Max steering angle slider** (0.20–0.75 rad, no relaunch) — sets
    `max_steering_rad` on `nav2_qcar_command_convert` directly, since Nav2 has
    no topic-based equivalent to `/speed_limit` for steering.
  - **EMERGENCY STOP** — stops motion only. Cancels the active Nav2 goal and
    holds the car's motor commands at zero; the LiDAR and every node keep
    running, so click it again to release and resume — no relaunch needed.
  - **STOP ALL** — full graceful shutdown (runs `scripts/stop.sh` for you),
    for when you're done and want everything, including the LiDAR, to spin
    down cleanly.

### Stopping mapping/navigation — and the spinning-LiDAR problem

**Normal stop: press `Ctrl+C` once in the launch terminal, then wait.**
Do *not* press it repeatedly, and do *not* close the terminal window.

Why this matters: the LiDAR motor is spun down by a `rplidar_close()` call at
the very end of the lidar node's main loop (`qcar2_nodes/src/lidar.cpp`). That
line only runs if the node exits **gracefully**. If the process is `SIGKILL`ed
— by closing the terminal, running `kill -9`, or by `ros2 launch` escalating
after you hammer `Ctrl+C` — that line never executes, and the motor is simply
left spinning. **Killing it harder is what causes the problem**, because the
spin state lives in the device, not in the process.

If `Ctrl+C` doesn't work or nodes are stuck, use the stop script from another
terminal. It sends `SIGINT` first and only escalates to `SIGKILL` for anything
still alive after 8 seconds:

```bash
scripts/stop.sh
```

If the LiDAR is *already* spinning after an unclean kill, no amount of further
killing will help — there is no process left to signal. Re-open the device and
close it properly instead:

```bash
scripts/stop_lidar.sh
```

That briefly restarts the lidar node and sends it a clean `SIGINT`, so
`rplidar_close()` actually runs. If it is *still* spinning after that, the only
remaining option is to power-cycle the QCar2.

```bash
scripts/stop.sh --hard   # skip the grace period; leaves the LiDAR spinning
                         # (follow with scripts/stop_lidar.sh)
```

Both scripts match ROS node **process names** only, so they are safe to run
while a `colcon build` is in progress.

### Rebuilding after you edit anything in `qcar2_rviz_gui`
```bash
scripts/build.sh
```
(Launch files and the `.rviz` config are symlink-installed, so edits there take
effect immediately. C++ changes — e.g. `lidar.cpp` or
`nav2_qcar_command_convert.cpp` — do need a real rebuild.)

---

## 7. Handy one-off commands (once `source scripts/env.sh` has been run)

```bash
ros2 topic list                      # see everything currently being published
ros2 topic hz /scan                  # check the LiDAR is actually streaming
ros2 topic echo /qcar2_battery       # check battery level
ros2 run tf2_tools view_frames       # dump the current TF tree to a PDF (debugging)
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}
                                      # if the car thinks it's boxed in by "ghost" obstacles
```

---

## 8. Troubleshooting

### Object detection / voice

- **"Object mapper is missing" or "Detector weights are missing" at launch** →
  run `scripts/install_deps.sh`. The launcher checks up front on purpose, so
  you find out before driving a whole mapping run, not at save time.
- **No objects ever get mapped** → press **SHOW DETECTIONS** in the console.
  - No boxes at all → the detector is not seeing your objects. Check the
    `Objects` panel says *detector ready*, then add better labels to
    `config/object_vocabulary.yaml` (concrete nouns work best).
  - **Amber** boxes → detected, but the LiDAR returned no range in that
    direction. That is the 2D-plane limitation: the object does not cross the
    LiDAR's horizontal plane ~19 cm off the floor. Nothing is wrong; that
    object simply cannot be placed geometrically.
  - Boxes appear but nothing is saved → fewer than `min_hits` (3) sightings.
    Drive past the object more slowly.
- **Labels land next to the object, consistently offset left or right** →
  that is the camera FOV assumption. Tune it:
  `ros2 param set /qcar2_object_mapper hfov_deg 110.0` (no intrinsics are
  published by the CSI driver, so this value is an estimate by design).
- **Detection is slow / the car stutters** → lower the total rate:
  `ros2 param set /qcar2_object_mapper detect_rate_hz 3.0`.
- **The mic button does nothing / "sounddevice not installed"** → run
  `scripts/install_deps.sh`. Check the car actually has a capture device with
  `pactl list short sources | grep -v monitor`.
- **Browser mic is refused** → browsers only allow microphone access over
  HTTPS or localhost. Use the SSH tunnel and open `http://localhost:18080`,
  or switch the source back to the car's own mic.
- **It reacts to my normal conversation** → it should not; say *"hey car"*
  first. If a bare **"stop"** is firing during chat and you would rather it
  did not, start with `use_voice:=false`, or launch the node with
  `emergency_stop_without_wake:=false`.
- **"I do not know that object"** → the spoken name did not match anything on
  the map. The console's `Objects` panel lists exactly what was mapped; add a
  `synonyms:` entry in `config/object_vocabulary.yaml` for what you naturally
  say.

### Autonomous exploration

- **The car does not move in auto mode** → it needs Nav2 *and* a map. The
  script waits for `/qcar2_explorer`; if it aborts, check `ros2 node list` for
  `bt_navigator` and `controller_server`. Also confirm E-STOP is not engaged.
- **It stops early** → it finishes when no *reachable* frontier is left.
  Frontiers that failed twice get blacklisted (Ackermann cars cannot reach
  some spots). Check `visited`/`skipped` in the Auto explore panel.
- **It never finishes** → the time budget (default 900 s) ends it. Raise
  `TIME_BUDGET` at the top of `scripts/start_mapping_auto.sh`.

### General

- **The LiDAR keeps spinning after I stopped everything** → the node was killed
  before it could run `rplidar_close()`. Run `scripts/stop_lidar.sh`, which
  re-opens the device and shuts it down properly. To avoid it next time, press
  `Ctrl+C` **once** and wait, or use `scripts/stop.sh` — closing the terminal
  or hammering `Ctrl+C` forces a `SIGKILL` that skips the spin-down. See
  "Stopping mapping/navigation" in section 6.
- **Nodes won't die / a launch is stuck** → `scripts/stop.sh` (graceful, waits
  8 s), then `scripts/stop.sh --hard` if something is truly wedged.
- **RViz is black at startup** → the old profile used `map` as its fixed
  frame, which does not exist until SLAM/localization publishes it. The
  supplied profile now starts in `base_link`, so its grid and QCar model are
  visible immediately. Close any old RViz window and relaunch using the
  project scripts (or reload `qcar2_full_gui.rviz` from RViz's File menu).
- **No map in navigation mode** → mapping has not produced a saved map yet.
  Run `scripts/save_map.sh my_room` while mapping, then start navigation with
  `scripts/start_navigate.sh map:=$PWD/maps/my_room.yaml`. The launcher now
  stops with a clear message instead of opening a misleading empty view.
- **RViz shows nothing / no TF** → make sure `scripts/start_mapping.sh` or
  `start_navigate.sh` is actually still running in its terminal and hasn't
  crashed; check that terminal's output for errors.
- **Camera panels blank** → the CSI ribbon cables can be finicky; check
  `ros2 topic hz /front/camera/csi_image` etc. Try `use_cameras:=false`
  then re-enable one at a time if one camera is misbehaving.
- **Map looks rotated/mirrored or scan doesn't line up with walls** → this
  is a LiDAR-mount TF issue; the mounting offset is set in
  `qcar2_ws/src/qcar2_nodes/src/fixed_lidar_frame.cpp` (already matches
  Quanser's tested value, shouldn't need changing).
- **Car doesn't move when you click a nav goal** → confirm
  `ros2 topic echo /cmd_vel` shows messages while a goal is active; if not,
  Nav2's controller isn't producing commands (path blocked / costmap
  covered in obstacles — try the `clear_entirely_local_costmap` command
  above).
- **"Workspace not built yet" from env.sh** → run `scripts/build.sh` first.
