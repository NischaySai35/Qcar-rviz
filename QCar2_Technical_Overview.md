# QCar2 Autonomous Indoor Vehicle: Technical Overview

| | |
|---|---|
| Document | QCar2 platform and software overview |
| Revision | 2.0 |
| Date | 2026-10-12 |
| Prepared by | Nischay Sai |
| Status | Platform data captured from the vehicle on the revision date; values marked *spec* are from vendor data sheets, all others were read from the running system |

---

## 1. Purpose and scope

This document describes the Quanser QCar2 research vehicle as currently configured, covering the compute platform, operating system and board support package, connectivity, security posture, vehicle hardware, and the autonomy software running on it. It is intended as a reference for teams planning to integrate additional functionality, in particular over-the-air (OTA) software update and cloud connectivity, and to demonstrate it on this vehicle.

Sections 2 to 7 describe the platform. Section 8 describes the vehicle hardware. Sections 9 and 10 cover OTA and cloud integration considerations specific to this vehicle. Sections 11 onward describe the application software.

---

## 2. Platform summary

| Item | Value |
|---|---|
| Vehicle | Quanser QCar2, 1/10-scale, rear-wheel drive, Ackermann steering |
| Compute module | NVIDIA Jetson AGX Orin 32GB (P3701-0004, SKU 699-13701-0004-501 rev E.0) |
| SoC | NVIDIA Tegra234 (Orin) |
| Carrier / device tree | Quanser QCar2 carrier; device tree compatible `p3737-0000+p3701-0004` (AGX Orin Developer Kit base) |
| BSP | NVIDIA JetPack 5.1.4 / Jetson Linux (L4T) R35.6.0, GCID 37391689, built 2024-08-28 |
| Kernel | Linux 5.10.216-tegra, PREEMPT, aarch64 (Quanser-modified build, package suffix `+qcar0`) |
| Operating system | Ubuntu 20.04.6 LTS (focal) |
| Root filesystem | ext4 on NVMe SSD (WD_BLACK SN770, 250 GB) |
| Power mode | MAXN (nvpmodel ID 0) |
| Connectivity | Wi-Fi 5 (802.11ac, 2.4/5 GHz) + Bluetooth, wired Ethernet, USB device-mode networking |
| Middleware | ROS 2 Humble (Fast DDS); ROS 1 Noetic also installed, not used |
| Vendor runtime | Quanser SDK / QUARC runtime 1.26.133 |
| GPU compute stack | CUDA 11.4, cuDNN 8.6.0, TensorRT 8.5.2, VPI 2.4.8 |
| TLS | OpenSSL 1.1.1f (Ubuntu patched 1.1.1f-1ubuntu2.24): TLS 1.2 and TLS 1.3 |

---

## 3. Compute module

### 3.1 Jetson AGX Orin 32GB

| Parameter | Value | Source |
|---|---|---|
| CPU | 8-core Arm Cortex-A78AE v8.2 (64-bit) | spec; 8 cores observed |
| CPU max clock | 2.2 GHz | observed 2188.8 MHz |
| GPU | NVIDIA Ampere, 1792 CUDA cores, 56 Tensor Cores | spec |
| GPU max clock | 930 MHz | observed 930.75 MHz |
| CUDA compute capability | 8.7 (sm_87) | toolchain |
| Deep-learning accelerators | 2 × NVDLA v2.0 (not used by this project) | spec |
| AI performance | 200 TOPS (INT8, sparse) | spec |
| Memory | 32 GB LPDDR5, 256-bit, 204.8 GB/s (29 GiB visible to Linux) | spec / observed |
| Swap | zram, 8 × 1.9 GB | observed |
| On-module storage | 64 GB eMMC 5.1 (59.3 GiB), not used as rootfs | spec / observed |
| Module power envelope | 15 W to 40 W | spec |
| Available power modes | MAXN (0), 15 W (1), 30 W (2), 40 W (3); currently MAXN | `/etc/nvpmodel.conf` |
| Trusted execution | OP-TEE present (`/dev/tee0`, `/dev/teepriv0`) | observed |
| TPM | none (`/dev/tpm*` absent) | observed |

### 3.2 Current resource usage

| Resource | Used by |
|---|---|
| GPU | YOLO-World + CLIP (PyTorch), Cosmos-Reason2 VLM (llama.cpp), Whisper (whisper.cpp) |
| CPU | ROS 2 nodes, Nav2 (planner, MPPI controller, costmaps), Cartographer SLAM, AMCL |
| Disk | 74 GB of 227 GB used (35 %) |

---

## 4. Storage and boot

### 4.1 Devices

| Device | Size | Role |
|---|---|---|
| `nvme0n1` (WD_BLACK SN770 250 GB) | 232.9 GiB | boot device; rootfs and all boot partitions |
| `mmcblk0` (module eMMC) | 59.3 GiB | not mounted; contents not inspected |

### 4.2 NVMe partition layout (GPT labels)

| Partition | Label | Size | Purpose |
|---|---|---|---|
| p1 | APP | 231.6 GiB | root filesystem (ext4) |
| p2 / p5 | A_kernel / B_kernel | 128 MiB each | kernel, slot A / B |
| p3 / p6 | A_kernel-dtb / B_kernel-dtb | 768 KiB each | kernel device tree, slot A / B |
| p4 / p7 | reserved_for_chain_A_user / B | 31.6 MiB each | reserved |
| p8 / p12 | recovery / recovery_alt | 80 MiB each | recovery kernel |
| p9 / p13 | recovery-dtb / recovery-dtb_alt | 512 KiB each | recovery device tree |
| p10 | RECROOTFS | 300 MiB | recovery root filesystem |
| p11 / p14 | esp / esp_alt | 64 MiB each | UEFI system partition |
| p15 | UDA | 400 MiB | user data area |

### 4.3 Boot redundancy state (from `nvbootctrl`)

| Item | State |
|---|---|
| Bootloader | UEFI; A/B slots enabled, `num_slots: 2`; current and active slot **A** |
| Capsule update status | 0 (no pending or failed capsule update) |
| Root filesystem A/B | **Not enabled** (single APP partition) |
| Kernel command line | `root=PARTUUID=… rootfstype=ext4 console=ttyTCU0,115200 console=tty0 net.ifnames=0 …` |
| Disk encryption | none (rootfs is plain ext4) |
| Secure Boot (fuse state) | **not verified**; reading fuse state requires root and was not performed |

---

## 5. Software baseline

### 5.1 BSP and system

| Component | Version |
|---|---|
| JetPack | 5.1.4 |
| L4T (Jetson Linux) | R35.6.0 (`nvidia-l4t-core 35.6.0-20240828020325`) |
| Kernel package | `nvidia-l4t-kernel 5.10.216-tegra-35.6.0-20240828020325+qcar0` |
| Bootloader package | `nvidia-l4t-bootloader 35.6.0-20240828020325` |
| Ubuntu | 20.04.6 LTS, arm64 |
| systemd / NTP | timesyncd active, clock synchronised, timezone UTC |
| Docker | 26.1.3 (service active) |
| Python (system) | 3.8.10 |

### 5.2 NVIDIA compute libraries

| Component | Version |
|---|---|
| CUDA | 11.4 (toolkit 11.4.19, nvcc 11.4) |
| cuDNN | 8.6.0.166 |
| TensorRT (libnvinfer) | 8.5.2 |
| VPI | 2.4.8 |
| OpenCV (NVIDIA build) | 4.5.4; Python `cv2` 4.12.0 |
| PyTorch | 2.1.0a0 (NVIDIA Jetson build nv23.06, CUDA enabled) |

### 5.3 Vendor and robotics software

| Component | Version |
|---|---|
| Quanser SDK, QUARC runtime, HIL runtime, `firmware-qcar2` | 1.26.133-1 |
| Quanser services (systemd) | `quarc_target_manager`, `quarc_power_monitor`, `quanser_license_manager` |
| ROS 2 Humble | `ros-base` 0.10.0, `navigation2` 1.1.12, `realsense2_camera` 4.54.1, RMW Fast DDS |
| ROS 1 Noetic | 1.5.0 (installed, sourced by default login shell; isolated from this project) |
| librealsense2 (system) | 2.49.0 |
| Cartographer ROS | vendored in workspace (built from source) |

### 5.4 Package sources (`/etc/apt/sources.list.d`)

| Source | Purpose |
|---|---|
| `ports.ubuntu.com` focal, focal-updates, focal-security, focal-backports | Ubuntu base |
| `repo.download.nvidia.com/jetson/common r35.6`, `…/ffmpeg r35.6` | NVIDIA L4T packages |
| `repo.quanser.com/debian/release` | Quanser SDK and QCar2 firmware |
| `packages.ros.org` | ROS 1 / ROS 2 |
| Docker, OSRF (Gazebo), deadsnakes PPA, editor repositories | development tooling |

No packages are held (`apt-mark showhold` is empty).

### 5.5 Support lifecycle notes

- Ubuntu 20.04 has left standard support; further security updates require Ubuntu Pro (ESM).
- OpenSSL 1.1.1 is end-of-life upstream; the Ubuntu package receives backported fixes only under the Ubuntu 20.04 support channel in use.
- JetPack 5 / L4T R35 is the last release line for Ubuntu 20.04 on Orin. Moving to JetPack 6 (Ubuntu 22.04) is a full reflash and would require Quanser support for the QCar2 kernel and carrier.

---

## 6. Connectivity

| Interface | Hardware / driver | State | Notes |
|---|---|---|---|
| Wi-Fi (`wlan0`) | Laird Connectivity 60 Series module, Marvell 88W8997 chipset, driver `lrdmwl_pcie` (Summit Linux backport 11.236.0.24), dual antenna | in use | 802.11a/b/g/n/ac (Wi-Fi 5), 2.4 GHz and 5 GHz. Observed link: 5 GHz, 80 MHz channel width, 20 dBm Tx power. Managed by NetworkManager; station (client) mode |
| Bluetooth (`hci0`) | same module, USB HCI | up | BlueZ 5.53; not used by the application |
| Ethernet (`eth0`) | Orin integrated MAC, driver `nvethernet`, RJ45 | no carrier | available for wired provisioning or test networks |
| USB device mode (`usb0`, `rndis0`, `l4tbr0`) | L4T USB gadget | down | standard L4T host-to-device networking over USB-C |
| CAN | Orin CAN controllers (`mttcan`); carrier exposes CAN-FD | not configured | driver not loaded; no `can*` interfaces |
| Cellular (LTE/5G) | — | **not fitted** | |
| GNSS | — | **not fitted** | |
| Docker bridge (`docker0`) | — | no containers running | 172.17.0.0/16 |

The vehicle has no dedicated telematics control unit. In Jetson terminology, `ttyTCU0` on the kernel command line is the *Tegra Combined UART* (serial debug console), not a telematics unit. Any TCU role in a demonstration would be performed by the Orin itself, by an attached modem, or by an external unit connected over Ethernet or CAN.

---

## 7. Security and transport

### 7.1 TLS / SSL support (OpenSSL 1.1.1f, system default configuration)

| Protocol | Available | Detail |
|---|---|---|
| SSL 2.0 | No | Removed from OpenSSL since 1.1.0; Python `ssl.HAS_SSLv2 = False` |
| SSL 3.0 | No | Not built in the Ubuntu OpenSSL package |
| TLS 1.0 / 1.1 | No (by policy) | Compiled in, but no cipher suites are permitted at Ubuntu's default security level 2 |
| TLS 1.2 | Yes | 27 cipher suites, AEAD first (ECDHE-ECDSA/RSA with AES-GCM and ChaCha20-Poly1305) |
| TLS 1.3 | Yes | `TLS_AES_256_GCM_SHA384`, `TLS_CHACHA20_POLY1305_SHA256`, `TLS_AES_128_GCM_SHA256` |

The same OpenSSL build backs Python `ssl`, `curl`/libcurl 7.68.0, OpenSSH, and `apt` HTTPS transport. GnuTLS 3.6.13 is also present. Trust store: `ca-certificates 20240203~20.04.1`. System time is NTP-synchronised, which certificate validation depends on.

### 7.2 Libraries available for client connectivity

| Need | Available on device |
|---|---|
| HTTPS / REST | Python `requests` 2.32.3, `aiohttp` 3.10.6, libcurl 7.68.0 |
| WebSocket | `aiohttp` |
| MQTT | **not installed** (no `paho-mqtt`, no `mosquitto` client) |
| Containers | Docker 26.1.3 |
| Hardware-backed key storage | OP-TEE present; no TPM. No keys are provisioned today |

### 7.3 Network-exposed services (listening on all interfaces)

| Port | Service | Notes |
|---|---|---|
| 22/tcp | OpenSSH 8.2p1 | password authentication at default (enabled); root login at default (key only) |
| 111/tcp | rpcbind | system default, not required by the application |
| 3389/tcp | xrdp | remote desktop (separate XFCE session) |
| 5900/tcp | VNC (vino) | live GNOME desktop |
| 16999, 17000/tcp | Quanser QUARC services | target manager |
| 8080/tcp | Vehicle operator console (HTTP + WebSocket + MJPEG) | **no TLS, no authentication**; present only while the application runs |

Localhost-only: 631 (CUPS), 53 (systemd-resolved), 8090 (vision-language model server), 8091 (speech-to-text server), ROS 2 ephemeral ports.

### 7.4 Current posture, summarised

- Host firewall (`ufw`) is inactive.
- The operator console is unauthenticated plain HTTP; it is designed for a trusted LAN. Remote use is through an SSH tunnel.
- Secure Boot fuse state, and whether the device has a burned PKC/SBK, has not been checked (requires root).
- Rootfs is unencrypted; there is no rootfs A/B.
- ROS 2 DDS traffic is unencrypted (SROS2 not enabled) and uses the default domain ID.

---

## 8. Vehicle hardware

### 8.1 Chassis and drivetrain (from the vehicle URDF and driver)

| Property | Value |
|---|---|
| Layout | 1/10 scale, rear-wheel drive, front Ackermann steering; cannot turn in place |
| Wheelbase | 0.25725 m |
| Track | ≈ 0.112 m |
| Wheel radius | 0.033 m |
| Steering range | ±0.5236 rad (±30°), mechanical stop |
| Minimum turning radius | 0.446 m |
| Navigation footprint | 0.43 m × 0.24 m |
| Drive motor | single brushed DC, quadrature encoder 720 × 4 counts/rev, gear ratio (13·19)/(70·30); ≈ 118,000 counts/s per 1 m/s |
| Operating speeds | 0.45 m/s forward, 0.20 m/s reverse (navigation limits) |
| Battery | 3S LiPo, 11.1 V nominal, 3300 mAh, XT60; ≈ 30 min driving |

### 8.2 Sensors

| Sensor | Model / specification | Mount (x, y, z) m in `base_link` | Interface |
|---|---|---|---|
| 2D LiDAR | Slamtec RPLIDAR A2M12, 20 Hz, ≤ 1680 points/rev, 12 m | (-0.014, 0, 0.194) | serial, 256000 baud |
| IMU | 6-axis; gyro ±250 °/s at 500 Hz, accel ±16 g at 1 kHz | (0.012, 0, 0.090) | Quanser HIL |
| Motor encoder | quadrature on drive motor | — | Quanser HIL |
| CSI cameras (×4) | IMX219-class, 160° fisheye; front, rear, left, right | front (0.198, 0, 0.110), rear (-0.165, 0, 0.110), right (0.016, -0.072, 0.110), left (0.016, 0.051, 0.110) | MIPI CSI, NVIDIA Argus |
| RGB-D camera | Intel RealSense D435 (USB 8086:0b07) | RGB (0.095, 0.034, 0.176) | USB 3 |
| Battery / current | battery voltage, electronics current, motor current; INA3221 power monitor | — | HIL analog; I²C |
| Microphones | 2 × PDM (stereo pair) on Orin DMIC2 | — | ALSA `hw:1,1`, PulseAudio |

### 8.3 Actuators and outputs

| Output | Interface | Notes |
|---|---|---|
| Steering servo | HIL channel 1000 (radians) | write-only; no steering position feedback exists |
| Drive motor PWM | HIL channel 11000 | clipped in the driver; deadband ≈ 0.03 PWM |
| Body lighting | HIL digital 11–26 | headlamps, brake, reverse, indicators (16 LEDs) |
| Underbody RGB strip | SPI | 33 LEDs |
| Speaker | 32 mm, 8 Ω, 0.1 W on Orin I2S1 | PulseAudio sink; ships muted |
| LCD | 2.7" 400 × 240 TFT | Quanser diagnostic display; not used by this project |

### 8.4 Hardware limitations relevant to software

- No steering angle sensor. Actual wheel angle is inferred from yaw rate and speed (δ = atan(L·ω/v)).
- The motor encoder does not reliably report direction; direction is derived from the sign of the applied throttle, latched only when the vehicle is near standstill.
- No mechanical brake; deceleration uses reverse throttle, which must be cut as soon as wheel speed stops falling.
- No per-wheel speed sensing, bumper switches, ultrasonics, GNSS or cellular.
- The LiDAR cannot see within ≈ 5–8 cm of the front bumper or below its scan plane (≈ 19 cm); the RealSense covers low obstacles ahead from ≈ 20–30 cm.
- The front steering has a mechanical centre offset on this unit. A learned steering bias is applied in the hardware driver and persisted across restarts.

### 8.5 Hardware usage map

| Hardware | Function in this project |
|---|---|
| LiDAR | SLAM, localisation, costmaps, object ranging, collision avoidance, safety shield |
| Encoder + IMU | wheel/IMU odometry, closed-loop speed control, stall and slip detection |
| CSI cameras | 360° object detection during mapping; visual checks during autonomous exploration |
| RealSense D435 | forward camera, low-obstacle detection, input to the vision-language model |
| Speaker / microphones | spoken status and answers; wake-word voice commands with echo cancellation |
| Battery monitor | console telemetry; spoken low-battery warnings (independent systemd service) |
| Wi-Fi | operator console, SSH and remote desktop access |

---

## 9. OTA update considerations for this vehicle

### 9.1 Update domains

| Domain | Contents | Size on disk | Delivery options | Notes |
|---|---|---|---|---|
| Bootloader / firmware | UEFI, MB1/MB2 | — | NVIDIA UEFI capsule update | A/B already active (slot A); safe rollback supported |
| Kernel + DTB | `nvidia-l4t-kernel …+qcar0` | 128 MiB partition | apt, or NVIDIA image-based OTA | Kernel is a Quanser build. An unpinned NVIDIA kernel update would replace it and could break QCar2 I/O. Kernel slots A/B exist |
| Root filesystem / OS | Ubuntu 20.04 + L4T userspace | 74 GB used | NVIDIA image-based OTA (`l4t_generate_ota_package.sh`), or apt | No rootfs A/B, so a failed rootfs update cannot be rolled back automatically |
| Vendor runtime | Quanser SDK, QUARC, `firmware-qcar2` 1.26.133 | — | apt (`repo.quanser.com`) | `firmware-qcar2` carries the vehicle microcontroller image (`/usr/lib/firmware/quanser/QCar2.qfl`); the `quanser-flash` tool is installed |
| Application | ROS 2 workspace (`qcar2_ws/install`) | 15 MB (source 12 MB) | file/archive sync, Debian package, or container | Python, launch files and configuration are symlink-installed; C++ requires a build |
| AI models | detector, ASR, TTS, VLM weights | 9.1 GB total (see §15) | delta or per-file transfer | Largest single file 4.8 GB (VLM) |
| Configuration / data | Nav2 parameters, object vocabulary, maps | < 1 MB | file sync | Changes take effect on next launch |

### 9.2 Constraints for applying an update on the vehicle

1. The vehicle must be stationary with the application stopped. Use `scripts/stop.sh`; it stops nodes gracefully so the motor output is zeroed and the LiDAR motor spins down. A forced kill leaves the LiDAR spinning.
2. Battery should be above 11.0 V. Low-battery warnings begin below 10.8 V.
3. Camera services: if CSI cameras were force-stopped, `nvargus-daemon` must be restarted before relaunch.
4. CSI camera nodes require the local display (`DISPLAY=:0`); this is handled in the launch files and must be preserved.
5. A reboot is required after kernel, bootloader or Quanser firmware updates.

### 9.3 Post-update verification

| Check | Command / indicator | Expected |
|---|---|---|
| BSP version | `cat /etc/nv_tegra_release` | expected L4T release |
| Boot slot | `nvbootctrl dump-slots-info` | expected slot active, status normal |
| LiDAR | `ros2 topic hz /scan` | ≈ 20 Hz |
| Vehicle I/O | `ros2 topic echo --once /qcar2_battery` | plausible voltage |
| Cameras | `ros2 topic hz /front/camera/csi_image` | frames arriving |
| GPU | model servers start (ports 8090/8091 respond) | healthy |
| Application | operator console reachable on port 8080 | map, scan and pose visible |

### 9.4 Gaps to close before a production-style OTA demonstration

- Enable rootfs A/B (requires reflash) or restrict the demonstration to the application and model domains.
- Pin (`apt-mark hold`) the Quanser kernel and firmware packages so a general update cannot replace them unintentionally.
- Confirm Secure Boot state and decide on update package signing and verification.
- Provision device identity (client certificate and key), ideally stored via OP-TEE.
- Add an MQTT or HTTPS client for the cloud platform; restrict exposed services (firewall, disable unused rpcbind/VNC/xrdp where not needed).

---

## 10. Cloud integration interfaces

The application uses ROS 2 internally. A cloud agent can be added as one more ROS 2 node, or as a separate process reading the operator-console WebSocket, without changes to the autonomy software.

| Data available for upload | Source | Rate |
|---|---|---|
| Battery voltage | `/qcar2_battery` | 1 kHz (decimate) |
| Vehicle pose (map frame) | TF `map → base_link` | 20–50 Hz |
| Speed, steering (commanded and inferred) | `/odom`, `/qcar2/steering_report` | 50 Hz |
| Navigation status (active, reached, failed) | `/navigate_to_pose/_action/status` | event |
| Safety events (shield, bump, stall, e-stop) | `/qcar2/shield_status`, `/qcar2/drive_blocked`, `/qcar2/drive_stalled`, `/qcar2_estop` | event |
| Map and labelled objects | `maps/<name>.pgm/.yaml/_objects.json` | on save |
| Camera snapshots | `/front/camera/preview` and CSI image topics | on request |
| Per-cycle drive diagnostics | `~/.ros/qcar2_speed_logs/speed_*.csv` | per run |

| Commands that can be accepted | Topic |
|---|---|
| Navigate to pose | `/goal_pose_raw` |
| Navigate to named object | `/qcar2/nav_to_object` |
| Emergency stop | `/qcar2_estop` |
| Speed limit | `/speed_limit` |
| Spoken message | `/qcar2/say` |

Outbound HTTPS (TLS 1.2/1.3) works with the installed libraries. MQTT requires installing a client library.

---

## 11. Application architecture

```
 Browser (operator console, port 8080)
        │  WebSocket JSON, MJPEG, control channel
        ▼
 qcar2_web_gui (rclpy + aiohttp) ─────────────── ROS 2 topics only ───────────────┐
                                                                                  │
 PERCEPTION                         LOCALISATION / MAPPING        AI SERVICES (localhost HTTP)
 lidar ──/scan                      Cartographer (mapping)        llama-server :8090  Cosmos-Reason2 VLM
 csi ×4 ──camera images             map_server + AMCL (nav)       whisper-server :8091 Whisper small.en
 RealSense ──colour + depth         wheel_imu_odometry
 object_mapper (YOLO-World + CLIP)
 depth_obstacles, bump_guard
                    │                          │
                    ▼                          ▼
              NAV2: bt_navigator → Smac Hybrid-A* → MPPI (20 Hz) → velocity_smoother
                    │ /cmd_vel
                    ▼
              nav2_qcar2_converter (Twist → steering angle + speed, 50 Hz, 300 ms watchdog)
                    │ /qcar2_motor_speed_cmd (single writer)
   safety_shield ───┤ /qcar2/shield_cmd
   bump_guard ──────┤ /qcar2/obstacle_clearance
   e-stop ──────────┤ /qcar2_estop
                    ▼
              qcar2_hardware (Quanser HIL: 1 kHz I/O, 15 ms speed loop, limits, steering bias)
                    ▼
              steering servo · drive motor · lighting
```

Design rules applied throughout:

- ROS 2 is the only transport between components; the browser communicates solely with the console node.
- Exactly one publisher drives the motor command topic at any time; all other drive sources go through the converter's override input.
- Every command relay carries a watchdog (converter 300 ms, safety shield 150 ms, clearance data 0.5 s).
- Speed and clearance limits are enforced in the hardware driver, so they apply to manual and autonomous driving alike.

---

## 12. Operating modes

| Mode | Entry point | Main components |
|---|---|---|
| Manual mapping | `scripts/start_mapping.sh <name>` | hardware, LiDAR, cameras, Cartographer (LiDAR + wheel/IMU), object labelling, operator drive pad; map autosaved every 10 s |
| Autonomous mapping | `scripts/start_mapping_auto.sh <name>` | as above, plus Nav2 on the live map, frontier explorer, room analyser, VLM assistance; saves and shuts down on completion (time budget 1800 s) |
| Navigation | `scripts/start_navigate.sh map:=<file>.yaml` | saved map, AMCL, full Nav2, safety shield, object navigation, voice commands, question answering, speech output |

Common launch arguments: `use_rviz`, `web_port`, `use_cameras`, `use_realsense`, `detect_objects`, `sensor_fusion`, `use_llm`, `llm_size` (8B / 2B / auto), `use_voice`, `safety_shield`, `go_next_room`, `time_budget_sec`.

---

## 13. Autonomy stack

### 13.1 Mapping and localisation

- **SLAM:** Cartographer 2D, 0.05 m grid. Default profile fuses LiDAR with wheel/IMU odometry (range 0.12–10 m, IMU enabled, odometry translation weight 1e3, rotation weight 1e5). A LiDAR-only profile (range 3.5 m) is retained as fallback.
- **Odometry:** encoder speed and gyro heading, with gyro bias calibrated at start-up; translation is frozen while the wheels are slipping against an obstacle.
- **Localisation (navigation):** AMCL (1500 particles) on the saved map; initial pose set by the operator.
- **Outputs per map:** `<name>.pgm`, `<name>.yaml`, `<name>_objects.json` (labelled landmarks).
- **TF tree:** `map → odom → base_link → {base_scan (static), cameras, IMU, wheels}`.

### 13.2 Navigation (Nav2)

| Element | Configuration |
|---|---|
| Global planner | SmacPlannerHybrid (Hybrid-A*), Reeds-Shepp motion model, minimum turning radius 0.55 m |
| Controller | MPPI, 20 Hz, 1000 samples × 56 steps × 0.05 s (2.8 s horizon), `vx` −0.20 to 0.45 m/s, `wz_max` 1.0 rad/s, Ackermann constraint 0.45 m |
| Costmaps | local 10 Hz (odom frame), global 5 Hz; layers: LiDAR obstacle, depth, bump, static, inflation (local 0.35 m, global 0.90 m) |
| Behaviour tree | custom Ackermann tree: no in-place spin, back-up then costmap clear as recovery, 120 s overall timeout per goal |
| Goal handling | 0.30 m position tolerance, any final heading; goal heading re-aimed to the approach direction |
| Velocity smoother | 0.45 m/s, 1.0 rad/s, accel 0.5 m/s², decel 0.7 m/s² |
| Command conversion | bicycle model, steering slew limit, 0.52 rad steering cap, manual override with dead-man timing |

Turning-radius ordering maintained in all tuning: planner 0.55 m > controller 0.45 m ≥ chassis 0.446 m.

---

## 14. Motor control and safety

| Layer | Implementation | Function |
|---|---|---|
| Path planning | Nav2 costmaps + MPPI obstacle critic | avoid known and sensed obstacles |
| Speed limits | explorer 0.20 m/s, navigation 0.45 m/s, manual 0.25 m/s default | limit requested speed |
| Safety shield | `safety_shield` (C++), navigation mode | tracks moving objects in the LiDAR scan, keeps a buffer around the vehicle (people 0.50 m front/rear, 0.30 m side), retreats smoothly, then resumes the goal |
| Clearance cap | `bump_guard` → hardware driver | caps speed by distance along the current steering arc; stops ≈ 8 cm short |
| Contact detection | `bump_guard` | detects stall or wheel slip (e.g. glass); marks a no-go line on the costmap |
| Speed loop | hardware driver | feed-forward + trim + P; throttle hard-clamped; ramped set-point; traction control; stall cut-off |
| Watchdogs | converter, shield, clearance | stale input results in stop |
| Emergency stop | console, voice ("stop"), `/qcar2_estop` | latched zero throttle, overrides all sources |
| Shutdown | ROS shutdown hook in hardware driver | zeroes motor output, persists steering bias |
| Battery warning | `qcar2-battery-voice.service` (root, independent of ROS) | spoken warnings below 10.8 V in 0.1 V steps |

The safety shield logic has an offline scenario test harness (`qcar2_nodes/test/test_shield_core.cpp`).

---

## 15. Perception and AI models

All inference runs on the vehicle. No audio or images leave the device.

| Model | Function | Runtime | Size | Latency (on vehicle) |
|---|---|---|---|---|
| YOLO-World v2 Large | open-vocabulary object detection from all cameras | Ultralytics 8.2.101, PyTorch, GPU | 90 MB | 50 ms/frame |
| YOLO-World v2 Small | fallback detector | same | 25 MB | 28 ms/frame |
| CLIP ViT-B/32 | verification of detection crops | PyTorch, GPU | ≈ 350 MB | — |
| Cosmos-Reason2 8B (Q4_K_M) + projector (F16) | vision-language model: scene description, question answering, exploration hints | llama.cpp server, CUDA, ctx 8192 | 4.8 GB + 1.1 GB | ≈ 2.4 s per image query |
| Cosmos-Reason2 2B (Q8_0) + projector (F16) | faster VLM option | same | 2.1 GB + 0.8 GB | ≈ 0.9 s scene description |
| Whisper small.en | speech-to-text after wake phrase | whisper.cpp server, CUDA | 466 MB | ≈ 0.3 s per sentence |
| Vosk small en-IN 0.4 | wake phrase and keyword spotting | Vosk/Kaldi, CPU | 55 MB | real time |
| Piper `en_US-ryan-medium` | text-to-speech | ONNX Runtime 1.14, CPU | 292 MB (incl. 4 voices) | — |

Detector evaluation (150 indoor images, project vocabulary): Large P/R 0.76/0.64; Small 0.69/0.55.

### 15.1 Object labelling

Detections from the four CSI cameras are converted to map coordinates by intersecting the bounding-box bearing sector with the nearest LiDAR cluster, using camera poses from the URDF. Sightings are fused in map coordinates, so an object seen by several cameras becomes one landmark. Labels are maintained by weighted vote; landmarks not re-observed in clear view are removed. The camera field of view is self-calibrated from LiDAR-matched detections, as the CSI driver publishes no intrinsics. The vocabulary is free text (`config/object_vocabulary.yaml`).

### 15.2 Autonomous exploration

A geometric room analyser determines the room outline, coverage and completion (≥ 90 % coverage and ≥ 85 % of floor observed at close range). The explorer drives to remaining frontiers through Nav2. The VLM provides advisory hints only (suspected glass, doorways); it cannot end the run or place obstacles without a physical contact confirming them.

### 15.3 Voice and question answering

- Wake phrase "hey car" detected by Vosk; the following sentence transcribed by Whisper. "Stop" is accepted without the wake phrase.
- Microphone source selectable: on-vehicle microphones (WebRTC echo cancellation) or the operator's browser.
- Answers are resolved in order: deterministic rules from the map and pose (counts, distances, directions, time), then the VLM with the current camera frame, then the language model alone. Map-derived answers take precedence. The language model does not command motion.

---

## 16. Operator interface and remote access

| Interface | Access | Function |
|---|---|---|
| Operator console | `http://<vehicle-ip>:8080` | 2D/3D map, costmaps, LiDAR, path, cameras, set pose/goal, drive pad, e-stop, speed limits, object list, questions, microphone control |
| Console with browser microphone | SSH tunnel to port 8080, opened as `localhost` | browsers allow microphone access only on HTTPS or localhost |
| RViz | launch with `use_rviz:=true` | engineering visualisation |
| SSH / VS Code Remote | port 22 | development |
| Remote desktop | xrdp (3389), VNC (5900) | full desktop |

---

## 17. Software components

### 17.1 Native (C++, package `qcar2_nodes`)

| Executable | Function |
|---|---|
| `qcar2_hardware` | HIL I/O at 1 kHz, sensor publication, speed loop, steering bias, limits, e-stop, safety-shield override |
| `lidar` | RPLIDAR driver |
| `csi` | CSI camera driver (one instance per camera) |
| `fixed_lidar_frame` | static LiDAR transform |
| `nav2_qcar2_converter` | Nav2 velocity to steering angle and speed, watchdog, manual override |
| `qcar2_safety_shield` | moving-object safety shield |

### 17.2 Python (package `qcar2_rviz_gui`)

| Node | Function |
|---|---|
| `qcar2_web_gui` | operator console server |
| `wheel_imu_odometry` | odometry |
| `qcar2_object_mapper`, `qcar2_object_nav` | object labelling, navigation to objects |
| `qcar2_explorer`, `qcar2_room_analyzer`, `qcar2_explore_vlm` | autonomous mapping |
| `qcar2_bump_guard`, `qcar2_depth_obstacles`, `qcar2_depth_view` | contact detection, depth obstacles, forward camera view |
| `qcar2_assistant`, `qcar2_voice_command`, `qcar2_announcer` | question answering, voice commands, speech output |
| `qcar2_goal_heading`, `qcar2_goal_reset`, `qcar2_nav_visualizer` | goal handling and visualisation helpers |

### 17.3 Workspace packages

| Package | Origin |
|---|---|
| `qcar2_nodes`, `qcar2_interfaces`, `qcar2` (URDF) | Quanser, modified |
| `qcar2_rviz_gui`, `qcar2_rviz_panels` | developed in this project |
| `cartographer_ros`, `pcl_conversions`, `rf2o_laser_odometry` | upstream, vendored |

### 17.4 Principal topics

| Topic | Content |
|---|---|
| `/scan` | LiDAR scan |
| `/qcar2_joint`, `/qcar2_imu`, `/qcar2_battery` | encoder, IMU, battery (1 kHz) |
| `/odom`, `/map` | odometry, occupancy grid |
| `/cmd_vel` → `/qcar2_motor_speed_cmd` | velocity command → vehicle command |
| `/qcar2_estop` | emergency stop (latched) |
| `/goal_pose_raw`, `/initialpose`, `/speed_limit` | navigation inputs |
| `/front/depth_scan`, `/qcar2/bump_obstacles`, `/qcar2/obstacle_clearance` | additional obstacle inputs |
| `/qcar2/shield_cmd`, `/qcar2/shield_status` | safety shield |
| `/qcar2/nav_to_object`, `/qcar2/say`, `/qcar2/say_now` | object navigation, speech |

---

## 18. Repository and operations

```
Qcar-rviz/
├── QCar2_Technical_Overview.md   this document
├── README.md           operating manual and troubleshooting
├── commands.md         command reference
├── qcar2_ws/src/       ROS 2 workspace
├── scripts/            build, environment, start, stop, battery service
├── maps/               saved maps and object labels
├── models/             model weights (not version-controlled)
└── archive/            superseded prototypes
```

External to the repository: `~/llama.cpp` (VLM server), `~/whisper.cpp` (speech-to-text server), `~/.qcar2_steering_bias`, `~/.ros/qcar2_speed_logs/`.

| Script | Function |
|---|---|
| `scripts/build.sh` | clean-environment `colcon build --symlink-install` |
| `source scripts/env.sh` | ROS 2 Humble environment (removes ROS 1 from the shell) |
| `scripts/start_mapping.sh`, `start_mapping_auto.sh`, `start_navigate.sh` | mode entry points |
| `scripts/stop.sh` (`--hard`, `--lidar`) | graceful shutdown; LiDAR spin-down |

---

## 19. Engineering notes

1. All steering and turning parameters derive from the 30° steering limit and 0.25725 m wheelbase.
2. Every command relay requires a watchdog, and only one node may write the motor command at a time.
3. Wheel speed direction must not be taken from the encoder.
4. Throttle limits are applied to the command at source rather than corrected after the fact.
5. Rigidly mounted sensor frames use static transforms.
6. No final heading is enforced at navigation goals.
7. Model outputs are advisory; map-derived facts take precedence and models do not command motion.
8. CSI cameras require the local display; a wedged camera daemon is recovered with `systemctl restart nvargus-daemon`.
9. Processes are stopped by exact name, never by pattern match, to avoid signalling concurrent builds.
10. Diagnostic ROS nodes must not be started alongside the running stack; they join the same graph.

---

## 20. Known limitations

- Objects entirely outside the LiDAR plane are only detected ahead of the vehicle (RealSense).
- Glass is confirmed only after physical contact.
- The safety shield cannot prevent contact with a person entering within ≈ 5 cm at full speed (≈ 0.11 s reaction latency).
- Recent changes to the speed loop, safety shield and exploration have been validated offline and in short vehicle trials, not in extended runs.
- VLM answers about camera content can be incorrect.
- CSI camera intrinsics are estimated, not factory calibrated.
- `nav2_collision_monitor` is bypassed due to a command-forwarding fault in this build.
- Platform items not yet verified: Secure Boot fuse state, eMMC contents, Ethernet link speed.
