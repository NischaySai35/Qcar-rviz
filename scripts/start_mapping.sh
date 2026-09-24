#!/bin/bash
# MAPPING MODE: drive the car around and build a new SLAM map from the
# browser console at http://<car-ip>:8080 (drive pad, live map, Save Map).
# No desktop is needed on the car.  use_rviz:=true / use_drive_gui:=true
# additionally open the old RViz / Tk windows.
# Usage: scripts/start_mapping.sh [extra ros2 launch args...]
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Select a real interface address so the URL is reachable from another laptop.
qcar_ip="$(ip -4 -o addr show scope global | awk '$2 != "docker0" {sub(/\/.*/, "", $4); print $4; exit}')"
if [ -z "$qcar_ip" ]; then
  echo "[start_mapping.sh] No LAN IPv4 address found. Connect the Orin to Wi-Fi or Ethernet and retry." >&2
  exit 1
fi
echo "[start_mapping.sh] Console URL: http://$qcar_ip:8080"
echo "[start_mapping.sh] Use a colon before 8080 (not a dot). On the Orin: http://localhost:8080"
echo "[start_mapping.sh] Laptop localhost tunnel: ssh -N -L 18080:127.0.0.1:8080 nvidia@$qcar_ip"
echo "[start_mapping.sh] Then open http://localhost:18080 on the laptop."

USER_ARGS=("$@")

# Detect if caller already specified GUI launch args.
has_use_rviz_arg=false
has_use_drive_gui_arg=false
for arg in "${USER_ARGS[@]}"; do
  case "$arg" in
    use_rviz:=*) has_use_rviz_arg=true ;;
    use_drive_gui:=*) has_use_drive_gui_arg=true ;;
  esac
done

# Only enable GUI apps when a real, accessible desktop display is available.
# Merely inheriting DISPLAY is not enough: a remote/SSH session can have a
# stale DISPLAY value, which previously made the drive GUI exit with TclError
# while the rest of mapping continued to run.
has_display=false
if [ -n "${DISPLAY:-}" ] && command -v xdpyinfo >/dev/null 2>&1 && \
   xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
  has_display=true
elif [ -n "${WAYLAND_DISPLAY:-}" ]; then
  # Wayland compositors commonly provide XWayland for Tk, but xdpyinfo cannot
  # query a pure Wayland socket. Preserve it for the GUI in that case.
  has_display=true
fi

if [ "$has_display" = false ]; then
  [ "$has_use_rviz_arg" = false ] && USER_ARGS+=("use_rviz:=false")
  [ "$has_use_drive_gui_arg" = false ] && USER_ARGS+=("use_drive_gui:=false")
  echo "[start_mapping.sh] No DISPLAY/WAYLAND_DISPLAY found; launching headless (use_rviz:=false use_drive_gui:=false)."
  echo "[start_mapping.sh] To force GUI, start from a desktop terminal or pass use_rviz:=true use_drive_gui:=true explicitly."
fi

# Force a clean ROS2-only environment. Using -l here pulls in login hooks that
# often reintroduce ROS1/Noetic, which is exactly what breaks rviz2 and the
# QCar ROS2 nodes.
env -i HOME="$HOME" USER="$USER" DISPLAY="${DISPLAY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" TERM="${TERM:-xterm}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games:/snap/bin" \
  bash --noprofile --norc -c '
    set -e
    source "$1/scripts/env.sh"
    shift
    for arg in "$@"; do
      if [ "$arg" = "sensor_fusion:=true" ]; then
        fusion_helper="$(ros2 pkg prefix qcar2_rviz_gui)/lib/qcar2_rviz_gui/wheel_imu_odometry.py"
        if [ ! -x "$fusion_helper" ]; then
          echo "[start_mapping.sh] Fused mapping helper is missing or not executable: $fusion_helper" >&2
          echo "[start_mapping.sh] Run scripts/build.sh, then retry." >&2
          exit 1
        fi
      fi
    done
    exec ros2 launch qcar2_rviz_gui mapping.launch.py "$@"
  ' bash "$PROJECT_DIR" "${USER_ARGS[@]}"
