#!/bin/bash
# Force-stop mapping / navigation cleanly.
#
# IMPORTANT: the LiDAR motor is spun down by rplidar_close() inside the lidar
# node, which only runs if that node exits *gracefully*. So this script sends
# SIGINT (same as Ctrl+C) first and gives the nodes time to shut down properly.
# Only processes still alive after the grace period get SIGKILLed.
#
# If anything had to be SIGKILLed, the LiDAR spin-down is run automatically
# afterwards (see spin_down_lidar below).
#
# Usage:  scripts/stop.sh          (graceful, recommended)
#         scripts/stop.sh --hard   (SIGKILL straight away, then spin the
#                                   LiDAR down)
#         scripts/stop.sh --lidar  (nothing running but the LiDAR is still
#                                   spinning, e.g. after a closed terminal:
#                                   just spin it down)
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Was 8. Every node here exits on SIGINT in about a second; anything still
# alive after 3 s is stuck, and waiting longer only delays the SIGKILL.
GRACE_SECONDS=3
HARD=0
[ "${1:-}" = "--hard" ] && HARD=1

# The RPLIDAR's motor keeps spinning until something sends it a stop command
# over serial, and that command lives in rplidar_close() at the end of the
# lidar node. A SIGKILLed node never runs it, and there is then no process
# left to kill -- so briefly re-open the device and close it PROPERLY: start
# the lidar node, let it enter its read loop, then SIGINT it.
spin_down_lidar() {
  if pgrep -x 'lidar' >/dev/null 2>&1; then
    echo "[stop.sh] A lidar node is still running. Stopping it cleanly first..."
    pkill -INT -x 'lidar'
    sleep 5
    pkill -9 -x 'lidar' 2>/dev/null
    sleep 1
  fi
  echo "[stop.sh] Re-opening the LiDAR so it can be shut down properly..."
  env -i HOME="$HOME" USER="$USER" \
    PATH="/usr/bin:/bin:/usr/local/bin:/usr/local/sbin:/usr/sbin:/sbin" \
    bash -c "
      source /opt/ros/humble/setup.bash
      source '$PROJECT_DIR/qcar2_ws/install/setup.bash'
      ros2 run qcar2_nodes lidar > /tmp/qcar2_stop_lidar.log 2>&1 &
      NODE_PID=\$!
      # SIGINT before the driver is in its read loop would skip the close path.
      sleep 5
      kill -INT \$NODE_PID 2>/dev/null
      for _ in \$(seq 10); do
        kill -0 \$NODE_PID 2>/dev/null || break
        sleep 1
      done
      kill -9 \$NODE_PID 2>/dev/null
    "
  echo "[stop.sh] LiDAR spin-down done. If it is STILL spinning, power-cycle the QCar2"
  echo "          (the motor state lives in the device)."
}

if [ "${1:-}" = "--lidar" ]; then
  spin_down_lidar
  exit 0
fi

# Matched against the process NAME only (pgrep -x), never the full command
# line. Matching command lines with `pgrep -f` is dangerous here: while
# compiling this workspace, the compiler's arguments contain strings like
# "qcar2_hardware.cpp", so a -f match would SIGINT an in-progress build.
EXEC_NAMES=(
  'qcar2_hardware' 'lidar' 'fixed_lidar_frame' 'csi'
  'nav2_qcar2_converter' 'nav2_qcar_command_convert'
  'rf2o_laser_odometry_node' 'cartographer_node'
  'cartographer_occupancy_grid_node' 'robot_state_publisher'
  'controller_server' 'planner_server' 'bt_navigator' 'behavior_server'
  'smoother_server' 'velocity_smoother' 'waypoint_follower'
  'collision_monitor' 'lifecycle_manager' 'map_server' 'amcl' 'rviz2'
  'llama-server' 'whisper-server'
)

# Python nodes all run as "python3", so they must be matched on the command
# line. These patterns are anchored to this project's install path so they
# cannot match a compiler, an editor, or anyone else's script.
CMDLINE_PATTERNS=(
  "qcar2_rviz_gui/qcar2_nav_visualizer\.py"
  "qcar2_rviz_gui/qcar2_pose_marker\.py"
  "qcar2_rviz_gui/qcar2_model_joint_state\.py"
  "qcar2_rviz_gui/image_preview_throttle\.py"
  "qcar2_rviz_gui/qcar2_drive_gui\.py"
  "qcar2_rviz_gui/wheel_imu_odometry\.py"
  "qcar2_rviz_gui/qcar2_speed_limit_gui\.py"
  "qcar2_rviz_gui/qcar2_goal_reset\.py"
  "qcar2_rviz_gui/qcar2_goal_heading\.py"
  "qcar2_rviz_gui/qcar2_announcer\.py"
  "qcar2_rviz_gui/qcar2_web_gui\.py"
  "qcar2_rviz_gui/qcar2_object_mapper\.py"
  "qcar2_rviz_gui/qcar2_object_nav\.py"
  "qcar2_rviz_gui/qcar2_voice_command\.py"
  "qcar2_rviz_gui/qcar2_explorer\.py"
  "qcar2_rviz_gui/qcar2_assistant\.py"
  "bin/ros2 launch qcar2_rviz_gui"
)

collect_pids() {
  local pids=()
  local pid

  for name in "${EXEC_NAMES[@]}"; do
    while read -r pid; do
      [ -n "$pid" ] && pids+=("$pid")
    done < <(pgrep -x -- "$name" 2>/dev/null)
  done

  for pattern in "${CMDLINE_PATTERNS[@]}"; do
    while read -r pid; do
      # Never target this script or its own subshell.
      [ -n "$pid" ] && [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ] && pids+=("$pid")
    done < <(pgrep -f -- "$pattern" 2>/dev/null)
  done

  printf '%s\n' "${pids[@]:-}" | sort -u | sed '/^$/d'
}

PIDS=$(collect_pids)
if [ -z "$PIDS" ]; then
  echo "[stop.sh] Nothing running."
  exit 0
fi

echo "[stop.sh] Found these QCar2 processes:"
for pid in $PIDS; do
  echo "   $pid  $(ps -p "$pid" -o comm= 2>/dev/null)"
done

if [ "$HARD" -eq 1 ]; then
  echo "[stop.sh] --hard: sending SIGKILL immediately."
  # shellcheck disable=SC2086
  kill -9 $PIDS 2>/dev/null
  sleep 1
  spin_down_lidar
  exit 0
fi

echo "[stop.sh] Sending SIGINT (clean shutdown, spins the LiDAR down)..."
# shellcheck disable=SC2086
kill -INT $PIDS 2>/dev/null

# Poll every 0.25 s rather than 1 s: most nodes exit well inside a second,
# and waiting out whole seconds is what made stopping feel sluggish.
for i in $(seq $((GRACE_SECONDS * 4))); do
  sleep 0.25
  REMAINING=$(collect_pids)
  if [ -z "$REMAINING" ]; then
    echo "[stop.sh] All nodes exited cleanly after $(awk "BEGIN{print $i/4}")s. LiDAR spun down."
    exit 0
  fi
done

REMAINING=$(collect_pids)
echo "[stop.sh] These did not exit within ${GRACE_SECONDS}s, forcing SIGKILL:"
LIDAR_KILLED=0
for pid in $REMAINING; do
  name="$(ps -p "$pid" -o comm= 2>/dev/null)"
  echo "   $pid  $name"
  [ "$name" = "lidar" ] && LIDAR_KILLED=1
done
# shellcheck disable=SC2086
kill -9 $REMAINING 2>/dev/null
sleep 0.2

# Only the lidar node's OWN clean exit spins the motor down, so the (10 s)
# spin-down is needed only if that process was the one force-killed -- not
# because some unrelated Python node was slow to exit.
if [ "$LIDAR_KILLED" -eq 1 ]; then
  echo "[stop.sh] The lidar node had to be killed, so the LiDAR may still be spinning."
  spin_down_lidar
else
  echo "[stop.sh] Done (LiDAR exited cleanly and is spun down)."
fi
