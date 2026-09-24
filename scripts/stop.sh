#!/bin/bash
# Force-stop mapping / navigation cleanly.
#
# IMPORTANT: the LiDAR motor is spun down by rplidar_close() inside the lidar
# node, which only runs if that node exits *gracefully*. So this script sends
# SIGINT (same as Ctrl+C) first and gives the nodes time to shut down properly.
# Only processes still alive after the grace period get SIGKILLed.
#
# Usage:  scripts/stop.sh          (graceful, recommended)
#         scripts/stop.sh --hard   (skip straight to SIGKILL - leaves the
#                                   LiDAR spinning; use scripts/stop_lidar.sh
#                                   afterwards to spin it down)
set -u

GRACE_SECONDS=8
HARD=0
[ "${1:-}" = "--hard" ] && HARD=1

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
  echo "[stop.sh] WARNING: the LiDAR will keep spinning. Run scripts/stop_lidar.sh next."
  # shellcheck disable=SC2086
  kill -9 $PIDS 2>/dev/null
  exit 0
fi

echo "[stop.sh] Sending SIGINT (clean shutdown, spins the LiDAR down)..."
# shellcheck disable=SC2086
kill -INT $PIDS 2>/dev/null

for i in $(seq "$GRACE_SECONDS"); do
  sleep 1
  REMAINING=$(collect_pids)
  if [ -z "$REMAINING" ]; then
    echo "[stop.sh] All nodes exited cleanly after ${i}s. LiDAR spun down."
    exit 0
  fi
done

REMAINING=$(collect_pids)
echo "[stop.sh] These did not exit within ${GRACE_SECONDS}s, forcing SIGKILL:"
for pid in $REMAINING; do
  echo "   $pid  $(ps -p "$pid" -o comm= 2>/dev/null)"
done
# shellcheck disable=SC2086
kill -9 $REMAINING 2>/dev/null
sleep 1

if pgrep -x 'lidar' >/dev/null 2>&1 || [ -n "$REMAINING" ]; then
  echo "[stop.sh] A node had to be killed, so the LiDAR may still be spinning."
  echo "[stop.sh] If you can still hear it: scripts/stop_lidar.sh"
fi
