#!/bin/bash
# AUTONOMOUS MAPPING: the car explores and maps the room by itself.
#
#   scripts/start_mapping_auto.sh [map_name] [extra ros2 launch args...]
#
# Default map_name is auto_map. What happens:
#   1. Mapping comes up exactly as scripts/start_mapping.sh does (Cartographer
#      + sensor fusion), plus the 4 cameras and object detection.
#   2. Nav2 starts on the LIVE SLAM map -- same MPPI controller, Hybrid-A*
#      planner and Ackermann behaviour tree that navigation mode uses.
#   3. qcar2_explorer.py repeatedly picks the best frontier (the boundary
#      between mapped and unknown space) and sends the car there, until the
#      room is covered, nothing reachable is left, or the time budget expires.
#   4. The map AND the detected objects are saved automatically.
#   5. Everything is shut down cleanly, including spinning the LiDAR down.
#
# Watch it from http://<car-ip>:8080 while it runs. You can take over at any
# time: the E-STOP button pauses it, the drive pad overrides it.
#
# Ctrl+C is safe at any point -- the map saved so far is written out first,
# then everything is stopped, and the car is left with no goal executing.
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAP_NAME="${1:-auto_map}"
if [ $# -gt 0 ]; then shift; fi
# Anything left over is passed through to the launch file.
EXTRA_ARGS=("$@")

# --- tuning -----------------------------------------------------------------
TIME_BUDGET=900          # hard stop for exploration, seconds
STARTUP_TIMEOUT=120      # max wait for the hardware node
NAV_TIMEOUT=90           # max extra wait for Nav2 to activate
# ----------------------------------------------------------------------------

MAP_PGID=""
SAVED=false

save_everything() {
  # Guard against running twice (Ctrl+C during the normal save path).
  if [ "$SAVED" = true ]; then return 0; fi
  SAVED=true
  # If the run aborted before mapping ever started there is nothing to save,
  # and attempting it buries the REAL error (usually a launch failure further
  # up) under a scary-looking map_saver stack. save_map.sh exits 3 for
  # "nothing to save", which is not a failure worth shouting about.
  echo "[auto] Saving map as '$MAP_NAME' ..."
  local rc=0
  "$PROJECT_DIR/scripts/save_map.sh" "$MAP_NAME" || rc=$?
  if [ "$rc" -eq 3 ]; then
    echo "[auto] Nothing was mapped, so nothing was saved."
    echo "[auto] Look further up for why the stack did not start."
  elif [ "$rc" -ne 0 ]; then
    echo "[auto] WARNING: map save failed (exit $rc)." >&2
  fi
}

cleanup() {
  save_everything
  echo "[auto] Stopping everything..."
  "$PROJECT_DIR/scripts/stop.sh" || true
  if [ -n "$MAP_PGID" ] && kill -0 "-$MAP_PGID" 2>/dev/null; then
    kill -INT "-$MAP_PGID" 2>/dev/null || true
    sleep 3
    kill -9 "-$MAP_PGID" 2>/dev/null || true
  fi
}
trap 'echo; echo "[auto] Interrupted."; cleanup; exit 130' INT TERM

echo "[auto] Autonomous mapping -> maps/$MAP_NAME"
echo "[auto] Exploration budget: ${TIME_BUDGET}s. Watch at http://<car-ip>:8080"

# --- 1. bring the stack up ---------------------------------------------------
setsid "$PROJECT_DIR/scripts/start_mapping.sh" \
  explore:=true \
  detect_objects:=true \
  time_budget_sec:="$TIME_BUDGET" \
  "${EXTRA_ARGS[@]}" &
MAP_PID=$!
MAP_PGID=$(ps -o pgid= -p "$MAP_PID" | tr -d ' ')

# env.sh sources /opt/ros/humble/setup.bash, which references unset vars;
# relax `set -u` just while sourcing it so this script does not exit early.
set +u
# shellcheck source=/dev/null
source "$PROJECT_DIR/scripts/env.sh"
set -u

# --- 2. wait for hardware ----------------------------------------------------
echo "[auto] Waiting for the QCar2 hardware node (up to ${STARTUP_TIMEOUT}s)..."
ready=false
for _ in $(seq "$STARTUP_TIMEOUT"); do
  if ! kill -0 "$MAP_PID" 2>/dev/null; then
    echo "[auto] Mapping exited during startup. Aborting." >&2
    cleanup
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -q '/qcar2_hardware'; then ready=true; break; fi
  sleep 1
done
if [ "$ready" != true ]; then
  echo "[auto] Hardware node never appeared. Aborting." >&2
  cleanup
  exit 1
fi
echo "[auto] Hardware is up."

# --- 3. wait for Nav2 + the explorer -----------------------------------------
echo "[auto] Waiting for Nav2 and the explorer (up to ${NAV_TIMEOUT}s)..."
nav_ready=false
for _ in $(seq "$NAV_TIMEOUT"); do
  if ! kill -0 "$MAP_PID" 2>/dev/null; then
    echo "[auto] Mapping exited while Nav2 was starting. Aborting." >&2
    cleanup
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -q '/qcar2_explorer'; then nav_ready=true; break; fi
  sleep 1
done
if [ "$nav_ready" != true ]; then
  echo "[auto] Explorer never started. Aborting." >&2
  cleanup
  exit 1
fi
echo "[auto] Exploring. The car is now driving itself."

# --- 4. follow progress until the explorer reports finished ------------------
# The explorer owns its own time budget; the +120s here is only a backstop in
# case its status topic stops updating, so this script can never hang forever.
DEADLINE=$(( $(date +%s) + TIME_BUDGET + 120 ))
last_report=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  if ! kill -0 "$MAP_PID" 2>/dev/null; then
    echo "[auto] Mapping stack exited."
    break
  fi
  status=$(timeout 3 ros2 topic echo --once /qcar2/explore_status std_msgs/msg/String 2>/dev/null \
           | sed -n 's/^data: //p' | tr -d "'" )
  if [ -n "$status" ]; then
    finished=$(/usr/bin/python3 -c "
import json,sys
try: d=json.loads(sys.argv[1])
except Exception: sys.exit(0)
print('yes' if d.get('finished') else 'no')
print(f\"visited={d.get('visited')} failed={d.get('failed')} elapsed={d.get('elapsed')}s\")
" "$status" 2>/dev/null)
    done_flag=$(echo "$finished" | sed -n 1p)
    detail=$(echo "$finished" | sed -n 2p)
    now=$(date +%s)
    if [ -n "$detail" ] && [ $(( now - last_report )) -ge 15 ]; then
      echo "[auto] $detail"
      last_report=$now
    fi
    if [ "$done_flag" = yes ]; then
      echo "[auto] Exploration reported complete."
      # Let the final map update and the last detections settle before saving.
      sleep 8
      break
    fi
  fi
  sleep 3
done

# --- 5. save and shut down ---------------------------------------------------
trap - INT TERM
cleanup
echo "[auto] Done. Navigate on it with:"
echo "  scripts/start_navigate.sh map:=$PROJECT_DIR/maps/$MAP_NAME.yaml"
