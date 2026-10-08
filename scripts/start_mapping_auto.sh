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
#   4. The map + objects are AUTOSAVED to maps/<map_name>.yaml/.pgm/
#      _objects.json every 10 s (same files overwritten), and saved once
#      more when exploration FINISHES.
#   5. Then everything shuts down by itself, LiDAR spun down, and this
#      script exits -- ready for scripts/start_navigate.sh.
#
# ROOM AWARENESS: qcar2_room_analyzer.py tracks the room outline and how much
# of it is mapped, and finishes at 90 % (or 80 % once progress stalls);
# qcar2_explore_vlm.py looks through the cameras with the vision model
# (Cosmos-Reason2: the 8B once fully downloaded, else the 2B) for glass,
# doorways and gaps not worth visiting, and reads out a final report.
# Only the starting room is mapped; to follow doorways into other rooms:
#   scripts/start_mapping_auto.sh my_map go_next_room:=true
# Other switches: use_llm:=false (geometry only), llm_size:=2B|8B.
#
# Watch it from http://<car-ip>:8080 while it runs. You can take over at any
# time: the E-STOP button pauses it, the drive pad overrides it.
#
# Ctrl+C stops everything immediately (~2 s); the last autosave (<= 10 s
# old) is what you keep.
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAP_NAME="${1:-auto_map}"
if [ $# -gt 0 ]; then shift; fi
# Anything left over is passed through to the launch file.
EXTRA_ARGS=("$@")

# --- tuning -----------------------------------------------------------------
# Hard stop for exploration, seconds. 900 -> 1800 (2026-10-06): the car now
# TRIES every place (try-first) at 0.20 m/s with a camera look at each stop,
# which takes longer; it still ends sooner once everything has been tried.
TIME_BUDGET=1800
STARTUP_TIMEOUT=120      # max wait for the hardware node
NAV_TIMEOUT=90           # max extra wait for Nav2 to activate
# ----------------------------------------------------------------------------

MAP_PID=""

# Stopping = signal start_mapping.sh, which takes the whole launch down in
# ~2 s, and wait for it.
#
# SIGTERM, not SIGINT: a script started in the background (&) by another
# script has SIGINT IGNORED, and bash cannot trap a signal that was ignored
# on entry -- a SIGINT here would do nothing and the fallback would run.
cleanup() {
  if [ -n "$MAP_PID" ] && kill -0 "$MAP_PID" 2>/dev/null; then
    kill -TERM "$MAP_PID" 2>/dev/null
    for _ in $(seq 50); do kill -0 "$MAP_PID" 2>/dev/null || return 0; sleep 0.1; done
    echo "[auto] start_mapping.sh did not finish; forcing stop." >&2
    "$PROJECT_DIR/scripts/stop.sh" || true
  fi
}
trap 'echo; echo "[auto] Interrupted."; cleanup; exit 130' INT TERM

echo "[auto] Autonomous mapping -> maps/$MAP_NAME"
echo "[auto] Exploration budget: ${TIME_BUDGET}s. Watch at http://<car-ip>:8080"

# --- 1. bring the stack up ---------------------------------------------------
# setsid so the terminal's Ctrl+C reaches only THIS script; cleanup() then
# stops start_mapping.sh (and with it the whole launch).
setsid "$PROJECT_DIR/scripts/start_mapping.sh" "$MAP_NAME" \
  explore:=true \
  detect_objects:=true \
  time_budget_sec:="$TIME_BUDGET" \
  "${EXTRA_ARGS[@]}" &
MAP_PID=$!

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
      sleep 3
      break
    fi
  fi
  sleep 3
done

# --- 5. save and shut down ---------------------------------------------------
# The final save is simply the next autosave: the console rewrites
# maps/$MAP_NAME.* every 10 s, so waiting one cycle after the map settled
# guarantees the saved files include the finished room. (A separate
# map_saver run here would race the autosave writing the same files.)
saved=false
if kill -0 "$MAP_PID" 2>/dev/null; then
  echo "[auto] Final save to maps/$MAP_NAME ..."
  final_from=$(date +%s)
  sleep 11
  yaml="$PROJECT_DIR/maps/$MAP_NAME.yaml"
  [ -f "$yaml" ] && [ "$(stat -c %Y "$yaml")" -ge "$final_from" ] && saved=true
fi
trap - INT TERM
cleanup
if [ "$saved" = true ]; then
  echo "[auto] Done. Navigate on it with:"
  echo "  scripts/start_navigate.sh map:=$PROJECT_DIR/maps/$MAP_NAME.yaml"
else
  echo "[auto] Stopped. No map was saved."
fi
