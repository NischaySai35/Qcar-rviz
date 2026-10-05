#!/bin/bash
# Sets up the shell for this project. This must be SOURCED, not executed:
#   source scripts/env.sh
#
# Your default terminal sources ROS1 Noetic (~/.bashrc). This script layers
# ROS2 Humble + this workspace's build on top for the current shell only —
# it does not touch ~/.bashrc or anything outside this folder.

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# This computer's interactive shell can still source ROS 1 Noetic / catkin
# paths from ~/.bashrc, and mixing those with ROS 2 breaks RViz and the QCar
# nodes at runtime. Strip both ROS1 and catkin overlay paths before sourcing
# the ROS2 Humble environment.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
unset ROS_ETC_DIR ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROSLISP_PACKAGE_DIRECTORIES
unset LD_PRELOAD

# Remove any stale ROS1 entries from the active environment before handing
# control to ROS2. This includes the PATH, library paths, and Python path.
for qcar_env_var in PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
                    LD_LIBRARY_PATH PKG_CONFIG_PATH PYTHONPATH; do
    qcar_env_value="${!qcar_env_var:-}"
    if [ -n "$qcar_env_value" ]; then
        filtered_value=""
        while IFS= read -r entry; do
            [ -z "$entry" ] && continue
            case "$entry" in
                /opt/ros/noetic*|/home/nvidia/catkin_ws*|*/catkin_ws*|/opt/ros/noetic|/home/nvidia/catkin_ws)
                    continue
                    ;;
                *)
                    filtered_value="${filtered_value:+$filtered_value:}$entry"
                    ;;
            esac
        done <<< "$qcar_env_value"
        export "$qcar_env_var=$filtered_value"
    fi
done

# Clear any stale ROS1 package hooks before entering ROS2.
unset ROSLISP_PACKAGE_DIRECTORIES

source /opt/ros/humble/setup.bash

if [ -f "$PROJECT_DIR/qcar2_ws/install/local_setup.bash" ]; then
    source "$PROJECT_DIR/qcar2_ws/install/local_setup.bash"
else
    echo "[env.sh] Workspace not built yet. Run scripts/build.sh first."
fi

export QCAR_RVIZ_MAPS_DIR="$PROJECT_DIR/maps"

# xrdp remote desktop and SSH/VS Code terminals leave this unset, and without
# it PulseAudio and speech-dispatcher cannot be found -- the car's speaker and
# mic then fail silently (see qcar2_announcer.py). start_mapping.sh passes it
# through its env -i, so setting it here covers every launch.
if [ -z "${XDG_RUNTIME_DIR:-}" ] && [ -d "/run/user/$(id -u)" ]; then
    export XDG_RUNTIME_DIR="/run/user/$(id -u)"
fi

# Save the map being built right now (and its detected objects) as
# maps/<name>.yaml + .pgm + <name>_objects.json.
#   qcar_save_map my_room
# Used by both mapping scripts when they stop, and by the console's Save Map
# button. Returns 3 when nothing is mapping (nothing to save), so callers can
# tell that apart from a real failure.
qcar_save_map() {
    local name="${1:-qcar_map}"
    local maps="$QCAR_RVIZ_MAPS_DIR"
    mkdir -p "$maps"
    # map_saver_cli's own error for "nothing is publishing /map" reads like a
    # crash; say the useful thing instead.
    if ! ros2 topic info /map 2>/dev/null | grep -qE 'Publisher count: [1-9]'; then
        echo "[save] Nothing is publishing /map, so there is no map to save." >&2
        return 3
    fi
    # Ask the object mapper to write the objects next to the map, so the
    # .pgm/.yaml/_objects.json always describe the same run. In the
    # background, IN PARALLEL with the map save: each ros2 CLI call costs 1-2 s
    # of start-up on this board, and running them one after another is what
    # made Ctrl+C feel slow. Harmless if no object mapper is running.
    rm -f "$maps/${name}_objects.json"
    # -w 1: wait until the mapper's subscription is discovered (a fresh CLI
    # node that publishes at once can lose the message); timeout keeps it
    # from waiting forever when object detection is off.
    timeout 6 ros2 topic pub -w 1 --once /qcar2/save_objects std_msgs/msg/String \
        "{data: '$name'}" >/dev/null 2>&1 &
    local obj_pid=$!
    # 10 s, not the 2 s default: map_saver_cli is a brand-new node every run
    # and DDS discovery of /map occasionally takes longer than 2 s here.
    ros2 run nav2_map_server map_saver_cli -f "$maps/$name" \
        --ros-args -p save_map_timeout:=10.0 >/dev/null 2>&1 || { wait $obj_pid; return 1; }
    echo "[save] Saved: $maps/$name.yaml and .pgm"
    wait $obj_pid
    local _
    for _ in $(seq 8); do
        [ -f "$maps/${name}_objects.json" ] && break
        sleep 0.25
    done
    if [ -f "$maps/${name}_objects.json" ]; then
        echo "[save] Saved: ${name}_objects.json ($(/usr/bin/python3 -c \
            "import json,sys;print(len(json.load(open(sys.argv[1]))['objects']))" \
            "$maps/${name}_objects.json" 2>/dev/null || echo '?') objects)"
    else
        echo "[save] (no objects file -- object detection was off or found nothing)"
    fi
    echo "[save] Navigate with:  scripts/start_navigate.sh map:=$maps/$name.yaml"
}

echo "[env.sh] ROS2 Humble + qcar2_ws sourced for this shell."
echo "[env.sh] Maps directory: $QCAR_RVIZ_MAPS_DIR"
