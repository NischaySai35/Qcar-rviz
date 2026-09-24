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

echo "[env.sh] ROS2 Humble + qcar2_ws sourced for this shell."
echo "[env.sh] Maps directory: $QCAR_RVIZ_MAPS_DIR"
