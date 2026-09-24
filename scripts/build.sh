#!/bin/bash
# Builds the ROS2 workspace inside this project (qcar2_ws).
#
# Runs the actual colcon build in a CLEAN environment (env -i), because this
# machine's default terminal has ROS1 Noetic sourced (~/.bashrc), and mixing
# ROS1 + ROS2 CMake paths makes some find_package() calls (e.g. PCL, used by
# cartographer_ros) fail even though the library is installed correctly.
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[build.sh] Building qcar2_ws in a clean ROS2-only environment (this can take a few minutes the first time)..."

env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/local/bin:/usr/local/sbin:/usr/sbin:/sbin" \
  bash -c "
    set -e
    source /opt/ros/humble/setup.bash
    cd '$PROJECT_DIR/qcar2_ws'
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  "

echo "[build.sh] Done. Now run:  source scripts/env.sh"
