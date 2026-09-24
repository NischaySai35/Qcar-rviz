#!/bin/bash
# Saves the map currently being built by mapping.launch.py (run this in a
# SEPARATE terminal while mapping is still running, before you shut it down).
# Usage: scripts/save_map.sh [map_name]   (default map_name: qcar_map)
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_DIR/scripts/env.sh"

NAME="${1:-qcar_map}"
mkdir -p "$PROJECT_DIR/maps"

# save_map_timeout defaults to 2.0s, which is a fresh-process DDS discovery
# race on this hardware: map_saver_cli starts a brand-new node every run and
# has to discover cartographer_occupancy_grid_node's /map publisher from
# scratch, which can occasionally take longer than 2s on a first contact
# between two particular nodes. That produced a reliable
# "Failed to spin map subscription" even while mapping was healthy and
# actively publishing. 10s gives discovery real room without meaningfully
# slowing down the common case where it succeeds in well under a second.
ros2 run nav2_map_server map_saver_cli -f "$PROJECT_DIR/maps/$NAME" \
    --ros-args -p save_map_timeout:=10.0
echo "[save_map.sh] Saved: $PROJECT_DIR/maps/$NAME.yaml and .pgm"
echo "[save_map.sh] Navigate with:  scripts/start_navigate.sh map:=$PROJECT_DIR/maps/$NAME.yaml"
