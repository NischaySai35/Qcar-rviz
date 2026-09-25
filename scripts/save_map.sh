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
# If nothing is publishing /map there is no map to save, and map_saver_cli's
# own failure ("Failed to spin map subscription") reads like something broke
# rather than "mapping was never running". Say the useful thing instead, and
# use a distinct exit code so callers can tell "nothing to save" apart from a
# genuine save failure.
if ! ros2 topic info /map 2>/dev/null | grep -qE 'Publisher count: [1-9]'; then
    echo "[save_map.sh] Nothing is publishing /map, so there is no map to save." >&2
    echo "[save_map.sh] Start mapping first (scripts/start_mapping.sh), drive around," >&2
    echo "[save_map.sh] then run this in a SECOND terminal while it is still running." >&2
    exit 3
fi

ros2 run nav2_map_server map_saver_cli -f "$PROJECT_DIR/maps/$NAME" \
    --ros-args -p save_map_timeout:=10.0
echo "[save_map.sh] Saved: $PROJECT_DIR/maps/$NAME.yaml and .pgm"

# Ask the object mapper (if it is running) to write the semantic layer next to
# the map that was just saved, so the .pgm/.yaml/_objects.json always describe
# the same run.  Harmless when mapping was started with detect_objects:=false:
# nothing is subscribed, the publish goes nowhere, and no file is written.
if ros2 node list 2>/dev/null | grep -q '/qcar2_object_mapper'; then
    ros2 topic pub --once /qcar2/save_objects std_msgs/msg/String "{data: '$NAME'}" \
        >/dev/null 2>&1 || true
    # The write is a filesystem round-trip on the node's side, not instant.
    for _ in $(seq 20); do
        [ -f "$PROJECT_DIR/maps/${NAME}_objects.json" ] && break
        sleep 0.25
    done
    if [ -f "$PROJECT_DIR/maps/${NAME}_objects.json" ]; then
        COUNT=$(/usr/bin/python3 -c "import json,sys;print(len(json.load(open(sys.argv[1]))['objects']))" \
            "$PROJECT_DIR/maps/${NAME}_objects.json" 2>/dev/null || echo '?')
        echo "[save_map.sh] Saved: ${NAME}_objects.json ($COUNT objects)"
    else
        echo "[save_map.sh] WARNING: object mapper is running but wrote no objects file." >&2
    fi
else
    echo "[save_map.sh] (no object mapper running -- geometric map only)"
fi

echo "[save_map.sh] Navigate with:  scripts/start_navigate.sh map:=$PROJECT_DIR/maps/$NAME.yaml"
