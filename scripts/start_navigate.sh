#!/bin/bash
# NAVIGATION MODE: localize on a saved map and click-to-drive from the
# browser console at http://<car-ip>:8080 (no desktop needed on the car).
# Pass use_rviz:=true to additionally open the old RViz window.
# Usage: scripts/start_navigate.sh [map:=/path/to/map.yaml] [extra args...]
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_DIR/scripts/env.sh"

# A map server cannot display a map until mapping has produced a YAML/PGM.
# Require the file explicitly instead of starting a misleading empty RViz.
qcar_map_file=""
for qcar_arg in "$@"; do
    case "$qcar_arg" in
        map:=*) qcar_map_file="${qcar_arg#map:=}" ;;
    esac
done
if [ -z "$qcar_map_file" ]; then
    # Use the first map in stable alphabetical order for the convenient
    # no-argument case. A user-supplied map:= argument always takes priority.
    mapfile -t qcar_maps < <(find "$PROJECT_DIR/maps" -maxdepth 1 -type f -name '*.yaml' -print | sort)
    if [ "${#qcar_maps[@]}" -gt 0 ]; then
        qcar_map_file="${qcar_maps[0]}"
        echo "[start_navigate] Using first saved map: $qcar_map_file"
        set -- "map:=$qcar_map_file" "$@"
    else
        echo "[start_navigate] No saved map exists in $PROJECT_DIR/maps. Create one with mapping mode, then run:"
        echo "  scripts/start_navigate.sh map:=$PROJECT_DIR/maps/<your_map>.yaml"
        exit 2
    fi
fi
if [ ! -f "$qcar_map_file" ]; then
    echo "[start_navigate] Map YAML not found: $qcar_map_file"
    exit 2
fi
# Select a real interface address so the URL is reachable from another laptop.
qcar_ip="$(ip -4 -o addr show scope global | awk '$2 != "docker0" {sub(/\/.*/, "", $4); print $4; exit}')"
if [ -z "$qcar_ip" ]; then
    echo "[start_navigate] No LAN IPv4 address found. Connect the Orin to Wi-Fi or Ethernet and retry." >&2
    exit 1
fi
echo "[start_navigate] Console URL: http://$qcar_ip:8080"
echo "[start_navigate] Use a colon before 8080 (not a dot). On the Orin: http://localhost:8080"
echo "[start_navigate] Laptop localhost tunnel: ssh -N -L 18080:127.0.0.1:8080 nvidia@$qcar_ip"
echo "[start_navigate] Then open http://localhost:18080 on the laptop."
ros2 launch qcar2_rviz_gui navigate.launch.py "$@"
