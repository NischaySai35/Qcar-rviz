#!/bin/bash
# Start eSync (ARM build) in Docker for the YOLO OTA drive demo.
#   - VIN qcar123: the vehicle the vendor's campaign targets.
#   - yolo_ota2/esync-campaigns is mounted at /downloads; the entrypoint
#     symlinks eSync's download dir there, so campaign packages land in the
#     folder yolo_ota2_demo.py watches.
#   - /etc/machine-id: without it esync-boot exits silently.
# Runs in the foreground (-it) so errors show here; Ctrl+C stops it cleanly.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SDK="$HERE/../docker_test/eSync_linux_SDK"

# Create it as this user first; if docker creates it, it is owned by root
# and the demo can't delete applied packages.
mkdir -p "$HERE/esync-campaigns"

docker build -t esync-sdk-arm-ota "$SDK"
docker rm -f esync-qcar123 >/dev/null 2>&1 || true
docker run -it --name esync-qcar123 \
    -v /etc/machine-id:/etc/machine-id:ro \
    -v "$HERE/esync-campaigns:/downloads" \
    esync-sdk-arm-ota qcar123
