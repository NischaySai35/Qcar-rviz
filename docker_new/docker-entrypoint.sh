#!/bin/bash

SDK_DIR=/opt/esync-sdk
ESYNC_BIN="$HOME/esync/bin"

if [ $# -ne 1 ]; then
    echo "Usage: docker run -d --name esync <image> <VIN>"
    exit 1
fi

stop_esync() {
    echo "Stopping eSync..."
    "$SDK_DIR/install_esync.sh" -p -y >/dev/null 2>&1
    pkill -u "$(id -un)" -f "^$ESYNC_BIN/" 2>/dev/null
    exit 0
}
trap stop_esync TERM INT

"$SDK_DIR/start_esync.sh" "$1" || exit 1

while pgrep -u "$(id -un)" -f "^$ESYNC_BIN/esync-boot" >/dev/null; do
    sleep 10 &
    wait $!
done
echo "ERROR: esync-boot is no longer running. Logs: $HOME/esync/data/log/"
exit 1
