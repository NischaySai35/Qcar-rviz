#!/bin/bash
# Copyright 2026 <Excelfore>
# Provides: start_esync.sh
# Short-Description: Installs and starts eSync with the VIN given on the command line.
# Usage: ./start_esync.sh <VIN>

red=196; green=40; cyan=87; normal=0

printc() {
    local color=$1; shift
    local fmt=$1; shift
    printf "\033[38;5;${color}m${fmt}\033[0m\n" "$@"
}

THIS_DIR="$(realpath "$(dirname "$0")")"

if [ $# -ne 1 ]; then
    echo "Usage: $0 <VIN>"
    echo "e.g., $0 96C0AF9AA1354BD7BB486EA8B95F8505"
    exit 1
fi

VIN="$1"
if ! [[ "$VIN" =~ ^[A-Za-z0-9_-]{1,64}$ ]]; then
    printc $red "ERROR: invalid VIN '$VIN' (use only letters, digits, '-' or '_', max 64 characters)"
    exit 1
fi

if [ ! -f "$THIS_DIR/sdk.cfg" ] || [ ! -x "$THIS_DIR/install_esync.sh" ]; then
    printc $red "ERROR: sdk.cfg / install_esync.sh not found in $THIS_DIR"
    exit 1
fi
if [ ! -f "$THIS_DIR/provision-key.json" ]; then
    printc $red "ERROR: provision-key.json not found in $THIS_DIR"
    exit 1
fi

. "$THIS_DIR/sdk.cfg"
export VIN

ESYNC_BIN="$ESYNC_INSTALL_DIR/esync/bin"
ESYNC_PROCS="esync-boot|esyncbus|esyncclient|agent|esync-uds-agent|policyagent|selfupdateagent|esync-bridge|edatx-can-da|log_aggregator|work-ua"


check_foreign_esync() {
    local foreign
    foreign="$(ps -eo user=,pid=,stat=,comm= | awk -v me="$(id -un)" -v re="^($ESYNC_PROCS)$" \
        '$4 ~ re && ($1 != me || $3 ~ /Z/)')"
    if [ -n "$foreign" ]; then
        printc $red "ERROR: eSync processes from another user, container or a dead install are running:"
        echo "$foreign"
        printc $red "Stop them first (e.g. 'sudo pkill esync-boot', or 'docker stop <container>' for Docker), then run this script again."
        exit 1
    fi
}

stop_all_esync() {
    "$THIS_DIR/install_esync.sh" -p -y >/dev/null 2>&1 || true
    pkill -u "$(id -un)" -f "^$ESYNC_BIN/" 2>/dev/null || true
    sleep 2
    pkill -9 -u "$(id -un)" -f "^$ESYNC_BIN/" 2>/dev/null || true
}

check_foreign_esync

if [ -d "$ESYNC_INSTALL_DIR/esync" ]; then
    current_vin="$(cat "$ESYNC_RUN_DIR/etc/.custom_vin" 2>/dev/null)"
    if [ "$current_vin" = "$VIN" ]; then
        printc $cyan "eSync already installed with VIN $VIN, restarting..."
        stop_all_esync
        "$THIS_DIR/install_esync.sh" -s -y || { printc $red "ERROR: eSync failed to start. Logs: $ESYNC_RUN_DIR/log/"; exit 1; }
        exit 0
    fi
    printc $cyan "eSync installed with VIN '${current_vin:-unknown}', reinstalling with VIN $VIN..."
    stop_all_esync
    "$THIS_DIR/install_esync.sh" -U -y || { printc $red "ERROR: uninstall failed"; exit 1; }
fi

printc $cyan "Installing and starting eSync with VIN: $VIN"
"$THIS_DIR/install_esync.sh" -i -y || { printc $red "ERROR: eSync installation failed. Logs: $ESYNC_RUN_DIR/log/"; exit 1; }
