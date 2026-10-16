#!/bin/bash
# Create the virtual CAN interface vcan0 now and on every boot.
#
# The OTA / cloud team reads vehicle data from SocketCAN interface vcan0 on
# the board. qcar2_can_bridge.py (started by every launch) writes the car's
# telemetry there; the frame layout is in
# qcar2_ws/src/qcar2_rviz_gui/config/qcar2_vcan.dbc.
#
# The vcan kernel module already ships with this L4T kernel
# (/lib/modules/$(uname -r)/kernel/drivers/net/can/vcan.ko) and can-utils
# (candump/cansend) is already installed, so nothing is downloaded.
#
# Usage:  sudo scripts/setup_vcan.sh            install + bring up vcan0
#         sudo scripts/setup_vcan.sh --remove   undo everything
# Check:  ip -details link show vcan0 ;  candump vcan0
set -eu

IFACE=vcan0
UNIT=/etc/systemd/system/vcan0.service
MODCONF=/etc/modules-load.d/vcan.conf

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0 $*" >&2
  exit 1
fi

if [ "${1:-}" = "--remove" ]; then
  systemctl disable --now vcan0.service 2>/dev/null || true
  rm -f "$UNIT" "$MODCONF"
  systemctl daemon-reload
  ip link del "$IFACE" 2>/dev/null || true
  echo "vcan0 removed."
  exit 0
fi

# Load the module at boot.
echo vcan > "$MODCONF"
modprobe vcan

# A oneshot unit creates and raises vcan0 at boot. txqueuelen 1000: the
# default of 10 drops frames when the bridge sends a burst of messages.
cat > "$UNIT" <<EOF
[Unit]
Description=Virtual CAN interface ${IFACE} (QCar2 telemetry for OTA)
After=systemd-modules-load.service
Before=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/sbin/modprobe vcan
ExecStart=/bin/sh -c 'ip link show ${IFACE} >/dev/null 2>&1 || ip link add dev ${IFACE} type vcan'
ExecStart=/sbin/ip link set ${IFACE} txqueuelen 1000
ExecStart=/sbin/ip link set up ${IFACE}
ExecStop=/sbin/ip link del ${IFACE}

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now vcan0.service

ip -details link show "$IFACE"
echo
echo "vcan0 is up and will be recreated on every boot."
echo "Watch the telemetry with:  candump -td vcan0"
