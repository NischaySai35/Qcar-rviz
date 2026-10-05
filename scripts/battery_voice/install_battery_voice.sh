#!/bin/bash
# Installs the QCar2 low-battery voice as a boot-time service.
# Run once:  sudo ~/Desktop/Qcar-rviz/scripts/battery_voice/install_battery_voice.sh
# Remove:    sudo systemctl disable --now qcar2-battery-voice
set -e
if [ "$(id -u)" != 0 ]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
install -D -m 755 "$HERE/qcar2_battery_voice.py" /usr/local/lib/qcar2/qcar2_battery_voice.py
install -m 644 "$HERE/qcar2-battery-voice.service" /etc/systemd/system/qcar2-battery-voice.service
systemctl daemon-reload
systemctl enable --now qcar2-battery-voice
sleep 3
systemctl --no-pager status qcar2-battery-voice | head -12
echo
echo "Installed. Logs: journalctl -u qcar2-battery-voice -f"
