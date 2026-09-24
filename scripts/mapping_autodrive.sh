#!/bin/bash
# MAPPING + SCRIPTED DRIVE
# ------------------------
# 1. Starts mapping exactly like scripts/start_mapping.sh, with the live GUI
#    forced ON (RViz + the drive console visible, NOT headless).
# 2. Waits for the QCar2 hardware node to come up.
# 3. Then this script drives the car itself on a fixed schedule:
#        forward 10 s  ->  wait 5 s  ->  reverse 10 s  ->  wait 20 s  ->  stop
# 4. Cleanly stops everything (scripts/stop.sh, which also spins the LiDAR down).
#
# Usage:  scripts/mapping_autodrive.sh
#
# Ctrl+C at any point: the car is commanded to zero and everything is shut
# down cleanly.
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- tuning -----------------------------------------------------------------
DRIVE_SPEED=0.30        # m/s throttle magnitude for both directions
FORWARD_SECONDS=10
WAIT1_SECONDS=5
REVERSE_SECONDS=10
WAIT2_SECONDS=20
STARTUP_TIMEOUT=90      # max seconds to wait for the hardware node
# -------------------------------------------------------------------------

MAP_PGID=""

cleanup() {
  # Best-effort: make sure the car is not left with a latched command, then
  # bring the whole stack down gracefully so the LiDAR motor spins down.
  echo "[mapping_autodrive] Stopping everything..."
  "$PROJECT_DIR/scripts/stop.sh" || true
  if [ -n "$MAP_PGID" ] && kill -0 "-$MAP_PGID" 2>/dev/null; then
    kill -INT "-$MAP_PGID" 2>/dev/null || true
    sleep 3
    kill -9 "-$MAP_PGID" 2>/dev/null || true
  fi
}
trap 'cleanup; exit 130' INT TERM

# --- 1. bring up mapping with the GUI visible ------------------------------
echo "[mapping_autodrive] Launching mapping (RViz + drive console visible)..."
setsid "$PROJECT_DIR/scripts/start_mapping.sh" \
  use_rviz:=true use_drive_gui:=true &
MAP_PID=$!
MAP_PGID=$(ps -o pgid= -p "$MAP_PID" | tr -d ' ')

# --- 2. wait for the hardware node ---------------------------------------
echo "[mapping_autodrive] Waiting for the QCar2 hardware node (up to ${STARTUP_TIMEOUT}s)..."
# env.sh sources /opt/ros/humble/setup.bash, which references unset vars;
# relax `set -u` just while sourcing it so this script does not exit early.
set +u
# shellcheck source=/dev/null
source "$PROJECT_DIR/scripts/env.sh"
set -u

ready=false
for _ in $(seq "$STARTUP_TIMEOUT"); do
  if ! kill -0 "$MAP_PID" 2>/dev/null; then
    echo "[mapping_autodrive] Mapping process exited during startup. Aborting." >&2
    cleanup
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -q '/qcar2_hardware'; then
    ready=true
    break
  fi
  sleep 1
done

if [ "$ready" != true ]; then
  echo "[mapping_autodrive] Hardware node never appeared. Aborting." >&2
  cleanup
  exit 1
fi

# Give the driver a moment to finish opening the motor/serial links.
sleep 3
echo "[mapping_autodrive] Hardware is up. Starting the scripted drive."

# --- 3. scripted drive --------------------------------------------------
# A short rclpy node owns the timing so a zero command is guaranteed on every
# exit path (normal end, exception, or SIGINT/SIGTERM).
DRIVE_PY="$(mktemp /tmp/qcar2_autodrive.XXXXXX.py)"
cat > "$DRIVE_PY" <<'PY'
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands

SPEED = float(sys.argv[1])
FORWARD_S = float(sys.argv[2])
WAIT1_S = float(sys.argv[3])
REVERSE_S = float(sys.argv[4])
WAIT2_S = float(sys.argv[5])
RATE_HZ = 20.0


class AutoDrive(Node):
    def __init__(self):
        super().__init__('qcar2_autodrive')
        self.pub = self.create_publisher(MotorCommands, '/qcar2_motor_speed_cmd', 10)

    def send(self, throttle, steering=0.0):
        msg = MotorCommands()
        msg.motor_names = ['steering_angle', 'motor_throttle']
        msg.values = [float(steering), float(throttle)]
        self.pub.publish(msg)

    def hold(self, throttle, seconds, label):
        self.get_logger().info(f'{label}: throttle={throttle:+.2f} m/s for {seconds:.0f}s')
        end = time.monotonic() + seconds
        period = 1.0 / RATE_HZ
        while rclpy.ok() and time.monotonic() < end:
            self.send(throttle)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def stop_burst(self, seconds, label):
        self.get_logger().info(f'{label}: stop for {seconds:.0f}s')
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.send(0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)


def main():
    rclpy.init()
    node = AutoDrive()

    def _bail(*_):
        for _ in range(10):
            node.send(0.0)
            time.sleep(0.02)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _bail)
    signal.signal(signal.SIGTERM, _bail)

    # Let discovery settle so the very first command is not dropped.
    time.sleep(1.0)
    try:
        node.hold(SPEED, FORWARD_S, 'FORWARD')
        node.stop_burst(WAIT1_S, 'WAIT 1')
        node.hold(-SPEED, REVERSE_S, 'REVERSE')
        node.stop_burst(WAIT2_S, 'WAIT 2')
    finally:
        for _ in range(10):
            node.send(0.0)
            time.sleep(0.02)
        node.get_logger().info('Scripted drive complete; car commanded to zero.')
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
PY

python3 "$DRIVE_PY" \
  "$DRIVE_SPEED" "$FORWARD_SECONDS" "$WAIT1_SECONDS" "$REVERSE_SECONDS" "$WAIT2_SECONDS"
DRIVE_RC=$?
rm -f "$DRIVE_PY"
echo "[mapping_autodrive] Drive sequence finished (rc=$DRIVE_RC)."

# --- 4. stop all ------------------------------------------------------------
trap - INT TERM
cleanup
echo "[mapping_autodrive] Done."
