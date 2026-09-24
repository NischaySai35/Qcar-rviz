#!/bin/bash
# Spin down a LiDAR that is still running after an unclean kill.
#
# WHY THIS IS NEEDED: the RPLIDAR's motor keeps spinning until something sends
# it a stop command over the serial link. That command lives in rplidar_close(),
# at the end of the lidar node's main loop. If the node was SIGKILLed (terminal
# closed, `kill -9`, or `ros2 launch` escalating after Ctrl+C was hammered),
# that line never ran -- so the device was simply left spinning. No amount of
# further killing will stop it, because there is no longer a process to kill.
#
# The fix is to briefly re-open the device and then close it *properly*: this
# starts the lidar node, waits, and sends it a clean SIGINT so rplidar_close()
# actually executes.
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if pgrep -x 'lidar' >/dev/null 2>&1; then
  echo "[stop_lidar.sh] A lidar node is still running. Stopping it cleanly first..."
  pkill -INT -x 'lidar'
  sleep 5
  pkill -9 -x 'lidar' 2>/dev/null
  sleep 1
fi

echo "[stop_lidar.sh] Re-opening the LiDAR so it can be shut down properly..."

env -i HOME="$HOME" USER="$USER" \
  PATH="/usr/bin:/bin:/usr/local/bin:/usr/local/sbin:/usr/sbin:/sbin" \
  bash -c "
    source /opt/ros/humble/setup.bash
    source '$PROJECT_DIR/qcar2_ws/install/setup.bash'
    ros2 run qcar2_nodes lidar > /tmp/qcar2_stop_lidar.log 2>&1 &
    NODE_PID=\$!
    # Give the driver time to open the serial port and enter its read loop;
    # SIGINT before that point would skip the close path we are after.
    sleep 5
    kill -INT \$NODE_PID 2>/dev/null
    # Wait for rplidar_close() to complete rather than racing it.
    for _ in \$(seq 10); do
      kill -0 \$NODE_PID 2>/dev/null || break
      sleep 1
    done
    kill -9 \$NODE_PID 2>/dev/null
  "

echo "[stop_lidar.sh] Done. The LiDAR should now be silent."
echo "[stop_lidar.sh] If it is STILL spinning, the only remaining option is to"
echo "                power-cycle the QCar2 (the motor state lives in the device)."
