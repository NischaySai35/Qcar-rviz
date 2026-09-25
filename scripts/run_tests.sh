#!/bin/bash
# Offline logic tests for the object-mapping / voice / exploration features.
#
#   scripts/run_tests.sh
#
# SAFE TO RUN AT ANY TIME, including while mapping or navigation is running:
# these import the node modules to exercise their pure logic, but never
# construct a ROS node and never call rclpy.init(), so nothing joins the live
# ROS graph. No hardware, no cameras and no driving are involved.
#
# What they cover (the claims that are otherwise only checkable by driving):
#   test_object_geometry  pixel -> bearing -> LiDAR range -> map coordinate,
#                         and that the SAME object seen by two different
#                         cameras lands on one landmark while genuinely
#                         distinct objects stay separate
#   test_object_nav       "go to the fridge" name matching, and standoff goals
#                         that stop short of an object instead of driving into it
#   test_voice_intent     that ordinary conversation does NOT move the car
#   test_explorer         frontier detection, goal scoring, dead-end blacklisting
set -u
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# env.sh sources /opt/ros/humble/setup.bash, which references unset vars.
set +u
# shellcheck source=/dev/null
source "$PROJECT_DIR/scripts/env.sh" >/dev/null 2>&1
set -u

# /usr/bin/python3 explicitly: bare `python3` is pyenv's 3.7 on this car, which
# cannot import rclpy or the JetPack numpy the nodes are written against.
PY=/usr/bin/python3
rc=0
for t in "$PROJECT_DIR"/tests/test_*.py; do
  name="$(basename "$t")"
  echo
  echo "================ $name ================"
  if ! "$PY" "$t"; then
    rc=1
  fi
done

echo
if [ "$rc" -eq 0 ]; then
  echo "[run_tests] ALL SUITES PASSED"
else
  echo "[run_tests] SOME SUITES FAILED" >&2
fi
exit "$rc"
