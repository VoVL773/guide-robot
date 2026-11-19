#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash
if [ -f /home/nvidia/guide_robot/guide_ws/install/setup.bash ]; then
  source /home/nvidia/guide_robot/guide_ws/install/setup.bash
fi
exec /usr/bin/python3 monitor.py "$@"
