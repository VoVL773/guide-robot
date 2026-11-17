#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash
exec /usr/bin/python3 monitor.py "$@"
