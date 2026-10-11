#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/bot/mx_jazzhands/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Preserve a caller's DDS profile. For Isaac interop, use the existing peer XML.
exec ros2 launch vanilla_bringup sensors.launch.xml "$@"
