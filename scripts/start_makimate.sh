#!/usr/bin/env bash
# Launch the full MakiMate ROS 2 stack.
# Used by the makimate.service systemd unit.
set -e
source /opt/ros/jazzy/setup.bash
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_DIR/install/setup.bash"
exec ros2 launch maki_operational_nodes presentation_mode_v3.launch.py
