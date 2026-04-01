#!/usr/bin/env bash
# Start only the MakiMate dashboard node (lightweight, no robot hardware needed).
# Used by the makimate.service systemd unit.
set -e
source /opt/ros/jazzy/setup.bash
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_DIR/install/setup.bash"
exec ros2 run makimate_dashboard makimate_dashboard
