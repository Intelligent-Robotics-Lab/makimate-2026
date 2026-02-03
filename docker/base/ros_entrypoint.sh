#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# If workspace exists and src has packages, build it dynamically

if [ -d "/maki_ws/src" ]; then
	cd /maki_ws
	echo "Building workspace (symlink install)..."
	colcon build --symlink-install || echo "Some packages failed to build; continuing anyways"
	source install/setup.bash
fi 

# Optional: copy expressions.yaml to legacy path if it exists
if [ -f "/maki_ws/src/makimate_dxl/makimate_dxl/expressions.yaml" ]; then
	mkdir -p /root/MakiMate/src/core/makimate_dxl/makimate_dxl
	cp /maki_ws/src/makimate_dxl/makimate_dxl/expressions.yaml \
	/root/MakiMate/src/core/makimate_dxl/makimate_dxl/expressions.yaml
fi



# If no CMD/arguments passed, default to bash
if [ $# -eq 0 ]; then
	exec bash
else 
	exec "$@"
fi
