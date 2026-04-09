#!/usr/bin/env bash
source ~/makimate-2026/install/setup.bash
ros2 topic pub --once /maki/awake std_msgs/msg/Bool "{data: true}" \
  --qos-durability transient_local --qos-reliability reliable
