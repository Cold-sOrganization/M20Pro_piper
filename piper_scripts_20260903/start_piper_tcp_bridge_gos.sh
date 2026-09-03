#!/usr/bin/env bash
set -Eeuo pipefail

exec sudo bash -lc '
source /opt/ros/foxy/setup.bash &&
source /home/user/agx_arm_ws/install/setup.bash &&
unset ROS_DISCOVERY_SERVER &&
export ROS_DOMAIN_ID=0 &&
export ROS_LOCALHOST_ONLY=0 &&
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
exec python3 /home/user/piper_tcp_bridge_gos.py
'
