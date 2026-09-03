#!/usr/bin/env bash
set -Eeuo pipefail

if ! lsusb -d 8086:0b3a >/dev/null 2>&1; then
    echo "错误：未找到 RealSense D435i（8086:0b3a）。" >&2
    exit 1
fi

exec sudo bash --noprofile --norc -c '
printf "128\n" > /sys/module/usbcore/parameters/usbfs_memory_mb
source /opt/robot/scripts/setup_ros2.sh
export ROS_DOMAIN_ID=0
exec m20-realsense-camera \
  enable_depth:=false \
  enable_color:=true \
  enable_gyro:=false \
  enable_accel:=false \
  pointcloud.enable:=false \
  align_depth.enable:=false \
  enable_sync:=false \
  rgb_camera.profile:=424,240,15
'
