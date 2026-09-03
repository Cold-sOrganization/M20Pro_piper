#!/usr/bin/env bash
set -Eeuo pipefail

if ! lsusb -d 1d50:606f >/dev/null 2>&1; then
    echo "错误：未找到 Piper USB-CAN（1d50:606f）。" >&2
    exit 1
fi

exec sudo bash -lc '
source /opt/ros/foxy/setup.bash &&
source /home/user/agx_arm_ws/install/setup.bash &&
export PYTHONPATH=/home/user/agx_arm_ws/vendor-python:$PYTHONPATH &&
exec ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args \
  -p can_interface:=gs_usb \
  -p can_port:=piper \
  -p can_index:=0 \
  -p arm_type:=piper \
  -p effector_type:=none \
  -p fw_version:=v190 \
  -p auto_enable:=false \
  -p control_enabled:=false
'
