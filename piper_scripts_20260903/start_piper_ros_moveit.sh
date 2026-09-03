#!/usr/bin/env bash
set -Eeuo pipefail

# 独立的 MoveIt/TCP 桥接场景启动脚本。
# 不读取、覆盖或修改 ~/start_piper_ros.sh。

driver_executable="/home/user/agx_arm_ws/install/agx_arm_ctrl/lib/agx_arm_ctrl/agx_arm_ctrl_single"

if pgrep -f "${driver_executable}" >/dev/null 2>&1; then
    echo "错误：检测到 Piper ROS 驱动已经运行。" >&2
    echo "请不要同时启动两套驱动，否则会争用同一个 USB-CAN。" >&2
    echo "当前进程：" >&2
    pgrep -af "${driver_executable}" >&2 || true
    exit 2
fi

if ! lsusb -d 1d50:606f >/dev/null 2>&1; then
    echo "错误：未找到 Piper USB-CAN（USB ID 1d50:606f）。" >&2
    echo "请检查机械臂供电和机器狗上的 USB-CAN 接线。" >&2
    exit 1
fi

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
    echo "错误：未找到 ROS 2 Foxy 环境。" >&2
    exit 1
fi

if [[ ! -f /home/user/agx_arm_ws/install/setup.bash ]]; then
    echo "错误：未找到 /home/user/agx_arm_ws 编译结果。" >&2
    exit 1
fi

echo "启动 Piper ROS 驱动（MoveIt/TCP 桥接模式）"
echo "  USB 后端：gs_usb / index 0"
echo "  自动使能：关闭"
echo "  ROS 控制入口：初始关闭，等待 feedback ready"
echo "  原启动脚本：未修改（/home/user/start_piper_ros.sh）"
echo
echo "启动后必须看到：Agx_arm feedback is ready"
echo "按 Ctrl+C 可停止本驱动。"

exec sudo bash -lc '
source /opt/ros/foxy/setup.bash &&
source /home/user/agx_arm_ws/install/setup.bash &&
export PYTHONPATH=/home/user/agx_arm_ws/vendor-python:$PYTHONPATH &&
unset ROS_DISCOVERY_SERVER &&
export ROS_DOMAIN_ID=0 &&
export ROS_LOCALHOST_ONLY=0 &&
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
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
