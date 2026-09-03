# M20 Pro GOS Piper 控制代码备份说明

## 备份来源与位置

- 来源主机：M20 Pro GOS（`user@10.21.31.104`）
- 来源工作空间：`/home/user/agx_arm_ws`
- 中转机备份目录：`/home/ccic/Desktop/M20Pro_Piper_GOS_code_20260901`
- 备份日期：2026-09-01

## 目录内容

- `src/agx_arm_ctrl/`：ROS 2 Piper 控制节点，包含 GOS 的 `gs_usb` 参数适配。
- `src/agx_arm_msgs/`：ROS 2 消息定义。
- `src/agx_arm_ros/`：官方附带的 CAN 配置脚本目录。
- `src/pyAgxArm/`：pyAgxArm 源码副本，包含用户态 CAN 通信适配。
- `vendor-python/`：GOS 实际运行时加载的 Python 依赖，包括 python-can 4.3.1、gs-usb 0.3.0、PyUSB 1.2.1 和 pyAgxArm。
- `scripts/piper_joint6_swing_test.py`：带反馈检查、使能和自动回位保护的第六关节摆动测试脚本。
- `M20Pro_Piper_GOS_code_20260901.tar.gz`：上述内容的原始压缩备份。

## 关键文件

- ROS 节点：`src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py`
- pyAgxArm 源码通信层：`src/pyAgxArm/pyAgxArm/protocols/can_protocol/comms/can_comm.py`
- GOS 实际运行通信层：`vendor-python/pyAgxArm/protocols/can_protocol/comms/can_comm.py`
- 控制测试脚本：`scripts/piper_joint6_swing_test.py`

GOS 启动时设置了：

```bash
export PYTHONPATH=/home/user/agx_arm_ws/vendor-python:$PYTHONPATH
```

因此运行时优先导入 `vendor-python/pyAgxArm`。修改代码时需要留意源码副本与运行副本；本次备份中两份 `can_comm.py` 的 SHA-256 相同。

## SHA-256 校验值

```text
d27c2aea6cbc1a49b1c6c9000c447cd11e597f69d1adc80cd31572d56c46af2a  M20Pro_Piper_GOS_code_20260901.tar.gz
78a431e6b7b6ac811571306344994d5b8367138c24c3e476c5f614fb7bc13adb  src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py
3852346867daee7e667eda7599336395f69391ace3d78cdde031e608d6ba706f  src/pyAgxArm/pyAgxArm/protocols/can_protocol/comms/can_comm.py
3852346867daee7e667eda7599336395f69391ace3d78cdde031e608d6ba706f  vendor-python/pyAgxArm/protocols/can_protocol/comms/can_comm.py
ed4516d61293de9cbbfc935bae95e3937319489d4694a42c6178ad738f3df271  scripts/piper_joint6_swing_test.py
```

## 使用提示

这份目录是从 GOS 原样复制的备份，不应直接覆盖其他工作区。移植时应先比较版本和差异。GOS 的通信链路为：

```text
agx_arm_ctrl → pyAgxArm → python-can(gs_usb) → gs_usb/PyUSB → USB-CAN → Piper
```

该链路不依赖 Linux `can0`。
