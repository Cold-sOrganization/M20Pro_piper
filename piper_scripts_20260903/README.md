# Piper 脚本备份说明

- 备份日期：2026-09-03
- 来源：机器狗 GOS，原目录 `/home/user`
- 中转机备份目录：`/home/ccic/Desktop/piper_scripts_20260903`
- 运行环境：Ubuntu 20.04、ROS 2 Foxy、`/home/user/agx_arm_ws`

这里保存的是脚本副本，便于查看、传输和恢复。脚本中的工作区路径、ROS 话题和设备地址均按机器狗环境编写，不应直接在中转机上运行。

## 文件说明

### Piper 运动与轨迹脚本

| 文件 | 作用 |
| --- | --- |
| `piper_zero.py` | 读取 `/feedback/joint_states`，向 `/control/move_j` 下发六个关节全零目标，并等待关节进入容差范围。不包含使能操作。 |
| `piper_demo.py` | 演示脚本：先归零，再小幅抬升机械臂，然后低力度反复开合夹爪。可通过参数限制抬升幅度、夹爪开度和循环次数。 |
| `piper_carry_hold.py` | 平滑闭合夹爪，以 S 曲线进入固定持物姿态 `[0, 1.05, -1.10, 0, 1.10, 0] rad` 并持续保持。按 `Ctrl+C` 后调用 `piper_zero.py` 归零退出。默认以 20 Hz 下发，速度可用 `--arm-speed` 调整。 |
| `piper_teach_record.py` | 在人工示教期间按指定频率记录 `/feedback/joint_states`，输出原始轨迹 JSON；夹爪版驱动运行时也会记录 `gripper`。 |
| `piper_smooth_trajectory.py` | 把原始录制轨迹转换为平滑轨迹 JSON，限制相邻关节步长，插值时间和夹爪数据，同时保留原始文件。 |
| `piper_replay_trajectory.py` | 回放轨迹 JSON；支持机械臂与夹爪、自动缓慢接近首点、起点距离保护、步长检查和 `--playback-speed` 速度参数。 |
| `piper_hold_then_return.py` | 回放指定轨迹后持续保持末端姿态和夹爪状态；按 `Ctrl+C` 后调用 `piper_zero.py` 归零退出。 |
| `piper_joint6_step.py` | 小幅测试第六关节。先读取完整六轴反馈，只允许第六关节在当前位置增减不超过 `0.02 rad`，然后发送一次完整六轴目标。 |
| `piper_joint6_step_fixed.py` | `piper_joint6_step.py` 的发送保持修正版；发布命令后固定等待 1 秒，降低节点过早退出导致消息未送达的概率。 |

### Piper 桥接与启动脚本

| 文件 | 作用 |
| --- | --- |
| `piper_tcp_bridge_gos.py` | GOS 侧 TCP/ROS 桥。监听 `10.21.31.104:29500`，把远端六轴命令转发到 `/control/joint_states`，回传关节反馈，并可代理 `/enable_agx_arm` 服务。使用前应确认来源网络可信。 |
| `start_piper_tcp_bridge_gos.sh` | 加载 ROS 2 Foxy 和机械臂工作区后，启动 `piper_tcp_bridge_gos.py`。 |
| `start_piper_ros.sh` | 启动 Piper ROS 2 驱动，使用 `gs_usb`、Piper 固件参数和 `effector_type=none`，不启用夹爪。自动使能和控制入口默认关闭。 |
| `start_piper_ros2.sh` | 当前内容与 `start_piper_ros.sh` 相同，是同一套无夹爪启动配置的另一文件名。 |
| `start_piper_ros_gripper.sh` | 启动带 AgileX 夹爪支持的 Piper ROS 2 驱动，关键参数为 `effector_type=agx_gripper`。录制或回放夹爪动作时使用。 |
| `start_piper_ros_moveit.sh` | MoveIt/TCP 桥接场景启动脚本。启动前检查驱动冲突、USB-CAN、ROS 环境和工作区，随后以无夹爪模式启动驱动。 |
| `start_piper_ros.sh.pre_moveit_20260901` | 2026-09-01 留存的 MoveIt 修改前备份；当前内容与 `start_piper_ros.sh` 相同，不作为首选入口。 |

### 相机辅助脚本

| 文件 | 作用 |
| --- | --- |
| `start_camera_ros.sh` | 启动 M20 RealSense 相机 ROS 节点，配置 424x240、15 FPS 等参数。 |
| `m20_camera_usb_diag.sh` | RealSense USB 诊断脚本：检查 USB 节点和权限、刷新 udev、枚举设备、查看视频接口、USB 描述符及相关内核日志。脚本需要较高系统权限。 |

## 常用流程

机械臂和夹爪脚本运行前，先在机器狗上启动带夹爪的驱动，并按现场流程手动使能机械臂。运动脚本本身不会自动使能。

运行 Python 脚本时使用 root ROS 环境，例如：

```bash
sudo bash -lc '
source /opt/ros/foxy/setup.bash
source /home/user/agx_arm_ws/install/setup.bash
python3 /home/user/piper_zero.py
'
```

示教、录制、平滑和回放的基本顺序：

```text
启动夹爪版驱动 -> 手动使能/进入示教 -> piper_teach_record.py
-> piper_smooth_trajectory.py -> piper_replay_trajectory.py
```

## 本次未复制内容

以下项目不属于脚本备份，因此未放入本目录：

- 工作区和缓存：`agx_arm_ws`、`__pycache__`
- 轨迹和运行数据：`piper_recorded_trajectory.json`、`piper_smoothed_*.json`
- USB/相机日志：`gs_usb_diagnose.txt`、`piper_gs_usb_test.txt`
- 离线程序、固件和源码构建物：`m20_realsense_offline`、`m20_rsusb_offline`、`upd72020x-load`、`upd72020x-load.c`、`K2026090.mem`、`Makefile`
- 机器狗原有 `/home/user/README.md`

## 安全提示

- 运行运动脚本前清空机械臂工作空间，保持急停可用。
- 先空载、低速验证，再逐步增加速度或负载。
- 不要同时启动两套 Piper 驱动，避免争用同一个 USB-CAN。
- `Ctrl+C` 是程序退出与归零流程，不替代硬件急停。
