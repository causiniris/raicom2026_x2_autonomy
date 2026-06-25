# Raicom2026 Control Gate 操作指南

本文档用于验证当前 `sim + MC + autonomy` 全链路，以及采集站立稳定性和 WALK_TO_ZONE_1 调优所需信息。

## 1. 启动 full stack

在宿主机执行：

```bash
docker exec x2_deploy bash -lc 'export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0; /home/agi/x2_deploy_workspace/scripts/run_all.sh'
```

如果已有旧进程在运行，先重启：

```bash
docker exec x2_deploy bash -lc 'for name in raicom2026_x2_autonomy mc_app_main aima-sim-app; do pids=$(pidof "$name" || true); if [ -n "$pids" ]; then kill -TERM $pids || true; fi; done; sleep 2; for name in raicom2026_x2_autonomy mc_app_main aima-sim-app; do pids=$(pidof "$name" || true); if [ -n "$pids" ]; then kill -KILL $pids || true; fi; done; export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0; nohup /home/agi/x2_deploy_workspace/scripts/run_all.sh > /tmp/raicom_run_all.log 2>&1 &'
```

理论上应看到：

- sim、MC、autonomy 都在 docker 内运行
- autonomy node 名称为 `/X2AutonomyNode`
- 控制 topic `/aima/mc/locomotion/velocity` 由 autonomy 发布，MC 订阅

## 2. 检查 ROS2 graph

```bash
docker exec x2_deploy bash -lc 'export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0; source /opt/ros/humble/setup.bash; ros2 node list'
```

理论上应看到类似：

```text
/X2AutonomyNode
/mc_ros2_node...
/aimrt_sim_node...
/McControlInspector
/RobotStateMonitor
```

检查关键 topic：

```bash
docker exec x2_deploy bash -lc 'export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0; source /opt/ros/humble/setup.bash; ros2 topic list | egrep "/aima/mc/locomotion/velocity|/aima/mc/leg_odometry|/aima/hal/joint/leg/state"'
```

理论上应看到：

```text
/aima/hal/joint/leg/state
/aima/mc/leg_odometry
/aima/mc/locomotion/velocity
```

检查 autonomy 是否能被 MC 消费：

```bash
docker exec x2_deploy bash -lc 'export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0; source /opt/ros/humble/setup.bash; ros2 topic info /aima/mc/locomotion/velocity -v'
```

理论上应看到：

- Publisher count: `1`
- publisher node: `X2AutonomyNode`
- Subscription count: `1` 或更多
- subscriber node: `mc_ros2_node...`

## 3. 观察 Control Gate

查看实时运行日志：

```bash
docker exec x2_deploy bash -lc 'tail -f /tmp/raicom_run_all.log'
```

理论上每个 control loop 会输出：

```text
[CONTROL GATE] stable=NO/YES pose_converged=NO/YES motion_allowed=NO/YES yaw_error=... height=...
```

判断规则：

- `stable=YES`：高度、roll/pitch、导数趋势、fall guard 都满足
- `pose_converged=YES`：机器人已连续稳定至少 3 秒
- `motion_allowed=YES`：允许 WALK_TO_ZONE_1 发送速度
- `motion_allowed=NO`：WALK/DANCE 被阻断，只允许站立/保持

## 4. 查看落盘日志

控制稳定性：

```bash
docker exec x2_deploy bash -lc 'tail -n 80 /home/agi/x2_deploy_workspace/raicom2026_x2_autonomy/logs/control_stability.log'
```

STAND_UP phase trace：

```bash
docker exec x2_deploy bash -lc 'cat /home/agi/x2_deploy_workspace/raicom2026_x2_autonomy/logs/stand_up_phase_trace.log'
```

STAND_UP 收敛过程：

```bash
docker exec x2_deploy bash -lc 'tail -n 80 /home/agi/x2_deploy_workspace/raicom2026_x2_autonomy/logs/stand_up_convergence.log'
```

odom delta tracking：

```bash
docker exec x2_deploy bash -lc 'tail -n 80 /home/agi/x2_deploy_workspace/raicom2026_x2_autonomy/logs/odom_delta_tracking.log'
```

理论上健康流程应为：

```text
PRE_STABILIZE -> GET_UP -> STAND_LOCK -> READY_TO_WALK -> WALK_TO_ZONE_1
```

并且进入 WALK 前应满足：

- `height > 0.25`
- `abs(roll) < 0.22`
- `abs(pitch) < 0.22`
- `stable=true`
- `pose_converged=true`
- `motion_allowed=true`
- `fall=false`

## 5. 直接 echo 关键反馈

leg odometry：

```bash
docker exec x2_deploy bash -lc 'export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0; source /opt/ros/humble/setup.bash; timeout 10 ros2 topic echo /aima/mc/leg_odometry --once'
```

leg joint state：

```bash
docker exec x2_deploy bash -lc 'export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0; source /opt/ros/humble/setup.bash; timeout 10 ros2 topic echo /aima/hal/joint/leg/state --once'
```

注意：如果 CLI 报 `aimdk_msgs/msg/JointStateArray is invalid`，说明当前 shell 没有正确加载该自定义消息环境。此时请反馈错误原文。

## 6. 你需要反馈给我的内容

请把以下内容贴回来：

1. `ros2 node list` 输出
2. `/aima/mc/locomotion/velocity -v` 输出
3. `/tmp/raicom_run_all.log` 中最近 30 行 `[CONTROL GATE]`
4. `stand_up_phase_trace.log` 全部内容
5. `stand_up_convergence.log` 最近 80 行
6. `odom_delta_tracking.log` 最近 80 行
7. 你肉眼观察到的机器人状态：
   - 是否从趴地/初始姿态抬起
   - 最高站立高度大约多少
   - 是否侧翻或前后倒
   - 是否进入 WALK_TO_ZONE_1
   - 如果进入 WALK，偏航方向是左偏、右偏还是原地打转

## 7. 当前调优目标

下一轮优化会基于你反馈的数据判断：

- 是 MC `GET_UP_DEFAULT` 本身没有完成
- 还是 STAND_LOCK 阶段阈值过严
- 还是 IMU/odom frame 与姿态判定不一致
- 还是 WALK yaw PID/frame correction 需要调参

在 `motion_allowed=false` 时，机器人不应该执行 WALK_TO_ZONE_1；如果你看到它仍然行走，请优先反馈 `/tmp/raicom_run_all.log` 和 `/aima/mc/locomotion/velocity -v`。
