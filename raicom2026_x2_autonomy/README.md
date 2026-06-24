# Raicom2026 X2 Autonomy

Raicom2026 比赛用户自主控制代码。

本目录与官方依赖目录隔离，禁止把官方源码移动或复制到这里：

- `mc/`
- `sim_mujoco/`
- `aimdk_msgs/`

## 构建

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build
```

## 运行

```bash
./build/raicom2026_x2_autonomy
```

## 控制方式说明

当前 autonomy 已升级为 ROS2 MC 走跑控制节点：

- ROS2 节点名：`X2AutonomyNode`
- 控制频率：50Hz
- 状态流转：`STAND_UP -> WALK_TO_ZONE_1 -> DANCE -> DONE`
- 输入源字段：`source = "raicom_autonomy"`
- topic 自动探测结果写入：`logs/control_topics.log`
- 官方接口依据：AimDK X2 `McLocomotionVelocity`
- 稳定控制：所有 WALK / DANCE 指令必须先通过 stability gate

控制链路：

```text
raicom2026_x2_autonomy
  -> StabilityController roll/pitch/height gate
  -> ROS2 topic: /aima/mc/locomotion/velocity
  -> aimdk_msgs/msg/McLocomotionVelocity
  -> MC 运动控制
  -> /aima/hal/joint/*/command
  -> sim_mujoco / MujocoSimModule
```

主要发布 topic：

- `/aima/mc/locomotion/velocity`

消息类型：

- `aimdk_msgs/msg/McLocomotionVelocity`

字段映射：

- `source = "raicom_autonomy"`
- `forward_velocity`：前进/后退速度，单位 m/s
- `lateral_velocity`：侧向速度，单位 m/s
- `angular_velocity`：偏航角速度，作为 yaw rate 使用，单位 rad/s

注意：AimDK 文档要求通过 `SetMcInputSource` 注册二次开发输入源。本地 SDK 预构建包缺少该服务依赖的 `CommonRequest` / `CommonTaskResponse` C++ 生成头，因此当前 C++ 节点先保证持续发布带唯一 `source` 的走跑控制消息。若 MC 仍丢弃未知 source，需要补齐 AimDK 生成头或使用官方 Python 示例注册输入源。

## 控制架构升级

已加入 `StabilityController`，控制节点会订阅：

- `/aima/hal/odom/state`
- `/aima/hal/imu/torso/state`
- `/aima/hal/imu/chest/state`

稳定性判断使用 base roll、pitch、height 和角速度反馈。每个 control loop 会打印：

```text
[STABILITY] roll=... pitch=... height=... status=STABLE/UNSTABLE
```

运行规则：

- `STAND_UP`：只执行 stabilization command。
- `WALK_TO_ZONE_1`：先稳定，再以低速前进，并持续叠加姿态修正。
- `DANCE`：只在稳定时执行低幅周期 yaw motion。
- 无姿态/高度反馈、反馈超时、roll/pitch 超阈值时，禁止 WALK / DANCE，进入 `GATE_BLOCK`，只发布稳定/阻尼修正。

## 如何验证

1. 启动仿真：

```bash
cd ../Raicom2026
./sim_mujoco/bin/start_sim.sh
```

2. 另开终端启动 MC：

```bash
cd ../Raicom2026/mc/bin
./em_run.sh
```

3. 另开终端启动 autonomy：

```bash
cd ../raicom2026_x2_autonomy
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build
./build/raicom2026_x2_autonomy
```

4. 检查：

- `logs/control_topics.log` 中是否出现 `/aima/mc/locomotion/velocity`
- 终端是否持续打印 topic、state、source、forward、lateral、yaw_rate
- 终端是否持续打印 `[STABILITY]`
- WALK / DANCE 前 `status` 是否为 `STABLE`
- 机器人是否从静止进入站立稳定状态
- 机器人是否低速向 zone 1 方向移动且不摔倒
- 机器人是否只在稳定时执行低幅 dance motion
