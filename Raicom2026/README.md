
# Raicom2026 比赛交付说明

## 1. 项目简介

本项目面向睿抗 2026 具身智能服务机器人赛题，目标是在官方 `sim_mujoco + mc` 环境上完成自主控制链路接入，实现从 `RESET` 后自动站立、进入 locomotion、导航到 `zone_1`，并继续执行后续动作。

当前交付目录由三部分组成：

- 官方环境层：`sim_mujoco/`、`mc/`、`aimdk-aarch64-1bde262f-artifacts/`、`aimdk_msgs/`
- 用户控制层：`raicom2026_x2_autonomy/`
- 运行辅助层：`scripts/`、`launch/`、`config/`

目录结构：

```text
Raicom2026/
├── aimdk-aarch64-1bde262f-artifacts/
├── aimdk_msgs/
├── mc/
├── sim_mujoco/
├── raicom2026_x2_autonomy/
├── scripts/
├── launch/
├── config/
├── Dockerfile
└── README.md
```

## 2. 系统架构

系统采用三层结构：

1. `sim_mujoco`
   负责机器人仿真、物理环境和可视化窗口。
2. `mc`
   负责底层运动控制执行，消费 locomotion 指令并输出机器人运动行为。
3. `raicom2026_x2_autonomy`
   负责自主状态流转、稳定性判断、里程计处理、目标点导航和到点后动作控制。

ROS2 通信链路如下：

```text
raicom2026_x2_autonomy
  -> /aima/mc/locomotion/velocity
  -> mc
  -> sim_mujoco

反馈订阅：
  /aima/hal/odom/state
  /aima/mc/leg_odometry
  /aima/hal/imu/torso/state
  /aima/hal/imu/chest/state
```

当前确认实际使用的控制 topic：

- `/aima/mc/locomotion/velocity`

当前确认实际使用的反馈 topic：

- `/aima/hal/odom/state`
- `/aima/mc/leg_odometry`
- `/aima/hal/imu/torso/state`
- `/aima/hal/imu/chest/state`

## 3. 控制流程

当前主流程为：

```text
RESET -> STAND_UP -> WALK_TO_ZONE_1 -> DANCE
```

对应运行逻辑：

1. `RESET`
   等待仿真中的 reset 姿态被检测到。
2. `STAND_UP`
   调用 `STAND_DEFAULT`，等待姿态稳定。
3. `WALK_TO_ZONE_1`
   切换到 `LOCOMOTION_DEFAULT`，开始向目标区域导航。
4. `DANCE`
   导航结束后进入预设的后续动作阶段。

当前自主入口脚本为：

```bash
/home/agi/x2_deploy_workspace/scripts/run_all.sh
```

## 4. 关键算法

### 4.1 yaw control

导航阶段会根据当前位置与目标点计算目标朝向 `yaw_target`，再由 `yaw_error = yaw_target - current_yaw` 生成偏航角速度指令，通过比例控制限制最大角速度，避免快速摆头。

到达 `zone_1` 后，系统会进入二阶段控制，切换到原地旋转模式，继续使用 `yaw_error` 收敛到目标朝向。

### 4.2 odom filtering

导航控制并不直接使用单帧原始 odom，而是对 odom 做滑动均值滤波，降低位置和偏航抖动。当前代码中同时保留：

- `raw_x/raw_y/raw_yaw`
- `filtered current_x/current_y/current_yaw`

最终导航和到点判定使用滤波后的位姿。

### 4.3 zone navigation

`zone_1` 使用固定目标坐标，当前默认值为：

```text
zone_1 = (-0.05, 1.60)
```

导航控制使用：

- 前向速度控制
- 侧向速度控制
- 偏航速度控制

并在距离目标小于停止阈值后切换到到点后原地旋转阶段。

## 5. 运行方式

以下命令基于 Ubuntu 22.04 + ROS2 Humble。

### 5.1 进入交付目录

压缩包解压后，必须进入 `Raicom2026/` 目录执行后续命令：

```bash
cd Raicom2026
```

原因是 Docker 挂载使用：

```bash
-v .:/home/agi/x2_deploy_workspace
```

因此当前目录下必须直接包含：

- `sim_mujoco/`
- `mc/`
- `raicom2026_x2_autonomy/`
- `scripts/`

### 5.2 Docker 镜像构建

```bash
docker build \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -t lingxi-x2-env:v1.0 .
```

### 5.3 Docker 启动

先允许 X11 显示：

```bash
xhost +
```

再启动容器：

```bash
docker run -it \
  --name=x2_deploy \
  --privileged \
  --net=host \
  --ipc=host \
  --pid=host \
  -e DISPLAY=$DISPLAY \
  -v /dev/input:/dev/input \
  -v /tmp:/tmp \
  -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \
  -v .:/home/agi/x2_deploy_workspace \
  -d lingxi-x2-env:v1.0
```

### 5.4 ROS Domain 设置

统一使用：

```bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
```

`scripts/run_all.sh` 已经内置这两个环境变量。

### 5.5 编译 `aimdk_msgs`

如果 `aimdk_msgs/` 尚未存在，需要在容器内构建：

```bash
docker start x2_deploy
docker exec -it x2_deploy /bin/bash

cd /home/agi/x2_deploy_workspace/aimdk-aarch64-1bde262f-artifacts
colcon build
cp -r ./install/aimdk_msgs/ /home/agi/x2_deploy_workspace/
```

### 5.6 启动 sim + mc + autonomy

推荐统一入口：

```bash
docker start x2_deploy
docker exec -it x2_deploy /home/agi/x2_deploy_workspace/scripts/run_all.sh
```

如果你已经 `cd` 到 `Raicom2026/` 根目录，也可以直接用更短的宿主机入口：

```bash
./start
```

`./start` 是宿主机包装脚本，会自动执行 `docker start x2_deploy` 并进入容器调用 `scripts/run_all.sh`。前提是 `x2_deploy` 容器已经按本文前面的方式创建完成。

该脚本会自动完成：

1. 设置 `ROS_DOMAIN_ID=20`
2. 构建 `raicom2026_x2_autonomy/build_docker`
3. 启动 `sim_mujoco/bin/start_sim.sh`
4. 启动 `mc/bin/em_run.sh`
5. 启动 autonomy 主节点

如果需要手动分开启动，可使用：

启动仿真：

```bash
docker start x2_deploy
docker exec -it x2_deploy /bin/bash
cd /home/agi/x2_deploy_workspace/sim_mujoco/bin
./start_sim.sh
```

启动 MC：

```bash
docker start x2_deploy
docker exec -it x2_deploy /bin/bash
cd /home/agi/x2_deploy_workspace/mc/bin
./em_run.sh
```

启动 autonomy：

```bash
docker start x2_deploy
docker exec -it x2_deploy /home/agi/x2_deploy_workspace/scripts/run_all.sh
```

## 6. Debug 方法

### 6.1 如何检查机器人是否在动

优先看 odom 变化。当前代码和日志中会输出 `odom_delta` 或导航距离变化。

可检查 autonomy 日志中的：

- `dist=...`
- `forward=... lateral=... yaw_error=...`
- `odom_delta=(..., ...)`

也可以在容器内直接查看 odom topic：

```bash
docker exec -it x2_deploy /bin/bash
source /opt/ros/humble/setup.bash
ros2 topic echo /aima/hal/odom/state
```

如果位置持续变化，说明机器人确实在运动。

### 6.2 如何检查 topic

列出 topic：

```bash
docker exec -it x2_deploy /bin/bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

检查控制 topic 是否存在：

```bash
ros2 topic info /aima/mc/locomotion/velocity
```

检查是否有控制消息：

```bash
ros2 topic echo /aima/mc/locomotion/velocity
```

检查 odom：

```bash
ros2 topic echo /aima/hal/odom/state
```

### 6.3 常见失败原因

#### domain mismatch

现象：

- autonomy 在发消息，但 MC 或 sim 没反应
- `ros2 topic list` 两边看到的话题不一致

处理：

- 确认宿主机和容器都使用 `ROS_DOMAIN_ID=20`
- 确认 `ROS_LOCALHOST_ONLY=0`

#### mc not connected

现象：

- `run_all.sh` 提示 `mc_app_main is not running`
- 机器人站立后不进入 locomotion

处理：

- 检查 `mc/bin/em_run.sh` 是否真正启动
- 容器内执行 `pgrep -f mc_app_main`

#### 仿真未启动或显示异常

现象：

- 没有 Mujoco 窗口
- `start_sim.sh` 报错

处理：

- 确认先执行了 `xhost +`
- 确认 Docker 启动参数包含 `-e DISPLAY=$DISPLAY`

## 7. 已完成成果说明

当前已完成的能力如下：

- 站立成功
  已实现 `RESET` 后自动调用 `STAND_DEFAULT` 并通过稳定性检查。
- zone1 到达
  已实现基于 odom 和 imu 的闭环导航，可向 `zone_1` 目标点移动。
- 基本导航实现
  已完成 yaw 对齐、位姿滤波、速度控制、目标点停止判定和到点后动作切换。

## 8. 交付打包说明

比赛提交建议只打包 `Raicom2026/` 目录，而不是更上层仓库。

原因：

- `run_all.sh` 固定要求容器挂载根目录为 `/home/agi/x2_deploy_workspace`
- 挂载根目录下必须直接看到 `sim_mujoco/`、`mc/`、`raicom2026_x2_autonomy/`、`scripts/`
- 如果把上层 `link_u_os_competition/` 整体作为运行根目录，路径会多出一层，运行入口会失配

推荐打包命令见最终交付说明。
