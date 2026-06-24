# Raicom2026 变更日志

本文件用于记录 Raicom2026 工程中的结构调整、运行方式、依赖状态和后续开发变更。所有记录均以不修改官方依赖目录为前提。

## 当前工程边界

### 官方依赖目录

以下目录视为官方交付内容或官方依赖产物，禁止修改：

- `mc/`
- `sim_mujoco/`
- `aimdk_msgs/`
- `aimdk-aarch64-1bde262f-artifacts/`

### 用户开发目录

用户新增内容仅允许放在以下区域：

- `../raicom2026_x2_autonomy/`
- `launch/`
- `scripts/`
- `config/`
- `DOCS/`

## 2026-06-24 结构整理记录

### 目录结构调整说明

已在 `Raicom2026/` 下新增用户侧扩展目录：

- `user_workspace/autonomy/`
- `user_workspace/control/`
- `user_workspace/perception/`
- `user_workspace/tasks/`
- `launch/`
- `scripts/`
- `config/`

其中 `user_workspace/` 用于后续按功能域沉淀开发资料或中间代码；当前核心可编译用户代码放在工程根目录同级的 `raicom2026_x2_autonomy/`。

### autonomy 仓库创建说明

已新增用户自主系统目录：

- `../raicom2026_x2_autonomy/`

当前包含：

- `README.md`
- `CMakeLists.txt`
- `src/main.cpp`
- `src/state_machine.cpp`
- `src/controller.cpp`
- `src/planner.cpp`

该目录实现了最小 C++ 运行骨架：

- 启动输出 `START AUTONOMY SYSTEM`
- 状态流转为 `STAND_UP -> WALK_TO_ZONE_1 -> DANCE -> DONE`
- `planner` 和 `controller` 当前为占位逻辑
- 使用独立 CMake 构建，不依赖修改官方包

### 仿真环境说明

官方仿真入口为：

```bash
./sim_mujoco/bin/start_sim.sh
```

已新增用户侧启动脚本：

```bash
./launch/run_autonomy.sh
```

该脚本会：

1. 构建 `../raicom2026_x2_autonomy`
2. 调用 `./sim_mujoco/bin/start_sim.sh` 启动仿真
3. 运行自主系统二进制 `../raicom2026_x2_autonomy/build/raicom2026_x2_autonomy`

注意：`start_sim.sh` 是官方脚本，当前未修改。

### git 仓库关系说明

当前工程应保持单一 git 根目录：

```text
link_u_os_competition/
```

当前检查结果显示只存在顶层 `.git`，未发现 `raicom2026_x2_autonomy/` 下存在嵌套 `.git`。后续禁止在任何子目录再次执行 `git init`，避免产生嵌套仓库。

`raicom2026_x2_autonomy/` 当前作为“逻辑独立用户代码目录”管理，不作为 git submodule，也不再作为嵌套 git repo。

### 当前运行方式说明

单独构建 autonomy：

```bash
cd ../raicom2026_x2_autonomy
cmake -S . -B build
cmake --build build
./build/raicom2026_x2_autonomy
```

从 Raicom2026 目录运行仿真和 autonomy：

```bash
cd Raicom2026
./launch/run_autonomy.sh
```

### 依赖说明

当前 autonomy 骨架只依赖：

- CMake
- C++17 编译器
- 标准 C++ 库

仿真环境依赖由官方 `sim_mujoco/`、`mc/`、`aimdk_msgs/` 和 `aimdk-aarch64-1bde262f-artifacts/` 提供。

当前环境检查中：

- `libgflags` 可在系统动态库缓存中找到
- `scene.xml` 存在
- `x2.xml` 存在
- `rosidl typesupport` 未在 `ldconfig` 输出中发现明确条目，需要在仿真或 ROS 消息链路报错时重点排查

## 变更规范（必须遵守）

以后每次修改必须追加记录，格式如下：

#### [日期] 修改内容

- 修改点：
- 修改原因：
- 影响范围：
- 如何验证：

## 2026-06-24 建立变更记录体系

- 修改点：新增本 `CHANGELOG.md`，补充目录结构、autonomy、仿真、git、运行方式和依赖说明。
- 修改原因：建立工程级可追踪变更系统，便于比赛期间快速定位结构变化和运行风险。
- 影响范围：仅新增文档，不修改官方目录，不改变仿真和构建行为。
- 如何验证：检查 `git status --short`，确认官方目录无修改；阅读本文件确认后续变更格式已定义。

#### [2026-06-24] autonomy 接入 ROS2 / MC 控制链路

- 修改点：在 `../raicom2026_x2_autonomy/` 中新增 `X2AutonomyNode` ROS2 控制节点，发布 `/aima/mc/locomotion/velocity` 和 `/mc/upper_body_command`，控制循环频率为 50Hz，并将 topic 探测结果记录到 `logs/control_topics.log`。
- 修改原因：将 autonomy 从状态机打印程序升级为可通过 ROS2 与 MC 通信的控制节点。
- 影响范围：仅修改 `../raicom2026_x2_autonomy/` 和 `launch/run_autonomy.sh`；未修改 `mc/`、`sim_mujoco/`、`aimdk_msgs/`、`aimdk-aarch64-*`。
- 如何验证：执行 `cmake -S . -B build && cmake --build build` 编译 autonomy；运行 `./build/raicom2026_x2_autonomy`，确认节点输出 `STAND_UP -> WALK_TO_ZONE_1 -> DANCE -> DONE`，并检查 `logs/control_topics.log` 中包含控制 topic。

#### [2026-06-25] autonomy 收敛到 AimDK MC 走跑控制接口

- 修改点：新增 `mc_ros2_controller.cpp/.h`，使用官方 `/aima/mc/locomotion/velocity` topic 和 `McLocomotionVelocity` 消息，50Hz 连续发布 `source=raicom_autonomy` 的速度控制；移除主程序对上半身控制 topic 的依赖。
- 修改原因：仿真机器人未运动的根因是 autonomy 没有接入 AimDK X2 的 MC locomotion velocity 控制接口。
- 影响范围：仅修改 `../raicom2026_x2_autonomy/` 文档和代码；未修改官方目录。
- 如何验证：编译 `raicom2026_x2_autonomy` 并运行二进制，确认终端持续打印 `/aima/mc/locomotion/velocity`、状态、source、forward/lateral/yaw_rate；检查 `logs/control_topics.log`。

#### [2026-06-25] 新增控制闭环运动诊断系统

- 修改点：新增 `robot_state_monitor.cpp/.h`，自动探测 `/aima/mc/state`、`/aima/robot/state`、`/joint_states`、`/odom` 及 AimDK HAL 状态 topic；实现 base motion、joint variance、控制命令与反馈关联、10Hz 日志和 5 秒无运动自动调试；生成 `DOCS/MOTION_DEBUG_REPORT.md`。
- 修改原因：需要区分问题发生在 ROS2 publish、MC 消费还是仿真反馈链路。
- 影响范围：仅修改 `../raicom2026_x2_autonomy/` 并新增/更新 `DOCS/MOTION_DEBUG_REPORT.md`；未修改官方目录。
- 如何验证：运行 autonomy，观察 `RobotStateMonitor` 输出；检查 `DOCS/MOTION_DEBUG_REPORT.md` 中的 publish、feedback、motion、topic 列表和可能原因分析。

#### [2026-06-25] 新增 MC control inspector 与最终 CASE 判定

- 修改点：新增 `mc_control_inspector.cpp/.h`，启动时打印 ROS2 topic，检查 MC feedback/arbitration topic 和 sim feedback topic，输出 `FULL CONTROL OK`、`COMMAND IGNORED`、`NO MC CONNECTION` 三类判定；报告中新增 MC connectivity status、arbitration status guess、control rejection reason。
- 修改原因：进一步区分控制未生效是 MC 未连接、仲裁覆盖，还是仿真未响应。
- 影响范围：仅修改 `../raicom2026_x2_autonomy/` 和 `DOCS/MOTION_DEBUG_REPORT.md`；未修改官方目录。
- 如何验证：运行 autonomy，若只存在 `/aima/mc/locomotion/velocity` 而没有 MC feedback topic，应打印 `❌ MC CONTROL LAYER NOT ACTIVE OR NOT CONNECTED` 并报告 `CASE 3: NO MC CONNECTION`。
