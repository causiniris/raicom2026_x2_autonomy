# 当前系统状态总结

## 当前目录结构

```text
link_u_os_competition/
├── Raicom2026/
│   ├── aimdk-aarch64-1bde262f-artifacts/
│   ├── aimdk_msgs/
│   ├── config/
│   ├── DOCS/
│   ├── launch/
│   ├── mc/
│   ├── scripts/
│   ├── sim_mujoco/
│   └── user_workspace/
└── raicom2026_x2_autonomy/
    ├── CMakeLists.txt
    ├── README.md
    └── src/
```

## git 结构是否正确

当前目标结构是单一 git root：

```text
link_u_os_competition/.git
```

当前检查结果：

- `git rev-parse --show-toplevel` 指向 `link_u_os_competition`
- 当前未发现 `raicom2026_x2_autonomy/.git`
- `raicom2026_x2_autonomy/` 作为逻辑独立用户代码目录，不作为嵌套 git repo
- 后续禁止执行子目录 `git init`

## 仿真是否正常

当前已确认：

- 官方仿真脚本存在：`Raicom2026/sim_mujoco/bin/start_sim.sh`
- 本次未修改 `sim_mujoco/`
- 模型文件存在：
  - `Raicom2026/sim_mujoco/configuration/robot/lx2501_3_t2d5/model_info/scene.xml`
  - `Raicom2026/sim_mujoco/configuration/robot/lx2501_3_t2d5/model_info/x2.xml`

当前未做结论：

- 本次未完整交互式启动仿真并验证图形/物理运行状态
- 需要在比赛机器或目标运行环境中执行 `./sim_mujoco/bin/start_sim.sh` 做最终确认

## autonomy 是否独立

当前 autonomy 代码位于：

```text
raicom2026_x2_autonomy/
```

当前特征：

- 独立 CMake 工程
- 不依赖修改官方目录
- 不包含官方包源码
- 已验证可编译
- 已验证可运行最小状态机

当前状态机：

```text
STAND_UP -> WALK_TO_ZONE_1 -> DANCE -> DONE
```

## 已知问题

### 1. rosidl typesupport 待确认

当前 `ldconfig` 检查未发现明确 `rosidl` 动态库条目。若后续出现消息类型支持错误，应检查：

- 是否已 `source ./install/setup.sh`
- `aimdk_msgs` 是否构建/安装完整
- `RMW_IMPLEMENTATION` 是否与仿真脚本一致
- `LD_LIBRARY_PATH` 是否包含官方安装路径

### 2. 仿真完整运行状态待人工确认

当前只确认了脚本和模型文件存在，未确认仿真窗口、物理状态、控制链路全部正常。

### 3. `launch/run_autonomy.sh` 依赖官方脚本行为

`launch/run_autonomy.sh` 会调用官方 `sim_mujoco/bin/start_sim.sh`。如果官方脚本后续改变参数、缓存或交互逻辑，启动脚本可能需要同步调整，但不得直接修改官方脚本。

### 4. `libgrid_map_msgs` 缺失风险

当前文档未在系统中确认 `libgrid_map_msgs` 是否存在。若后续构建或运行出现 `libgrid_map_msgs` 相关缺失，应按依赖问题处理，不应修改官方依赖目录。
