# 环境检测文档

本文档用于 Raicom2026 工程的基础环境检查。执行检查时必须避免修改官方目录。

## 必须检测项

### 1. 仿真是否正常启动

检测脚本是否存在：

```bash
test -x ./sim_mujoco/bin/start_sim.sh
```

手动启动仿真：

```bash
./sim_mujoco/bin/start_sim.sh
```

检查标准：

- 脚本存在且具有可执行权限
- 启动后可以选择机器人或读取缓存机器人
- 未出现动态库缺失、配置缺失或模型加载失败

当前状态：

- `./sim_mujoco/bin/start_sim.sh` 存在
- 本次未修改该官方脚本

### 2. 机器人模型是否加载成功

检测模型文件：

```bash
find ./sim_mujoco -name scene.xml -o -name x2.xml
```

必须确认：

- `scene.xml` 存在
- `x2.xml` 存在
- 启动仿真时日志中没有模型解析失败

当前已发现文件：

```text
./sim_mujoco/configuration/robot/lx2501_3_t2d5/model_info/scene.xml
./sim_mujoco/configuration/robot/lx2501_3_t2d5/model_info/x2.xml
```

### 3. 依赖是否完整

检测 `libgflags`：

```bash
ldconfig -p | grep libgflags
```

当前状态：

- `libgflags` 已在系统动态库缓存中发现

检测 `rosidl typesupport`：

```bash
ldconfig -p | grep rosidl
```

当前状态：

- 当前检查未发现明确 `rosidl typesupport` 动态库条目
- 如果后续出现 `rosidl_typesupport_cpp`、`rosidl_typesupport_c` 或消息类型支持相关错误，应优先检查 ROS 环境变量和 `aimdk_msgs` 安装结果

建议检查：

```bash
source ./install/setup.sh
env | grep -E 'AMENT|COLCON|ROS|RMW'
```

### 4. autonomy 是否可编译

进入 autonomy 目录：

```bash
cd ../raicom2026_x2_autonomy
cmake -S . -B build
cmake --build build
```

运行 autonomy：

```bash
./build/raicom2026_x2_autonomy
```

预期输出包含：

```text
START AUTONOMY SYSTEM
STATE: STAND_UP
STATE: WALK_TO_ZONE_1
STATE: DANCE
AUTONOMY SYSTEM STOPPED
```

当前状态：

- CMake 配置已验证成功
- CMake 编译已验证成功
- autonomy 二进制已验证可运行

### 5. git 状态是否正确

必须确认只有一个 git root：

```bash
git rev-parse --show-toplevel
find .. -path '*/.git' -type d | sort
```

正确结果应只有：

```text
../.git
```

当前状态：

- 当前工程 git root 为 `link_u_os_competition`
- 当前检查只发现顶层 `.git`
- 后续禁止在 `raicom2026_x2_autonomy/` 或其他子目录执行 `git init`

## 推荐完整检查流程

```bash
cd Raicom2026
test -x ./sim_mujoco/bin/start_sim.sh
find ./sim_mujoco -name scene.xml -o -name x2.xml
ldconfig -p | grep libgflags
cd ../raicom2026_x2_autonomy
cmake -S . -B build
cmake --build build
./build/raicom2026_x2_autonomy
cd ..
git rev-parse --show-toplevel
find . -path '*/.git' -type d | sort
```
