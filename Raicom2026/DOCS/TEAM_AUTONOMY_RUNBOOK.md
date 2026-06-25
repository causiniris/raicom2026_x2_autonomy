# X2 Autonomy Team Runbook

## Current Scope

This version is a minimal autonomy loop for simulation:

1. `RESET`
2. `STAND_DEFAULT`
3. stability check
4. `LOCOMOTION_DEFAULT`
5. navigate toward `zone_1`

Current default `zone_1` target:

- `x = 0.0`
- `y = 1.75`

This target is implemented in:

- `raicom2026_x2_autonomy/src/preset_motion_wrapper.cpp`

## Current Code Changes

Only the autonomy side has been changed.

Key files:

- `raicom2026_x2_autonomy/src/main.cpp`
  - flow is now `RESET -> STAND -> LOCOMOTION -> ZONE1 NAV`
- `raicom2026_x2_autonomy/src/stability_wait_node.cpp`
  - relaxed reset and stand detection for current sim behavior
- `raicom2026_x2_autonomy/src/localization/centroid_filter.h`
- `raicom2026_x2_autonomy/src/localization/centroid_filter.cpp`
  - added filtered pose support
- `raicom2026_x2_autonomy/src/preset_motion_wrapper.cpp`
  - zone1 navigation uses filtered pose
  - MC input source name is unique per process
  - default target is currently `(0.0, 1.75)`

## How To Start

### 1. Go to repository root

On host:

```bash
cd ~/link_u_os_competition/Raicom2026
```

### 2. Make sure container is running

```bash
docker start x2_deploy
```

If the container does not exist yet, create it from this directory:

```bash
xhost +
docker run -it \
  --name=x2_deploy \
  --privileged \
  --net=host \
  --ipc=host \
  --pid=host \
  -e DISPLAY="$DISPLAY" \
  -v /dev/input:/dev/input \
  -v /tmp:/tmp \
  -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \
  -v .:/home/agi/x2_deploy_workspace \
  -d lingxi-x2-env:v1.0
```

### 3. Start simulation window

Open one terminal on host:

```bash
docker exec -it x2_deploy bash
```

Then inside container:

```bash
cd /home/agi/x2_deploy_workspace/sim_mujoco/bin
./start_sim.sh
```

### 4. Start autonomy

Open another terminal on host:

```bash
cd ~/link_u_os_competition/Raicom2026
docker exec x2_deploy bash -lc 'pkill -f raicom2026_x2_autonomy || true'
docker exec -it x2_deploy /home/agi/x2_deploy_workspace/scripts/run_all.sh
```

`run_all.sh` will:

- build `raicom2026_x2_autonomy`
- start sim dependencies if needed
- start MC if needed
- start autonomy node

## Expected Behavior

After pressing `Reset` in simulation:

1. robot stands up
2. autonomy switches to locomotion mode
3. robot walks toward `zone_1 = (0.0, 1.75)`

## Log Check

On host:

```bash
cd ~/link_u_os_competition/Raicom2026
docker exec x2_deploy bash -lc 'latest=$(ls -t /home/agi/x2_deploy_workspace/raicom2026_x2_autonomy/logs/raicom2026_x2_autonomy_*.log | head -n 1); echo "$latest"; grep -E "STEP1_RESULT|STEP2_RESULT|STEP3_RESULT|NAV_GOAL|input source" "$latest"'
```

Expected key lines:

- `STEP1_RESULT stable_stand=true`
- `STEP2_RESULT locomotion_mode=true`
- `STEP3_RESULT filtered_navigation=true`
- `NAV_GOAL input source registered ...`
- `NAV_GOAL enabling locomotion control toward zone_1=(0.00, 1.75)`

## Common Problems

### 1. Robot stands but does not walk

Check latest log for:

- `input source registration rejected`

If present, restart autonomy:

```bash
cd ~/link_u_os_competition/Raicom2026
docker exec x2_deploy bash -lc 'pkill -f raicom2026_x2_autonomy || true'
docker exec -it x2_deploy /home/agi/x2_deploy_workspace/scripts/run_all.sh
```

### 2. Autonomy stuck in reset wait

Check latest log for repeated:

- `RESET_WAIT detected=NO`

This means `STEP 1` has not passed yet.

### 3. Simulation window does not open

On host:

```bash
xhost +
docker start x2_deploy
```

Then start sim again from container:

```bash
docker exec -it x2_deploy bash
cd /home/agi/x2_deploy_workspace/sim_mujoco/bin
./start_sim.sh
```

### 4. Wrong target point

Current default target is hardcoded in:

- `raicom2026_x2_autonomy/src/preset_motion_wrapper.cpp`

It can also be overridden per run:

```bash
docker exec -e X2_ZONE1_X=0.0 -e X2_ZONE1_Y=1.75 -it x2_deploy /home/agi/x2_deploy_workspace/scripts/run_all.sh
```

## Validation Standard

Pass:

- robot stands after reset
- log shows `STEP1_RESULT=true`
- log shows `STEP2_RESULT=true`
- log shows `STEP3_RESULT=true`
- robot moves toward `zone_1`

Fail:

- robot stays in place after `STEP2_RESULT=true`
- input source registration is rejected
- robot walks in obviously wrong direction
- no `NAV_GOAL` progress logs
