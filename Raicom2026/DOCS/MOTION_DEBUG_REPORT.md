# MOTION DEBUG REPORT

## 结论

- 是否收到 odom/state：是
- 是否 publish 成功：是，控制节点已创建 publisher `/aima/mc/locomotion/velocity`
- MC connectivity status：MC topics and sim feedback topics are present
- arbitration status guess：source accepted or motion path active
- control rejection reason：none
- 是否 MC 接收：存在 MC 相关 topic，可能已接入
- 机器人是否运动：是
- 观测结果：机器人已检测到运动

## 最终判定

### CASE 1: FULL CONTROL OK

- ✔ MC receiving commands
- ✔ feedback exists
- ✔ robot moving

## 当前控制命令

- state: `STAND_UP`
- topic: `/aima/mc/locomotion/velocity`
- source: `pnc`
- forward_velocity: `0`
- lateral_velocity: `0`
- yaw_rate/angular_velocity: `0`
- control_active: `false`

## MC Control Inspector

- publish_topic_exists: `true`
- mc_feedback_exists: `true`
- sim_feedback_exists: `true`
- override_suspected: `false`
- source_missing: `false`

## 反馈状态

- odom_received: `true`
- joint_received: `false`
- base_position: `(-1.51782, -1.84283, 0.0711547)`
- base_velocity: `(0.00528392, -0.000200707, 0.000545751)`
- delta_position: `0.890411`
- joint_angle_variance: `0`
- debug_mode: `false`

## State Topic 时间戳

- `/aima/hal/joint/arm/command` type=`aimdk_msgs/msg/JointCommandArray` received=`false`
- `/aima/hal/joint/arm/state` type=`aimdk_msgs/msg/JointStateArray` received=`false`
- `/aima/hal/joint/hand/command` type=`aimdk_msgs/msg/HandCommandArray` received=`false`
- `/aima/hal/joint/hand/state` type=`aimdk_msgs/msg/HandStateArray` received=`false`
- `/aima/hal/joint/head/command` type=`aimdk_msgs/msg/JointCommandArray` received=`false`
- `/aima/hal/joint/head/state` type=`aimdk_msgs/msg/JointStateArray` received=`false`
- `/aima/hal/joint/leg/command` type=`aimdk_msgs/msg/JointCommandArray` received=`false`
- `/aima/hal/joint/leg/state` type=`aimdk_msgs/msg/JointStateArray` received=`false`
- `/aima/hal/joint/waist/command` type=`aimdk_msgs/msg/JointCommandArray` received=`false`
- `/aima/hal/joint/waist/state` type=`aimdk_msgs/msg/JointStateArray` received=`false`
- `/aima/hal/odom/state` type=`nav_msgs/msg/Odometry` received=`true` age_sec=`1535.62`
- `/aima/mc/common/state` type=`aimdk_msgs/msg/McCommonState` received=`false`
- `/aima/mc/leg_odometry` type=`nav_msgs/msg/Odometry` received=`false`

## 当前 Topic 列表

- /aima/hal/imu/chest/state [sensor_msgs/msg/Imu]
- /aima/hal/imu/torso/state [sensor_msgs/msg/Imu]
- /aima/hal/joint/arm/command [aimdk_msgs/msg/JointCommandArray]
- /aima/hal/joint/arm/state [aimdk_msgs/msg/JointStateArray]
- /aima/hal/joint/hand/command [aimdk_msgs/msg/HandCommandArray]
- /aima/hal/joint/hand/state [aimdk_msgs/msg/HandStateArray]
- /aima/hal/joint/head/command [aimdk_msgs/msg/JointCommandArray]
- /aima/hal/joint/head/state [aimdk_msgs/msg/JointStateArray]
- /aima/hal/joint/leg/command [aimdk_msgs/msg/JointCommandArray]
- /aima/hal/joint/leg/state [aimdk_msgs/msg/JointStateArray]
- /aima/hal/joint/waist/command [aimdk_msgs/msg/JointCommandArray]
- /aima/hal/joint/waist/state [aimdk_msgs/msg/JointStateArray]
- /aima/hal/odom/state [nav_msgs/msg/Odometry]
- /aima/hds/process/info/soc0 [aimdk_msgs/msg/ProcessInfo]
- /aima/internal/event/probe/soc0/mc [aimdk_msgs/msg/EventProbeInfoArray]
- /aima/mc/body_pose [aimdk_msgs/msg/McBodyPose]
- /aima/mc/common/state [aimdk_msgs/msg/McCommonState]
- /aima/mc/leg_odometry [nav_msgs/msg/Odometry]
- /aima/mc/locomotion/velocity [aimdk_msgs/msg/McLocomotionVelocity]
- /aima/mc/rl/debug [aimdk_msgs/msg/McRLDebug]
- /aima/mc_debug_f64 [std_msgs/msg/Float64MultiArray]
- /aima/teleop_bridge/vr_data [aimdk_msgs/msg/VRData]
- /aimrte/sm/node_state/pb_3Aaimdk_2Eprotocol_2ESmNodeStateChannel [ros2_plugin_proto/msg/RosMsgWrapper]
- /mc/upper_body_command [aimdk_msgs/msg/UpperBodyCommandArray]
- /parameter_events [rcl_interfaces/msg/ParameterEvent]
- /rosout [rcl_interfaces/msg/Log]
- /tf [tf2_msgs/msg/TFMessage]
- /tf_static [tf2_msgs/msg/TFMessage]

## 可能原因分析

- 控制闭环有效；已检测到 base 或 joint motion。
