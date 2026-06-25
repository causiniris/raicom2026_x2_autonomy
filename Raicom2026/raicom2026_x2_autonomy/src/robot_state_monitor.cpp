#include "robot_state_monitor.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <sstream>

namespace {
constexpr double kBaseMotionThresholdMeters = 0.03;
constexpr double kJointVarianceThreshold = 0.0005;

std::filesystem::path workspaceDocsDir() {
  if (const char* workspace = std::getenv("X2_DEPLOY_WORKSPACE")) {
    return std::filesystem::path(workspace) / "DOCS";
  }
  return std::filesystem::current_path().parent_path() / "DOCS";
}

bool hasType(const std::vector<std::string>& types, const std::string& target) {
  return std::find(types.begin(), types.end(), target) != types.end();
}

bool isCandidateStateTopic(const std::string& topic) {
  return topic == "/aima/mc/state" ||
         topic == "/aima/robot/state" ||
         topic == "/joint_states" ||
         topic == "/odom" ||
         topic == "/aima/hal/odom/state" ||
         topic == "/aima/mc/common/state" ||
         topic == "/aima/mc/leg_odometry" ||
         topic.find("/aima/hal/joint/") == 0;
}
}  // namespace

RobotStateMonitor::RobotStateMonitor(
    std::shared_ptr<McRos2Controller> controller,
    std::shared_ptr<McControlInspector> inspector)
    : Node("RobotStateMonitor"),
      controller_(std::move(controller)),
      inspector_(std::move(inspector)),
      odom_received_(false),
      joint_received_(false),
      robot_moving_(false),
      not_moving_warned_(false),
      debug_mode_(false),
      initial_x_(0.0),
      initial_y_(0.0),
      initial_z_(0.0),
      current_x_(0.0),
      current_y_(0.0),
      current_z_(0.0),
      current_vx_(0.0),
      current_vy_(0.0),
      current_vz_(0.0),
      base_delta_(0.0),
      joint_variance_(0.0) {
  first_control_active_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_odom_time_ = first_control_active_time_;
  last_joint_time_ = first_control_active_time_;
  initial_pose_time_ = first_control_active_time_;
  last_motion_time_ = first_control_active_time_;

  discovery_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotStateMonitor::discoveryLoop, this));
  logging_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotStateMonitor::loggingLoop, this));
  report_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotStateMonitor::reportLoop, this));

  discoveryLoop();
  writeReport();
}

void RobotStateMonitor::discoveryLoop() {
  known_topics_ = get_topic_names_and_types();

  for (const auto& [topic, types] : known_topics_) {
    if (isCandidateStateTopic(topic)) {
      auto& stamp = state_topic_stamps_[topic];
      stamp.type = types.empty() ? "unknown" : types.front();
    }
  }

  subscribeDetectedTopics();
}

void RobotStateMonitor::subscribeDetectedTopics() {
  for (const auto& [topic, types] : known_topics_) {
    if ((topic == "/odom" || topic == "/aima/hal/odom/state" || topic == "/aima/mc/leg_odometry") &&
        hasType(types, "nav_msgs/msg/Odometry")) {
      subscribeOdomTopic(topic);
    }

    if ((topic == "/joint_states" || topic.find("/aima/hal/joint/") == 0) &&
        hasType(types, "sensor_msgs/msg/JointState")) {
      subscribeJointTopic(topic);
    }
  }
}

void RobotStateMonitor::subscribeOdomTopic(const std::string& topic) {
  for (const auto& sub : odom_subs_) {
    if (sub->get_topic_name() == topic) {
      return;
    }
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
  odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      topic,
      qos,
      [this, topic](nav_msgs::msg::Odometry::SharedPtr msg) { handleOdom(msg, topic); }));
  RCLCPP_INFO(get_logger(), "Subscribed odom topic: %s", topic.c_str());
}

void RobotStateMonitor::subscribeJointTopic(const std::string& topic) {
  for (const auto& sub : joint_subs_) {
    if (sub->get_topic_name() == topic) {
      return;
    }
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
  joint_subs_.push_back(create_subscription<sensor_msgs::msg::JointState>(
      topic,
      qos,
      [this, topic](sensor_msgs::msg::JointState::SharedPtr msg) { handleJointState(msg, topic); }));
  RCLCPP_INFO(get_logger(), "Subscribed joint topic: %s", topic.c_str());
}

void RobotStateMonitor::handleOdom(
    const nav_msgs::msg::Odometry::SharedPtr msg,
    const std::string& topic) {
  const auto now_time = now();
  auto& stamp = state_topic_stamps_[topic];
  stamp.type = "nav_msgs/msg/Odometry";
  stamp.last_received = now_time;
  stamp.received = true;

  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_z_ = msg->pose.pose.position.z;
  current_vx_ = msg->twist.twist.linear.x;
  current_vy_ = msg->twist.twist.linear.y;
  current_vz_ = msg->twist.twist.linear.z;

  if (!odom_received_) {
    initial_x_ = current_x_;
    initial_y_ = current_y_;
    initial_z_ = current_z_;
    initial_pose_time_ = now_time;
  }

  odom_received_ = true;
  last_odom_time_ = now_time;
  updateMotionState();
}

void RobotStateMonitor::handleJointState(
    const sensor_msgs::msg::JointState::SharedPtr msg,
    const std::string& topic) {
  const auto now_time = now();
  auto& stamp = state_topic_stamps_[topic];
  stamp.type = "sensor_msgs/msg/JointState";
  stamp.last_received = now_time;
  stamp.received = true;

  joint_received_ = true;
  last_joint_time_ = now_time;

  std::vector<double> leg_positions;
  for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    const auto& name = msg->name[i];
    if (name.find("hip") != std::string::npos ||
        name.find("knee") != std::string::npos ||
        name.find("ankle") != std::string::npos) {
      leg_positions.push_back(msg->position[i]);
    }
  }

  if (!leg_positions.empty()) {
    const double mean = std::accumulate(leg_positions.begin(), leg_positions.end(), 0.0) /
                        static_cast<double>(leg_positions.size());
    double variance = 0.0;
    for (double value : leg_positions) {
      const double diff = value - mean;
      variance += diff * diff;
    }
    joint_variance_ = variance / static_cast<double>(leg_positions.size());
  }

  updateMotionState();
}

void RobotStateMonitor::updateMotionState() {
  base_delta_ = std::sqrt(
      std::pow(current_x_ - initial_x_, 2.0) +
      std::pow(current_y_ - initial_y_, 2.0) +
      std::pow(current_z_ - initial_z_, 2.0));

  const bool base_moving = base_delta_ > kBaseMotionThresholdMeters ||
                           std::sqrt(current_vx_ * current_vx_ + current_vy_ * current_vy_ + current_vz_ * current_vz_) > 0.03;
  const bool joints_moving = joint_variance_ > kJointVarianceThreshold;
  robot_moving_ = base_moving || joints_moving;

  if (robot_moving_) {
    last_motion_time_ = now();
  }
}

void RobotStateMonitor::loggingLoop() {
  const auto command = controller_->lastCommand();
  const bool control_active = controller_->controlActive();
  const auto now_time = now();

  if (control_active && first_control_active_time_.nanoseconds() == 0) {
    first_control_active_time_ = now_time;
  }

  updateMotionState();

  if (control_active && robot_moving_) {
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "✅ ROBOT IS MOVING state=%s cmd=(%.3f, %.3f, %.3f) pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f) joint_var=%.6f",
        stateName(controller_->currentState()),
        command.forward_velocity,
        command.lateral_velocity,
        command.yaw_rate,
        current_x_,
        current_y_,
        current_z_,
        current_vx_,
        current_vy_,
        current_vz_,
        joint_variance_);
  } else if (control_active && base_delta_ < kBaseMotionThresholdMeters) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "⚠ ROBOT NOT MOVING - CONTROL NOT EFFECTIVE state=%s cmd=(%.3f, %.3f, %.3f) delta=%.4f odom=%s joint=%s",
        stateName(controller_->currentState()),
        command.forward_velocity,
        command.lateral_velocity,
        command.yaw_rate,
        base_delta_,
        hasRecentOdom() ? "recent" : "missing",
        hasRecentJointState() ? "recent" : "missing");
  }

  if (control_active &&
      first_control_active_time_.nanoseconds() != 0 &&
      !robot_moving_ &&
      (now_time - first_control_active_time_).seconds() > 5.0) {
    debug_mode_ = true;
    if (!not_moving_warned_) {
      printDebugDump();
      not_moving_warned_ = true;
    }
  }
}

void RobotStateMonitor::reportLoop() {
  writeReport();
}

bool RobotStateMonitor::hasRecentOdom() const {
  return odom_received_ && (now() - last_odom_time_).seconds() < 1.0;
}

bool RobotStateMonitor::hasRecentJointState() const {
  return joint_received_ && (now() - last_joint_time_).seconds() < 1.0;
}

std::string RobotStateMonitor::topicListText() const {
  std::ostringstream out;
  for (const auto& [topic, types] : known_topics_) {
    out << "- " << topic;
    if (!types.empty()) {
      out << " [" << types.front() << "]";
    }
    out << "\n";
  }
  return out.str();
}

std::string RobotStateMonitor::observedResultText() const {
  if (robot_moving_) {
    return "机器人已检测到运动";
  }
  if (!odom_received_ && !joint_received_) {
    return "未收到可解码 odom/joint feedback，无法证明机器人运动";
  }
  return "已收到反馈但未超过运动阈值";
}

void RobotStateMonitor::printDebugDump() {
  RCLCPP_WARN(get_logger(), "AUTO DEBUG MODE ENABLED: no motion detected after 5 seconds");
  RCLCPP_WARN(get_logger(), "Subscribed odom topics: %zu, joint topics: %zu", odom_subs_.size(), joint_subs_.size());
  for (const auto& [topic, stamp] : state_topic_stamps_) {
    const std::string age = stamp.received ? std::to_string((now() - stamp.last_received).seconds()) + "s" : "never";
    RCLCPP_WARN(get_logger(), "state topic=%s type=%s last_received=%s", topic.c_str(), stamp.type.c_str(), age.c_str());
  }
  RCLCPP_WARN(get_logger(), "Known ROS2 topics:\n%s", topicListText().c_str());
}

void RobotStateMonitor::writeReport() {
  const auto docs_dir = workspaceDocsDir();
  std::filesystem::create_directories(docs_dir);
  std::ofstream report(docs_dir / "MOTION_DEBUG_REPORT.md", std::ios::trunc);
  const auto command = controller_->lastCommand();
  const auto inspection = inspector_->snapshot(robot_moving_);

  report << "# MOTION DEBUG REPORT\n\n";
  report << "## 结论\n\n";
  report << "- 是否收到 odom/state：" << ((odom_received_ || joint_received_) ? "是" : "否") << "\n";
  report << "- 是否 publish 成功：是，控制节点已创建 publisher `" << controller_->velocityTopic() << "`\n";
  report << "- MC connectivity status：" << inspection.mc_connectivity_status << "\n";
  report << "- arbitration status guess：" << inspection.arbitration_status_guess << "\n";
  report << "- control rejection reason：" << inspection.control_rejection_reason << "\n";
  report << "- 是否 MC 接收：" << (inspection.mc_feedback_exists ? "存在 MC 相关 topic，可能已接入" : "未检测到 MC feedback/arbitration topic") << "\n";
  report << "- 机器人是否运动：" << (robot_moving_ ? "是" : "否/未确认") << "\n";
  report << "- 观测结果：" << observedResultText() << "\n\n";

  report << "## 最终判定\n\n";
  if (inspection.control_case == ControlCase::FULL_CONTROL_OK) {
    report << "### CASE 1: FULL CONTROL OK\n\n";
    report << "- ✔ MC receiving commands\n";
    report << "- ✔ feedback exists\n";
    report << "- ✔ robot moving\n\n";
  } else if (inspection.control_case == ControlCase::COMMAND_IGNORED) {
    report << "### CASE 2: COMMAND IGNORED\n\n";
    report << "- ✔ publish OK\n";
    report << "- ❌ no motion\n";
    report << "- " << (inspection.mc_feedback_exists ? "✔ MC state/topic exists" : "❌ no MC state") << "\n";
    report << "- conclusion: arbitration override or source not registered\n\n";
  } else {
    report << "### CASE 3: NO MC CONNECTION\n\n";
    report << "- ❌ no MC topics\n";
    report << "- conclusion: sim bridge broken, MC not running, or ROS graph/RMW mismatch\n\n";
  }

  report << "## 当前控制命令\n\n";
  report << "- state: `" << stateName(controller_->currentState()) << "`\n";
  report << "- topic: `" << controller_->velocityTopic() << "`\n";
  report << "- source: `" << controller_->sourceName() << "`\n";
  report << "- forward_velocity: `" << command.forward_velocity << "`\n";
  report << "- lateral_velocity: `" << command.lateral_velocity << "`\n";
  report << "- yaw_rate/angular_velocity: `" << command.yaw_rate << "`\n";
  report << "- control_active: `" << (controller_->controlActive() ? "true" : "false") << "`\n\n";

  report << "## MC Control Inspector\n\n";
  report << "- publish_topic_exists: `" << (inspection.publish_topic_exists ? "true" : "false") << "`\n";
  report << "- mc_feedback_exists: `" << (inspection.mc_feedback_exists ? "true" : "false") << "`\n";
  report << "- sim_feedback_exists: `" << (inspection.sim_feedback_exists ? "true" : "false") << "`\n";
  report << "- override_suspected: `" << (inspection.override_suspected ? "true" : "false") << "`\n";
  report << "- source_missing: `" << (inspection.source_missing ? "true" : "false") << "`\n\n";

  report << "## 反馈状态\n\n";
  report << "- odom_received: `" << (odom_received_ ? "true" : "false") << "`\n";
  report << "- joint_received: `" << (joint_received_ ? "true" : "false") << "`\n";
  report << "- base_position: `(" << current_x_ << ", " << current_y_ << ", " << current_z_ << ")`\n";
  report << "- base_velocity: `(" << current_vx_ << ", " << current_vy_ << ", " << current_vz_ << ")`\n";
  report << "- delta_position: `" << base_delta_ << "`\n";
  report << "- joint_angle_variance: `" << joint_variance_ << "`\n";
  report << "- debug_mode: `" << (debug_mode_ ? "true" : "false") << "`\n\n";

  report << "## State Topic 时间戳\n\n";
  for (const auto& [topic, stamp] : state_topic_stamps_) {
    report << "- `" << topic << "` type=`" << stamp.type << "` received=`"
           << (stamp.received ? "true" : "false") << "`";
    if (stamp.received) {
      report << " age_sec=`" << (now() - stamp.last_received).seconds() << "`";
    }
    report << "\n";
  }

  report << "\n## 当前 Topic 列表\n\n";
  report << topicListText() << "\n";

  report << "## 可能原因分析\n\n";
  if (!known_topics_.count(controller_->velocityTopic())) {
    report << "- publish 可能未进入 ROS graph：未在 topic list 中看到控制 topic。\n";
  } else if (inspection.control_case == ControlCase::NO_MC_CONNECTION) {
    report << "- publish 已存在，但没有检测到 MC feedback/arbitration topic；当前断点是 MC 未运行、sim bridge 未连通或 RMW/ROS graph 不一致。\n";
  } else if (!odom_received_ && !joint_received_) {
    report << "- publish 已存在，但没有可解码反馈；问题可能在 MC 未消费、仿真未启动、RMW 不一致，或反馈 topic 不是标准 Odometry/JointState。\n";
  } else if (!robot_moving_) {
    report << "- 已收到反馈但未运动；问题可能在 MC 输入源仲裁、source 未注册、机器人模式不允许走跑，或仿真未响应 MC 输出。\n";
  } else {
    report << "- 控制闭环有效；已检测到 base 或 joint motion。\n";
  }
}
