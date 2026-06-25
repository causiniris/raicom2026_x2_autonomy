#include "ros2_control_node.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace {
constexpr double kPi = 3.14159265358979323846;

bool isControlTopic(const std::string& topic) {
  return topic.find("command") != std::string::npos ||
         topic.find("control") != std::string::npos ||
         topic.find("locomotion") != std::string::npos ||
         topic.find("upper_body") != std::string::npos ||
         topic.find("joint") != std::string::npos;
}
}  // namespace

X2AutonomyNode::X2AutonomyNode()
    : Node("X2AutonomyNode"),
      state_(AutonomyState::STAND_UP),
      state_start_time_(now()),
      sequence_(0),
      neutral_arm_pose_({0.25, 0.0, 0.0, -0.45, 0.0, 0.0, 0.0,
                         0.25, 0.0, 0.0, -0.45, 0.0, 0.0, 0.0}) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  locomotion_velocity_pub_ = create_publisher<LocomotionVelocity>(
      "/aima/mc/locomotion/velocity", qos);
  upper_body_pub_ = create_publisher<UpperBodyCommandArray>(
      "/mc/upper_body_command", qos);

  control_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                     std::bind(&X2AutonomyNode::controlLoop, this));
  topic_debug_timer_ = create_wall_timer(std::chrono::seconds(2),
                                         std::bind(&X2AutonomyNode::topicDebugLoop, this));

  updateKnownTopics();
  writeControlTopicLog();
  RCLCPP_INFO(get_logger(), "START AUTONOMY SYSTEM");
  RCLCPP_INFO(get_logger(), "STATE: %s", stateName(state_));
}

void X2AutonomyNode::setState(AutonomyState state) {
  if (state_ == state) {
    return;
  }

  state_ = state;
  state_start_time_ = now();
  RCLCPP_INFO(get_logger(), "STATE: %s", stateName(state_));
}

AutonomyState X2AutonomyNode::state() const {
  return state_;
}

bool X2AutonomyNode::finished() const {
  return state_ == AutonomyState::DONE;
}

void X2AutonomyNode::controlLoop() {
  const double elapsed = (now() - state_start_time_).seconds();

  if (state_ == AutonomyState::STAND_UP) {
    publishStandUp();
    if (elapsed >= 3.0) {
      setState(AutonomyState::WALK_TO_ZONE_1);
    }
    return;
  }

  if (state_ == AutonomyState::WALK_TO_ZONE_1) {
    publishWalkToZone1();
    if (elapsed >= 5.0) {
      setState(AutonomyState::DANCE);
    }
    return;
  }

  if (state_ == AutonomyState::DANCE) {
    publishDance();
    if (elapsed >= 6.0) {
      setState(AutonomyState::DONE);
    }
    return;
  }

  publishStop();
}

void X2AutonomyNode::topicDebugLoop() {
  updateKnownTopics();
  writeControlTopicLog();

  std::ostringstream stream;
  for (const auto& [topic, types] : known_topics_) {
    if (isControlTopic(topic)) {
      stream << topic << " ";
    }
  }

  RCLCPP_INFO(get_logger(), "control topics: %s", stream.str().c_str());
}

void X2AutonomyNode::publishStandUp() {
  publishStop();

  UpperBodyCommandArray upper_body;
  fillHeader(upper_body.header);
  upper_body.source = "pnc";
  upper_body.hand_sub_mode = UpperBodyCommandArray::HAND_CLAW_OPEN_CLOSE;
  upper_body.head_pos = {0.0, 0.0};
  upper_body.arm_pos = neutral_arm_pose_;
  upper_body.hand_pos = {0.0, 0.0};
  upper_body_pub_->publish(upper_body);
}

void X2AutonomyNode::publishWalkToZone1() {
  LocomotionVelocity velocity;
  fillHeader(velocity.header);
  velocity.source = "pnc";
  velocity.forward_velocity = 0.18;
  velocity.lateral_velocity = 0.0;
  velocity.angular_velocity = 0.0;
  locomotion_velocity_pub_->publish(velocity);
}

void X2AutonomyNode::publishDance() {
  publishStop();

  const double elapsed = (now() - state_start_time_).seconds();
  const double wave = std::sin(2.0 * kPi * 0.8 * elapsed);

  UpperBodyCommandArray upper_body;
  fillHeader(upper_body.header);
  upper_body.source = "pnc";
  upper_body.hand_sub_mode = UpperBodyCommandArray::HAND_CLAW_OPEN_CLOSE;
  upper_body.head_pos = {0.25 * wave, 0.10 * std::sin(2.0 * kPi * 0.4 * elapsed)};
  upper_body.arm_pos = {
      0.25, 0.35 * wave, 0.0, -0.45, 0.0, 0.0, 0.0,
      0.25, -0.35 * wave, 0.0, -0.45, 0.0, 0.0, 0.0,
  };
  upper_body.hand_pos = {0.5 + 0.5 * wave, 0.5 - 0.5 * wave};
  upper_body_pub_->publish(upper_body);
}

void X2AutonomyNode::publishStop() {
  LocomotionVelocity velocity;
  fillHeader(velocity.header);
  velocity.source = "pnc";
  velocity.forward_velocity = 0.0;
  velocity.lateral_velocity = 0.0;
  velocity.angular_velocity = 0.0;
  locomotion_velocity_pub_->publish(velocity);
}

void X2AutonomyNode::fillHeader(aimdk_msgs::msg::MessageHeader& header) {
  header.stamp = now();
  header.frame_id = "map";
  header.sequence = sequence_++;
  header.meas_stamp = header.stamp;
}

void X2AutonomyNode::updateKnownTopics() {
  known_topics_ = get_topic_names_and_types();
}

void X2AutonomyNode::writeControlTopicLog() const {
  recordTopicLog(known_topics_);
}

void X2AutonomyNode::recordTopicLog(
    const std::map<std::string, std::vector<std::string>>& topics) const {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/control_topics.log", std::ios::trunc);

  log << "ROS2 topic list\n";
  for (const auto& [topic, types] : topics) {
    log << topic;
    if (!types.empty()) {
      log << " [";
      for (std::size_t i = 0; i < types.size(); ++i) {
        log << types[i];
        if (i + 1 < types.size()) {
          log << ", ";
        }
      }
      log << "]";
    }
    log << "\n";
  }

  log << "\nControl topics\n";
  for (const auto& [topic, types] : topics) {
    if (isControlTopic(topic)) {
      log << topic;
      if (!types.empty()) {
        log << " [" << types.front() << "]";
      }
      log << "\n";
    }
  }
}
