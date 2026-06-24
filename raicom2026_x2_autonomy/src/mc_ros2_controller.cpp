#include "mc_ros2_controller.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kControlPeriodSeconds = 0.02;
constexpr double kFeedbackTimeoutSeconds = 0.5;
constexpr double kWalkForwardVelocity = 0.16;
constexpr double kDanceYawRate = 0.16;

bool isControlTopic(const std::string& topic) {
  return topic.find("locomotion") != std::string::npos ||
         topic.find("velocity") != std::string::npos ||
         topic.find("command") != std::string::npos ||
         topic.find("control") != std::string::npos;
}
}  // namespace

McRos2Controller::McRos2Controller()
    : Node(kNodeName),
      state_(AutonomyState::STAND_UP),
      state_start_time_(now()),
      last_feedback_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      sequence_(0),
      last_forward_velocity_(0.0),
      last_lateral_velocity_(0.0),
      last_yaw_rate_(0.0) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  velocity_pub_ = create_publisher<LocomotionVelocity>(kVelocityTopic, qos);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aima/hal/odom/state",
      qos,
      std::bind(&McRos2Controller::handleOdom, this, std::placeholders::_1));
  torso_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/torso/state",
      qos,
      std::bind(&McRos2Controller::handleImu, this, std::placeholders::_1));
  chest_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/chest/state",
      qos,
      std::bind(&McRos2Controller::handleImu, this, std::placeholders::_1));

  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodSeconds * 1000.0)),
      std::bind(&McRos2Controller::controlLoop, this));
  debug_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&McRos2Controller::topicDebugLoop, this));

  writeControlTopicLog();
  RCLCPP_INFO(get_logger(), "START AUTONOMY SYSTEM");
  RCLCPP_INFO(get_logger(), "MC velocity topic: %s", kVelocityTopic);
  RCLCPP_INFO(get_logger(), "MC source: %s", kSource);
  RCLCPP_WARN(
      get_logger(),
      "SetMcInputSource service client is disabled because this SDK bundle is missing generated CommonRequest/CommonTaskResponse C++ headers; velocity.source is still set to %s",
      kSource);
  RCLCPP_INFO(get_logger(), "STATE: %s", stateName(state_));
}

bool McRos2Controller::finished() const {
  return state_ == AutonomyState::DONE;
}

AutonomyState McRos2Controller::currentState() const {
  return state_;
}

VelocityCommandSnapshot McRos2Controller::lastCommand() const {
  return {last_forward_velocity_, last_lateral_velocity_, last_yaw_rate_};
}

bool McRos2Controller::controlActive() const {
  return std::abs(last_forward_velocity_) > 0.01 ||
         std::abs(last_lateral_velocity_) > 0.01 ||
         std::abs(last_yaw_rate_) > 0.01;
}

const char* McRos2Controller::velocityTopic() const {
  return kVelocityTopic;
}

const char* McRos2Controller::sourceName() const {
  return kSource;
}

void McRos2Controller::controlLoop() {
  const double elapsed = (now() - state_start_time_).seconds();

  if (state_ == AutonomyState::STAND_UP) {
    publishStabilizationOnly();
    if (elapsed >= 3.0) {
      setState(AutonomyState::WALK_TO_ZONE_1);
    }
    return;
  }

  if (state_ == AutonomyState::WALK_TO_ZONE_1) {
    publishStabilityCheckedVelocity(kWalkForwardVelocity, 0.0, computeHeadingToZone1());
    if (elapsed >= 6.0) {
      setState(AutonomyState::DANCE);
    }
    return;
  }

  if (state_ == AutonomyState::DANCE) {
    const double yaw_rate = kDanceYawRate * std::sin(2.0 * kPi * 0.25 * elapsed);
    publishStabilityCheckedVelocity(0.0, 0.0, yaw_rate);
    if (elapsed >= 6.0) {
      setState(AutonomyState::DONE);
    }
    return;
  }

  publishVelocity(0.0, 0.0, 0.0);
}

void McRos2Controller::topicDebugLoop() {
  writeControlTopicLog();
  RCLCPP_INFO(
      get_logger(),
      "topic=%s state=%s source=%s forward=%.3f lateral=%.3f yaw_rate=%.3f stability=%s roll=%.3f pitch=%.3f height=%.3f",
      kVelocityTopic,
      stateName(state_),
      kSource,
      last_forward_velocity_,
      last_lateral_velocity_,
      last_yaw_rate_,
      last_stability_command_.status.c_str(),
      stability_state_.roll,
      stability_state_.pitch,
      stability_state_.base_height);
}

void McRos2Controller::setState(AutonomyState state) {
  if (state_ == state) {
    return;
  }

  state_ = state;
  state_start_time_ = now();
  RCLCPP_INFO(get_logger(), "STATE: %s", stateName(state_));
}

void McRos2Controller::publishVelocity(
    double forward_velocity,
    double lateral_velocity,
    double yaw_rate) {
  LocomotionVelocity command;
  fillHeader(command.header);
  command.source = kSource;
  command.forward_velocity = forward_velocity;
  command.lateral_velocity = lateral_velocity;
  command.angular_velocity = yaw_rate;

  velocity_pub_->publish(command);

  last_forward_velocity_ = forward_velocity;
  last_lateral_velocity_ = lateral_velocity;
  last_yaw_rate_ = yaw_rate;
}

void McRos2Controller::publishStabilizationOnly() {
  last_stability_command_ = stability_controller_.update(stability_state_);
  publishVelocity(
      last_stability_command_.forward_correction,
      last_stability_command_.lateral_correction,
      last_stability_command_.yaw_correction);

  RCLCPP_INFO(
      get_logger(),
      "[STABILITY] roll=%.3f pitch=%.3f height=%.3f status=%s mode=STABILIZE_ONLY correction=(%.3f, %.3f, %.3f)",
      stability_state_.roll,
      stability_state_.pitch,
      stability_state_.base_height,
      last_stability_command_.status.c_str(),
      last_stability_command_.forward_correction,
      last_stability_command_.lateral_correction,
      last_stability_command_.yaw_correction);
}

void McRos2Controller::publishStabilityCheckedVelocity(
    double forward_velocity,
    double lateral_velocity,
    double yaw_rate) {
  last_stability_command_ = stability_controller_.update(stability_state_);

  const bool feedback_recent =
      last_feedback_time_.nanoseconds() != 0 &&
      (now() - last_feedback_time_).seconds() <= kFeedbackTimeoutSeconds;

  if (!feedback_recent || !last_stability_command_.stable) {
    publishVelocity(
        last_stability_command_.forward_correction,
        last_stability_command_.lateral_correction,
        last_stability_command_.yaw_correction);
    RCLCPP_INFO(
        get_logger(),
        "[STABILITY] roll=%.3f pitch=%.3f height=%.3f status=UNSTABLE mode=GATE_BLOCK feedback=%s correction=(%.3f, %.3f, %.3f)",
        stability_state_.roll,
        stability_state_.pitch,
        stability_state_.base_height,
        feedback_recent ? "recent" : "missing",
        last_stability_command_.forward_correction,
        last_stability_command_.lateral_correction,
        last_stability_command_.yaw_correction);
    return;
  }

  publishVelocity(
      forward_velocity + last_stability_command_.forward_correction,
      lateral_velocity + last_stability_command_.lateral_correction,
      yaw_rate + last_stability_command_.yaw_correction);
  RCLCPP_INFO(
      get_logger(),
      "[STABILITY] roll=%.3f pitch=%.3f height=%.3f status=STABLE mode=MOTION_ALLOWED correction=(%.3f, %.3f, %.3f)",
      stability_state_.roll,
      stability_state_.pitch,
      stability_state_.base_height,
      last_stability_command_.forward_correction,
      last_stability_command_.lateral_correction,
      last_stability_command_.yaw_correction);
}

void McRos2Controller::handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  stability_state_.base_height = msg->pose.pose.position.z;
  stability_state_.has_height = true;
  stability_state_.yaw_rate = msg->twist.twist.angular.z;
  updateOrientationFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  last_feedback_time_ = now();
}

void McRos2Controller::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  stability_state_.roll_rate = msg->angular_velocity.x;
  stability_state_.pitch_rate = msg->angular_velocity.y;
  stability_state_.yaw_rate = msg->angular_velocity.z;
  updateOrientationFromQuaternion(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  last_feedback_time_ = now();
}

void McRos2Controller::updateOrientationFromQuaternion(double x, double y, double z, double w) {
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  stability_state_.roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    stability_state_.pitch = std::copysign(kPi / 2.0, sinp);
  } else {
    stability_state_.pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  stability_state_.yaw = std::atan2(siny_cosp, cosy_cosp);
  stability_state_.has_orientation = true;
}

double McRos2Controller::computeHeadingToZone1() const {
  const double elapsed = (now() - state_start_time_).seconds();
  return 0.06 * std::sin(2.0 * kPi * 0.18 * elapsed);
}

void McRos2Controller::fillHeader(aimdk_msgs::msg::MessageHeader& header) {
  header.stamp = now();
  header.frame_id = "base_link";
  header.sequence = sequence_++;
  header.meas_stamp = header.stamp;
}

void McRos2Controller::writeControlTopicLog() {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/control_topics.log", std::ios::trunc);
  const auto topics = get_topic_names_and_types();

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

  log << "\nActive command\n";
  log << "topic=" << kVelocityTopic << "\n";
  log << "state=" << stateName(state_) << "\n";
  log << "source=" << kSource << "\n";
  log << "forward_velocity=" << last_forward_velocity_ << "\n";
  log << "lateral_velocity=" << last_lateral_velocity_ << "\n";
  log << "yaw_rate=" << last_yaw_rate_ << "\n";
  log << "stability_status=" << last_stability_command_.status << "\n";
  log << "roll=" << stability_state_.roll << "\n";
  log << "pitch=" << stability_state_.pitch << "\n";
  log << "base_height=" << stability_state_.base_height << "\n";
  log << "source_registration=source_field_only\n";
}
