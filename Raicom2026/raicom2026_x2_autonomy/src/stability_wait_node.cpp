#include "stability_wait_node.h"

#include <cmath>
#include <chrono>

namespace {
constexpr double kFeedbackTimeoutSeconds = 0.5;
constexpr double kResetDropHeightMax = 0.10;
constexpr double kResetDropHeightRateMax = 0.08;
constexpr double kResetDropTiltMax = 0.35;
constexpr double kStandingResetHeightMin = 0.45;
constexpr double kStandingResetHeightMax = 0.95;
constexpr double kStandingResetTiltMax = 0.20;
constexpr double kMaxStableRollPitch = 0.30;
constexpr double kMaxStableHeightRate = 0.04;
constexpr double kMaxStableTiltRate = 0.12;
}

StabilityWaitNode::StabilityWaitNode()
    : Node("stability_wait_node"),
      last_feedback_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      last_sample_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      height_(0.0),
      previous_height_(0.0),
      roll_(0.0),
      pitch_(0.0),
      previous_roll_(0.0),
      previous_pitch_(0.0),
      height_rate_(0.0),
      roll_rate_(0.0),
      pitch_rate_(0.0),
      has_height_(false),
      has_orientation_(false),
      has_derivative_(false) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto leg_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aima/hal/odom/state", qos, std::bind(&StabilityWaitNode::handleOdom, this, std::placeholders::_1));
  leg_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aima/mc/leg_odometry", leg_qos, std::bind(&StabilityWaitNode::handleOdom, this, std::placeholders::_1));
  torso_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/torso/state", qos, std::bind(&StabilityWaitNode::handleImu, this, std::placeholders::_1));
  chest_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/chest/state", qos, std::bind(&StabilityWaitNode::handleImu, this, std::placeholders::_1));
}

void StabilityWaitNode::handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const rclcpp::Time sample_time = now();
  previous_height_ = height_;
  previous_roll_ = roll_;
  previous_pitch_ = pitch_;

  height_ = msg->pose.pose.position.z;
  has_height_ = true;
  last_feedback_time_ = sample_time;

  if (last_sample_time_.nanoseconds() != 0) {
    const double dt = (sample_time - last_sample_time_).seconds();
    if (dt > 1e-4) {
      height_rate_ = (height_ - previous_height_) / dt;
      roll_rate_ = (roll_ - previous_roll_) / dt;
      pitch_rate_ = (pitch_ - previous_pitch_) / dt;
      has_derivative_ = true;
    }
  }
  last_sample_time_ = sample_time;
}

void StabilityWaitNode::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  updateOrientation(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  last_feedback_time_ = now();
}

void StabilityWaitNode::updateOrientation(double x, double y, double z, double w) {
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll_ = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  pitch_ = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  has_orientation_ = true;
}

bool StabilityWaitNode::stableNow() const {
  return last_feedback_time_.nanoseconds() != 0 &&
         (now() - last_feedback_time_).seconds() <= kFeedbackTimeoutSeconds &&
         has_height_ &&
         has_orientation_ &&
         has_derivative_ &&
         std::abs(roll_) <= kMaxStableRollPitch &&
         std::abs(pitch_) <= kMaxStableRollPitch &&
         std::abs(height_rate_) <= kMaxStableHeightRate &&
         std::abs(roll_rate_) <= kMaxStableTiltRate &&
         std::abs(pitch_rate_) <= kMaxStableTiltRate;
}

bool StabilityWaitNode::resetDropNow() const {
  const bool crouch_reset =
      height_ <= kResetDropHeightMax &&
      std::abs(height_rate_) <= kResetDropHeightRateMax &&
      std::abs(roll_) <= kResetDropTiltMax &&
      std::abs(pitch_) <= kResetDropTiltMax;

  const bool standing_reset =
      height_ >= kStandingResetHeightMin &&
      height_ <= kStandingResetHeightMax &&
      std::abs(height_rate_) <= kResetDropHeightRateMax &&
      std::abs(roll_) <= kStandingResetTiltMax &&
      std::abs(pitch_) <= kStandingResetTiltMax;

  return last_feedback_time_.nanoseconds() != 0 &&
         (now() - last_feedback_time_).seconds() <= kFeedbackTimeoutSeconds &&
         has_height_ &&
         has_orientation_ &&
         has_derivative_ &&
         (crouch_reset || standing_reset);
}

bool StabilityWaitNode::waitForResetDrop(double timeout_seconds) {
  const rclcpp::Time start = now();
  rclcpp::Time reset_since(0, 0, get_clock()->get_clock_type());

  while (rclcpp::ok() && (now() - start).seconds() < timeout_seconds) {
    rclcpp::spin_some(shared_from_this());
    const bool reset_ready = resetDropNow();
    if (reset_ready) {
      if (reset_since.nanoseconds() == 0) {
        reset_since = now();
      }
      if ((now() - reset_since).seconds() >= 0.5) {
        RCLCPP_INFO(
            get_logger(),
            "[RESET_WAIT] reset pose detected height=%.3f roll=%.3f pitch=%.3f",
            height_, roll_, pitch_);
        return true;
      }
    } else {
      reset_since = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[RESET_WAIT] detected=%s height=%.3f roll=%.3f pitch=%.3f height_rate=%.3f",
        reset_ready ? "YES" : "NO", height_, roll_, pitch_, height_rate_);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  RCLCPP_ERROR(
      get_logger(),
      "[RESET_WAIT] timeout waiting reset pose height=%.3f roll=%.3f pitch=%.3f",
      height_, roll_, pitch_);
  return false;
}

bool StabilityWaitNode::waitUntilStable(double required_seconds, double timeout_seconds) {
  rclcpp::Time stable_since(0, 0, get_clock()->get_clock_type());
  const rclcpp::Time start = now();

  while (rclcpp::ok() && (now() - start).seconds() < timeout_seconds) {
    rclcpp::spin_some(shared_from_this());
    const bool stable = stableNow();
    if (stable) {
      if (stable_since.nanoseconds() == 0) {
        stable_since = now();
      }
      if ((now() - stable_since).seconds() >= required_seconds) {
        RCLCPP_INFO(get_logger(), "[STABILITY_WAIT] stable %.2fs height=%.3f roll=%.3f pitch=%.3f",
                    required_seconds, height_, roll_, pitch_);
        return true;
      }
    } else {
      stable_since = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[STABILITY_WAIT] stable=%s height=%.3f roll=%.3f pitch=%.3f rates=(%.3f,%.3f,%.3f)",
        stable ? "YES" : "NO", height_, roll_, pitch_, height_rate_, roll_rate_, pitch_rate_);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  RCLCPP_ERROR(get_logger(), "[STABILITY_WAIT] timeout height=%.3f roll=%.3f pitch=%.3f",
               height_, roll_, pitch_);
  return false;
}
