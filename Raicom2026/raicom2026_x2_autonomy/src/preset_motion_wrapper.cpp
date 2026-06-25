#include "preset_motion_wrapper.h"

#include <cmath>
#include <algorithm>
#include <chrono>

namespace {
constexpr double kZone1X = 1.0;
constexpr double kZone1Y = 0.0;
constexpr double kStopDistance = 0.20;
constexpr double kMaxForwardVelocity = 0.18;
constexpr double kMaxLateralVelocity = 0.08;
constexpr double kMaxYawRate = 0.50;
constexpr double kForwardKp = 0.45;
constexpr double kLateralKp = 0.35;
constexpr double kYawKp = 1.20;
constexpr int32_t kMcInputActionAdd = 1001;
constexpr int32_t kInputSourcePriority = 40;
constexpr int32_t kInputSourceTimeoutMs = 1000;
constexpr const char* kInputSourceName = "node";

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}
}  // namespace

PresetMotionWrapper::PresetMotionWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      current_x_(0.0),
      current_y_(0.0),
      current_yaw_(0.0),
      has_odom_(false),
      has_yaw_(false),
      input_source_registered_(false),
      navigation_active_(false) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto leg_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

  locomotion_pub_ = node_->create_publisher<LocomotionVelocity>(
      "/aima/mc/locomotion/velocity", qos);
  input_source_client_ = node_->create_client<SetMcInputSource>(
      "/aimdk_5Fmsgs/srv/SetMcInputSource");
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/aima/hal/odom/state", qos, std::bind(&PresetMotionWrapper::handleOdom, this, std::placeholders::_1));
  leg_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/aima/mc/leg_odometry", leg_qos, std::bind(&PresetMotionWrapper::handleOdom, this, std::placeholders::_1));
  torso_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/torso/state", qos, std::bind(&PresetMotionWrapper::handleImu, this, std::placeholders::_1));
  chest_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/chest/state", qos, std::bind(&PresetMotionWrapper::handleImu, this, std::placeholders::_1));
}

bool PresetMotionWrapper::runPresetMotion(int area_id, int motion_id, bool interrupt) {
  (void)area_id;
  (void)motion_id;
  (void)interrupt;
  return true;
}

bool PresetMotionWrapper::registerInputSource() {
  if (input_source_registered_) {
    return true;
  }

  const auto wait_start = std::chrono::steady_clock::now();
  while (!input_source_client_->wait_for_service(std::chrono::seconds(2))) {
    if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(8)) {
      RCLCPP_ERROR(node_->get_logger(), "[NAV_GOAL] waiting SetMcInputSource timed out");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[NAV_GOAL] waiting for SetMcInputSource service...");
  }

  auto request = std::make_shared<SetMcInputSource::Request>();
  request->action.value = kMcInputActionAdd;
  request->input_source.name = kInputSourceName;
  request->input_source.priority = kInputSourcePriority;
  request->input_source.timeout = kInputSourceTimeoutMs;

  for (int attempt = 0; attempt < 8; ++attempt) {
    request->request.header.stamp = node_->now();
    auto future = input_source_client_->async_send_request(request);
    const auto ret = rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(250));
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[NAV_GOAL] retry register input source attempt=%d", attempt);
      continue;
    }

    const auto response = future.get();
    if (response && response->response.header.code == 0) {
      input_source_registered_ = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "[NAV_GOAL] input source registered name=%s priority=%d timeout_ms=%d task_id=%lu",
          kInputSourceName,
          kInputSourcePriority,
          kInputSourceTimeoutMs,
          response->response.task_id);
      return true;
    }

    RCLCPP_ERROR(
        node_->get_logger(),
        "[NAV_GOAL] input source registration rejected attempt=%d",
        attempt);
    return false;
  }

  RCLCPP_ERROR(node_->get_logger(), "[NAV_GOAL] failed to register input source");
  return false;
}

void PresetMotionWrapper::publishZone1Goal() {
  if (!registerInputSource()) {
    return;
  }

  navigation_active_ = true;
  RCLCPP_INFO(
      node_->get_logger(),
      "[NAV_GOAL] enabling locomotion control toward zone_1=(%.2f, %.2f)",
      kZone1X,
      kZone1Y);

  if (navigation_timer_) {
    navigation_timer_->cancel();
  }

  navigation_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PresetMotionWrapper::publishNavigationVelocity, this));
}

void PresetMotionWrapper::handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  const double x = msg->pose.pose.orientation.x;
  const double y = msg->pose.pose.orientation.y;
  const double z = msg->pose.pose.orientation.z;
  const double w = msg->pose.pose.orientation.w;
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  has_odom_ = true;
  has_yaw_ = true;
}

void PresetMotionWrapper::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double x = msg->orientation.x;
  const double y = msg->orientation.y;
  const double z = msg->orientation.z;
  const double w = msg->orientation.w;
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  has_yaw_ = true;
}

void PresetMotionWrapper::publishNavigationVelocity() {
  LocomotionVelocity velocity;
  velocity.header.stamp = node_->now();
  velocity.header.meas_stamp = velocity.header.stamp;
  velocity.header.frame_id = "base_link";
  velocity.header.sequence = 0;
  velocity.source = kInputSourceName;

  if (!navigation_active_ || !has_odom_ || !has_yaw_) {
    velocity.forward_velocity = 0.0;
    velocity.lateral_velocity = 0.0;
    velocity.angular_velocity = 0.0;
    locomotion_pub_->publish(velocity);
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[NAV_GOAL] waiting feedback active=%s has_odom=%s has_yaw=%s",
        navigation_active_ ? "YES" : "NO",
        has_odom_ ? "YES" : "NO",
        has_yaw_ ? "YES" : "NO");
    return;
  }

  const double dx = kZone1X - current_x_;
  const double dy = kZone1Y - current_y_;
  const double distance = std::hypot(dx, dy);

  if (distance <= kStopDistance) {
    navigation_active_ = false;
    velocity.forward_velocity = 0.0;
    velocity.lateral_velocity = 0.0;
    velocity.angular_velocity = 0.0;
    locomotion_pub_->publish(velocity);
    RCLCPP_INFO(
        node_->get_logger(),
        "[NAV_GOAL] zone_1 reached current=(%.3f, %.3f) distance=%.3f",
        current_x_,
        current_y_,
        distance);
    return;
  }

  const double cos_yaw = std::cos(current_yaw_);
  const double sin_yaw = std::sin(current_yaw_);
  const double forward_base = dx * cos_yaw + dy * sin_yaw;
  const double lateral_base = -dx * sin_yaw + dy * cos_yaw;
  const double yaw_target = std::atan2(dy, dx);
  double yaw_error = yaw_target - current_yaw_;
  while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

  velocity.forward_velocity = clamp(kForwardKp * forward_base, 0.0, kMaxForwardVelocity);
  velocity.lateral_velocity = clamp(kLateralKp * lateral_base, -kMaxLateralVelocity, kMaxLateralVelocity);
  velocity.angular_velocity = clamp(kYawKp * yaw_error, -kMaxYawRate, kMaxYawRate);

  if (std::abs(yaw_error) > 0.45) {
    velocity.forward_velocity = std::min(velocity.forward_velocity, 0.06);
  }

  locomotion_pub_->publish(velocity);
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[NAV_GOAL] pos=(%.3f,%.3f) dist=%.3f forward=%.3f lateral=%.3f yaw_error=%.3f",
      current_x_,
      current_y_,
      distance,
      velocity.forward_velocity,
      velocity.lateral_velocity,
      yaw_error);
}
