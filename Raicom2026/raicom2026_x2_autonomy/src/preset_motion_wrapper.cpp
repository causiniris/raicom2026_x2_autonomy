#include "preset_motion_wrapper.h"

#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <unistd.h>

namespace {
constexpr double kDefaultZone1X = -0.10;
constexpr double kDefaultZone1Y = 1.60;
constexpr double kStopDistance = 0.20;
constexpr double kRotateAfterArrivalDegrees = 150.0;
constexpr double kRotateCompletionTolerance = 0.08;
constexpr double kMaxForwardVelocity = 0.28;
constexpr double kMaxLateralVelocity = 0.10;
constexpr double kMaxYawRate = 0.70;
constexpr double kForwardKp = 0.60;
constexpr double kLateralKp = 0.35;
constexpr double kYawKp = 1.35;
constexpr int32_t kMcInputActionAdd = 1001;
constexpr int32_t kInputSourcePriority = 40;
constexpr int32_t kInputSourceTimeoutMs = 1000;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double readTargetCoordinate(const char* env_name, double default_value) {
  const char* value = std::getenv(env_name);
  if (value == nullptr) {
    return default_value;
  }

  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  return (end != value) ? parsed : default_value;
}

double degreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}
}  // namespace

PresetMotionWrapper::PresetMotionWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      input_source_name_("node_" + std::to_string(static_cast<long>(::getpid()))),
      pose_filter_(0.2, 5),
      target_x_(readTargetCoordinate("X2_ZONE1_X", kDefaultZone1X)),
      target_y_(readTargetCoordinate("X2_ZONE1_Y", kDefaultZone1Y)),
      current_x_(0.0),
      current_y_(0.0),
      current_yaw_(0.0),
      rotate_target_yaw_(0.0),
      raw_x_(0.0),
      raw_y_(0.0),
      raw_yaw_(0.0),
      has_odom_(false),
      has_yaw_(false),
      input_source_registered_(false),
      navigation_active_(false),
      rotate_after_arrival_active_(false) {
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
  request->input_source.name = input_source_name_;
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
          input_source_name_.c_str(),
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
  rotate_after_arrival_active_ = false;
  RCLCPP_INFO(
      node_->get_logger(),
      "[NAV_GOAL] enabling locomotion control toward zone_1=(%.2f, %.2f) source=%s",
      target_x_,
      target_y_,
      input_source_name_.c_str());

  if (navigation_timer_) {
    navigation_timer_->cancel();
  }

  navigation_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PresetMotionWrapper::publishNavigationVelocity, this));
}

void PresetMotionWrapper::handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const double x = msg->pose.pose.orientation.x;
  const double y = msg->pose.pose.orientation.y;
  const double z = msg->pose.pose.orientation.z;
  const double w = msg->pose.pose.orientation.w;
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  raw_x_ = msg->pose.pose.position.x;
  raw_y_ = msg->pose.pose.position.y;
  raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);

  const CentroidFilterOutput filtered = pose_filter_.update(*msg);
  if (filtered.sliding_mean.valid) {
    current_x_ = filtered.sliding_mean.x;
    current_y_ = filtered.sliding_mean.y;
    current_yaw_ = filtered.sliding_mean.yaw;
    has_odom_ = true;
    has_yaw_ = true;
  }
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

double PresetMotionWrapper::normalizeAngle(double angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void PresetMotionWrapper::publishNavigationVelocity() {
  LocomotionVelocity velocity;
  velocity.header.stamp = node_->now();
  velocity.header.meas_stamp = velocity.header.stamp;
  velocity.header.frame_id = "base_link";
  velocity.header.sequence = 0;
  velocity.source = input_source_name_;

  if ((!navigation_active_ && !rotate_after_arrival_active_) || !has_odom_ || !has_yaw_) {
    velocity.forward_velocity = 0.0;
    velocity.lateral_velocity = 0.0;
    velocity.angular_velocity = 0.0;
    locomotion_pub_->publish(velocity);
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[NAV_GOAL] waiting feedback nav_active=%s rotate_active=%s has_odom=%s has_yaw=%s",
        navigation_active_ ? "YES" : "NO",
        rotate_after_arrival_active_ ? "YES" : "NO",
        has_odom_ ? "YES" : "NO",
        has_yaw_ ? "YES" : "NO");
    return;
  }

  const double dx = target_x_ - current_x_;
  const double dy = target_y_ - current_y_;
  const double distance = std::hypot(dx, dy);

  if (navigation_active_ && distance <= kStopDistance) {
    navigation_active_ = false;
    rotate_after_arrival_active_ = true;
    rotate_target_yaw_ =
        normalizeAngle(current_yaw_ - degreesToRadians(kRotateAfterArrivalDegrees));
    RCLCPP_INFO(
        node_->get_logger(),
        "[NAV_GOAL] zone_1 reached current=(%.3f, %.3f) distance=%.3f rotate_target_yaw=%.3f",
        current_x_,
        current_y_,
        distance,
        rotate_target_yaw_);
  }

  double yaw_error = 0.0;
  if (rotate_after_arrival_active_) {
    yaw_error = normalizeAngle(rotate_target_yaw_ - current_yaw_);
    velocity.forward_velocity = 0.0;
    velocity.lateral_velocity = 0.0;
    velocity.angular_velocity = clamp(kYawKp * yaw_error, -kMaxYawRate, kMaxYawRate);

    if (std::abs(yaw_error) <= kRotateCompletionTolerance) {
      rotate_after_arrival_active_ = false;
      velocity.angular_velocity = 0.0;
      RCLCPP_INFO(
          node_->get_logger(),
          "[NAV_GOAL] post-zone rotation complete yaw=%.3f target=%.3f",
          current_yaw_,
          rotate_target_yaw_);
    }
  } else if (navigation_active_) {
    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    const double forward_base = dx * cos_yaw + dy * sin_yaw;
    const double lateral_base = -dx * sin_yaw + dy * cos_yaw;
    const double yaw_target = std::atan2(dy, dx);
    yaw_error = normalizeAngle(yaw_target - current_yaw_);

    velocity.forward_velocity = clamp(kForwardKp * forward_base, 0.0, kMaxForwardVelocity);
    velocity.lateral_velocity = clamp(kLateralKp * lateral_base, -kMaxLateralVelocity, kMaxLateralVelocity);
    velocity.angular_velocity = clamp(kYawKp * yaw_error, -kMaxYawRate, kMaxYawRate);

    if (std::abs(yaw_error) > 0.45) {
      velocity.forward_velocity = std::min(velocity.forward_velocity, 0.12);
    }
  } else {
    velocity.forward_velocity = 0.0;
    velocity.lateral_velocity = 0.0;
    velocity.angular_velocity = 0.0;
  }

  locomotion_pub_->publish(velocity);
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[NAV_GOAL] raw=(%.3f,%.3f,%.3f) filtered=(%.3f,%.3f,%.3f) dist=%.3f forward=%.3f lateral=%.3f yaw_error=%.3f",
      raw_x_,
      raw_y_,
      raw_yaw_,
      current_x_,
      current_y_,
      current_yaw_,
      distance,
      velocity.forward_velocity,
      velocity.lateral_velocity,
      yaw_error);
}
