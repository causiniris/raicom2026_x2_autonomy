#include "localization/centroid_filter.h"

#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

CentroidFilter::CentroidFilter(double alpha, std::size_t window_size)
    : alpha_(alpha > 0.0 ? alpha : 0.2),
      window_size_(window_size > 0 ? window_size : 5),
      ema_initialized_(false) {}

void CentroidFilter::reset() {
  ema_initialized_ = false;
  ema_pose_ = PoseSample{};
  window_.clear();
}

CentroidFilterOutput CentroidFilter::update(const nav_msgs::msg::Odometry& odom) {
  const PoseSample sample{
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      extractYaw(odom),
  };

  if (!ema_initialized_) {
    ema_pose_ = sample;
    ema_initialized_ = true;
  } else {
    ema_pose_.x = alpha_ * sample.x + (1.0 - alpha_) * ema_pose_.x;
    ema_pose_.y = alpha_ * sample.y + (1.0 - alpha_) * ema_pose_.y;
    const double yaw_delta = normalizeAngle(sample.yaw - ema_pose_.yaw);
    ema_pose_.yaw = normalizeAngle(ema_pose_.yaw + alpha_ * yaw_delta);
  }

  window_.push_back(sample);
  while (window_.size() > window_size_) {
    window_.pop_front();
  }

  return output();
}

CentroidFilterOutput CentroidFilter::output() const {
  CentroidFilterOutput result;

  if (ema_initialized_) {
    result.ema.x = ema_pose_.x;
    result.ema.y = ema_pose_.y;
    result.ema.yaw = ema_pose_.yaw;
    result.ema.valid = true;
  }

  if (!window_.empty()) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_sin = 0.0;
    double sum_cos = 0.0;

    for (const PoseSample& sample : window_) {
      sum_x += sample.x;
      sum_y += sample.y;
      sum_sin += std::sin(sample.yaw);
      sum_cos += std::cos(sample.yaw);
    }

    const double scale = 1.0 / static_cast<double>(window_.size());
    result.sliding_mean.x = sum_x * scale;
    result.sliding_mean.y = sum_y * scale;
    result.sliding_mean.yaw = std::atan2(sum_sin * scale, sum_cos * scale);
    result.sliding_mean.valid = true;
  }

  return result;
}

double CentroidFilter::normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double CentroidFilter::extractYaw(const nav_msgs::msg::Odometry& odom) {
  const double x = odom.pose.pose.orientation.x;
  const double y = odom.pose.pose.orientation.y;
  const double z = odom.pose.pose.orientation.z;
  const double w = odom.pose.pose.orientation.w;

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}
