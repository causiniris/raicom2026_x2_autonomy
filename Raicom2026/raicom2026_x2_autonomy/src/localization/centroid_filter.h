#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include <cstddef>
#include <deque>

struct FilteredPose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool valid = false;
};

struct CentroidFilterOutput {
  FilteredPose ema;
  FilteredPose sliding_mean;
};

class CentroidFilter {
 public:
  explicit CentroidFilter(double alpha = 0.2, std::size_t window_size = 5);

  void reset();
  CentroidFilterOutput update(const nav_msgs::msg::Odometry& odom);
  CentroidFilterOutput output() const;

 private:
  struct PoseSample {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  static double normalizeAngle(double angle);
  static double extractYaw(const nav_msgs::msg::Odometry& odom);

  double alpha_;
  std::size_t window_size_;
  bool ema_initialized_;
  PoseSample ema_pose_;
  std::deque<PoseSample> window_;
};
