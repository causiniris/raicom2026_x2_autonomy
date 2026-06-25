#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class StabilityWaitNode : public rclcpp::Node {
 public:
  StabilityWaitNode();

  bool waitForResetDrop(double timeout_seconds);
  bool waitUntilStable(double required_seconds, double timeout_seconds);

 private:
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void updateOrientation(double x, double y, double z, double w);
  bool resetDropNow() const;
  bool stableNow() const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr torso_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr chest_imu_sub_;

  rclcpp::Time last_feedback_time_;
  rclcpp::Time last_sample_time_;
  double height_;
  double previous_height_;
  double roll_;
  double pitch_;
  double previous_roll_;
  double previous_pitch_;
  double height_rate_;
  double roll_rate_;
  double pitch_rate_;
  bool has_height_;
  bool has_orientation_;
  bool has_derivative_;
};
