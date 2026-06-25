#pragma once

#include "localization/centroid_filter.h"

#include <aimdk_msgs/msg/mc_locomotion_velocity.hpp>
#include <aimdk_msgs/srv/set_mc_input_source.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

class PresetMotionWrapper {
 public:
  explicit PresetMotionWrapper(const rclcpp::Node::SharedPtr& node);

  bool runPresetMotion(int area_id, int motion_id, bool interrupt = false);
  void publishZone1Goal();

 private:
  using LocomotionVelocity = aimdk_msgs::msg::McLocomotionVelocity;
  using SetMcInputSource = aimdk_msgs::srv::SetMcInputSource;

  bool registerInputSource();
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishNavigationVelocity();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<LocomotionVelocity>::SharedPtr locomotion_pub_;
  rclcpp::Client<SetMcInputSource>::SharedPtr input_source_client_;
  rclcpp::TimerBase::SharedPtr navigation_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr torso_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr chest_imu_sub_;
  std::string input_source_name_;
  CentroidFilter pose_filter_;
  double target_x_;
  double target_y_;
  double current_x_;
  double current_y_;
  double current_yaw_;
  double raw_x_;
  double raw_y_;
  double raw_yaw_;
  bool has_odom_;
  bool has_yaw_;
  bool input_source_registered_;
  bool navigation_active_;
};
