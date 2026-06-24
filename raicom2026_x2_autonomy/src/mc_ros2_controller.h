#pragma once

#include "autonomy_state.h"
#include "stability_controller.h"

#include <aimdk_msgs/msg/mc_locomotion_velocity.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

struct VelocityCommandSnapshot {
  double forward_velocity;
  double lateral_velocity;
  double yaw_rate;
};

class McRos2Controller : public rclcpp::Node {
 public:
  McRos2Controller();

  bool finished() const;
  AutonomyState currentState() const;
  VelocityCommandSnapshot lastCommand() const;
  bool controlActive() const;
  const char* velocityTopic() const;
  const char* sourceName() const;

 private:
  using LocomotionVelocity = aimdk_msgs::msg::McLocomotionVelocity;

  void controlLoop();
  void topicDebugLoop();
  void setState(AutonomyState state);
  void publishVelocity(double forward_velocity, double lateral_velocity, double yaw_rate);
  void publishStabilizationOnly();
  void publishStabilityCheckedVelocity(double forward_velocity, double lateral_velocity, double yaw_rate);
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void updateOrientationFromQuaternion(double x, double y, double z, double w);
  double computeHeadingToZone1() const;
  void fillHeader(aimdk_msgs::msg::MessageHeader& header);
  void writeControlTopicLog();

  static constexpr const char* kNodeName = "X2AutonomyNode";
  static constexpr const char* kVelocityTopic = "/aima/mc/locomotion/velocity";
  static constexpr const char* kSource = "raicom_autonomy";

  rclcpp::Publisher<LocomotionVelocity>::SharedPtr velocity_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr torso_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr chest_imu_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  StabilityController stability_controller_;
  StabilityState stability_state_;
  StabilityCommand last_stability_command_;

  AutonomyState state_;
  rclcpp::Time state_start_time_;
  rclcpp::Time last_feedback_time_;
  uint32_t sequence_;

  double last_forward_velocity_;
  double last_lateral_velocity_;
  double last_yaw_rate_;
};
