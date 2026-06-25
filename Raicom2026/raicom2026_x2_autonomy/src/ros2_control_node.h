#pragma once

#include "autonomy_state.h"

#include <aimdk_msgs/msg/mc_locomotion_velocity.hpp>
#include <aimdk_msgs/msg/upper_body_command_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <map>
#include <string>
#include <vector>

class X2AutonomyNode : public rclcpp::Node {
 public:
  X2AutonomyNode();

  void setState(AutonomyState state);
  AutonomyState state() const;
  bool finished() const;
  void writeControlTopicLog() const;

 private:
  using LocomotionVelocity = aimdk_msgs::msg::McLocomotionVelocity;
  using UpperBodyCommandArray = aimdk_msgs::msg::UpperBodyCommandArray;

  void controlLoop();
  void topicDebugLoop();
  void publishStandUp();
  void publishWalkToZone1();
  void publishDance();
  void publishStop();

  void fillHeader(aimdk_msgs::msg::MessageHeader& header);
  void updateKnownTopics();
  void recordTopicLog(const std::map<std::string, std::vector<std::string>>& topics) const;

  rclcpp::Publisher<LocomotionVelocity>::SharedPtr locomotion_velocity_pub_;
  rclcpp::Publisher<UpperBodyCommandArray>::SharedPtr upper_body_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr topic_debug_timer_;

  AutonomyState state_;
  rclcpp::Time state_start_time_;
  mutable uint32_t sequence_;
  std::map<std::string, std::vector<std::string>> known_topics_;

  std::array<double, 14> neutral_arm_pose_;
};
