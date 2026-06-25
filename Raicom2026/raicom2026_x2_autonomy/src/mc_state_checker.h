#pragma once

#include <aimdk_msgs/srv/set_mc_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

class McStateChecker {
 public:
  explicit McStateChecker(const rclcpp::Node::SharedPtr& node);

  bool setAction(const std::string& action_name, const std::string& source = "rc");
  bool waitForAction(const std::string& action_name, double timeout_seconds);

 private:
  using SetMcAction = aimdk_msgs::srv::SetMcAction;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SetMcAction>::SharedPtr set_action_client_;
  std::string last_confirmed_action_;
};
