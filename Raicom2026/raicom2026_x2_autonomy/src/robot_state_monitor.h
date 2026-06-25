#pragma once

#include "mc_control_inspector.h"
#include "mc_ros2_controller.h"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

class RobotStateMonitor : public rclcpp::Node {
 public:
  RobotStateMonitor(
      std::shared_ptr<McRos2Controller> controller,
      std::shared_ptr<McControlInspector> inspector);

 private:
  struct TopicStamp {
    std::string type;
    rclcpp::Time last_received;
    bool received = false;
  };

  void discoveryLoop();
  void loggingLoop();
  void reportLoop();
  void subscribeDetectedTopics();
  void subscribeOdomTopic(const std::string& topic);
  void subscribeJointTopic(const std::string& topic);
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& topic);
  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& topic);
  void updateMotionState();
  void writeReport();
  void printDebugDump();
  bool hasRecentOdom() const;
  bool hasRecentJointState() const;
  std::string topicListText() const;
  std::string observedResultText() const;

  std::shared_ptr<McRos2Controller> controller_;
  std::shared_ptr<McControlInspector> inspector_;

  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr logging_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::map<std::string, std::vector<std::string>> known_topics_;
  std::map<std::string, TopicStamp> state_topic_stamps_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_subs_;

  bool odom_received_;
  bool joint_received_;
  bool robot_moving_;
  bool not_moving_warned_;
  bool debug_mode_;

  rclcpp::Time first_control_active_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_joint_time_;
  rclcpp::Time initial_pose_time_;
  rclcpp::Time last_motion_time_;

  double initial_x_;
  double initial_y_;
  double initial_z_;
  double current_x_;
  double current_y_;
  double current_z_;
  double current_vx_;
  double current_vy_;
  double current_vz_;
  double base_delta_;
  double joint_variance_;
};
