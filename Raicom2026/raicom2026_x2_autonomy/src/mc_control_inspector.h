#pragma once

#include "mc_ros2_controller.h"

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

enum class ControlCase {
  FULL_CONTROL_OK,
  COMMAND_IGNORED,
  NO_MC_CONNECTION
};

struct McInspectionSnapshot {
  bool publish_topic_exists = false;
  bool mc_feedback_exists = false;
  bool sim_feedback_exists = false;
  bool override_suspected = false;
  bool source_missing = false;
  ControlCase control_case = ControlCase::NO_MC_CONNECTION;
  std::string mc_connectivity_status;
  std::string arbitration_status_guess;
  std::string control_rejection_reason;
  std::string topics_text;
};

class McControlInspector : public rclcpp::Node {
 public:
  explicit McControlInspector(std::shared_ptr<McRos2Controller> controller);

  McInspectionSnapshot snapshot(bool robot_moving) const;

 private:
  void inspectLoop();
  void printStartupSanityCheck();
  bool hasTopic(const std::string& topic) const;
  bool hasTopicPrefix(const std::string& prefix) const;
  bool hasTopicContaining(const std::string& text) const;
  std::string topicsText() const;
  static const char* caseName(ControlCase control_case);

  std::shared_ptr<McRos2Controller> controller_;
  rclcpp::TimerBase::SharedPtr inspect_timer_;
  std::map<std::string, std::vector<std::string>> known_topics_;
  bool printed_no_mc_;
};
