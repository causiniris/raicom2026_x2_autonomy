#include "mc_control_inspector.h"

#include <sstream>

namespace {
bool containsAny(const std::string& value, const std::vector<std::string>& needles) {
  for (const auto& needle : needles) {
    if (value.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}
}  // namespace

McControlInspector::McControlInspector(std::shared_ptr<McRos2Controller> controller)
    : Node("McControlInspector"),
      controller_(std::move(controller)),
      printed_no_mc_(false) {
  known_topics_ = get_topic_names_and_types();
  printStartupSanityCheck();

  inspect_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&McControlInspector::inspectLoop, this));
}

McInspectionSnapshot McControlInspector::snapshot(bool robot_moving) const {
  McInspectionSnapshot result;
  result.publish_topic_exists = hasTopic(controller_->velocityTopic());
  result.source_missing = std::string(controller_->sourceName()).empty();
  result.mc_feedback_exists =
      hasTopic("/aima/mc/state") ||
      hasTopic("/aima/mc/arbitration_state") ||
      hasTopic("/aima/mc/common/state") ||
      hasTopic("/aima/mc/leg_odometry");
  result.sim_feedback_exists =
      hasTopic("/aima/hal/state") ||
      hasTopic("/aima/hal/odom/state") ||
      hasTopic("/odom") ||
      hasTopic("/joint_states") ||
      hasTopicPrefix("/aima/hal/joint/");
  result.override_suspected = result.publish_topic_exists && !robot_moving && result.mc_feedback_exists;
  result.topics_text = topicsText();

  if (result.source_missing) {
    result.control_case = ControlCase::COMMAND_IGNORED;
    result.mc_connectivity_status = "UNKNOWN";
    result.arbitration_status_guess = "CONTROL SOURCE MISSING - MC WILL IGNORE COMMAND";
    result.control_rejection_reason = "locomotion velocity message source field is empty";
    return result;
  }

  if (result.publish_topic_exists && result.mc_feedback_exists && result.sim_feedback_exists && robot_moving) {
    result.control_case = ControlCase::FULL_CONTROL_OK;
    result.mc_connectivity_status = "MC topics and sim feedback topics are present";
    result.arbitration_status_guess = "source accepted or motion path active";
    result.control_rejection_reason = "none";
    return result;
  }

  if (!result.mc_feedback_exists) {
    result.control_case = ControlCase::NO_MC_CONNECTION;
    result.mc_connectivity_status = "no MC feedback/arbitration topics detected";
    result.arbitration_status_guess = "not observable";
    result.control_rejection_reason = "MC control layer not active, not connected, or not in the same ROS graph";
    return result;
  }

  result.control_case = ControlCase::COMMAND_IGNORED;
  result.mc_connectivity_status = "MC topics detected";
  result.arbitration_status_guess = "arbitration override or source not registered";
  result.control_rejection_reason = "publish OK but no robot motion detected";
  return result;
}

void McControlInspector::inspectLoop() {
  known_topics_ = get_topic_names_and_types();
  const auto snap = snapshot(false);

  if (!snap.mc_feedback_exists && !printed_no_mc_) {
    RCLCPP_ERROR(get_logger(), "❌ MC CONTROL LAYER NOT ACTIVE OR NOT CONNECTED");
    printed_no_mc_ = true;
  }

  if (snap.source_missing) {
    RCLCPP_ERROR(get_logger(), "CONTROL SOURCE MISSING - MC WILL IGNORE COMMAND");
  }

  RCLCPP_INFO(
      get_logger(),
      "MC_INSPECT case=%s publish=%s mc_feedback=%s sim_feedback=%s source=%s",
      caseName(snap.control_case),
      snap.publish_topic_exists ? "true" : "false",
      snap.mc_feedback_exists ? "true" : "false",
      snap.sim_feedback_exists ? "true" : "false",
      controller_->sourceName());
}

void McControlInspector::printStartupSanityCheck() {
  RCLCPP_INFO(get_logger(), "ROS2 topics at startup:\n%s", topicsText().c_str());
  RCLCPP_INFO(
      get_logger(),
      "MC sanity: publish_topic=%s mc_related=%s sim_feedback=%s",
      hasTopic(controller_->velocityTopic()) ? "true" : "false",
      ((hasTopic("/aima/mc/state") ||
        hasTopic("/aima/mc/arbitration_state") ||
        hasTopic("/aima/mc/common/state") ||
        hasTopic("/aima/mc/leg_odometry")) ? "true" : "false"),
      ((hasTopic("/odom") || hasTopic("/joint_states") || hasTopicPrefix("/aima/hal/")) ? "true" : "false"));
}

bool McControlInspector::hasTopic(const std::string& topic) const {
  return known_topics_.find(topic) != known_topics_.end();
}

bool McControlInspector::hasTopicPrefix(const std::string& prefix) const {
  for (const auto& [topic, _] : known_topics_) {
    if (topic.find(prefix) == 0) {
      return true;
    }
  }
  return false;
}

bool McControlInspector::hasTopicContaining(const std::string& text) const {
  for (const auto& [topic, _] : known_topics_) {
    if (topic.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string McControlInspector::topicsText() const {
  std::ostringstream out;
  for (const auto& [topic, types] : known_topics_) {
    out << "- " << topic;
    if (!types.empty()) {
      out << " [" << types.front() << "]";
    }
    out << "\n";
  }
  return out.str();
}

const char* McControlInspector::caseName(ControlCase control_case) {
  switch (control_case) {
    case ControlCase::FULL_CONTROL_OK:
      return "CASE 1: FULL CONTROL OK";
    case ControlCase::COMMAND_IGNORED:
      return "CASE 2: COMMAND IGNORED";
    case ControlCase::NO_MC_CONNECTION:
      return "CASE 3: NO MC CONNECTION";
  }
  return "UNKNOWN";
}
