#include "mc_state_checker.h"

#include <chrono>

McStateChecker::McStateChecker(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
  set_action_client_ = node_->create_client<SetMcAction>("/aimdk_5Fmsgs/srv/SetMcAction");
}

bool McStateChecker::setAction(const std::string& action_name, const std::string& source) {
  const rclcpp::Time wait_start = node_->now();
  while (rclcpp::ok() && !set_action_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[MC_STATE] waiting for SetMcAction service action=%s",
        action_name.c_str());
    if ((node_->now() - wait_start).seconds() > 60.0) {
      break;
    }
  }

  if (!set_action_client_->service_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "[MC_STATE] SetMcAction service unavailable");
    return false;
  }

  auto request = std::make_shared<SetMcAction::Request>();
  request->header.stamp = node_->now();
  request->source = source;
  request->command.action_desc = action_name;

  for (int attempt = 0; attempt < 8; ++attempt) {
    request->header.stamp = node_->now();
    auto future = set_action_client_->async_send_request(request);
    const auto ret = rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(250));
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      continue;
    }
    const auto response = future.get();
    const bool ok = response &&
                    (response->response.header.code == 0 ||
                     response->response.status.value == 1);
    RCLCPP_INFO(
        node_->get_logger(),
        "[MC_STATE] set action=%s ok=%s code=%ld status=%d message=%s",
        action_name.c_str(),
        ok ? "true" : "false",
        response ? response->response.header.code : -1,
        response ? response->response.status.value : -1,
        response ? response->response.message.c_str() : "<null>");
    if (ok) {
      last_confirmed_action_ = action_name;
    }
    return ok;
  }

  RCLCPP_ERROR(node_->get_logger(), "[MC_STATE] SetMcAction timed out action=%s", action_name.c_str());
  return false;
}

bool McStateChecker::waitForAction(const std::string& action_name, double timeout_seconds) {
  const rclcpp::Time start = node_->now();
  while (rclcpp::ok() && (node_->now() - start).seconds() < timeout_seconds) {
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[MC_STATE] confirmed=%s target=%s",
        last_confirmed_action_.c_str(),
        action_name.c_str());
    if (last_confirmed_action_ == action_name) {
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_ERROR(node_->get_logger(), "[MC_STATE] action wait timeout target=%s", action_name.c_str());
  return false;
}
