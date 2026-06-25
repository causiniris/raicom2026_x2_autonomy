#ifndef AIMDK_MSGS__MSG__DETAIL__COMMON_TASK_RESPONSE__TRAITS_HPP_
#define AIMDK_MSGS__MSG__DETAIL__COMMON_TASK_RESPONSE__TRAITS_HPP_

#include <sstream>
#include <string>
#include <type_traits>

#include "aimdk_msgs/msg/detail/common_task_response__struct.hpp"
#include "aimdk_msgs/msg/detail/common_state__traits.hpp"
#include "aimdk_msgs/msg/detail/response_header__traits.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace aimdk_msgs {
namespace msg {

inline void to_flow_style_yaml(const CommonTaskResponse& msg, std::ostream& out) {
  out << "{header: ";
  to_flow_style_yaml(msg.header, out);
  out << ", task_id: ";
  rosidl_generator_traits::value_to_yaml(msg.task_id, out);
  out << ", state: ";
  to_flow_style_yaml(msg.state, out);
  out << "}";
}

inline void to_block_style_yaml(
    const CommonTaskResponse& msg, std::ostream& out, size_t indentation = 0) {
  if (indentation > 0) {
    out << std::string(indentation, ' ');
  }
  out << "header:\n";
  to_block_style_yaml(msg.header, out, indentation + 2);
  if (indentation > 0) {
    out << std::string(indentation, ' ');
  }
  out << "task_id: ";
  rosidl_generator_traits::value_to_yaml(msg.task_id, out);
  out << "\n";
  if (indentation > 0) {
    out << std::string(indentation, ' ');
  }
  out << "state:\n";
  to_block_style_yaml(msg.state, out, indentation + 2);
}

inline std::string to_yaml(const CommonTaskResponse& msg, bool use_flow_style = false) {
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg
}  // namespace aimdk_msgs

namespace rosidl_generator_traits {

template<>
inline const char* data_type<aimdk_msgs::msg::CommonTaskResponse>() {
  return "aimdk_msgs::msg::CommonTaskResponse";
}

template<>
inline const char* name<aimdk_msgs::msg::CommonTaskResponse>() {
  return "aimdk_msgs/msg/CommonTaskResponse";
}

template<>
struct has_fixed_size<aimdk_msgs::msg::CommonTaskResponse>
    : std::integral_constant<bool,
          has_fixed_size<aimdk_msgs::msg::ResponseHeader>::value &&
          has_fixed_size<aimdk_msgs::msg::CommonState>::value> {};

template<>
struct has_bounded_size<aimdk_msgs::msg::CommonTaskResponse>
    : std::integral_constant<bool,
          has_bounded_size<aimdk_msgs::msg::ResponseHeader>::value &&
          has_bounded_size<aimdk_msgs::msg::CommonState>::value> {};

template<>
struct is_message<aimdk_msgs::msg::CommonTaskResponse> : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIMDK_MSGS__MSG__DETAIL__COMMON_TASK_RESPONSE__TRAITS_HPP_
