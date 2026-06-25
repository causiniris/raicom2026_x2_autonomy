#ifndef AIMDK_MSGS__MSG__DETAIL__COMMON_TASK_RESPONSE__STRUCT_HPP_
#define AIMDK_MSGS__MSG__DETAIL__COMMON_TASK_RESPONSE__STRUCT_HPP_

#include <memory>

#include "aimdk_msgs/msg/detail/common_state__struct.hpp"
#include "aimdk_msgs/msg/detail/response_header__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"

namespace aimdk_msgs {
namespace msg {

template<class ContainerAllocator>
struct CommonTaskResponse_ {
  using Type = CommonTaskResponse_<ContainerAllocator>;

  explicit CommonTaskResponse_(
      rosidl_runtime_cpp::MessageInitialization init =
          rosidl_runtime_cpp::MessageInitialization::ALL)
      : header(init), state(init) {
    if (init == rosidl_runtime_cpp::MessageInitialization::ALL ||
        init == rosidl_runtime_cpp::MessageInitialization::ZERO) {
      task_id = 0;
    }
  }

  explicit CommonTaskResponse_(
      const ContainerAllocator& alloc,
      rosidl_runtime_cpp::MessageInitialization init =
          rosidl_runtime_cpp::MessageInitialization::ALL)
      : header(alloc, init), state(alloc, init) {
    if (init == rosidl_runtime_cpp::MessageInitialization::ALL ||
        init == rosidl_runtime_cpp::MessageInitialization::ZERO) {
      task_id = 0;
    }
  }

  using _header_type = aimdk_msgs::msg::ResponseHeader_<ContainerAllocator>;
  _header_type header;
  using _task_id_type = uint64_t;
  _task_id_type task_id;
  using _state_type = aimdk_msgs::msg::CommonState_<ContainerAllocator>;
  _state_type state;

  Type& set__header(const _header_type& arg) {
    header = arg;
    return *this;
  }
  Type& set__task_id(const uint64_t& arg) {
    task_id = arg;
    return *this;
  }
  Type& set__state(const _state_type& arg) {
    state = arg;
    return *this;
  }

  bool operator==(const CommonTaskResponse_& other) const {
    return header == other.header &&
           task_id == other.task_id &&
           state == other.state;
  }
  bool operator!=(const CommonTaskResponse_& other) const {
    return !(*this == other);
  }
};

using CommonTaskResponse = CommonTaskResponse_<std::allocator<void>>;

}  // namespace msg
}  // namespace aimdk_msgs

#endif  // AIMDK_MSGS__MSG__DETAIL__COMMON_TASK_RESPONSE__STRUCT_HPP_
