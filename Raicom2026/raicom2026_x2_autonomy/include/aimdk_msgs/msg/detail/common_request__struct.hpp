#ifndef AIMDK_MSGS__MSG__DETAIL__COMMON_REQUEST__STRUCT_HPP_
#define AIMDK_MSGS__MSG__DETAIL__COMMON_REQUEST__STRUCT_HPP_

#include <memory>

#include "aimdk_msgs/msg/detail/request_header__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"

namespace aimdk_msgs {
namespace msg {

template<class ContainerAllocator>
struct CommonRequest_ {
  using Type = CommonRequest_<ContainerAllocator>;

  explicit CommonRequest_(
      rosidl_runtime_cpp::MessageInitialization init =
          rosidl_runtime_cpp::MessageInitialization::ALL)
      : header(init) {}

  explicit CommonRequest_(
      const ContainerAllocator& alloc,
      rosidl_runtime_cpp::MessageInitialization init =
          rosidl_runtime_cpp::MessageInitialization::ALL)
      : header(alloc, init) {}

  using _header_type = aimdk_msgs::msg::RequestHeader_<ContainerAllocator>;
  _header_type header;

  Type& set__header(const _header_type& arg) {
    header = arg;
    return *this;
  }

  bool operator==(const CommonRequest_& other) const {
    return header == other.header;
  }
  bool operator!=(const CommonRequest_& other) const {
    return !(*this == other);
  }
};

using CommonRequest = CommonRequest_<std::allocator<void>>;

}  // namespace msg
}  // namespace aimdk_msgs

#endif  // AIMDK_MSGS__MSG__DETAIL__COMMON_REQUEST__STRUCT_HPP_
