cmake_minimum_required(VERSION 3.10)

project(raicom2026_x2_autonomy LANGUAGES CXX)

if(EXISTS "/opt/ros/humble")
  list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(RAICOM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(AIMDK_MSGS_INCLUDE_DIR
  "${RAICOM_ROOT}/aimdk-aarch64-1bde262f-artifacts/src/aimdk_msgs/prebuilt_aarch64/include/aimdk_msgs"
)
set(AIMDK_MSGS_LIBRARY_DIR "${RAICOM_ROOT}/mc/lib")

add_executable(raicom2026_x2_autonomy
  src/main.cpp
  src/state_machine.cpp
  src/controller.cpp
  src/planner.cpp
  src/stability_controller.cpp
  src/mc_ros2_controller.cpp
  src/mc_control_inspector.cpp
  src/robot_state_monitor.cpp
)

target_include_directories(raicom2026_x2_autonomy PRIVATE
  "${AIMDK_MSGS_INCLUDE_DIR}"
)

target_link_directories(raicom2026_x2_autonomy PRIVATE
  "${AIMDK_MSGS_LIBRARY_DIR}"
)

ament_target_dependencies(raicom2026_x2_autonomy
  rclcpp
  builtin_interfaces
  nav_msgs
  sensor_msgs
)

target_link_libraries(raicom2026_x2_autonomy
  aimdk_msgs__rosidl_typesupport_cpp
  aimdk_msgs__rosidl_typesupport_fastrtps_cpp
  aimdk_msgs__rosidl_typesupport_introspection_cpp
)

set_target_properties(raicom2026_x2_autonomy PROPERTIES
  BUILD_RPATH "${AIMDK_MSGS_LIBRARY_DIR}"
  INSTALL_RPATH "${AIMDK_MSGS_LIBRARY_DIR}"
)
