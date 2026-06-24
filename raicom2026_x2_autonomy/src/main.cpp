#include <iostream>
#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>
#include <vector>

#include "mc_control_inspector.h"
#include "mc_ros2_controller.h"
#include "robot_state_monitor.h"

namespace {
void preloadAimdkTypesupport(const char* argv0) {
  namespace fs = std::filesystem;

  fs::path exe_path = fs::absolute(argv0);
  if (fs::exists(exe_path)) {
    exe_path = fs::weakly_canonical(exe_path);
  }

  const fs::path autonomy_dir = exe_path.parent_path().parent_path();
  const fs::path lib_dir = autonomy_dir.parent_path() / "Raicom2026" / "mc" / "lib";
  const std::vector<std::string> libraries = {
      "libaimdk_msgs__rosidl_typesupport_cpp.so",
      "libaimdk_msgs__rosidl_typesupport_fastrtps_cpp.so",
      "libaimdk_msgs__rosidl_typesupport_introspection_cpp.so",
  };

  for (const auto& library : libraries) {
    const fs::path full_path = lib_dir / library;
    void* handle = dlopen(full_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (handle == nullptr) {
      std::cerr << "Failed to preload " << full_path << ": " << dlerror() << std::endl;
    }
  }
}
}  // namespace

int main(int argc, char** argv) {
  std::filesystem::create_directories("logs");
  setenv("ROS_LOG_DIR", "logs", 0);
  preloadAimdkTypesupport(argv[0]);

  rclcpp::init(argc, argv);

  std::cout << "START AUTONOMY SYSTEM" << std::endl;
  auto controller = std::make_shared<McRos2Controller>();
  auto inspector = std::make_shared<McControlInspector>(controller);
  auto monitor = std::make_shared<RobotStateMonitor>(controller, inspector);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(inspector);
  executor.add_node(monitor);

  while (rclcpp::ok() && !controller->finished()) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }

  std::cout << "AUTONOMY SYSTEM STOPPED" << std::endl;
  rclcpp::shutdown();
  return 0;
}
