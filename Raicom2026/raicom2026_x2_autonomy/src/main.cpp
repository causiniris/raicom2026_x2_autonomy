#include <iostream>
#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>
#include <vector>

#include "mc_state_checker.h"
#include "preset_motion_wrapper.h"
#include "stability_wait_node.h"

namespace {
void preloadAimdkTypesupport(const char* argv0) {
  namespace fs = std::filesystem;

  fs::path exe_path = fs::absolute(argv0);
  if (fs::exists(exe_path)) {
    exe_path = fs::weakly_canonical(exe_path);
  }

  const fs::path autonomy_dir = exe_path.parent_path().parent_path();
  const fs::path lib_dir = autonomy_dir.parent_path() / "mc" / "lib";
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
  auto orchestrator = std::make_shared<rclcpp::Node>("raicom2026_official_mc_orchestrator");
  auto stability_wait = std::make_shared<StabilityWaitNode>();
  McStateChecker mc_state(orchestrator);
  PresetMotionWrapper preset(orchestrator);

  RCLCPP_INFO(orchestrator->get_logger(), "[FLOW] STEP 1 wait for user reset drop");

  auto run_action = [&](const std::string& action_name, double timeout_seconds) {
    RCLCPP_INFO(orchestrator->get_logger(), "[FLOW] SetMcAction %s", action_name.c_str());
    return mc_state.setAction(action_name, "node") &&
           mc_state.waitForAction(action_name, timeout_seconds);
  };

  bool ok = stability_wait->waitForResetDrop(120.0);

  if (ok) {
    RCLCPP_INFO(orchestrator->get_logger(), "[FLOW] STEP 1 set stand action after reset");
    ok = run_action("STAND_DEFAULT", 8.0);
  }

  if (ok) {
    RCLCPP_INFO(orchestrator->get_logger(), "[FLOW] STEP 1 wait stable after STAND_DEFAULT");
    ok = stability_wait->waitUntilStable(3.0, 15.0);
  }

  if (ok) {
    RCLCPP_INFO(
        orchestrator->get_logger(),
        "[STEP1_RESULT] stable_stand=true mc_ready=true detail=STAND_DEFAULT accepted and stability check passed");
  } else {
    RCLCPP_ERROR(
      orchestrator->get_logger(),
      "[STEP1_RESULT] stable_stand=false mc_ready=false detail=reset, stand action, or stability check failed");
  }

  bool locomotion_ready = false;
  if (ok) {
    RCLCPP_INFO(orchestrator->get_logger(), "[FLOW] STEP 2 set locomotion mode only");
    locomotion_ready = run_action("LOCOMOTION_DEFAULT", 8.0);
    if (locomotion_ready) {
      RCLCPP_INFO(
          orchestrator->get_logger(),
          "[STEP2_RESULT] locomotion_mode=true velocity_commanding=false detail=LOCOMOTION_DEFAULT accepted");
      RCLCPP_INFO(
          orchestrator->get_logger(),
          "[FLOW] STEP 3 start filtered zone1 navigation");
      preset.publishZone1Goal();
      RCLCPP_INFO(
          orchestrator->get_logger(),
          "[STEP3_RESULT] filtered_navigation=true zone1_targeting=true detail=PresetMotionWrapper uses filtered pose");
    } else {
      RCLCPP_ERROR(
          orchestrator->get_logger(),
          "[STEP2_RESULT] locomotion_mode=false velocity_commanding=false detail=LOCOMOTION_DEFAULT failed");
    }
  } else {
    RCLCPP_INFO(
        orchestrator->get_logger(),
        "[STEP2_RESULT] locomotion_mode=false velocity_commanding=false detail=skipped because STEP 1 failed");
  }

  RCLCPP_INFO(
      orchestrator->get_logger(),
      "[FLOW] navigation active only after STEP 1 and STEP 2 succeed");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(orchestrator);
  executor.add_node(stability_wait);

  while (rclcpp::ok()) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }

  std::cout << "AUTONOMY SYSTEM STOPPED" << std::endl;
  rclcpp::shutdown();
  return 0;
}
