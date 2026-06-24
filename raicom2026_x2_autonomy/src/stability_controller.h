#pragma once

#include <string>

struct StabilityState {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double base_height = 0.0;
  double roll_rate = 0.0;
  double pitch_rate = 0.0;
  double yaw_rate = 0.0;
  bool has_orientation = false;
  bool has_height = false;
  bool has_joint_state = false;
};

struct StabilityCommand {
  double forward_correction = 0.0;
  double lateral_correction = 0.0;
  double yaw_correction = 0.0;
  bool stable = false;
  std::string status;
};

class StabilityController {
 public:
  StabilityController();

  StabilityCommand update(const StabilityState& state);
  bool isStable(const StabilityState& state) const;

  double rollThreshold() const;
  double pitchThreshold() const;

 private:
  static double clamp(double value, double min_value, double max_value);

  double roll_threshold_;
  double pitch_threshold_;
  double kp_pitch_;
  double kd_pitch_;
  double kp_roll_;
  double kd_roll_;
  double kp_yaw_damping_;
  double max_forward_correction_;
  double max_lateral_correction_;
  double max_yaw_correction_;
};
