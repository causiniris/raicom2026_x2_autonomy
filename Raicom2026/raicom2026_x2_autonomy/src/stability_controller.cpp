#include "stability_controller.h"

#include <algorithm>
#include <cmath>

StabilityController::StabilityController()
    : roll_threshold_(0.22),
      pitch_threshold_(0.22),
      kp_pitch_(0.45),
      kd_pitch_(0.08),
      kp_roll_(0.35),
      kd_roll_(0.06),
      kp_yaw_damping_(0.08),
      max_forward_correction_(0.12),
      max_lateral_correction_(0.08),
      max_yaw_correction_(0.16) {}

StabilityCommand StabilityController::update(const StabilityState& state) {
  StabilityCommand command;
  command.stable = isStable(state);
  command.status = command.stable ? "STABLE" : "UNSTABLE";

  if (!state.has_orientation) {
    return command;
  }

  const double pitch_pd = -(kp_pitch_ * state.pitch + kd_pitch_ * state.pitch_rate);
  const double roll_pd = -(kp_roll_ * state.roll + kd_roll_ * state.roll_rate);
  const double yaw_damping = -kp_yaw_damping_ * state.yaw_rate;

  command.forward_correction = clamp(
      pitch_pd,
      -max_forward_correction_,
      max_forward_correction_);
  command.lateral_correction = clamp(
      roll_pd,
      -max_lateral_correction_,
      max_lateral_correction_);
  command.yaw_correction = clamp(
      yaw_damping,
      -max_yaw_correction_,
      max_yaw_correction_);

  return command;
}

bool StabilityController::isStable(const StabilityState& state) const {
  if (!state.has_orientation || !state.has_height) {
    return false;
  }

  return std::abs(state.roll) <= roll_threshold_ &&
         std::abs(state.pitch) <= pitch_threshold_ &&
         state.base_height > 0.03;
}

double StabilityController::rollThreshold() const {
  return roll_threshold_;
}

double StabilityController::pitchThreshold() const {
  return pitch_threshold_;
}

double StabilityController::clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}
