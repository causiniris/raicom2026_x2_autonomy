#pragma once

#include "autonomy_state.h"
#include "stability_controller.h"

#include <aimdk_msgs/msg/mc_locomotion_velocity.hpp>
#include <aimdk_msgs/msg/upper_body_command_array.hpp>
#include <aimdk_msgs/srv/set_mc_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

struct VelocityCommandSnapshot {
  double forward_velocity;
  double lateral_velocity;
  double yaw_rate;
};

enum class StandUpPhase {
  PRE_STABILIZE,
  SIT_INIT,
  STAND_PROBE,
  SIT_UP,
  STAND_LOCK,
  READY_TO_WALK,
};

class McRos2Controller : public rclcpp::Node {
 public:
  McRos2Controller();

  bool finished() const;
  AutonomyState currentState() const;
  VelocityCommandSnapshot lastCommand() const;
  bool controlActive() const;
  const char* velocityTopic() const;
  const char* sourceName() const;

 private:
  using LocomotionVelocity = aimdk_msgs::msg::McLocomotionVelocity;
  using UpperBodyCommandArray = aimdk_msgs::msg::UpperBodyCommandArray;
  using SetMcAction = aimdk_msgs::srv::SetMcAction;

  void controlLoop();
  void runStandUpPhase();
  void topicDebugLoop();
  void setState(AutonomyState state);
  void setStandUpPhase(StandUpPhase phase, const char* reason);
  void publishVelocity(double forward_velocity, double lateral_velocity, double yaw_rate);
  void publishStabilizationOnly();
  void publishStabilityCheckedVelocity(double forward_velocity, double lateral_velocity, double yaw_rate);
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void updateOrientationFromQuaternion(double x, double y, double z, double w);
  void updateFeedbackDerivative();
  double computeYawPid(double yaw_error);
  void resetWalkReference();
  void updateWalkDelta();
  void updateWalkTargetInBaseFrame();
  bool hasRecentFeedback() const;
  bool isFallDetected() const;
  bool isStandStable() const;
  bool controlGateStable() const;
  bool controlGatePoseConverged() const;
  bool controlGateMotionAllowed() const;
  bool hasStandFeedback() const;
  bool isFeedbackSettledForAction() const;
  bool hasStableStandHeight() const;
  bool hasStableStandOrientation() const;
  bool hasNoStandOscillation() const;
  bool isGetUpActionCompleteByFeedback() const;
  bool isStandReadyToWalk() const;
  void resetOfficialStandAction(const std::string& action_name);
  void requestOfficialStandUpAction();
  void handleStandUpActionResponse(rclcpp::Client<SetMcAction>::SharedFuture future);
  void updateControlGate();
  void logControlGate();
  void publishGuardedStop();
  void publishCommand(double forward_velocity, double lateral_velocity, double yaw_rate);
  void publishCounterbalanceUpperBody(double intensity);
  double smoothAxis(double target, double previous, double alpha, double max_delta) const;
  static double clamp(double value, double min_value, double max_value);
  static double normalizeAngle(double angle);
  void fillHeader(aimdk_msgs::msg::MessageHeader& header);
  void writeControlTopicLog();
  void appendControlStabilityLog(
      double target_forward,
      double target_lateral,
      double target_yaw_rate,
      const char* mode);
  void appendStateTransitionTrace(AutonomyState from, AutonomyState to, const char* reason);
  void appendOdomDeltaLog();
  void appendStandUpPhaseTrace(StandUpPhase from, StandUpPhase to, const char* reason);
  void appendStandUpConvergenceLog();
  static const char* standUpPhaseName(StandUpPhase phase);

  static constexpr const char* kNodeName = "X2AutonomyNode";
  static constexpr const char* kVelocityTopic = "/aima/mc/locomotion/velocity";
  static constexpr const char* kSource = "pnc";

  rclcpp::Publisher<LocomotionVelocity>::SharedPtr velocity_pub_;
  rclcpp::Publisher<UpperBodyCommandArray>::SharedPtr upper_body_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr torso_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr chest_imu_sub_;
  rclcpp::Client<SetMcAction>::SharedPtr set_mc_action_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  StabilityController stability_controller_;
  StabilityState stability_state_;
  StabilityCommand last_stability_command_;

  AutonomyState state_;
  StandUpPhase stand_up_phase_;
  rclcpp::Time state_start_time_;
  rclcpp::Time stand_phase_start_time_;
  rclcpp::Time last_feedback_time_;
  rclcpp::Time stable_since_time_;
  rclcpp::Time last_stand_action_request_time_;
  rclcpp::Time stand_action_accept_time_;
  rclcpp::Time previous_feedback_sample_time_;
  rclcpp::Time control_gate_stable_since_time_;
  uint32_t sequence_;

  double last_forward_velocity_;
  double last_lateral_velocity_;
  double last_yaw_rate_;
  double filtered_forward_velocity_;
  double filtered_lateral_velocity_;
  double filtered_yaw_rate_;
  double current_x_;
  double current_y_;
  double current_z_;
  double walk_start_x_;
  double walk_start_y_;
  double walk_start_yaw_;
  double walk_target_x_;
  double walk_target_y_;
  double walk_forward_delta_;
  double walk_lateral_delta_;
  double target_forward_base_;
  double target_lateral_base_;
  double target_distance_;
  double yaw_error_to_target_;
  double yaw_target_;
  double yaw_error_integral_;
  double previous_yaw_error_;
  double previous_height_;
  double previous_roll_;
  double previous_pitch_;
  double height_rate_;
  double roll_rate_estimate_;
  double pitch_rate_estimate_;
  double stand_probe_start_x_;
  double stand_probe_start_y_;
  double stand_probe_start_height_;
  double stand_probe_start_pitch_;
  bool has_odom_;
  bool has_leg_odom_;
  bool has_feedback_derivative_;
  bool walk_reference_set_;
  bool walk_target_set_;
  bool control_gate_stable_;
  bool control_gate_pose_converged_;
  bool control_gate_motion_allowed_;
  bool previous_yaw_error_valid_;
  bool stand_action_requested_;
  bool stand_action_accepted_;
  bool stand_action_fallback_used_;
  bool safe_damping_requested_;
  bool stand_probe_attempted_;
  std::string stand_action_name_;
};
