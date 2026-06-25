#include "mc_ros2_controller.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kControlPeriodSeconds = 0.02;
constexpr double kFeedbackTimeoutSeconds = 0.5;
constexpr double kStandReadySeconds = 5.0;
constexpr double kControlGatePoseConvergedSeconds = 3.0;
constexpr double kWalkTargetForwardMeters = 0.45;
constexpr double kZone1WorldDeltaX = 0.45;
constexpr double kZone1WorldDeltaY = 0.0;
constexpr double kWalkTargetToleranceMeters = 0.04;
constexpr double kWalkLateralToleranceMeters = 0.06;
constexpr double kWalkYawStartThreshold = 0.18;
constexpr double kWalkYawSlowThreshold = 0.35;
constexpr double kWalkTimeoutSeconds = 20.0;
constexpr double kMaxForwardVelocity = 0.16;
constexpr double kMaxLateralVelocity = 0.04;
constexpr double kMaxYawRate = 0.10;
constexpr double kFilterAlpha = 0.12;
constexpr double kMaxForwardStep = 0.006;
constexpr double kMaxLateralStep = 0.004;
constexpr double kMaxYawStep = 0.006;
constexpr double kYawKp = 0.55;
constexpr double kYawKi = 0.02;
constexpr double kYawKd = 0.04;
constexpr double kYawIntegralLimit = 0.25;
constexpr double kLateralKp = 0.22;
constexpr double kFallRollPitchRadians = 0.42;
constexpr double kFallAngularRate = 2.2;
constexpr double kFallMinHeight = 0.035;
constexpr double kStandMinHeight = 0.25;
constexpr double kStandRollPitchThreshold = 0.22;
constexpr double kGetUpCompletionTiltThreshold = 0.70;
constexpr double kPreStabilizeMaxTiltRate = 1.2;
constexpr double kGetUpCompletionMaxHeightRate = 0.08;
constexpr double kGetUpCompletionMaxTiltRate = 0.80;
constexpr double kPreActionMaxHeightRate = 0.03;
constexpr double kStandMaxHeightRate = 0.025;
constexpr double kStandMaxTiltRate = 0.16;
constexpr double kStandMaxImuTiltRate = 0.35;
constexpr double kLowPoseAbortHeight = 0.12;
constexpr double kGetUpStallSeconds = 6.0;
constexpr double kGetUpStallHeight = 0.10;
constexpr double kGetUpStallPitch = 0.35;
constexpr bool kEnableAutomaticSitUp = false;
constexpr bool kEnablePostureInit = false;
constexpr bool kEnableStandProbe = true;
constexpr double kStandProbeDelaySeconds = 1.0;
constexpr double kStandProbeTimeoutSeconds = 2.0;
constexpr double kStandProbeMaxPitch = 0.18;
constexpr double kStandProbeMaxRoll = 0.22;
constexpr double kStandProbeMaxBackwardDrift = 0.025;
constexpr double kStandProbeMaxHeightDrop = 0.025;

bool isControlTopic(const std::string& topic) {
  return topic.find("locomotion") != std::string::npos ||
         topic.find("velocity") != std::string::npos ||
         topic.find("command") != std::string::npos ||
         topic.find("control") != std::string::npos;
}
}  // namespace

McRos2Controller::McRos2Controller()
    : Node(kNodeName),
      state_(AutonomyState::STAND_UP),
      stand_up_phase_(StandUpPhase::PRE_STABILIZE),
      state_start_time_(now()),
      stand_phase_start_time_(now()),
      last_feedback_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      stable_since_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      last_stand_action_request_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      stand_action_accept_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      previous_feedback_sample_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      control_gate_stable_since_time_(rclcpp::Time(0, 0, get_clock()->get_clock_type())),
      sequence_(0),
      last_forward_velocity_(0.0),
      last_lateral_velocity_(0.0),
      last_yaw_rate_(0.0),
      filtered_forward_velocity_(0.0),
      filtered_lateral_velocity_(0.0),
      filtered_yaw_rate_(0.0),
      current_x_(0.0),
      current_y_(0.0),
      current_z_(0.0),
      walk_start_x_(0.0),
      walk_start_y_(0.0),
      walk_start_yaw_(0.0),
      walk_target_x_(0.0),
      walk_target_y_(0.0),
      walk_forward_delta_(0.0),
      walk_lateral_delta_(0.0),
      target_forward_base_(0.0),
      target_lateral_base_(0.0),
      target_distance_(0.0),
      yaw_error_to_target_(0.0),
      yaw_target_(0.0),
      yaw_error_integral_(0.0),
      previous_yaw_error_(0.0),
      previous_height_(0.0),
      previous_roll_(0.0),
      previous_pitch_(0.0),
      height_rate_(0.0),
      roll_rate_estimate_(0.0),
      pitch_rate_estimate_(0.0),
      stand_probe_start_x_(0.0),
      stand_probe_start_y_(0.0),
      stand_probe_start_height_(0.0),
      stand_probe_start_pitch_(0.0),
      has_odom_(false),
      has_leg_odom_(false),
      has_feedback_derivative_(false),
      walk_reference_set_(false),
      walk_target_set_(false),
      control_gate_stable_(false),
      control_gate_pose_converged_(false),
      control_gate_motion_allowed_(false),
      previous_yaw_error_valid_(false),
      stand_action_requested_(false),
      stand_action_accepted_(false),
      stand_action_fallback_used_(false),
      safe_damping_requested_(false),
      stand_probe_attempted_(false),
      stand_action_name_() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto leg_odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

  velocity_pub_ = create_publisher<LocomotionVelocity>(kVelocityTopic, qos);
  upper_body_pub_ = create_publisher<UpperBodyCommandArray>("/mc/upper_body_command", qos);
  set_mc_action_client_ = create_client<SetMcAction>("/aimdk_5Fmsgs/srv/SetMcAction");
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aima/hal/odom/state",
      qos,
      std::bind(&McRos2Controller::handleOdom, this, std::placeholders::_1));
  leg_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aima/mc/leg_odometry",
      leg_odom_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        has_leg_odom_ = true;
        handleOdom(msg);
      });
  torso_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/torso/state",
      qos,
      std::bind(&McRos2Controller::handleImu, this, std::placeholders::_1));
  chest_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/aima/hal/imu/chest/state",
      qos,
      std::bind(&McRos2Controller::handleImu, this, std::placeholders::_1));

  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodSeconds * 1000.0)),
      std::bind(&McRos2Controller::controlLoop, this));
  debug_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&McRos2Controller::topicDebugLoop, this));

  writeControlTopicLog();
  std::ofstream("logs/control_stability.log", std::ios::trunc)
      << "time state mode target filtered roll pitch yaw height stable pose_converged motion_allowed yaw_error fall\n";
  std::ofstream("logs/state_transition_trace.log", std::ios::trunc)
      << "time from to reason odom_delta roll pitch height\n";
  std::ofstream("logs/odom_delta_tracking.log", std::ios::trunc)
      << "time state position walk_delta target_forward forward_error lateral_error yaw_error\n";
  std::ofstream("logs/stand_up_phase_trace.log", std::ios::trunc)
      << "time from to reason action height roll pitch height_rate roll_rate pitch_rate ready\n";
  std::ofstream("logs/stand_up_convergence.log", std::ios::trunc)
      << "time phase action accepted leg_odom height roll pitch height_rate roll_rate pitch_rate imu_rates get_up_complete stable_height stable_orientation no_oscillation ready stable_for\n";
  RCLCPP_INFO(get_logger(), "START AUTONOMY SYSTEM");
  RCLCPP_INFO(get_logger(), "MC velocity topic: %s", kVelocityTopic);
  RCLCPP_INFO(get_logger(), "MC source: %s", kSource);
  RCLCPP_WARN(
      get_logger(),
      "STAND_UP safe mode: DAMPING_DEFAULT hold, then one guarded STAND_DEFAULT probe with drift/tilt abort.");
  RCLCPP_INFO(get_logger(), "STATE: %s", stateName(state_));
}

bool McRos2Controller::finished() const {
  return state_ == AutonomyState::DONE;
}

AutonomyState McRos2Controller::currentState() const {
  return state_;
}

VelocityCommandSnapshot McRos2Controller::lastCommand() const {
  return {last_forward_velocity_, last_lateral_velocity_, last_yaw_rate_};
}

bool McRos2Controller::controlActive() const {
  return std::abs(last_forward_velocity_) > 0.01 ||
         std::abs(last_lateral_velocity_) > 0.01 ||
         std::abs(last_yaw_rate_) > 0.01;
}

const char* McRos2Controller::velocityTopic() const {
  return kVelocityTopic;
}

const char* McRos2Controller::sourceName() const {
  return kSource;
}

void McRos2Controller::controlLoop() {
  const double elapsed = (now() - state_start_time_).seconds();
  last_stability_command_ = stability_controller_.update(stability_state_);
  if (state_ == AutonomyState::WALK_TO_ZONE_1) {
    updateWalkTargetInBaseFrame();
  }
  updateControlGate();
  logControlGate();
  appendControlStabilityLog(0.0, 0.0, 0.0, "CONTROL_GATE");

  if (state_ != AutonomyState::STAND_UP &&
      isFallDetected() &&
      state_ != AutonomyState::FALL_GUARD) {
    setState(AutonomyState::FALL_GUARD);
    publishGuardedStop();
    return;
  }

  if (state_ == AutonomyState::FALL_GUARD) {
    publishGuardedStop();
    if (isStandStable() && elapsed >= 3.0) {
      setState(AutonomyState::STAND_UP);
    }
    return;
  }

  if (state_ == AutonomyState::STAND_UP) {
    publishCounterbalanceUpperBody(1.0);
    const bool action_stalled_low =
        (now() - stand_phase_start_time_).seconds() >= kGetUpStallSeconds &&
        stand_up_phase_ == StandUpPhase::SIT_UP &&
        stability_state_.base_height < kGetUpStallHeight &&
        std::abs(stability_state_.pitch) > kGetUpStallPitch;
    const bool stand_lock_collapsed =
        (now() - stand_phase_start_time_).seconds() >= 1.5 &&
        stand_up_phase_ == StandUpPhase::STAND_LOCK &&
        stability_state_.base_height < kLowPoseAbortHeight;
    if (stand_up_phase_ != StandUpPhase::PRE_STABILIZE &&
        (isFallDetected() || action_stalled_low || stand_lock_collapsed)) {
      if (!safe_damping_requested_) {
        resetOfficialStandAction("DAMPING_DEFAULT");
        safe_damping_requested_ = true;
        RCLCPP_ERROR(
            get_logger(),
            "[STAND_UP] aborting get-up and switching to DAMPING_DEFAULT height=%.3f roll=%.3f pitch=%.3f",
            stability_state_.base_height,
            stability_state_.roll,
            stability_state_.pitch);
      }
      requestOfficialStandUpAction();
      return;
    }
    runStandUpPhase();
    return;
  }

  if (state_ == AutonomyState::WALK_TO_ZONE_1) {
    updateWalkTargetInBaseFrame();
    const double forward_error = target_forward_base_;
    const double yaw_cmd = computeYawPid(yaw_error_to_target_);
    const double yaw_abs = std::abs(yaw_error_to_target_);
    const double forward_scale =
        yaw_abs > kWalkYawSlowThreshold ? 0.0 :
        (yaw_abs > kWalkYawStartThreshold ? 0.35 : 1.0);
    const double forward_cmd = forward_scale * clamp(0.80 * forward_error, 0.0, kMaxForwardVelocity);
    const double lateral_cmd = clamp(kLateralKp * target_lateral_base_, -kMaxLateralVelocity, kMaxLateralVelocity);
    publishStabilityCheckedVelocity(forward_cmd, lateral_cmd, yaw_cmd);

    const bool target_reached =
        target_distance_ <= kWalkTargetToleranceMeters &&
        std::abs(target_lateral_base_) <= kWalkLateralToleranceMeters;
    if (target_reached) {
      setState(AutonomyState::DONE);
    } else if (elapsed >= kWalkTimeoutSeconds) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[ODOM_TRACK] walking beyond nominal timeout target=%.3f forward_delta=%.3f lateral_delta=%.3f",
          kWalkTargetForwardMeters,
          walk_forward_delta_,
          walk_lateral_delta_);
    }
    return;
  }

  if (state_ == AutonomyState::DANCE) {
    publishCommand(0.0, 0.0, 0.0);
    return;
  }

  publishCommand(0.0, 0.0, 0.0);
}

void McRos2Controller::runStandUpPhase() {
  appendStandUpConvergenceLog();

  switch (stand_up_phase_) {
    case StandUpPhase::PRE_STABILIZE:
      if (!hasStandFeedback()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[STAND_UP][PRE_STABILIZE] waiting for odom and IMU feedback leg_odom=%s",
            has_leg_odom_ ? "true" : "false");
        return;
      }
      if (!has_leg_odom_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[STAND_UP][PRE_STABILIZE] /aima/mc/leg_odometry has no samples; using available odom feedback for gate");
      }
      if (!isFeedbackSettledForAction()) {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[STAND_UP][PRE_STABILIZE] feedback not settled height=%.3f roll=%.3f pitch=%.3f rates=(%.3f, %.3f, %.3f)",
            stability_state_.base_height,
            stability_state_.roll,
            stability_state_.pitch,
            height_rate_,
            roll_rate_estimate_,
            pitch_rate_estimate_);
        return;
      }
      if (has_leg_odom_ && hasStableStandHeight() && hasStableStandOrientation()) {
        resetOfficialStandAction("STAND_DEFAULT");
        setStandUpPhase(StandUpPhase::STAND_LOCK, "already_upright_stable_height");
        return;
      }
      resetOfficialStandAction(kEnablePostureInit ? "SIT_JOINT_DEFAULT" : "DAMPING_DEFAULT");
      setStandUpPhase(StandUpPhase::SIT_INIT, "feedback_settled_crouch_pose");
      return;

    case StandUpPhase::SIT_INIT:
      requestOfficialStandUpAction();
      if (!kEnablePostureInit) {
        if (kEnableStandProbe &&
            !stand_probe_attempted_ &&
            stand_action_accepted_ &&
            (now() - stand_phase_start_time_).seconds() >= kStandProbeDelaySeconds &&
            isFeedbackSettledForAction()) {
          stand_probe_attempted_ = true;
          stand_probe_start_x_ = current_x_;
          stand_probe_start_y_ = current_y_;
          stand_probe_start_height_ = stability_state_.base_height;
          stand_probe_start_pitch_ = stability_state_.pitch;
          resetOfficialStandAction("STAND_DEFAULT");
          setStandUpPhase(StandUpPhase::STAND_PROBE, "guarded_stand_probe_start");
          return;
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[STAND_UP][SAFE_HOLD] holding DAMPING_DEFAULT; stand_probe_attempted=%s posture/get-up actions disabled.",
            stand_probe_attempted_ ? "true" : "false");
        return;
      }
      if (!kEnableAutomaticSitUp) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[STAND_UP][SAFE_HOLD] holding SIT_JOINT_DEFAULT; automatic SIT_UP_DEFAULT is disabled after backward-fall/fence contact.");
        return;
      }
      if (stand_action_accepted_ &&
          (now() - stand_phase_start_time_).seconds() >= 1.0 &&
          isFeedbackSettledForAction()) {
        resetOfficialStandAction("SIT_UP_DEFAULT");
        setStandUpPhase(StandUpPhase::SIT_UP, "sit_joint_ready");
      }
      return;

    case StandUpPhase::STAND_PROBE: {
      requestOfficialStandUpAction();
      const double elapsed = (now() - stand_phase_start_time_).seconds();
      const double backward_drift = std::abs(current_y_ - stand_probe_start_y_);
      const double height_drop = stand_probe_start_height_ - stability_state_.base_height;
      const bool unsafe_probe =
          std::abs(stability_state_.pitch) > kStandProbeMaxPitch ||
          std::abs(stability_state_.roll) > kStandProbeMaxRoll ||
          backward_drift > kStandProbeMaxBackwardDrift ||
          height_drop > kStandProbeMaxHeightDrop ||
          isFallDetected();
      if (unsafe_probe) {
        RCLCPP_ERROR(
            get_logger(),
            "[STAND_UP][STAND_PROBE] abort: height=%.3f roll=%.3f pitch=%.3f dy=%.3f height_drop=%.3f",
            stability_state_.base_height,
            stability_state_.roll,
            stability_state_.pitch,
            current_y_ - stand_probe_start_y_,
            height_drop);
        resetOfficialStandAction("DAMPING_DEFAULT");
        setStandUpPhase(StandUpPhase::SIT_INIT, "stand_probe_unsafe_damping");
        return;
      }
      if (stand_action_accepted_ && isGetUpActionCompleteByFeedback()) {
        resetOfficialStandAction("STAND_DEFAULT");
        setStandUpPhase(StandUpPhase::STAND_LOCK, "stand_probe_feedback_complete");
        return;
      }
      if (elapsed >= kStandProbeTimeoutSeconds) {
        RCLCPP_WARN(
            get_logger(),
            "[STAND_UP][STAND_PROBE] timeout without standing: height=%.3f roll=%.3f pitch=%.3f",
            stability_state_.base_height,
            stability_state_.roll,
            stability_state_.pitch);
        resetOfficialStandAction("DAMPING_DEFAULT");
        setStandUpPhase(StandUpPhase::SIT_INIT, "stand_probe_timeout_damping");
      }
      return;
    }

    case StandUpPhase::SIT_UP:
      requestOfficialStandUpAction();
      if (stand_action_accepted_ && isGetUpActionCompleteByFeedback()) {
        resetOfficialStandAction("STAND_DEFAULT");
        setStandUpPhase(StandUpPhase::STAND_LOCK, "sit_up_feedback_complete");
      }
      return;

    case StandUpPhase::STAND_LOCK:
      requestOfficialStandUpAction();
      if (stand_action_accepted_ && controlGateMotionAllowed()) {
        stable_since_time_ = now();
        setStandUpPhase(StandUpPhase::READY_TO_WALK, "posture_converged");
      }
      return;

    case StandUpPhase::READY_TO_WALK:
      if (!controlGateMotionAllowed()) {
        stable_since_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        resetOfficialStandAction("STAND_DEFAULT");
        setStandUpPhase(StandUpPhase::STAND_LOCK, "ready_gate_broken");
        return;
      }
      if (stable_since_time_.nanoseconds() == 0) {
        stable_since_time_ = now();
        return;
      }
      if ((now() - stable_since_time_).seconds() >= kStandReadySeconds) {
        resetWalkReference();
        setState(AutonomyState::WALK_TO_ZONE_1);
      }
      return;
  }
}

void McRos2Controller::topicDebugLoop() {
  writeControlTopicLog();
  RCLCPP_INFO(
      get_logger(),
      "topic=%s state=%s stand_phase=%s source=%s forward=%.3f lateral=%.3f yaw_rate=%.3f stability=%s roll=%.3f pitch=%.3f height=%.3f rates=(%.3f, %.3f, %.3f) odom_delta=(%.3f, %.3f) yaw_error=%.3f",
      kVelocityTopic,
      stateName(state_),
      state_ == AutonomyState::STAND_UP ? standUpPhaseName(stand_up_phase_) : "-",
      kSource,
      last_forward_velocity_,
      last_lateral_velocity_,
      last_yaw_rate_,
      last_stability_command_.status.c_str(),
      stability_state_.roll,
      stability_state_.pitch,
      stability_state_.base_height,
      height_rate_,
      roll_rate_estimate_,
      pitch_rate_estimate_,
      walk_forward_delta_,
      walk_lateral_delta_,
      normalizeAngle(yaw_target_ - stability_state_.yaw));
  appendOdomDeltaLog();
}

void McRos2Controller::setState(AutonomyState state) {
  if (state_ == state) {
    return;
  }

  const AutonomyState previous = state_;
  state_ = state;
  state_start_time_ = now();
  if (state_ == AutonomyState::STAND_UP) {
    stand_up_phase_ = StandUpPhase::PRE_STABILIZE;
    stand_phase_start_time_ = now();
    stable_since_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    control_gate_stable_since_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_stand_action_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    stand_action_accept_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    stand_action_requested_ = false;
    stand_action_accepted_ = false;
    stand_action_fallback_used_ = false;
    safe_damping_requested_ = false;
    stand_probe_attempted_ = false;
    stand_action_name_.clear();
    walk_target_set_ = false;
  }
  if (state_ == AutonomyState::WALK_TO_ZONE_1) {
    resetWalkReference();
  }
  appendStateTransitionTrace(previous, state_, "controller_guarded_transition");
  RCLCPP_INFO(get_logger(), "STATE: %s", stateName(state_));
}

void McRos2Controller::setStandUpPhase(StandUpPhase phase, const char* reason) {
  if (stand_up_phase_ == phase) {
    return;
  }

  const StandUpPhase previous = stand_up_phase_;
  stand_up_phase_ = phase;
  stand_phase_start_time_ = now();
  appendStandUpPhaseTrace(previous, phase, reason);
  RCLCPP_INFO(
      get_logger(),
      "[STAND_UP] PHASE: %s -> %s reason=%s height=%.3f roll=%.3f pitch=%.3f rates=(%.3f, %.3f, %.3f) action=%s",
      standUpPhaseName(previous),
      standUpPhaseName(phase),
      reason,
      stability_state_.base_height,
      stability_state_.roll,
      stability_state_.pitch,
      height_rate_,
      roll_rate_estimate_,
      pitch_rate_estimate_,
      stand_action_name_.c_str());
}

void McRos2Controller::publishVelocity(
    double forward_velocity,
    double lateral_velocity,
    double yaw_rate) {
  const double saturated_forward = clamp(forward_velocity, -kMaxForwardVelocity, kMaxForwardVelocity);
  const double saturated_lateral = clamp(lateral_velocity, -kMaxLateralVelocity, kMaxLateralVelocity);
  const double saturated_yaw_rate = clamp(yaw_rate, -kMaxYawRate, kMaxYawRate);

  filtered_forward_velocity_ = smoothAxis(
      saturated_forward, filtered_forward_velocity_, kFilterAlpha, kMaxForwardStep);
  filtered_lateral_velocity_ = smoothAxis(
      saturated_lateral, filtered_lateral_velocity_, kFilterAlpha, kMaxLateralStep);
  filtered_yaw_rate_ = smoothAxis(
      saturated_yaw_rate, filtered_yaw_rate_, kFilterAlpha, kMaxYawStep);

  LocomotionVelocity command;
  fillHeader(command.header);
  command.source = kSource;
  command.forward_velocity = filtered_forward_velocity_;
  command.lateral_velocity = filtered_lateral_velocity_;
  command.angular_velocity = filtered_yaw_rate_;

  velocity_pub_->publish(command);

  last_forward_velocity_ = filtered_forward_velocity_;
  last_lateral_velocity_ = filtered_lateral_velocity_;
  last_yaw_rate_ = filtered_yaw_rate_;
}

void McRos2Controller::publishStabilizationOnly() {
  publishCommand(0.0, 0.0, 0.0);
  appendControlStabilityLog(
      0.0,
      0.0,
      0.0,
      "STABILIZE_ONLY");

  RCLCPP_INFO(
      get_logger(),
      "[STABILITY] roll=%.3f pitch=%.3f height=%.3f status=%s mode=STABILIZE_ONLY command=(0.000, 0.000, 0.000)",
      stability_state_.roll,
      stability_state_.pitch,
      stability_state_.base_height,
      last_stability_command_.status.c_str());
}

void McRos2Controller::publishStabilityCheckedVelocity(
    double forward_velocity,
    double lateral_velocity,
    double yaw_rate) {
  if (!controlGateMotionAllowed()) {
    publishCommand(0.0, 0.0, 0.0);
    appendControlStabilityLog(
        0.0,
        0.0,
        0.0,
        "GATE_BLOCK");
    RCLCPP_INFO(
        get_logger(),
        "[CONTROL GATE] stable=%s pose_converged=%s motion_allowed=NO yaw_error=%.3f height=%.3f command=(0.000, 0.000, 0.000)",
        control_gate_stable_ ? "YES" : "NO",
        control_gate_pose_converged_ ? "YES" : "NO",
        yaw_error_to_target_,
        stability_state_.base_height);
    RCLCPP_INFO(
        get_logger(),
        "[STABILITY] roll=%.3f pitch=%.3f height=%.3f mode=GATE_BLOCK command=(0.000, 0.000, 0.000)",
        stability_state_.roll,
        stability_state_.pitch,
        stability_state_.base_height);
    return;
  }

  publishCommand(
      forward_velocity,
      lateral_velocity,
      yaw_rate);
  appendControlStabilityLog(
      forward_velocity,
      lateral_velocity,
      yaw_rate,
      "MOTION_ALLOWED");
  RCLCPP_INFO(
      get_logger(),
      "[STABILITY] roll=%.3f pitch=%.3f height=%.3f status=STABLE mode=MOTION_ALLOWED target=(%.3f, %.3f, %.3f)",
      stability_state_.roll,
      stability_state_.pitch,
      stability_state_.base_height,
      forward_velocity,
      lateral_velocity,
      yaw_rate);
}

void McRos2Controller::handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_z_ = msg->pose.pose.position.z;
  has_odom_ = true;
  stability_state_.base_height = msg->pose.pose.position.z;
  stability_state_.has_height = true;
  stability_state_.yaw_rate = msg->twist.twist.angular.z;
  updateOrientationFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  updateFeedbackDerivative();
  last_feedback_time_ = now();
  updateWalkDelta();
}

void McRos2Controller::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  stability_state_.roll_rate = msg->angular_velocity.x;
  stability_state_.pitch_rate = msg->angular_velocity.y;
  stability_state_.yaw_rate = msg->angular_velocity.z;
  updateOrientationFromQuaternion(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  updateFeedbackDerivative();
  last_feedback_time_ = now();
}

void McRos2Controller::updateOrientationFromQuaternion(double x, double y, double z, double w) {
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  stability_state_.roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    stability_state_.pitch = std::copysign(kPi / 2.0, sinp);
  } else {
    stability_state_.pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  stability_state_.yaw = std::atan2(siny_cosp, cosy_cosp);
  stability_state_.has_orientation = true;
}

void McRos2Controller::updateFeedbackDerivative() {
  if (!stability_state_.has_orientation || !stability_state_.has_height) {
    return;
  }

  const rclcpp::Time sample_time = now();
  if (previous_feedback_sample_time_.nanoseconds() == 0) {
    previous_feedback_sample_time_ = sample_time;
    previous_height_ = stability_state_.base_height;
    previous_roll_ = stability_state_.roll;
    previous_pitch_ = stability_state_.pitch;
    return;
  }

  const double dt = (sample_time - previous_feedback_sample_time_).seconds();
  if (dt < 0.005) {
    return;
  }

  height_rate_ = (stability_state_.base_height - previous_height_) / dt;
  roll_rate_estimate_ = normalizeAngle(stability_state_.roll - previous_roll_) / dt;
  pitch_rate_estimate_ = (stability_state_.pitch - previous_pitch_) / dt;
  previous_feedback_sample_time_ = sample_time;
  previous_height_ = stability_state_.base_height;
  previous_roll_ = stability_state_.roll;
  previous_pitch_ = stability_state_.pitch;
  has_feedback_derivative_ = true;
}

double McRos2Controller::computeYawPid(double yaw_error) {
  yaw_error = normalizeAngle(yaw_error);
  yaw_error_integral_ = clamp(
      yaw_error_integral_ + yaw_error * kControlPeriodSeconds,
      -kYawIntegralLimit,
      kYawIntegralLimit);
  const double derivative =
      previous_yaw_error_valid_ ? (yaw_error - previous_yaw_error_) / kControlPeriodSeconds : 0.0;
  previous_yaw_error_ = yaw_error;
  previous_yaw_error_valid_ = true;
  return clamp(
      kYawKp * yaw_error + kYawKi * yaw_error_integral_ + kYawKd * derivative,
      -kMaxYawRate,
      kMaxYawRate);
}

void McRos2Controller::resetWalkReference() {
  walk_start_x_ = current_x_;
  walk_start_y_ = current_y_;
  walk_start_yaw_ = stability_state_.yaw;
  walk_target_x_ = walk_start_x_ + kZone1WorldDeltaX;
  walk_target_y_ = walk_start_y_ + kZone1WorldDeltaY;
  walk_target_set_ = true;
  yaw_target_ = std::atan2(walk_target_y_ - current_y_, walk_target_x_ - current_x_);
  yaw_error_integral_ = 0.0;
  previous_yaw_error_ = 0.0;
  previous_yaw_error_valid_ = false;
  walk_reference_set_ = has_odom_;
  updateWalkDelta();
  updateWalkTargetInBaseFrame();
}

void McRos2Controller::updateWalkDelta() {
  if (!walk_reference_set_ || !has_odom_) {
    walk_forward_delta_ = 0.0;
    walk_lateral_delta_ = 0.0;
    return;
  }

  const double dx = current_x_ - walk_start_x_;
  const double dy = current_y_ - walk_start_y_;
  const double cos_yaw = std::cos(walk_start_yaw_);
  const double sin_yaw = std::sin(walk_start_yaw_);
  walk_forward_delta_ = dx * cos_yaw + dy * sin_yaw;
  walk_lateral_delta_ = -dx * sin_yaw + dy * cos_yaw;
}

void McRos2Controller::updateWalkTargetInBaseFrame() {
  if (!walk_target_set_ || !has_odom_) {
    target_forward_base_ = kWalkTargetForwardMeters;
    target_lateral_base_ = 0.0;
    target_distance_ = kWalkTargetForwardMeters;
    yaw_error_to_target_ = 0.0;
    return;
  }

  const double dx = walk_target_x_ - current_x_;
  const double dy = walk_target_y_ - current_y_;
  const double cos_yaw = std::cos(stability_state_.yaw);
  const double sin_yaw = std::sin(stability_state_.yaw);
  target_forward_base_ = dx * cos_yaw + dy * sin_yaw;
  target_lateral_base_ = -dx * sin_yaw + dy * cos_yaw;
  target_distance_ = std::hypot(dx, dy);
  yaw_target_ = std::atan2(dy, dx);
  yaw_error_to_target_ = normalizeAngle(yaw_target_ - stability_state_.yaw);
}

bool McRos2Controller::hasRecentFeedback() const {
  return last_feedback_time_.nanoseconds() != 0 &&
         (now() - last_feedback_time_).seconds() <= kFeedbackTimeoutSeconds;
}

bool McRos2Controller::isFallDetected() const {
  if (!hasRecentFeedback() || !stability_state_.has_orientation) {
    return false;
  }

  const bool excessive_tilt =
      std::abs(stability_state_.roll) > kFallRollPitchRadians ||
      std::abs(stability_state_.pitch) > kFallRollPitchRadians;
  const bool excessive_rate =
      std::abs(stability_state_.roll_rate) > kFallAngularRate ||
      std::abs(stability_state_.pitch_rate) > kFallAngularRate;
  const bool too_low =
      stability_state_.has_height && stability_state_.base_height < kFallMinHeight;
  return excessive_tilt || excessive_rate || too_low;
}

bool McRos2Controller::isStandStable() const {
  return isStandReadyToWalk();
}

bool McRos2Controller::controlGateStable() const {
  return hasStableStandHeight() &&
         hasStableStandOrientation() &&
         hasNoStandOscillation() &&
         !isFallDetected();
}

bool McRos2Controller::controlGatePoseConverged() const {
  return control_gate_stable_ &&
         control_gate_stable_since_time_.nanoseconds() != 0 &&
         (now() - control_gate_stable_since_time_).seconds() >= kControlGatePoseConvergedSeconds;
}

bool McRos2Controller::controlGateMotionAllowed() const {
  return control_gate_stable_ && control_gate_pose_converged_;
}

bool McRos2Controller::hasStandFeedback() const {
  return hasRecentFeedback() &&
         has_odom_ &&
         stability_state_.has_orientation &&
         stability_state_.has_height &&
         has_feedback_derivative_;
}

bool McRos2Controller::isFeedbackSettledForAction() const {
  return hasStandFeedback() &&
         std::abs(height_rate_) <= kPreActionMaxHeightRate &&
         std::abs(stability_state_.roll_rate) <= kPreStabilizeMaxTiltRate &&
         std::abs(stability_state_.pitch_rate) <= kPreStabilizeMaxTiltRate &&
         std::abs(roll_rate_estimate_) <= kPreStabilizeMaxTiltRate &&
         std::abs(pitch_rate_estimate_) <= kPreStabilizeMaxTiltRate;
}

bool McRos2Controller::hasStableStandHeight() const {
  return hasStandFeedback() && stability_state_.base_height > kStandMinHeight;
}

bool McRos2Controller::hasStableStandOrientation() const {
  return hasStandFeedback() &&
         std::abs(stability_state_.roll) <= kStandRollPitchThreshold &&
         std::abs(stability_state_.pitch) <= kStandRollPitchThreshold;
}

bool McRos2Controller::hasNoStandOscillation() const {
  return hasStandFeedback() &&
         std::abs(height_rate_) <= kStandMaxHeightRate &&
         std::abs(roll_rate_estimate_) <= kStandMaxTiltRate &&
         std::abs(pitch_rate_estimate_) <= kStandMaxTiltRate &&
         std::abs(stability_state_.roll_rate) <= kStandMaxImuTiltRate &&
         std::abs(stability_state_.pitch_rate) <= kStandMaxImuTiltRate;
}

bool McRos2Controller::isGetUpActionCompleteByFeedback() const {
  return hasStableStandHeight() &&
         std::abs(stability_state_.roll) <= kGetUpCompletionTiltThreshold &&
         std::abs(stability_state_.pitch) <= kGetUpCompletionTiltThreshold &&
         std::abs(height_rate_) <= kGetUpCompletionMaxHeightRate &&
         std::abs(roll_rate_estimate_) <= kGetUpCompletionMaxTiltRate &&
         std::abs(pitch_rate_estimate_) <= kGetUpCompletionMaxTiltRate &&
         std::abs(stability_state_.roll_rate) <= kPreStabilizeMaxTiltRate &&
         std::abs(stability_state_.pitch_rate) <= kPreStabilizeMaxTiltRate;
}

bool McRos2Controller::isStandReadyToWalk() const {
  return controlGateMotionAllowed();
}

void McRos2Controller::resetOfficialStandAction(const std::string& action_name) {
  stand_action_name_ = action_name;
  stand_action_requested_ = false;
  stand_action_accepted_ = false;
  stand_action_fallback_used_ = false;
  last_stand_action_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  stand_action_accept_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

void McRos2Controller::requestOfficialStandUpAction() {
  if (stand_action_requested_) {
    return;
  }

  if (stand_action_name_.empty()) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[STAND_UP] no official MC action selected for phase %s",
        standUpPhaseName(stand_up_phase_));
    return;
  }

  if (last_stand_action_request_time_.nanoseconds() != 0 &&
      (now() - last_stand_action_request_time_).seconds() < 1.0) {
    return;
  }

  if (!set_mc_action_client_->service_is_ready()) {
    if (!set_mc_action_client_->wait_for_service(std::chrono::milliseconds(10))) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[STAND_UP] waiting for official MC SetMcAction service");
      return;
    }
  }

  auto request = std::make_shared<SetMcAction::Request>();
  request->header.stamp = now();
  request->source = "rc";
  request->command.action_desc = stand_action_name_;

  stand_action_requested_ = true;
  last_stand_action_request_time_ = now();
  const char* auto_next =
      stand_action_name_ == "SIT_JOINT_DEFAULT"
          ? "official sit posture init"
          : (stand_action_name_ == "SIT_UP_DEFAULT"
                 ? "official sit-up to stand"
                 : (stand_action_name_ == "STAND_DEFAULT"
                        ? "official stable stand auto-balance"
                        : (stand_action_name_ == "DAMPING_DEFAULT"
                               ? "safe damping stop"
                               : "official MC action")));
  RCLCPP_INFO(
      get_logger(),
      "[STAND_UP] calling official MC action %s (auto-next: %s)",
      stand_action_name_.c_str(),
      auto_next);
  set_mc_action_client_->async_send_request(
      request,
      std::bind(&McRos2Controller::handleStandUpActionResponse, this, std::placeholders::_1));
}

void McRos2Controller::handleStandUpActionResponse(rclcpp::Client<SetMcAction>::SharedFuture future) {
  const auto response = future.get();
  if (!response) {
    RCLCPP_ERROR(get_logger(), "[STAND_UP] SetMcAction response is null");
    stand_action_requested_ = false;
    return;
  }

  stand_action_accepted_ = response->response.status.value == 1;
  if (stand_action_accepted_) {
    stand_action_accept_time_ = now();
    RCLCPP_INFO(
        get_logger(),
        "[STAND_UP] official MC action %s accepted",
        stand_action_name_.c_str());
  } else {
    RCLCPP_ERROR(
        get_logger(),
        "[STAND_UP] official MC action %s rejected status=%d message=%s",
        stand_action_name_.c_str(),
        response->response.status.value,
        response->response.message.c_str());
    stand_action_requested_ = false;
    stand_action_accept_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

void McRos2Controller::updateControlGate() {
  control_gate_stable_ = controlGateStable();
  if (control_gate_stable_) {
    if (control_gate_stable_since_time_.nanoseconds() == 0) {
      control_gate_stable_since_time_ = now();
    }
  } else {
    control_gate_stable_since_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  control_gate_pose_converged_ = controlGatePoseConverged();
  control_gate_motion_allowed_ = controlGateMotionAllowed();
}

void McRos2Controller::logControlGate() {
  RCLCPP_INFO(
      get_logger(),
      "[CONTROL GATE] stable=%s pose_converged=%s motion_allowed=%s yaw_error=%.3f height=%.3f roll=%.3f pitch=%.3f rates=(%.3f, %.3f, %.3f)",
      control_gate_stable_ ? "YES" : "NO",
      control_gate_pose_converged_ ? "YES" : "NO",
      control_gate_motion_allowed_ ? "YES" : "NO",
      yaw_error_to_target_,
      stability_state_.base_height,
      stability_state_.roll,
      stability_state_.pitch,
      height_rate_,
      roll_rate_estimate_,
      pitch_rate_estimate_);
}

void McRos2Controller::publishGuardedStop() {
  publishCommand(0.0, 0.0, 0.0);
  appendControlStabilityLog(0.0, 0.0, 0.0, "FALL_GUARD");
  RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "[FALL_GUARD] roll=%.3f pitch=%.3f height=%.3f rates=(%.3f, %.3f, %.3f)",
      stability_state_.roll,
      stability_state_.pitch,
      stability_state_.base_height,
      stability_state_.roll_rate,
      stability_state_.pitch_rate,
      stability_state_.yaw_rate);
}

void McRos2Controller::publishCommand(double forward_velocity, double lateral_velocity, double yaw_rate) {
  publishVelocity(forward_velocity, lateral_velocity, yaw_rate);
}

void McRos2Controller::publishCounterbalanceUpperBody(double intensity) {
  UpperBodyCommandArray upper_body;
  fillHeader(upper_body.header);
  upper_body.header.frame_id = "mc_upper_body";
  upper_body.source = "remote_teleop_pc";
  upper_body.hand_sub_mode = 0;
  const double gain = clamp(intensity, 0.0, 1.0);
  upper_body.head_pos = {0.0, 0.10 * gain};
  upper_body.arm_pos = {
      0.65 * gain, 0.18 * gain, 0.0, -0.55, 0.0, 0.0, 0.0,
      0.65 * gain, -0.18 * gain, 0.0, -0.55, 0.0, 0.0, 0.0,
  };
  upper_body.hand_pos = {};
  upper_body_pub_->publish(upper_body);
}

double McRos2Controller::smoothAxis(double target, double previous, double alpha, double max_delta) const {
  const double filtered = previous + alpha * (target - previous);
  return previous + clamp(filtered - previous, -max_delta, max_delta);
}

double McRos2Controller::clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}

double McRos2Controller::normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void McRos2Controller::fillHeader(aimdk_msgs::msg::MessageHeader& header) {
  header.stamp = now();
  header.frame_id = "base_link";
  header.sequence = sequence_++;
  header.meas_stamp = header.stamp;
}

void McRos2Controller::writeControlTopicLog() {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/control_topics.log", std::ios::trunc);
  const auto topics = get_topic_names_and_types();

  log << "ROS2 topic list\n";
  for (const auto& [topic, types] : topics) {
    log << topic;
    if (!types.empty()) {
      log << " [";
      for (std::size_t i = 0; i < types.size(); ++i) {
        log << types[i];
        if (i + 1 < types.size()) {
          log << ", ";
        }
      }
      log << "]";
    }
    log << "\n";
  }

  log << "\nControl topics\n";
  for (const auto& [topic, types] : topics) {
    if (isControlTopic(topic)) {
      log << topic;
      if (!types.empty()) {
        log << " [" << types.front() << "]";
      }
      log << "\n";
    }
  }

  log << "\nActive command\n";
  log << "topic=" << kVelocityTopic << "\n";
  log << "state=" << stateName(state_) << "\n";
  log << "stand_up_phase=" << (state_ == AutonomyState::STAND_UP ? standUpPhaseName(stand_up_phase_) : "-") << "\n";
  log << "source=" << kSource << "\n";
  log << "forward_velocity=" << last_forward_velocity_ << "\n";
  log << "lateral_velocity=" << last_lateral_velocity_ << "\n";
  log << "yaw_rate=" << last_yaw_rate_ << "\n";
  log << "stability_status=" << last_stability_command_.status << "\n";
  log << "roll=" << stability_state_.roll << "\n";
  log << "pitch=" << stability_state_.pitch << "\n";
  log << "yaw=" << stability_state_.yaw << "\n";
  log << "base_height=" << stability_state_.base_height << "\n";
  log << "height_rate=" << height_rate_ << "\n";
  log << "roll_rate_estimate=" << roll_rate_estimate_ << "\n";
  log << "pitch_rate_estimate=" << pitch_rate_estimate_ << "\n";
  log << "control_gate_stable=" << (control_gate_stable_ ? "true" : "false") << "\n";
  log << "control_gate_pose_converged=" << (control_gate_pose_converged_ ? "true" : "false") << "\n";
  log << "control_gate_motion_allowed=" << (control_gate_motion_allowed_ ? "true" : "false") << "\n";
  log << "control_gate_yaw_error=" << yaw_error_to_target_ << "\n";
  log << "walk_target_base_forward=" << target_forward_base_ << "\n";
  log << "walk_target_base_lateral=" << target_lateral_base_ << "\n";
  log << "walk_target_distance=" << target_distance_ << "\n";
  log << "stand_ready_to_walk=" << (isStandReadyToWalk() ? "true" : "false") << "\n";
  log << "walk_forward_delta=" << walk_forward_delta_ << "\n";
  log << "walk_lateral_delta=" << walk_lateral_delta_ << "\n";
  log << "walk_target_forward=" << kWalkTargetForwardMeters << "\n";
  log << "fall_guard=" << (state_ == AutonomyState::FALL_GUARD ? "true" : "false") << "\n";
  log << "source_registration=source_field_only\n";
}

void McRos2Controller::appendControlStabilityLog(
    double target_forward,
    double target_lateral,
    double target_yaw_rate,
    const char* mode) {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/control_stability.log", std::ios::app);
  log << now().seconds()
      << " state=" << stateName(state_)
      << " mode=" << mode
      << " target=(" << target_forward << "," << target_lateral << "," << target_yaw_rate << ")"
      << " filtered=(" << last_forward_velocity_ << "," << last_lateral_velocity_ << "," << last_yaw_rate_ << ")"
      << " roll=" << stability_state_.roll
      << " pitch=" << stability_state_.pitch
      << " yaw=" << stability_state_.yaw
      << " height=" << stability_state_.base_height
      << " stable=" << (control_gate_stable_ ? "true" : "false")
      << " pose_converged=" << (control_gate_pose_converged_ ? "true" : "false")
      << " motion_allowed=" << (control_gate_motion_allowed_ ? "true" : "false")
      << " yaw_error=" << yaw_error_to_target_
      << " fall=" << (isFallDetected() ? "true" : "false")
      << "\n";
}

void McRos2Controller::appendStateTransitionTrace(
    AutonomyState from,
    AutonomyState to,
    const char* reason) {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/state_transition_trace.log", std::ios::app);
  log << now().seconds()
      << " from=" << stateName(from)
      << " to=" << stateName(to)
      << " reason=" << reason
      << " odom_delta=(" << walk_forward_delta_ << "," << walk_lateral_delta_ << ")"
      << " roll=" << stability_state_.roll
      << " pitch=" << stability_state_.pitch
      << " height=" << stability_state_.base_height
      << "\n";
}

void McRos2Controller::appendStandUpPhaseTrace(
    StandUpPhase from,
    StandUpPhase to,
    const char* reason) {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/stand_up_phase_trace.log", std::ios::app);
  log << now().seconds()
      << " from=" << standUpPhaseName(from)
      << " to=" << standUpPhaseName(to)
      << " reason=" << reason
      << " action=" << stand_action_name_
      << " height=" << stability_state_.base_height
      << " roll=" << stability_state_.roll
      << " pitch=" << stability_state_.pitch
      << " height_rate=" << height_rate_
      << " roll_rate=" << roll_rate_estimate_
      << " pitch_rate=" << pitch_rate_estimate_
      << " ready=" << (isStandReadyToWalk() ? "true" : "false")
      << "\n";
}

void McRos2Controller::appendStandUpConvergenceLog() {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/stand_up_convergence.log", std::ios::app);
  const double stable_for =
      control_gate_stable_since_time_.nanoseconds() == 0 ? 0.0 : (now() - control_gate_stable_since_time_).seconds();
  log << now().seconds()
      << " phase=" << standUpPhaseName(stand_up_phase_)
      << " action=" << stand_action_name_
      << " accepted=" << (stand_action_accepted_ ? "true" : "false")
      << " leg_odom=" << (has_leg_odom_ ? "true" : "false")
      << " height=" << stability_state_.base_height
      << " roll=" << stability_state_.roll
      << " pitch=" << stability_state_.pitch
      << " height_rate=" << height_rate_
      << " roll_rate=" << roll_rate_estimate_
      << " pitch_rate=" << pitch_rate_estimate_
      << " imu_rates=(" << stability_state_.roll_rate << "," << stability_state_.pitch_rate << ")"
      << " get_up_complete=" << (isGetUpActionCompleteByFeedback() ? "true" : "false")
      << " stable_height=" << (hasStableStandHeight() ? "true" : "false")
      << " stable_orientation=" << (hasStableStandOrientation() ? "true" : "false")
      << " no_oscillation=" << (hasNoStandOscillation() ? "true" : "false")
      << " ready=" << (isStandReadyToWalk() ? "true" : "false")
      << " pose_converged=" << (control_gate_pose_converged_ ? "true" : "false")
      << " motion_allowed=" << (control_gate_motion_allowed_ ? "true" : "false")
      << " stable_for=" << stable_for
      << "\n";
}

const char* McRos2Controller::standUpPhaseName(StandUpPhase phase) {
  switch (phase) {
    case StandUpPhase::PRE_STABILIZE:
      return "PRE_STABILIZE";
    case StandUpPhase::SIT_INIT:
      return "SIT_INIT";
    case StandUpPhase::STAND_PROBE:
      return "STAND_PROBE";
    case StandUpPhase::SIT_UP:
      return "SIT_UP";
    case StandUpPhase::STAND_LOCK:
      return "STAND_LOCK";
    case StandUpPhase::READY_TO_WALK:
      return "READY_TO_WALK";
  }

  return "UNKNOWN";
}

void McRos2Controller::appendOdomDeltaLog() {
  std::filesystem::create_directories("logs");
  std::ofstream log("logs/odom_delta_tracking.log", std::ios::app);
  log << now().seconds()
      << " state=" << stateName(state_)
      << " position=(" << current_x_ << "," << current_y_ << "," << current_z_ << ")"
      << " walk_delta=(" << walk_forward_delta_ << "," << walk_lateral_delta_ << ")"
      << " target_forward=" << kWalkTargetForwardMeters
      << " target_base=(" << target_forward_base_ << "," << target_lateral_base_ << ")"
      << " forward_error=" << target_forward_base_
      << " lateral_error=" << target_lateral_base_
      << " yaw_error=" << yaw_error_to_target_
      << "\n";
}
