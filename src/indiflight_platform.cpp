// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file indiflight_platform.cpp
*
* IndiflightPlatform class implementation
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#include <array>
#include <cinttypes>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "as2_platform_indiflight/indiflight_platform.hpp"

double convert_deg_s_to_rad_s(double deg_s)
{
  return deg_s / 180.0 * M_PI;
}

namespace as2_platform_indiflight
{

namespace
{
/**
 * @brief Name of a mocap rigid body, accepting both the string form and the
 * integer an unquoted numeric name yields in YAML.
 */
}  // namespace

IndiflightPlatform::IndiflightPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options), thrust_map_(4)
{
  readParameters();
  configureSensors();
  initChannels();
  if (use_thrust_map_) {
    thrust_map_.initialize(this);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Thrust map disabled. Thrust will be mapped directly to throttle.");
  }

  // Debug topics
  if (!debug_rc_command_topic_.empty()) {
    debug_rc_command_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
      debug_rc_command_topic_, 1);
  }
  if (!debug_og_timestamp_topic_.empty()) {
    og_timestamp_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>(
      debug_og_timestamp_topic_, rclcpp::SensorDataQoS());
  }
  if (!debug_pi_status_topic_.empty()) {
    debug_pi_status_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
      debug_pi_status_topic_, 1);
  }
  if (!debug_aux_topic_.empty()) {
    debug_aux_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
      debug_aux_topic_, 1);
  }

  // Callbacks are registered unconditionally: a firmware build that does not
  // emit one of these messages never triggers its callback.
  pi_protocol_client_.setEkfInputsCallback(
    [this](const pi_EKF_INPUTS_t & msg) {onPiProtocolEkfInputs(msg);});
  pi_protocol_client_.setAuxCallback(
    [this](const pi_AUX_t & msg) {onPiAux(msg);});
  pi_protocol_client_.setStatusCallback(
    [this](const pi_PI_STATUS_t & msg) {onPiStatus(msg);});
  pi_protocol_client_.setBatteryCallback(
    [this](const pi_BATTERY_t & msg) {onPiBattery(msg);});

  // The only link to the FC: without it the platform has neither commands nor
  // state, so a failure here is fatal rather than degraded.
  if (!pi_protocol_client_.connect(pi_protocol_device_, pi_protocol_baudrate_)) {
    throw std::runtime_error(
            "Could not connect to pi-protocol device " + pi_protocol_device_);
  }
  RCLCPP_INFO(
    this->get_logger(), "pi-protocol connected on %s @ %d baud",
    pi_protocol_device_.c_str(), pi_protocol_baudrate_);

  // Also used by the POSITION command path, which runs whether or not the
  // external pose uplink is enabled.
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  // Feed for the FC's onboard EKF. Created after the pi-protocol connection:
  // sendExternalPose() is a no-op until the first downlink message provides an
  // FC tick anyway.
  if (external_pose_enable_) {
    if (!external_pose_mocap_topic_.empty()) {
      external_rigid_bodies_sub_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        external_pose_mocap_topic_, rclcpp::SensorDataQoS(),
        std::bind(&IndiflightPlatform::onExternalRigidBodies, this, std::placeholders::_1));
      RCLCPP_INFO(
        this->get_logger(), "Forwarding '%s' body '%s' to the FC EKF as EXTERNAL_POSE",
        external_pose_mocap_topic_.c_str(), external_pose_rigid_body_name_.c_str());
    } else if (!external_pose_pose_topic_.empty()) {
      external_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        external_pose_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&IndiflightPlatform::onExternalPose, this, std::placeholders::_1));
      RCLCPP_INFO(
        this->get_logger(), "Forwarding '%s' to the FC EKF as EXTERNAL_POSE",
        external_pose_pose_topic_.c_str());
    } else {
      external_pose_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / external_pose_rate_),
        std::bind(&IndiflightPlatform::sendExternalPoseFromTf, this));
      RCLCPP_INFO(
        this->get_logger(), "Forwarding %s->%s TF to the FC EKF as EXTERNAL_POSE at %.1f Hz",
        earth_frame_id_.c_str(), base_link_frame_id_.c_str(), external_pose_rate_);
    }
  }

  // One second never trips on a healthy link: EKF_INPUTS alone arrives at 500Hz.
  link_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&IndiflightPlatform::checkLink, this));

  // Clear layout dimensions if they were set in a previous publication
  debug_rc_command_.layout.dim.clear();

  // Configure the array layout
  std_msgs::msg::MultiArrayDimension dim;
  dim.size = channel_values_.size();
  dim.stride = 1;
  dim.label = "rc_channels";
  debug_rc_command_.layout.dim.push_back(dim);
  publishDebugRc();
}

void IndiflightPlatform::readParameters()
{
  external_odom_ = getParameter<bool>("external_odom");

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");
  // Not namespaced, unlike the two above: the global reference is shared.
  earth_frame_id_ = "earth";
  earth_frame_id_ = getParameter("global_ref_frame", earth_frame_id_);

  pi_protocol_device_ = getParameter<std::string>("pi_protocol.device");
  pi_protocol_baudrate_ = getParameter<int>("pi_protocol.baudrate");

  // true: header.stamp is the FC sample instant reconstructed by
  // pi_protocol_clock_sync_. false: the arrival time, as elsewhere in AS2.
  // debug/platform/og_timestamp carries the raw FC time_ref either way.
  use_fcu_stamps_ = getParameter("use_fcu_stamps", use_fcu_stamps_);
  num_rotors_ = getParameter("num_rotors", num_rotors_);
  if (num_rotors_ < 1 || num_rotors_ > 6) {
    throw std::runtime_error("num_rotors must be in [1, 6]");
  }
  debug_rc_command_topic_ = getParameter("debug_topics.rc_command", debug_rc_command_topic_);
  debug_og_timestamp_topic_ = getParameter("debug_topics.og_timestamp", debug_og_timestamp_topic_);
  debug_pi_status_topic_ = getParameter("debug_topics.pi_status", debug_pi_status_topic_);
  debug_aux_topic_ = getParameter("debug_topics.aux", debug_aux_topic_);

  imu_gyro_covariance_ = getParameter("imu.covariance.gyro", imu_gyro_covariance_);
  imu_accel_covariance_ = getParameter("imu.covariance.accel", imu_accel_covariance_);

  // Rotation (rad) from indiflight's FRD firmware frame to the body frame.
  // The default r = pi is FRD -> FLU.
  desired_frame_roll_ = M_PI;
  desired_frame_roll_ = getParameter("desired_frame_T.r", desired_frame_roll_);
  desired_frame_pitch_ = getParameter("desired_frame_T.p", desired_frame_pitch_);
  desired_frame_yaw_ = getParameter("desired_frame_T.y", desired_frame_yaw_);
  desired_frame_rotation_ =
    (Eigen::AngleAxisd(desired_frame_yaw_, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(desired_frame_pitch_, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(desired_frame_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();

  // Command limits
  max_thrust_ = getParameter<double>("thrust.max");
  min_thrust_ = getParameter<double>("thrust.min");
  max_roll_rate_ = getParameter<double>("roll_rate.max");
  min_roll_rate_ = getParameter<double>("roll_rate.min");
  max_pitch_rate_ = getParameter<double>("pitch_rate.max");
  min_pitch_rate_ = getParameter<double>("pitch_rate.min");
  max_yaw_rate_ = getParameter<double>("yaw_rate.max");
  min_yaw_rate_ = getParameter<double>("yaw_rate.min");
  // Convert limits from deg/s to rad/s for the control slopes and the limiters.
  max_roll_rate_ = convert_deg_s_to_rad_s(max_roll_rate_);
  min_roll_rate_ = convert_deg_s_to_rad_s(min_roll_rate_);
  max_pitch_rate_ = convert_deg_s_to_rad_s(max_pitch_rate_);
  min_pitch_rate_ = convert_deg_s_to_rad_s(min_pitch_rate_);
  max_yaw_rate_ = convert_deg_s_to_rad_s(max_yaw_rate_);
  min_yaw_rate_ = convert_deg_s_to_rad_s(min_yaw_rate_);
  computeControlSlopes();

  use_thrust_map_ = getParameter<bool>("use_thrust_map");
  limit_output_ = getParameter<bool>("limit_output");
  limit_roll_percent_ = getParameter<double>("limit_roll_percent");
  limit_pitch_percent_ = getParameter<double>("limit_pitch_percent");
  limit_yaw_percent_ = getParameter<double>("limit_yaw_percent");
  limit_thrust_percent_ = getParameter<double>("limit_thrust_percent");

  alpha_voltage_ = getParameter<double>("alpha_voltage");
  min_cell_voltage_ = getParameter<double>("min_cell_voltage");
  max_cell_voltage_ = getParameter<double>("max_cell_voltage");

  // Pose forwarded to the FC's onboard EKF as EXTERNAL_POSE (NED), in every
  // control mode. The non-empty topic parameter selects the source:
  //   both empty      -> the earth->base_link TF, polled at `rate`
  //   pose_topic set  -> that geometry_msgs/PoseStamped topic
  //   mocap_topic set -> that mocap4r2_msgs/RigidBodies topic, body picked by
  //                      rigid_body_name
  // Feeding the FC from TF while AS2 consumes the FC estimate would be a loop.
  external_pose_enable_ = getParameter<bool>("external_pose.enable");
  if (external_pose_enable_) {
    external_pose_rate_ = getParameter("external_pose.rate", external_pose_rate_);
    external_pose_pose_topic_ = getParameter("external_pose.pose_topic", external_pose_pose_topic_);
    if (external_pose_pose_topic_.empty()) {
      external_pose_mocap_topic_ = getParameter(
        "external_pose.mocap_topic",
        external_pose_mocap_topic_);
      if (!external_pose_mocap_topic_.empty()) {
        external_pose_rigid_body_name_ = getParameter<std::string>("external_pose.rigid_body_name");
        if (external_pose_rigid_body_name_.empty()) {
          RCLCPP_FATAL(
            this->get_logger(),
            "external_pose.rigid_body_name must be set when external_pose.mocap_topic is set");
        }
      }
    }
    if (!external_pose_pose_topic_.empty() && !external_pose_mocap_topic_.empty()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "external_pose pose_topic and mocap_topic are mutually exclusive");
    }
    if (external_pose_mocap_topic_.empty() && external_pose_pose_topic_.empty() &&
      external_pose_rate_ <= 0.0)
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "external_pose.rate must be > 0 when polling TF");
    }
  }
}

IndiflightPlatform::~IndiflightPlatform()
{
  pi_protocol_client_.disconnect();
}

void IndiflightPlatform::computeControlSlopes()
{
  roll_slope_ = (max_roll_rate_ - min_roll_rate_) / static_cast<double>(PULSE_RANGE);
  pitch_slope_ = (max_pitch_rate_ - min_pitch_rate_) / static_cast<double>(PULSE_RANGE);
  yaw_slope_ = (max_yaw_rate_ - min_yaw_rate_) / static_cast<double>(PULSE_RANGE);
}

void IndiflightPlatform::configureSensors()
{
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  battery_sensor_ptr_ = std::make_unique<as2::sensors::Battery>("battery", this);
  motor_sensor_ptr_ =
    std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::JointState>>(
    "motor_angular_speed", this);
}

void IndiflightPlatform::initChannels()
{
  // 0 roll, 1 pitch, 2 throttle, 3 yaw: the 4 channels RC_OVERRIDE carries.
  // ARM, offboard and killswitch live on the physical radio, outside it.
  channel_values_.clear();
  channel_values_.resize(4, 1000);
  channel_values_[RC_CHANNELS::ROLL] = 1500;
  channel_values_[RC_CHANNELS::PITCH] = 1500;
  channel_values_[RC_CHANNELS::THROTTLE] = 1000;
  channel_values_[RC_CHANNELS::YAW] = 1500;
}

bool IndiflightPlatform::ownSetArmingState(bool state)
{
  // ARM lives on the physical radio, outside RC_OVERRIDE's 4 channels. The
  // actual state is read back via PI_STATUS in onPiStatus().
  RCLCPP_WARN(
    this->get_logger(),
    "Arming is physical-radio-controlled on this platform - AS2 cannot arm/disarm it. "
    "Actual arm state is reported via pi-protocol PI_STATUS.");
  (void)state;
  return false;
}

bool IndiflightPlatform::ownSetOffboardControl(bool offboard)
{
  // Same reasoning as ownSetArmingState(): the PI OVERRIDE AUX switch lives on
  // the physical radio. Actual offboard (= PI_OVERRIDE_ACTIVE) state is read
  // back via PI_STATUS in onPiStatus().
  RCLCPP_WARN(
    this->get_logger(),
    "Offboard is driven by the PI OVERRIDE AUX switch on the physical radio - AS2 cannot set "
    "it. Actual state is reported via pi-protocol PI_STATUS.");
  (void)offboard;
  return false;
}

bool IndiflightPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  // as2::AerialPlatform forwards the raw request without checking it against
  // control_modes.yaml, so this switch is the real capability gate. UNSET is
  // absent on purpose: it is the mode the platform starts in, not one that can
  // be requested.
  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER:
      if (!acceptHover()) {
        return false;
      }
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      if (!acceptFcPositionControl("POSITION")) {
        return false;
      }
      break;
    case as2_msgs::msg::ControlMode::ACRO:
      break;
    default:
      RCLCPP_ERROR(
        this->get_logger(), "Control mode [%s] not supported by this platform",
        as2::control_mode::controlModeToString(msg).c_str());
      return false;
  }
  RCLCPP_INFO(
    this->get_logger(), "Control mode set: [%s]",
    as2::control_mode::controlModeToString(msg).c_str());
  return true;
}

bool IndiflightPlatform::ownSendCommand()
{
  switch (getControlMode().control_mode) {
    case as2_msgs::msg::ControlMode::POSITION:
      return sendPositionCommand();
    case as2_msgs::msg::ControlMode::HOVER:
      return sendHoverCommand();
    case as2_msgs::msg::ControlMode::ACRO:
      return sendAcroCommand();
    case as2_msgs::msg::ControlMode::UNSET:
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Control mode UNSET: no command sent");
      return false;
    default:
      RCLCPP_ERROR(
        this->get_logger(), "Control mode [%s] not supported by this platform",
        as2::control_mode::controlModeToString(getControlMode()).c_str());
      return false;
  }
}

bool IndiflightPlatform::toEarthFrame(geometry_msgs::msg::PoseStamped & pose) const
{
  if (pose.header.frame_id == earth_frame_id_) {
    return true;
  }
  // Zero timeout: use the latest cached transform. Both callers run on the
  // executor thread (command timer / external pose subscription) and must
  // never block it.
  return tf_handler_->tryConvert(pose, earth_frame_id_, std::chrono::nanoseconds::zero());
}

bool IndiflightPlatform::acceptFcPositionControl(const char * mode) const
{
  // POS_SETPOINT is tracked by the FC's own position controller, which only
  // runs on a converged EKF, and nothing else on this link feeds that EKF.
  if (!external_pose_enable_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s rejected: it runs on the FC position controller, which needs external_pose.enable",
      mode);
    return false;
  }
  return true;
}

bool IndiflightPlatform::acceptHover()
{
  if (!acceptFcPositionControl("HOVER")) {
    return false;
  }

  // getControlMode() is still the mode being left: as2::AerialPlatform stores
  // the new one only after this returns.
  const uint8_t previous_mode = getControlMode().control_mode;
  if (previous_mode == as2_msgs::msg::ControlMode::HOVER) {
    // Already hovering. Keeping the latched pose.
    return true;
  }
  // TODO(fjanguita): Remove when FC accepts offboard change from RATES to HOVER
  if (previous_mode != as2_msgs::msg::ControlMode::POSITION) {
    RCLCPP_ERROR(
      this->get_logger(),
      "HOVER rejected: only reachable from POSITION, the FC has no position setpoint "
      "to hold otherwise");
    return false;
  }

  // Latched once, on the transition. Reading the pose in the command path
  // instead would make the reference chase the vehicle as it drifts.
  if (!latchHoverPose()) {
    // Coming from POSITION the FC already holds the last commanded setpoint,
    // so hovering there is still correct, just not where the vehicle ended up.
    RCLCPP_WARN(
      this->get_logger(),
      "Entering HOVER without a fresh pose, the FC keeps its last setpoint");
  }
  return true;
}

bool IndiflightPlatform::latchHoverPose()
{
  hover_pose_.reset();
  try {
    hover_pose_ = tf_handler_->getPoseStamped(
      earth_frame_id_, base_link_frame_id_, tf2::TimePointZero, std::chrono::nanoseconds::zero());
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
      this->get_logger(), "No %s->%s transform to latch as the hover reference: %s",
      earth_frame_id_.c_str(), base_link_frame_id_.c_str(), e.what());
    return false;
  }
  return true;
}

bool IndiflightPlatform::sendHoverCommand()
{
  // Nothing latched: the FC holds whatever setpoint it already had, which is
  // the best available answer and what it does on its own anyway.
  if (!hover_pose_) {
    return true;
  }
  return sendPoseSetpoint(*hover_pose_);
}

bool IndiflightPlatform::sendPositionCommand()
{
  geometry_msgs::msg::PoseStamped pose = command_pose_msg_;
  // An empty frame_id means no pose reference has arrived yet: a
  // default-constructed pose would command the origin with a zero quaternion
  // (NaN yaw). Refuse it.
  if (pose.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "POSITION mode active but no pose reference received yet - not sent");
    return false;
  }
  if (!toEarthFrame(pose)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "POSITION command in frame '%s', could not convert to '%s' - not sent",
      pose.header.frame_id.c_str(), earth_frame_id_.c_str());
    return false;
  }

  return sendPoseSetpoint(pose);
}

bool IndiflightPlatform::sendPoseSetpoint(const geometry_msgs::msg::PoseStamped & pose)
{
  const Eigen::Vector3d ned = enuToNed(
    Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  const double yaw_ned_deg = yawEnuRadToNedDeg(
    as2::frame::getYawFromQuaternion(pose.pose.orientation));

  // Velocity feed-forward zero: in POSITION mode the twist reference is a
  // speed limit, not a feed-forward. TRAJECTORY mode would carry one.
  if (!pi_protocol_client_.sendPosSetpoint(
      static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z()),
      0.0f, 0.0f, 0.0f, static_cast<float>(yaw_ned_deg)))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Could not send POS_SETPOINT (no FC downlink tick yet, or write failed)");
    return false;
  }
  return true;
}

bool IndiflightPlatform::sendAcroCommand()
{
  double thrust = this->command_thrust_msg_.thrust;
  double roll = this->command_twist_msg_.twist.angular.x;
  double pitch = this->command_twist_msg_.twist.angular.y;
  // YAW is inverted, since for positive yaw the drone rotates counterclockwise,
  // so the u_sec value shall decrease, not increase.
  double yaw = (-1) * this->command_twist_msg_.twist.angular.z;

  // saturate thrust
  thrust = std::clamp(thrust, min_thrust_, max_thrust_);
  roll = std::clamp(roll, min_roll_rate_, max_roll_rate_);
  pitch = std::clamp(pitch, min_pitch_rate_, max_pitch_rate_);
  yaw = std::clamp(yaw, min_yaw_rate_, max_yaw_rate_);

  // convert to pulse width
  uint16_t roll_pulse = static_cast<uint16_t>(1500 + roll / roll_slope_);
  uint16_t pitch_pulse = static_cast<uint16_t>(1500 + pitch / pitch_slope_);
  uint16_t yaw_pulse = static_cast<uint16_t>(1500 + yaw / roll_slope_);
  uint16_t throttle_pulse = 1000;
  if (use_thrust_map_) {
    throttle_pulse = thrust_map_.getThrottle_useconds(thrust, voltage_);
  } else {
    double thust_normalized = thrust / max_thrust_;
    throttle_pulse = static_cast<uint16_t>(1000 + thust_normalized * 1000);
  }

  if (limit_output_) {
    roll_pulse = std::clamp(
      roll_pulse, static_cast<uint16_t>(1500 - limit_roll_percent_ * 500),
      static_cast<uint16_t>(1500 + limit_roll_percent_ * 500));
    pitch_pulse = std::clamp(
      pitch_pulse, static_cast<uint16_t>(1500 - limit_pitch_percent_ * 500),
      static_cast<uint16_t>(1500 + limit_pitch_percent_ * 500));
    yaw_pulse = std::clamp(
      yaw_pulse, static_cast<uint16_t>(1500 - limit_yaw_percent_ * 500),
      static_cast<uint16_t>(1500 + limit_yaw_percent_ * 500));
    throttle_pulse = std::clamp(
      throttle_pulse, static_cast<uint16_t>(1000),
      static_cast<uint16_t>(1000 + limit_thrust_percent_ * 1000));
  }

  // set the values
  channel_values_[RC_CHANNELS::ROLL] = roll_pulse;
  channel_values_[RC_CHANNELS::PITCH] = pitch_pulse;
  channel_values_[RC_CHANNELS::THROTTLE] = throttle_pulse;
  channel_values_[RC_CHANNELS::YAW] = yaw_pulse;

  publishDebugRc();

  bool out = pi_protocol_client_.sendRcOverride(roll_pulse, pitch_pulse, yaw_pulse, throttle_pulse);
  if (!out) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Could not send RC_OVERRIDE to the flight controller");
    return false;
  }
  return true;
}

void IndiflightPlatform::ownKillSwitch()
{
  RCLCPP_ERROR(
    this->get_logger(),
    "Kill-switch is physical-radio-only on this platform - AS2 cannot cut motors from here.");
}

void IndiflightPlatform::ownStopPlatform()
{
  RCLCPP_ERROR(
    this->get_logger(),
    "Stop is physical-radio-only on this platform - AS2 cannot stop it from here.");
}

void IndiflightPlatform::rotateImuToDesiredFrame(
  Eigen::Vector3d & angular_velocity, Eigen::Vector3d & linear_acceleration) const
{
  angular_velocity = desired_frame_rotation_ * angular_velocity;
  linear_acceleration = desired_frame_rotation_ * linear_acceleration;
}

void IndiflightPlatform::onPiProtocolEkfInputs(const pi_EKF_INPUTS_t & msg)
{
  // Fixed-point decode, exact inverse of telemetry/pi.c's piSendEkfInputs()
  // encode - matches pi-protocol's EKF_INPUTS.yaml field comments.
  constexpr float kAccelLsbToMps2 = 9.81f / 2048.f;                        // +-16g full scale
  constexpr float kGyroLsbToRadps = (2000.f * M_PI / 180.f) / 32768.f;  // +-2000 deg/s full scale

  // Captured first, right after piParse() hands off the struct, to keep the
  // clock-sync offset sample as tight as possible.
  const int64_t host_now_ns = this->get_clock()->now().nanoseconds();
  // Fed even when use_fcu_stamps_ is false, to avoid a cold start if the
  // parameter is flipped later.
  const int64_t synced_ns = pi_protocol_clock_sync_.sync(msg.time_us, host_now_ns);
  const rclcpp::Time stamp(use_fcu_stamps_ ? synced_ns : host_now_ns);

  Eigen::Vector3d angular_velocity(
    msg.p * kGyroLsbToRadps, msg.q * kGyroLsbToRadps, msg.r * kGyroLsbToRadps);
  Eigen::Vector3d linear_acceleration(
    msg.x * kAccelLsbToMps2, msg.y * kAccelLsbToMps2, msg.z * kAccelLsbToMps2);
  // Rotate out of indiflight's FRD firmware frame into desired_frame_T (default
  // FLU) before publishing, so base_link_frame_id_ actually matches its contents.
  rotateImuToDesiredFrame(angular_velocity, linear_acceleration);
  publishImuSample(stamp, msg.time_us, angular_velocity, linear_acceleration);

  sensor_msgs::msg::JointState motor_msg;
  motor_msg.header.stamp = stamp;
  motor_msg.header.frame_id = base_link_frame_id_;
  // Betaflight mixer output order (quad X: [RR, FR, RL, FL], hex X adds
  // [MR, ML]), not the indi_controller convention used elsewhere in the
  // workspace. Plain rad/s, unlike x/y/z/p/q/r above.
  const std::array<uint16_t, 6> omegas = {
    msg.omega1, msg.omega2, msg.omega3, msg.omega4, msg.omega5, msg.omega6};
  for (int i = 0; i < num_rotors_; i++) {
    motor_msg.name.emplace_back("motor" + std::to_string(i));
    motor_msg.velocity.emplace_back(static_cast<double>(omegas[i]));
  }
  // sensor_measurements/motor_angular_speed (as2::sensors::Sensor)
  motor_sensor_ptr_->updateData(motor_msg);
}

rclcpp::Time IndiflightPlatform::fcStamp(uint32_t fc_time_us)
{
  const int64_t host_now_ns = this->get_clock()->now().nanoseconds();
  if (!use_fcu_stamps_) {
    return rclcpp::Time(host_now_ns);
  }
  return rclcpp::Time(pi_protocol_clock_sync_.toHostTime(fc_time_us).value_or(host_now_ns));
}

void IndiflightPlatform::publishImuSample(
  const rclcpp::Time & stamp, uint32_t fc_time_us,
  const Eigen::Vector3d & angular_velocity, const Eigen::Vector3d & linear_acceleration)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = stamp;
  imu_msg.header.frame_id = base_link_frame_id_;
  imu_msg.angular_velocity.x = angular_velocity.x();
  imu_msg.angular_velocity.y = angular_velocity.y();
  imu_msg.angular_velocity.z = angular_velocity.z();
  imu_msg.linear_acceleration.x = linear_acceleration.x();
  imu_msg.linear_acceleration.y = linear_acceleration.y();
  imu_msg.linear_acceleration.z = linear_acceleration.z();
  imu_msg.angular_velocity_covariance[0] = imu_gyro_covariance_;
  imu_msg.angular_velocity_covariance[4] = imu_gyro_covariance_;
  imu_msg.angular_velocity_covariance[8] = imu_gyro_covariance_;
  imu_msg.linear_acceleration_covariance[0] = imu_accel_covariance_;
  imu_msg.linear_acceleration_covariance[4] = imu_accel_covariance_;
  imu_msg.linear_acceleration_covariance[8] = imu_accel_covariance_;
  // Orientation not provided on this channel.
  imu_msg.orientation_covariance[0] = -1.0;
  // sensor_measurements/imu (as2::sensors::Imu) - used by AS2's state
  // estimator; this used to come from the now-removed MSP onImu().
  imu_sensor_ptr_->updateAndPublish(imu_msg);

  // Raw FC time_us preserved alongside the corrected stamp - see
  // pi_protocol::ClockSync for why time_us can't be used as header.stamp directly.
  if (!og_timestamp_pub_) {
    return;
  }
  sensor_msgs::msg::TimeReference time_ref_msg;
  time_ref_msg.header.stamp = stamp;
  time_ref_msg.header.frame_id = base_link_frame_id_;
  time_ref_msg.time_ref = rclcpp::Time(static_cast<int64_t>(fc_time_us) * 1000);
  time_ref_msg.source = "indiflight_fc_micros";
  og_timestamp_pub_->publish(time_ref_msg);
}

void IndiflightPlatform::updatePlatformState(bool armed, bool offboard)
{
  // Written directly rather than through setArmingState()/setOffboardControl():
  // those gate on ownSetArmingState()/ownSetOffboardControl(), which always
  // fail here, so a report would never reach platform_info_msg_. Edge-triggered
  // because the source messages arrive continuously.
  if (armed != set_arm_) {
    set_arm_ = armed;
    platform_info_msg_.armed = armed;
    handleStateMachineEvent(
      armed ? as2_msgs::msg::PlatformStateMachineEvent::ARM :
      as2_msgs::msg::PlatformStateMachineEvent::DISARM);
  }
  if (offboard != set_offboard_) {
    set_offboard_ = offboard;
    platform_info_msg_.offboard = offboard;
  }
}

void IndiflightPlatform::onPiAux(const pi_AUX_t & msg)
{
  if (!debug_aux_pub_) {
    return;
  }
  const std::array<int16_t, 14> channels = {
    msg.aux_1, msg.aux_2, msg.aux_3, msg.aux_4, msg.aux_5, msg.aux_6, msg.aux_7,
    msg.aux_8, msg.aux_9, msg.aux_10, msg.aux_11, msg.aux_12, msg.aux_13, msg.aux_14};

  as2_msgs::msg::UInt16MultiArrayStamped debug_msg;
  debug_msg.layout.dim.resize(1);
  debug_msg.layout.dim[0].size = channels.size();
  debug_msg.layout.dim[0].label = "aux_1..aux_14";
  debug_msg.data.reserve(channels.size());
  for (const int16_t value : channels) {
    debug_msg.data.emplace_back(static_cast<uint16_t>(std::max<int16_t>(value, 0)));
  }
  debug_msg.stamp = fcStamp(msg.time_us);
  debug_aux_pub_->publish(debug_msg);
}

void IndiflightPlatform::sendExternalPoseFromTf()
{
  geometry_msgs::msg::PoseStamped pose;
  try {
    // TimePointZero + zero timeout: the latest buffered transform, never a
    // blocking wait. Its header.stamp is the transform's own, hence the dedup.
    pose = tf_handler_->getPoseStamped(
      earth_frame_id_, base_link_frame_id_, tf2::TimePointZero, std::chrono::nanoseconds::zero());
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No %s->%s transform available for EXTERNAL_POSE: %s",
      earth_frame_id_.c_str(), base_link_frame_id_.c_str(), e.what());
    return;
  }
  sendExternalPose(pose);
}

void IndiflightPlatform::onExternalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose = *msg;
  if (!toEarthFrame(pose)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "External pose in frame '%s', could not convert to '%s' - dropped",
      pose.header.frame_id.c_str(), earth_frame_id_.c_str());
    return;
  }
  sendExternalPose(pose);
}

void IndiflightPlatform::onExternalRigidBodies(
  const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{
  for (const auto & body : msg->rigidbodies) {
    if (body.rigid_body_name != external_pose_rigid_body_name_) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = body.pose;
    // if (!toEarthFrame(pose)) {
    //   RCLCPP_WARN_THROTTLE(
    //     this->get_logger(), *this->get_clock(), 1000,
    //     "Rigid body '%s' in frame '%s', could not convert to '%s' - dropped",
    //     body.rigid_body_name.c_str(), pose.header.frame_id.c_str(), earth_frame_id_.c_str());
    //   return;
    // }
    // msg from mocap should be in earth frame
    pose.header.frame_id = earth_frame_id_;
    sendExternalPose(pose);
    return;
  }
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Rigid body '%s' not present in '%s'",
    external_pose_rigid_body_name_.c_str(), external_pose_mocap_topic_.c_str());
}

void IndiflightPlatform::sendExternalPose(const geometry_msgs::msg::PoseStamped & pose)
{
  // Skip a pose already forwarded: the TF poll runs faster than the estimator
  // publishes. Re-sending one under a fresh FC timestamp would make the FC
  // read a stale measurement as current.
  const int64_t stamp_ns = rclcpp::Time(pose.header.stamp).nanoseconds();
  if (last_external_pose_stamp_ns_ && stamp_ns <= *last_external_pose_stamp_ns_) {
    return;
  }

  // Same ENU->NED convention as sendPositionCommand() - the FC's NED datum is
  // defined by this very message, so estimation and setpoints must agree (see
  // conversions.hpp).
  const Eigen::Vector3d ned = enuToNed(
    Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  const Eigen::Quaterniond q_ned_frd = enuFluToNedFrd(
    Eigen::Quaterniond(
      pose.pose.orientation.w, pose.pose.orientation.x,
      pose.pose.orientation.y, pose.pose.orientation.z));

  // Stamped with the instant the pose was sampled, mapped into the FC clock,
  // rather than with the latest FC tick: the firmware then sees the real age of
  // the measurement and refuses it past EKF_MAX_MEAS_AGE_US, instead of fusing
  // a stale pose as if it were current.
  const auto fc_time_us = pi_protocol_clock_sync_.toFcTime(stamp_ns);
  if (!fc_time_us) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No FC clock offset yet, cannot stamp EXTERNAL_POSE");
    return;
  }

  // Velocity zero: the FC EKF measurement vector is position + quaternion
  // only (flight/ekf.c ekf_Z), so the wire velocity is never fused.
  if (!pi_protocol_client_.sendExternalPose(
      *fc_time_us,
      static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z()),
      0.0f, 0.0f, 0.0f,
      static_cast<float>(q_ned_frd.w()), static_cast<float>(q_ned_frd.x()),
      static_cast<float>(q_ned_frd.y()), static_cast<float>(q_ned_frd.z())))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Could not send EXTERNAL_POSE to the flight controller");
    return;
  }
  last_external_pose_stamp_ns_ = stamp_ns;
}

void IndiflightPlatform::onPiBattery(const pi_BATTERY_t & msg)
{
  float voltage_filtered = alpha_voltage_ * voltage_ + (1 - alpha_voltage_) * msg.voltage;
  float max_batt_voltage = max_cell_voltage_ * msg.cell_count;
  float min_batt_voltage = min_cell_voltage_ * msg.cell_count;

  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = fcStamp(msg.time_us);
  battery_msg.voltage = voltage_filtered;
  battery_msg.current = msg.current;
  battery_msg.percentage = (voltage_filtered - min_batt_voltage) /
    (max_batt_voltage - min_batt_voltage);
  // No wire equivalent of MSP's battery.capacity_mAh (a static config value on
  // the FC, not telemetry) - battery_msg.charge is left at its default.

  battery_sensor_ptr_->updateData(battery_msg);

  voltage_ = voltage_filtered;
}

void IndiflightPlatform::onPiStatus(const pi_PI_STATUS_t & msg)
{
  // Keep in sync with indiflight/src/main/telemetry/pi.h's PI_STATUS_FLAG_* macros.
  constexpr uint8_t kFlagArmed = 1 << 0;
  constexpr uint8_t kFlagPiOverrideActive = 1 << 1;
  constexpr uint8_t kFlagRxLinkValid = 1 << 2;

  const bool armed = msg.flags & kFlagArmed;
  const bool override_active = msg.flags & kFlagPiOverrideActive;
  const bool rx_link_valid = msg.flags & kFlagRxLinkValid;

  updatePlatformState(armed, override_active);

  // Debug
  if (!debug_pi_status_pub_) {
    return;
  }

  as2_msgs::msg::UInt16MultiArrayStamped debug_msg;
  debug_msg.layout.dim.resize(1);
  debug_msg.layout.dim[0].size = 3;
  debug_msg.layout.dim[0].label = "armed,pi_override_active,rx_link_valid";
  debug_msg.data = {
    static_cast<uint16_t>(armed), static_cast<uint16_t>(override_active),
    static_cast<uint16_t>(rx_link_valid)};
  debug_msg.stamp = fcStamp(msg.time_us);
  debug_pi_status_pub_->publish(debug_msg);
}

void IndiflightPlatform::checkLink()
{
  const pi_protocol::LinkStats stats = pi_protocol_client_.stats();
  if (stats.frames_parsed > 0) {
    link_check_timer_->cancel();
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "FC link up: " << stats.frames_parsed << " frames decoded from " <<
        stats.bytes_read << " bytes");
    return;
  }
  if (stats.bytes_read == 0) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No bytes from the FC on " << pi_protocol_device_ << ". The port opened, so the FC is "
        "not sending: check that FUNCTION_TELEMETRY_PI is assigned to this port in the "
        "firmware, and that it is the one wired here.");
    return;
  }
  RCLCPP_WARN_STREAM_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    stats.bytes_read << " bytes from the FC but no frame decoded (" << stats.frames_gated <<
      " rejected by the stamp gate). The link carries data that does not parse: the firmware "
      "is most likely built against a different pi-protocol message table, or a different "
      "baudrate.");
}

void IndiflightPlatform::publishDebugRc()
{
  if (!debug_rc_command_pub_) {
    return;
  }
  // Assign the values from `channel_values_` to the `debug_rc_` message
  debug_rc_command_.data = channel_values_;

  // Publish the message
  debug_rc_command_.stamp = this->now();
  debug_rc_command_pub_->publish(debug_rc_command_);
}

}  // namespace as2_platform_indiflight
