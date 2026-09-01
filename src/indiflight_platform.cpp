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

#include <algorithm>
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
  if (!debug_rc_topic_.empty()) {
    debug_rc_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
      debug_rc_topic_, 1);
  }

  // Callbacks are registered unconditionally: a firmware build that does not
  // emit one of these messages never triggers its callback.
  pi_protocol_client_.setEkfInputsCallback(
    [this](const pi_EKF_INPUTS_t & msg) {onPiProtocolEkfInputs(msg);});
  pi_protocol_client_.setMotorStateCallback(
    [this](const pi_MOTOR_STATE_t & msg) {onPiProtocolMotorState(msg);});
  pi_protocol_client_.setRcCallback(
    [this](const pi_RC_t & msg) {onPiRc(msg);});
  pi_protocol_client_.setStatusCallback(
    [this](const pi_PI_STATUS_t & msg) {onPiStatus(msg);});
  pi_protocol_client_.setBatteryCallback(
    [this](const pi_BATTERY_t & msg) {onPiBattery(msg);});
  pi_protocol_client_.setTimesyncCallback(
    [this](const pi_TIMESYNC_t & msg) {onPiTimesync(msg);});

  // The only link to the FC: without it the platform has neither commands nor
  // state, so a failure here is fatal rather than degraded.
  if (!pi_protocol_client_.connect(pi_protocol_device_, pi_protocol_baudrate_)) {
    throw std::runtime_error(
            "Could not connect to pi-protocol device " + pi_protocol_device_);
  }
  RCLCPP_INFO(
    this->get_logger(), "pi-protocol connected on %s @ %d baud",
    pi_protocol_device_.c_str(), pi_protocol_baudrate_);

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

  // One exchange a second holds the offset without the link's floor latency in
  // it, for 16 bytes against a downlink already carrying hundreds per second.
  // The first one goes out here so the offset does not wait on the timer: the
  // EXTERNAL_POSE uplink is refused until an offset exists.
  timesync_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&IndiflightPlatform::requestTimesync, this));
  requestTimesync();

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

  base_link_frame_id_ = this->getBaseFrameId();
  odom_frame_id_ = this->getOdomFrameId();
  // Not namespaced, unlike the two above: the global reference is shared.
  earth_frame_id_ = this->getEarthFrameId();

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
  debug_rc_topic_ = getParameter("debug_topics.rc", debug_rc_topic_);

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

  mass_ = getParameter<double>("mass");
  if (mass_ <= 0.0) {
    throw std::runtime_error("mass must be strictly positive, it divides the commanded thrust");
  }

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

  const std::string command_send_mode_str = getParameter<std::string>("command_send_mode");
  if (command_send_mode_str == "acro_setpoint") {
    command_send_mode_ = CommandSendMode::ACRO_SETPOINT;
  } else if (command_send_mode_str == "rc_override") {
    command_send_mode_ = CommandSendMode::RC_OVERRIDE;
  } else if (command_send_mode_str == "auto") {
    command_send_mode_ = CommandSendMode::AUTO;
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Unknown command_send_mode '%s', defaulting to auto",
      command_send_mode_str.c_str());
    command_send_mode_ = CommandSendMode::AUTO;
  }

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
  // Sticks centred, throttle at minimum, every other channel at midpoint.
  channel_values_.clear();
  channel_values_.resize(4, 1500);
  channel_values_[RC_CHANNELS::ROLL] = 1500;
  channel_values_[RC_CHANNELS::PITCH] = 1500;
  channel_values_[RC_CHANNELS::THROTTLE] = 1000;
  channel_values_[RC_CHANNELS::YAW] = 1500;
}

bool IndiflightPlatform::ownSetArmingState(bool state)
{
  // The firmware refuses the arming channel to PI OVERRIDE, so this node cannot
  // arm whatever it sends. The actual state is read back via PI_STATUS.
  RCLCPP_WARN(
    this->get_logger(),
    "Arming is physical-radio-controlled on this platform - AS2 cannot arm/disarm it. "
    "Actual arm state is reported via pi-protocol PI_STATUS.");
  (void)state;
  return false;
}

bool IndiflightPlatform::ownSetOffboardControl(bool offboard)
{
  // Same reasoning as ownSetArmingState(): the firmware also refuses the channel
  // carrying PI OVERRIDE, so control is handed over by the pilot alone. The
  // actual state is read back via PI_STATUS in onPiStatus().
  RCLCPP_WARN(
    this->get_logger(),
    "Offboard is driven by the PI OVERRIDE switch on the physical radio - AS2 cannot set "
    "it. Actual state is reported via pi-protocol PI_STATUS.");
  (void)offboard;
  return false;
}

bool IndiflightPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  // The FC position controller tracks setpoints in the global ENU frame
  setCommandPoseFrameId(earth_frame_id_);
  setCommandTwistFrameId(earth_frame_id_);

  // UNSET is absent on purpose: it is the mode the platform starts in, not one
  // that can be requested.
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
    case as2_msgs::msg::ControlMode::SPEED:
      if (!acceptFcPositionControl("SPEED")) {
        return false;
      }
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      if (!acceptFcPositionControl("TRAJECTORY")) {
        return false;
      }
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
    case as2_msgs::msg::ControlMode::BODY_RATES:
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
    case as2_msgs::msg::ControlMode::SPEED:
      return sendSpeedCommand();
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      return sendTrajectoryCommand();
    case as2_msgs::msg::ControlMode::ATTITUDE:
      return sendAttitudeCommand();
    case as2_msgs::msg::ControlMode::BODY_RATES:
      return sendBodyRatesCommand();
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
  // as2::AerialPlatform already delivered the command in the declared frame
  const geometry_msgs::msg::PoseStamped & pose = command_pose_msg_;
  // An empty frame_id means no pose reference has arrived yet: a
  // default-constructed pose would command the origin with a zero quaternion
  // (NaN yaw). Refuse it.
  if (pose.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "POSITION mode active but no pose reference received yet - not sent");
    return false;
  }

  // In POSITION mode the twist reference is a speed limit, which go_to fills in
  const Eigen::Vector3d limit_ned = enuToNed(
    Eigen::Vector3d(
      command_twist_msg_.twist.linear.x, command_twist_msg_.twist.linear.y,
      command_twist_msg_.twist.linear.z)).cwiseAbs();

  return sendPoseSetpoint(pose, limit_ned, SETPOINT_POSITION);
}

bool IndiflightPlatform::sendSpeedCommand()
{
  const Eigen::Vector3d vel_ned = enuToNed(
    Eigen::Vector3d(
      command_twist_msg_.twist.linear.x, command_twist_msg_.twist.linear.y,
      command_twist_msg_.twist.linear.z));

  // The FC tracks yaw as an absolute angle in every mode, so the reference is
  // the pose command's yaw, which as2 keeps populated alongside the twist.
  geometry_msgs::msg::PoseStamped yaw_ref = command_pose_msg_;
  if (yaw_ref.header.frame_id.empty()) {
    yaw_ref.pose.orientation.w = 1.0;
  }

  return sendPoseSetpoint(yaw_ref, vel_ned, SETPOINT_VELOCITY);
}

bool IndiflightPlatform::sendTrajectoryCommand()
{
  if (command_trajectory_msg_.setpoints.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TRAJECTORY mode active but no setpoint received yet - not sent");
    return false;
  }

  const auto & setpoint = command_trajectory_msg_.setpoints.front();

  geometry_msgs::msg::PoseStamped pose;
  pose.header = command_trajectory_msg_.header;
  pose.pose.position.x = setpoint.position.x;
  pose.pose.position.y = setpoint.position.y;
  pose.pose.position.z = setpoint.position.z;
  as2::frame::eulerToQuaternion(0.0, 0.0, setpoint.yaw_angle, pose.pose.orientation);
  if (!toEarthFrame(pose)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TRAJECTORY command in frame '%s', could not convert to '%s' - not sent",
      pose.header.frame_id.c_str(), earth_frame_id_.c_str());
    return false;
  }

  const Eigen::Vector3d vel_ned = enuToNed(
    Eigen::Vector3d(setpoint.twist.x, setpoint.twist.y, setpoint.twist.z));

  return sendPoseSetpoint(pose, vel_ned, SETPOINT_TRAJECTORY);
}

float IndiflightPlatform::yawRateForMode(uint8_t & mode)
{
  if (getControlMode().yaw_mode != as2_msgs::msg::ControlMode::YAW_SPEED) {
    return 0.0f;
  }
  mode |= SETPOINT_YAW_RATE;
  // ENU to NED reverses the sense of rotation about the vertical axis
  return static_cast<float>(-command_twist_msg_.twist.angular.z);
}

bool IndiflightPlatform::sendPoseSetpoint(
  const geometry_msgs::msg::PoseStamped & pose,
  const Eigen::Vector3d & vel_ned, uint8_t mode)
{
  const float yaw_rate_ned = yawRateForMode(mode);
  const Eigen::Vector3d ned = enuToNed(
    Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  const double yaw_ned = yawEnuRadToNedRad(
    as2::frame::getYawFromQuaternion(pose.pose.orientation));

  const float scalar_c = (mode & SETPOINT_YAW_RATE) ?
    yaw_rate_ned : static_cast<float>(yaw_ned);

  if (!pi_protocol_client_.sendSetpoint(
      static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z()),
      static_cast<float>(vel_ned.x()), static_cast<float>(vel_ned.y()),
      static_cast<float>(vel_ned.z()), scalar_c, mode))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Could not send SETPOINT (no FC downlink tick yet, or write failed)");
    return false;
  }
  return true;
}

Eigen::Vector3d IndiflightPlatform::specificForceFrd() const
{
  const double thrust = std::clamp<double>(command_thrust_msg_.thrust, min_thrust_, max_thrust_);
  return fluToFrd(Eigen::Vector3d(0.0, 0.0, thrust / mass_));
}

bool IndiflightPlatform::sendAttitudeCommand()
{
  const Eigen::Quaterniond q_ned_frd = enuFluToNedFrd(
    Eigen::Quaterniond(
      command_pose_msg_.pose.orientation.w, command_pose_msg_.pose.orientation.x,
      command_pose_msg_.pose.orientation.y, command_pose_msg_.pose.orientation.z));
  const Eigen::Vector3d spf = specificForceFrd();

  if (!pi_protocol_client_.sendSetpoint(
      static_cast<float>(q_ned_frd.x()), static_cast<float>(q_ned_frd.y()),
      static_cast<float>(q_ned_frd.z()), static_cast<float>(spf.x()),
      static_cast<float>(spf.y()), static_cast<float>(spf.z()),
      static_cast<float>(q_ned_frd.w()), SETPOINT_ATTITUDE))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Could not send ATTITUDE SETPOINT to the flight controller");
    return false;
  }
  return true;
}

bool IndiflightPlatform::sendAcroSetpoint()
{
  const Eigen::Vector3d rates = fluToFrd(
    Eigen::Vector3d(
      command_twist_msg_.twist.angular.x, command_twist_msg_.twist.angular.y,
      command_twist_msg_.twist.angular.z));
  const Eigen::Vector3d spf = specificForceFrd();

  if (!pi_protocol_client_.sendSetpoint(
      static_cast<float>(rates.x()), static_cast<float>(rates.y()),
      static_cast<float>(rates.z()), static_cast<float>(spf.x()),
      static_cast<float>(spf.y()), static_cast<float>(spf.z()), 0.0f, SETPOINT_ACRO))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Could not send ACRO SETPOINT to the flight controller");
    return false;
  }
  return true;
}

bool IndiflightPlatform::sendBodyRatesCommand()
{
  switch (command_send_mode_) {
    case CommandSendMode::ACRO_SETPOINT:
      return sendAcroSetpoint();
    case CommandSendMode::RC_OVERRIDE:
      return sendRcOverrideCommand();
    case CommandSendMode::AUTO:
    default:
      // The pilot's switch picks the offboard language, and the two are exclusive
      return fc_pos_ctl_active_ ? sendAcroSetpoint() : sendRcOverrideCommand();
  }
}

bool IndiflightPlatform::sendRcOverrideCommand()
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
  uint16_t yaw_pulse = static_cast<uint16_t>(1500 + yaw / yaw_slope_);
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
  // Motor speed is no longer sourced from here -- see onPiProtocolMotorState().
  // EKF_INPUTS capped at 6 motors, carried no commanded-output field, and its
  // omega came from the same indiRun state MOTOR_STATE now carries directly.
}

void IndiflightPlatform::onPiProtocolMotorState(const pi_MOTOR_STATE_t & msg)
{
  // Betaflight mixer output order (quad X: [RR, FR, RL, FL]), not the
  // indi_controller convention used elsewhere in the workspace.
  //
  // TIMING: velocity[i] and effort[i] are NOT computed from each other. u
  // (effort) is the command indi.c computed from the PREVIOUS control tick's
  // omega, before that tick's indiUpdateActuatorState() call overwrote omega
  // with a fresh DSHOT-telemetry reading -- so effort[i] here corresponds to
  // velocity[i] one control tick EARLIER, not the velocity[i] alongside it in
  // this same message. See msgs/MOTOR_STATE.yaml for the full derivation.
  const rclcpp::Time stamp = fcStamp(msg.time_us);

  sensor_msgs::msg::JointState motor_msg;
  motor_msg.header.stamp = stamp;
  motor_msg.header.frame_id = base_link_frame_id_;
  const std::array<uint16_t, 4> omegas = {msg.omega1, msg.omega2, msg.omega3, msg.omega4};
  const std::array<int16_t, 4> us = {msg.u1, msg.u2, msg.u3, msg.u4};
  for (int i = 0; i < num_rotors_; i++) {
    motor_msg.name.emplace_back("motor" + std::to_string(i));
    motor_msg.velocity.emplace_back(static_cast<double>(omegas[i]));
    motor_msg.effort.emplace_back(static_cast<double>(us[i]) / 32767.0);
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

void IndiflightPlatform::onPiRc(const pi_RC_t & msg)
{
  if (!debug_rc_pub_) {
    return;
  }
  const std::array<int16_t, 16> channels = {
    msg.channel_1, msg.channel_2, msg.channel_3, msg.channel_4,
    msg.channel_5, msg.channel_6, msg.channel_7, msg.channel_8,
    msg.channel_9, msg.channel_10, msg.channel_11, msg.channel_12,
    msg.channel_13, msg.channel_14, msg.channel_15, msg.channel_16};

  as2_msgs::msg::UInt16MultiArrayStamped debug_msg;
  debug_msg.layout.dim.resize(1);
  debug_msg.layout.dim[0].size = channels.size();
  debug_msg.layout.dim[0].label = "channel_1..channel_16";
  debug_msg.data.reserve(channels.size());
  for (const int16_t value : channels) {
    debug_msg.data.emplace_back(static_cast<uint16_t>(std::max<int16_t>(value, 0)));
  }
  debug_msg.stamp = fcStamp(msg.time_us);
  debug_rc_pub_->publish(debug_msg);
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

  // Mocap gives position and attitude; the velocity would have to be
  // differentiated here and the FC would write it straight into the state.
  if (!pi_protocol_client_.sendExternalPose(
      *fc_time_us,
      static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z()),
      0.0f, 0.0f, 0.0f,
      static_cast<float>(q_ned_frd.w()), static_cast<float>(q_ned_frd.x()),
      static_cast<float>(q_ned_frd.y()), static_cast<float>(q_ned_frd.z()),
      EXT_POSE_USE_POS | EXT_POSE_USE_QUAT))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Could not send EXTERNAL_POSE to the flight controller");
    return;
  }

  // The client reports success for a skipped pose, so the count is the only signal
  const uint64_t skipped = pi_protocol_client_.stats().ext_pose_skipped;
  if (skipped > ext_pose_skipped_seen_) {
    ext_pose_skipped_seen_ = skipped;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "EXTERNAL_POSE does not advance the FC tick (%" PRIu64 " dropped): the FC EKF is "
      "not being corrected", skipped);
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
  constexpr uint8_t kFlagEkfConverged = 1 << 3;
  constexpr uint8_t kFlagPosCtlActive = 1 << 4;

  const bool armed = msg.flags & kFlagArmed;
  const bool override_active = msg.flags & kFlagPiOverrideActive;
  const bool rx_link_valid = msg.flags & kFlagRxLinkValid;
  fc_ekf_converged_ = msg.flags & kFlagEkfConverged;
  fc_pos_ctl_active_ = msg.flags & kFlagPosCtlActive;

  // POS_CTL is the offboard family, PI OVERRIDE the manual one
  updatePlatformState(armed, override_active || fc_pos_ctl_active_);

  // Debug
  if (!debug_pi_status_pub_) {
    return;
  }

  as2_msgs::msg::UInt16MultiArrayStamped debug_msg;
  debug_msg.layout.dim.resize(1);
  debug_msg.layout.dim[0].size = 5;
  debug_msg.layout.dim[0].label =
    "armed,pi_override_active,rx_link_valid,ekf_converged,pos_ctl_active";
  debug_msg.data = {
    static_cast<uint16_t>(armed), static_cast<uint16_t>(override_active),
    static_cast<uint16_t>(rx_link_valid), static_cast<uint16_t>(fc_ekf_converged_),
    static_cast<uint16_t>(fc_pos_ctl_active_)};
  debug_msg.stamp = fcStamp(msg.time_us);
  debug_pi_status_pub_->publish(debug_msg);
}

void IndiflightPlatform::requestTimesync()
{
  pi_protocol_client_.sendTimesyncRequest(this->get_clock()->now().nanoseconds());
}

void IndiflightPlatform::onPiTimesync(const pi_TIMESYNC_t & msg)
{
  // Read first: everything done before it counts as round trip the FC never
  // spent, and lands in the offset as error.
  const int64_t host_recv_ns = this->get_clock()->now().nanoseconds();
  const auto rtt_ns = pi_protocol_clock_sync_.syncRoundTrip(
    msg.fc_time_us, static_cast<int64_t>(msg.host_ns), host_recv_ns);
  if (!rtt_ns) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "TIMESYNC seq %u discarded: round trip too slow to place the FC stamp", msg.seq);
    return;
  }
  // The first exchange is a state change worth seeing: it is what releases the
  // EXTERNAL_POSE uplink, and its round trip is the only measure of the link's
  // latency anything reports. The rest are routine.
  if (!timesync_locked_) {
    timesync_locked_ = true;
    RCLCPP_INFO(
      this->get_logger(), "FC clock locked over TIMESYNC, round trip %.3f ms", *rtt_ns / 1e6);
    return;
  }
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 10000,
    "TIMESYNC seq %u round trip %.3f ms", msg.seq, *rtt_ns / 1e6);
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
