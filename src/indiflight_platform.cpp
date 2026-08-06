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
 * @file pixhawk_platform.cpp
 *
 * MavlinkPlatform class implementation
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 */

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <iostream>

#include "as2_platform_indiflight/indiflight_platform.hpp"
#include "msp/msp_msg.hpp"

double convert_deg_s_to_rad_s(double deg_s)
{
  return deg_s / 180.0 * M_PI;
}


void notImplemented()
{
  throw std::runtime_error("NOT IMPLEMENTED");
}


namespace as2_platform_indiflight
{

void IndiflightPlatform::readParameters()
{
  this->declare_parameter<bool>("external_odom");

  // pi-protocol config parameters - the only serial link this node depends on
  // at runtime now (see the constructor's connect() call).
  this->declare_parameter<bool>("pi_protocol.enable", pi_protocol_enable_);
  this->declare_parameter<std::string>("pi_protocol.device", pi_protocol_device_);
  this->declare_parameter<int>("pi_protocol.baudrate", pi_protocol_baudrate_);

  // IMU covariance parameters, used by onPiProtocolEkfInputs().
  this->declare_parameter<float>("imu.covariance.gyro");
  this->declare_parameter<float>("imu.covariance.accel");
  this->declare_parameter<float>("imu.covariance.orientation");

  // Rotation (rad) from indiflight's FRD firmware frame to the user's
  // preferred body frame, applied to IMU samples by rotateImuToDesiredFrame().
  // Default desired_frame_T.r = pi implements FRD -> FLU (keep X, negate Y and Z).
  // Warn (rather than require, like the parameters above) since this default
  // is a sensible fallback and silently flying with a wrong frame convention
  // is the kind of bug that's easy to miss until it's in the air.
  const auto & param_overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  if (!param_overrides.count("desired_frame_T.r") || !param_overrides.count("desired_frame_T.p") ||
    !param_overrides.count("desired_frame_T.y"))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "desired_frame_T.r/p/y not fully specified - missing value(s) will use the default "
      "FRD -> FLU rotation (r=%.5f, p=%.5f, y=%.5f)", M_PI, 0.0, 0.0);
  }
  this->declare_parameter<float>("desired_frame_T.r", static_cast<float>(M_PI));
  this->declare_parameter<float>("desired_frame_T.p", 0.0f);
  this->declare_parameter<float>("desired_frame_T.y", 0.0f);

  // Set publishers frequency. Set frequency to 0 to disable publication
  this->declare_parameter<float>("battery_hz");
  this->declare_parameter<float>("altitude_hz");
  this->declare_parameter<float>("attitude_hz");
  this->declare_parameter<float>("rc_hz");
  this->declare_parameter<float>("motor_hz");

  this->declare_parameter<float>("alpha_voltage");
  this->declare_parameter<float>("min_cell_voltage");
  this->declare_parameter<float>("max_cell_voltage");

  this->declare_parameter<float>("yaw_rate.min");
  this->declare_parameter<float>("yaw_rate.max");
  this->declare_parameter<float>("pitch_rate.min");
  this->declare_parameter<float>("pitch_rate.max");
  this->declare_parameter<float>("roll_rate.min");
  this->declare_parameter<float>("roll_rate.max");
  this->declare_parameter<float>("thrust.min");
  this->declare_parameter<float>("thrust.max");

  this->declare_parameter<bool>("use_thrust_map");

  // true (default): header.stamp on IMU/motor messages is reconstructed
  // host time for the instant the FC actually sampled the measurement
  // (via pi_protocol_clock_sync_, latency-jitter-rejected but subject to
  // whatever FC clock rate error exists - see indi_experiment clock-offset
  // investigation notes). false: header.stamp is simply this node's
  // get_clock()->now() at message-arrival time, matching the convention
  // used elsewhere in AS2 - trades away latency-jitter rejection and the
  // (usually small) FC-instant accuracy for guaranteed consistency with
  // other now()-stamped topics. debug/platform/og_timestamp keeps
  // publishing both the stamp actually used AND the raw FC time_ref
  // regardless of this setting, so the true offset can still be recovered
  // post-hoc either way.
  this->declare_parameter<bool>("use_fcu_stamps", use_fcu_stamps_);

  this->declare_parameter<bool>("limit_output");
  this->declare_parameter<float>("limit_roll_percent");
  this->declare_parameter<float>("limit_pitch_percent");
  this->declare_parameter<float>("limit_yaw_percent");
  this->declare_parameter<float>("limit_thrust_percent");


  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");

  external_odom_ = this->get_parameter("external_odom").as_bool();
  use_fcu_stamps_ = this->get_parameter("use_fcu_stamps").as_bool();

  pi_protocol_enable_ = this->get_parameter("pi_protocol.enable").as_bool();
  pi_protocol_device_ = this->get_parameter("pi_protocol.device").as_string();
  pi_protocol_baudrate_ = this->get_parameter("pi_protocol.baudrate").as_int();

  imu_gyro_covariance_ = this->get_parameter("imu.covariance.gyro").as_double();
  imu_accel_covariance_ = this->get_parameter("imu.covariance.accel").as_double();
  imu_orientation_covariance_ = this->get_parameter("imu.covariance.orientation").as_double();

  desired_frame_roll_ = this->get_parameter("desired_frame_T.r").as_double();
  desired_frame_pitch_ = this->get_parameter("desired_frame_T.p").as_double();
  desired_frame_yaw_ = this->get_parameter("desired_frame_T.y").as_double();
  desired_frame_rotation_ =
    (Eigen::AngleAxisd(desired_frame_yaw_, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(desired_frame_pitch_, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(desired_frame_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();

  battery_hz_ = this->get_parameter("battery_hz").as_double();
  altitude_hz_ = this->get_parameter("altitude_hz").as_double();
  attitude_hz_ = this->get_parameter("attitude_hz").as_double();
  rc_hz_ = this->get_parameter("rc_hz").as_double();
  motor_hz_ = this->get_parameter("motor_hz").as_double();

  alpha_voltage_ = this->get_parameter("alpha_voltage").as_double();
  min_cell_voltage_ = this->get_parameter("min_cell_voltage").as_double();
  max_cell_voltage_ = this->get_parameter("max_cell_voltage").as_double();

  max_thrust_ = this->get_parameter("thrust.max").as_double();
  min_thrust_ = this->get_parameter("thrust.min").as_double();
  max_pitch_rate_ = convert_deg_s_to_rad_s(this->get_parameter("pitch_rate.max").as_double());
  min_pitch_rate_ = convert_deg_s_to_rad_s(this->get_parameter("pitch_rate.min").as_double());
  max_roll_rate_ = convert_deg_s_to_rad_s(this->get_parameter("roll_rate.max").as_double());
  min_roll_rate_ = convert_deg_s_to_rad_s(this->get_parameter("roll_rate.min").as_double());
  max_yaw_rate_ = convert_deg_s_to_rad_s(this->get_parameter("yaw_rate.max").as_double());
  min_yaw_rate_ = convert_deg_s_to_rad_s(this->get_parameter("yaw_rate.min").as_double());

  use_thrust_map_ = this->get_parameter("use_thrust_map").as_bool();

  limit_output_ = this->get_parameter("limit_output").as_bool();
  limit_roll_percent_ = this->get_parameter("limit_roll_percent").as_double();
  limit_pitch_percent_ = this->get_parameter("limit_pitch_percent").as_double();
  limit_yaw_percent_ = this->get_parameter("limit_yaw_percent").as_double();
  limit_thrust_percent_ = this->get_parameter("limit_thrust_percent").as_double();

  RCLCPP_INFO(
    this->get_logger(), "pi-protocol device: %s @ %d baud",
    pi_protocol_device_.c_str(), pi_protocol_baudrate_);
  RCLCPP_INFO(this->get_logger(), "External odometry mode: %s", external_odom_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "IMU/motor header.stamp source: %s",
    use_fcu_stamps_ ? "FC-instant (pi_protocol_clock_sync_)" : "arrival time (now())");
  RCLCPP_INFO(
    this->get_logger(), "Simulation mode: %s",
    this->get_parameter("use_sim_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Thrust bounds: [%f, %f]", min_thrust_, max_thrust_);
  RCLCPP_INFO(this->get_logger(), "Pitch rate bounds: [%f, %f]", min_pitch_rate_, max_pitch_rate_);
  RCLCPP_INFO(this->get_logger(), "Roll rate bounds: [%f, %f]", min_roll_rate_, max_roll_rate_);
  RCLCPP_INFO(this->get_logger(), "Yaw rate bounds: [%f, %f]", min_yaw_rate_, max_yaw_rate_);
  RCLCPP_INFO(
    this->get_logger(), "desired_frame_T rotation (r,p,y): [%f, %f, %f]",
    desired_frame_roll_, desired_frame_pitch_, desired_frame_yaw_);
  computeControlSlopes();

  RCLCPP_INFO(this->get_logger(), "Limiting output: %s", limit_output_ ? "true" : "false");
  if (limit_output_) {
    RCLCPP_INFO(this->get_logger(), "Roll limit: %f", limit_roll_percent_);
    RCLCPP_INFO(this->get_logger(), "Pitch limit: %f", limit_pitch_percent_);
    RCLCPP_INFO(this->get_logger(), "Yaw limit: %f", limit_yaw_percent_);
    RCLCPP_INFO(this->get_logger(), "Thrust limit: %f", limit_thrust_percent_);
  }
}


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

  // Create all publishers BEFORE starting any subscriptions/connections.
  // pi_protocol_client_.connect() starts a reader thread that can invoke
  // callbacks immediately, so any publisher used in a callback must exist
  // before connect() returns, otherwise the first message races with
  // publisher construction and causes a null-dereference crash.
  debug_rc_command_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
    "debug/rc/command", 1);
  og_timestamp_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>(
    "debug/platform/og_timestamp", rclcpp::SensorDataQoS());
  attitude_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("attitude", 1);
  debug_motors_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
    "debug/motors", 1);
  debug_pi_status_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
    "debug/pi_status", 1);
  if (rc_hz_ > 0.0) {
    debug_rc_read_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
      "debug/rc/read", 1);
  }

  // pi-protocol is now the only link this node depends on at runtime: RC_OVERRIDE
  // commands, and PI_STATUS/BATTERY telemetry for arm/offboard state and thrust-map
  // voltage correction. Unlike the old MSP-optional framing this replaced, a
  // connect failure here is fatal - there is no other command/state link left.
  if (pi_protocol_enable_) {
    pi_protocol_client_.setEkfInputsCallback(
      [this](const pi_EKF_INPUTS_t & msg) {onPiProtocolEkfInputs(msg);});
    pi_protocol_client_.setStatusCallback(
      [this](const pi_PI_STATUS_t & msg) {onPiStatus(msg);});
    pi_protocol_client_.setBatteryCallback(
      [this](const pi_BATTERY_t & msg) {onPiBattery(msg);});
    if (!pi_protocol_client_.connect(pi_protocol_device_, pi_protocol_baudrate_)) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not connect to pi-protocol device %s",
        pi_protocol_device_.c_str());
      throw std::runtime_error("Could not connect to pi-protocol device");
    }
    RCLCPP_INFO(
      this->get_logger(), "pi-protocol connected on %s @ %d baud",
      pi_protocol_device_.c_str(), pi_protocol_baudrate_);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "pi_protocol.enable is false - this node has no command/state link to the FC.");
  }

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

void IndiflightPlatform::configureSensors()
{
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  battery_sensor_ptr_ = std::make_unique<as2::sensors::Battery>("battery", this);
  motor_sensor_ptr_ =
    std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::JointState>>("motor_angular_speed", this);
  // gps_sensor_ptr_ = std::make_unique<as2::sensors::GPS>("gps", this);

  // odometry_raw_estimation_ptr_ =
  //   std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
}

bool IndiflightPlatform::ownSetArmingState(bool state)
{
  // ARM lives on the physical radio underneath indiflight's PI OVERRIDE mode,
  // permanently outside pi-protocol's 4-channel RC_OVERRIDE - this node can't
  // set it. Actual arm state is read back from the FC via PI_STATUS and
  // reported upward from onPiStatus(), independent of this call's return value.
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
  // ONLY SUPPORTS ACRO MODE
  if (msg.control_mode != as2_msgs::msg::ControlMode::ACRO) {
    RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
    return false;
  }
  return true;
}

bool IndiflightPlatform::ownSendCommand()
{
  // ONLY ACRO MODE IS SUPPORTED

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
    RCLCPP_ERROR(this->get_logger(), "Could not send RC_OVERRIDE to flight controller");
    return false;
  }
  return true;
}

void IndiflightPlatform::ownKillSwitch()
{
  // The hard-kill AUX channel this used to drive over MSP lives on the
  // physical radio now, outside pi-protocol's 4-channel RC_OVERRIDE - this
  // node has no way to guarantee a motor cut. Not attempting a best-effort
  // "throttle to minimum" here on purpose: with idle-throttle/airmode that
  // wouldn't actually stop the motors, and sending it could read as a real
  // kill when it isn't one. Real kill authority stays with the safety pilot.
  RCLCPP_WARN(
    this->get_logger(),
    "Kill-switch is physical-radio-only on this platform - AS2 cannot cut motors from here.");
}

void IndiflightPlatform::ownStopPlatform() {RCLCPP_WARN(this->get_logger(), "NOT IMPLEMENTED");}

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
  constexpr float kGyroLsbToRadps = (2000.f * M_PI / 180.f) / 32768.f;     // +-2000 deg/s full scale

  // Captured first, right after piParse() hands off the struct, to keep the
  // clock-sync offset sample as tight as possible.
  const int64_t host_now_ns = this->get_clock()->now().nanoseconds();
  // Always feed the filter, even when use_fcu_stamps_ is false, so it stays
  // warmed up (and so og_timestamp's header.stamp - see below - reflects
  // whichever mode is actually selected without a cold-start gap if the
  // param is flipped later).
  const int64_t synced_ns = pi_protocol_clock_sync_.sync(msg.time_us, host_now_ns);
  const rclcpp::Time stamp(use_fcu_stamps_ ? synced_ns : host_now_ns);

  Eigen::Vector3d angular_velocity(
    msg.p * kGyroLsbToRadps, msg.q * kGyroLsbToRadps, msg.r * kGyroLsbToRadps);
  Eigen::Vector3d linear_acceleration(
    msg.x * kAccelLsbToMps2, msg.y * kAccelLsbToMps2, msg.z * kAccelLsbToMps2);
  // Rotate out of indiflight's FRD firmware frame into desired_frame_T (default
  // FLU) before publishing, so base_link_frame_id_ actually matches its contents.
  rotateImuToDesiredFrame(angular_velocity, linear_acceleration);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = stamp;
  imu_msg.header.frame_id = base_link_frame_id_;
  imu_msg.angular_velocity.x = angular_velocity.x();
  imu_msg.angular_velocity.y = angular_velocity.y();
  imu_msg.angular_velocity.z = angular_velocity.z();
  imu_msg.linear_acceleration.x = linear_acceleration.x();
  imu_msg.linear_acceleration.y = linear_acceleration.y();
  imu_msg.linear_acceleration.z = linear_acceleration.z();
  // no deg->rad conversion needed here, pi-protocol's gyro is already rad/s.
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

  sensor_msgs::msg::JointState motor_msg;
  motor_msg.header.stamp = stamp;
  motor_msg.header.frame_id = base_link_frame_id_;
  // Indexed in Betaflight's own mixer output order ([RR, FR, RL, FL], see
  // mixer_init.c mixerQuadX[]) - NOT the indi_controller/simulator convention
  // ([FR, RR, RL, FL]) used elsewhere in the wider workspace. omega1-4 are
  // sent as plain rad/s with no scale factor (unlike x/y/z/p/q/r above).
  motor_msg.name = {"motor0", "motor1", "motor2", "motor3"};
  motor_msg.velocity = {
    static_cast<double>(msg.omega1), static_cast<double>(msg.omega2),
    static_cast<double>(msg.omega3), static_cast<double>(msg.omega4)};
  // sensor_measurements/motor_angular_speed (as2::sensors::Sensor)
  motor_sensor_ptr_->updateData(motor_msg);

  // Raw FC time_us preserved alongside the corrected stamp, shared by both
  // topics above since they now always come from the same synchronized
  // sample - see PiProtocolClockSync for why time_us can't be used as
  // header.stamp directly.
  sensor_msgs::msg::TimeReference time_ref_msg;
  time_ref_msg.header.stamp = stamp;
  time_ref_msg.header.frame_id = base_link_frame_id_;
  time_ref_msg.time_ref = rclcpp::Time(static_cast<int64_t>(msg.time_us) * 1000);
  time_ref_msg.source = "indiflight_fc_micros";
  og_timestamp_pub_->publish(time_ref_msg);
}

void IndiflightPlatform::onAltitude(const msp::msg::Altitude & altitude)
{
  std::cout << "Altitude: " << altitude << std::endl;
}

void IndiflightPlatform::onAttitude(const msp::msg::Attitude & attitude)
{
  geometry_msgs::msg::QuaternionStamped attitude_msg;
  attitude_msg.header.stamp = this->get_clock()->now();
  attitude_msg.header.frame_id = odom_frame_id_;
  // print attitude
  RCLCPP_INFO(
    this->get_logger(), "Attitude: roll: %f, pitch: %f, yaw: %f",
    static_cast<double>(attitude.roll),
    static_cast<double>(attitude.pitch),
    static_cast<double>(attitude.yaw));

  // Convert Euler angles (degrees) to quaternion
  double roll_rad = attitude.roll * M_PI / 180.0;
  double pitch_rad = attitude.pitch * M_PI / 180.0;
  double yaw_rad = attitude.yaw * M_PI / 180.0;

  as2::frame::eulerToQuaternion(roll_rad, pitch_rad, yaw_rad, attitude_msg.quaternion);

  attitude_pub_->publish(attitude_msg);
}

void IndiflightPlatform::onMotor(const msp::msg::Motor & motor)
{
  as2_msgs::msg::UInt16MultiArrayStamped debug_motor_msg;
  debug_motor_msg.layout.dim.resize(1);
  debug_motor_msg.layout.dim[0].size = motor.motor.size();
  debug_motor_msg.data.reserve(motor.motor.size());

  for (auto motor_value : motor.motor) {
    debug_motor_msg.data.emplace_back(motor_value);
  }

  debug_motor_msg.stamp = this->now();
  debug_motors_pub_->publish(debug_motor_msg);
}

void IndiflightPlatform::onPiBattery(const pi_BATTERY_t & msg)
{
  float voltage_filtered = alpha_voltage_ * voltage_ + (1 - alpha_voltage_) * msg.voltage;
  float max_batt_voltage = max_cell_voltage_ * msg.cell_count;
  float min_batt_voltage = min_cell_voltage_ * msg.cell_count;

  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = this->get_clock()->now();
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

  // This reports FC-driven state - it must NOT go through
  // setArmingState()/setOffboardControl() (as2_core::AerialPlatform). Those
  // gate on ownSetArmingState()/ownSetOffboardControl(), which this platform
  // deliberately always fails (arm/offboard are physical-radio-only - AS2
  // cannot command them, see ownSetArmingState() above). Routing a *report*
  // through that same gate means platform_info_msg_ could never actually
  // reflect the FC's real state: ownSetArmingState() returning false is
  // exactly what was silently blocking .armed from ever becoming true here,
  // regardless of what PI_STATUS said. Update platform_info_msg_ directly
  // instead - protected members, accessible from this derived class -
  // replicating what setArmingState()/setOffboardControl() do on success.
  //
  // Edge-triggered on purpose too: PI_STATUS arrives continuously (~50Hz),
  // and handleStateMachineEvent() isn't meant to be re-fired every tick for
  // a state that hasn't changed.
  if (armed != set_arm_) {
    set_arm_ = armed;
    platform_info_msg_.armed = armed;
    handleStateMachineEvent(
      armed ? as2_msgs::msg::PlatformStateMachineEvent::ARM :
      as2_msgs::msg::PlatformStateMachineEvent::DISARM);
  }
  if (override_active != set_offboard_) {
    set_offboard_ = override_active;
    platform_info_msg_.offboard = override_active;
  }

  as2_msgs::msg::UInt16MultiArrayStamped debug_msg;
  debug_msg.layout.dim.resize(1);
  debug_msg.layout.dim[0].size = 3;
  debug_msg.layout.dim[0].label = "armed,pi_override_active,rx_link_valid";
  debug_msg.data = {
    static_cast<uint16_t>(armed), static_cast<uint16_t>(override_active),
    static_cast<uint16_t>(rx_link_valid)};
  debug_msg.stamp = this->now();
  debug_pi_status_pub_->publish(debug_msg);
}

void IndiflightPlatform::onRc(const msp::msg::Rc & rc)
{
  as2_msgs::msg::UInt16MultiArrayStamped debug_rc_msg;
  debug_rc_msg.layout.dim.resize(1);
  debug_rc_msg.layout.dim[0].size = rc.channels.size();
  debug_rc_msg.data.reserve(rc.channels.size());

  for (auto channel_value : rc.channels) {
    debug_rc_msg.data.emplace_back(channel_value);
  }

  rcArm(debug_rc_msg.data[4]);
  rcOffboard(debug_rc_msg.data[6]);

  debug_rc_msg.stamp = this->now();
  debug_rc_read_pub_->publish(debug_rc_msg);
}

void IndiflightPlatform::publishDebugRc()
{
  // Assign the values from `channel_values_` to the `debug_rc_` message
  debug_rc_command_.data = channel_values_;

  // Publish the message
  debug_rc_command_.stamp = this->now();
  debug_rc_command_pub_->publish(debug_rc_command_);
}

void IndiflightPlatform::rcArm(int channel)
{
  if (channel > 1500) {
    if (!set_arm_) {
      RCLCPP_INFO(this->get_logger(), "ARM received, arming...\n");
      set_arm_ = true;
      setArmingState(set_arm_);
    }
  } else {
    if (set_arm_) {
      RCLCPP_INFO(this->get_logger(), "DISARM received, disarming...\n");
      set_arm_ = false;
      setArmingState(set_arm_);
    }
  }
}

void IndiflightPlatform::rcOffboard(int channel)
{
  if (channel > 1500) {
    if (!set_offboard_) {
      RCLCPP_INFO(this->get_logger(), "OFFBOARD received, offboard ON...\n");
      set_offboard_ = true;
      setOffboardControl(set_offboard_);
    }
  } else {
    if (set_offboard_) {
      RCLCPP_INFO(this->get_logger(), "OFFBOARD received, offboard OFF...\n");
      set_offboard_ = false;
      setOffboardControl(set_offboard_);
    }
  }
}


}  // namespace as2_platform_indiflight
