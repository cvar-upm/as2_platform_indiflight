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

#include "as2_platform_indiflight/betaflight_platform.hpp"
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

void BetaflightPlatform::readParameters()
{
  this->declare_parameter<std::string>("device");
  this->declare_parameter<int>("baudrate");
  this->declare_parameter<bool>("external_odom");

  // pi-protocol config parameters (indiflight high-rate telemetry, separate UART from MSP)
  this->declare_parameter<bool>("pi_protocol.enable", pi_protocol_enable_);
  this->declare_parameter<std::string>("pi_protocol.device", pi_protocol_device_);
  this->declare_parameter<int>("pi_protocol.baudrate", pi_protocol_baudrate_);

  // IMU config parameters
  this->declare_parameter<float>("imu.frequency");
  this->declare_parameter<float>("imu.covariance.gyro");
  this->declare_parameter<float>("imu.covariance.accel");
  this->declare_parameter<float>("imu.covariance.orientation");

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

  this->declare_parameter<bool>("limit_output");
  this->declare_parameter<float>("limit_roll_percent");
  this->declare_parameter<float>("limit_pitch_percent");
  this->declare_parameter<float>("limit_yaw_percent");
  this->declare_parameter<float>("limit_thrust_percent");


  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");

  device_ = this->get_parameter("device").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  external_odom_ = this->get_parameter("external_odom").as_bool();

  pi_protocol_enable_ = this->get_parameter("pi_protocol.enable").as_bool();
  pi_protocol_device_ = this->get_parameter("pi_protocol.device").as_string();
  pi_protocol_baudrate_ = this->get_parameter("pi_protocol.baudrate").as_int();

  imu_hz_ = this->get_parameter("imu.frequency").as_double();
  imu_gyro_covariance_ = this->get_parameter("imu.covariance.gyro").as_double();
  imu_accel_covariance_ = this->get_parameter("imu.covariance.accel").as_double();
  imu_orientation_covariance_ = this->get_parameter("imu.covariance.orientation").as_double();

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

  RCLCPP_INFO(this->get_logger(), "Device: %s", device_.c_str());
  RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);
  RCLCPP_INFO(this->get_logger(), "External odometry mode: %s", external_odom_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "Simulation mode: %s",
    this->get_parameter("use_sim_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Thrust bounds: [%f, %f]", min_thrust_, max_thrust_);
  RCLCPP_INFO(this->get_logger(), "Pitch rate bounds: [%f, %f]", min_pitch_rate_, max_pitch_rate_);
  RCLCPP_INFO(this->get_logger(), "Roll rate bounds: [%f, %f]", min_roll_rate_, max_roll_rate_);
  RCLCPP_INFO(this->get_logger(), "Yaw rate bounds: [%f, %f]", min_yaw_rate_, max_yaw_rate_);
  computeControlSlopes();

  RCLCPP_INFO(this->get_logger(), "Limiting output: %s", limit_output_ ? "true" : "false");
  if (limit_output_) {
    RCLCPP_INFO(this->get_logger(), "Roll limit: %f", limit_roll_percent_);
    RCLCPP_INFO(this->get_logger(), "Pitch limit: %f", limit_pitch_percent_);
    RCLCPP_INFO(this->get_logger(), "Yaw limit: %f", limit_yaw_percent_);
    RCLCPP_INFO(this->get_logger(), "Thrust limit: %f", limit_thrust_percent_);
  }
}


BetaflightPlatform::BetaflightPlatform(const rclcpp::NodeOptions & options)
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

  auto out = fcu_.connect(device_, baudrate_, 0.0, true);
  if (!out) {
    RCLCPP_ERROR(this->get_logger(), "Could not connect to device %s", device_.c_str());
    throw std::runtime_error("Could not connect to device");
  }
  fcu_.setLoggingLevel(msp::client::LoggingLevel::INFO);
  fcu_.setControlSource(fcu::ControlSource::MSP);

  // Create all publishers BEFORE starting any subscriptions.
  // subscribe() starts a timer thread that fires immediately on an already-open
  // port, so any publisher used in a callback must exist before the matching
  // subscribe() call returns, otherwise the first response races with
  // publisher construction and causes a null-dereference crash.
  debug_rc_command_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
    "debug/rc/command", 1);
  raw_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("raw_imu", 1);
  imu_high_rate_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "imu_high_rate", rclcpp::SensorDataQoS());
  attitude_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("attitude", 1);
  debug_motors_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
    "debug/motors", 1);
  if (rc_hz_ > 0.0) {
    debug_rc_read_pub_ = this->create_publisher<as2_msgs::msg::UInt16MultiArrayStamped>(
      "debug/rc/read", 1);
  }

  fcu_.subscribe(&BetaflightPlatform::onStatus, this, 1);
  if (imu_hz_ > 0.0) {
    fcu_.subscribe(&BetaflightPlatform::onImu, this, 1.0 / imu_hz_);
  } else {
    RCLCPP_WARN(this->get_logger(), "IMU frequency is set to 0, IMU data will not be published");
  }
  if (battery_hz_ > 0.0) {
    fcu_.subscribe(&BetaflightPlatform::onBattery, this, 1.0 / battery_hz_);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Battery frequency is set to 0, battery data will not be published");
  }
  if (attitude_hz_ > 0.0) {
    fcu_.subscribe(&BetaflightPlatform::onAttitude, this, 1.0 / attitude_hz_);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Attitude frequency is set to 0, attitude data will not be published");
  }
  if (altitude_hz_ > 0.0) {
    fcu_.subscribe(&BetaflightPlatform::onAltitude, this, 1.0 / altitude_hz_);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Altitude frequency is set to 0, altitude data will not be published");
  }
  if (motor_hz_ > 0.0) {
    fcu_.subscribe(&BetaflightPlatform::onMotor, this, 1.0 / motor_hz_);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Motor frequency is set to 0, motor throttle data will not be published");
  }
  if (rc_hz_ > 0.0) {
    fcu_.subscribe(&BetaflightPlatform::onRc, this, 1.0 / rc_hz_);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "RC frequency is set to 0, RC data will not be published");
  }
  box_names_ = fcu_.getBoxNames();

  if (pi_protocol_enable_) {
    pi_protocol_client_.setImuCallback([this](const pi_IMU_t & imu) {onPiProtocolImu(imu);});
    if (!pi_protocol_client_.connect(pi_protocol_device_, pi_protocol_baudrate_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not connect to pi-protocol device %s - continuing without high-rate IMU",
        pi_protocol_device_.c_str());
      // Non-fatal on purpose: a bad second UART must not prevent flight control from starting.
    } else {
      RCLCPP_INFO(
        this->get_logger(), "pi-protocol connected on %s @ %d baud",
        pi_protocol_device_.c_str(), pi_protocol_baudrate_);
    }
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

void BetaflightPlatform::configureSensors()
{
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  battery_sensor_ptr_ = std::make_unique<as2::sensors::Battery>("battery", this);
  // gps_sensor_ptr_ = std::make_unique<as2::sensors::GPS>("gps", this);

  // odometry_raw_estimation_ptr_ =
  //   std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
}

bool BetaflightPlatform::ownSetArmingState(bool state)
{
  // TODO(miferco97): check if this is correct
  // NOTE: arming here goes through the RC ARM channel (MSP_SET_RAW_RC), not
  // MSP_SET_ARMING_DISABLED. On indiflight, MSP_SET_ARMING_DISABLED no longer
  // persistently blocks future arming (it only disarms if currently armed) -
  // see indiflight/src/main/msp/msp.c, ARMING_DISABLED_MSP is commented out.
  // Don't assume that command is a standing safety interlock on this firmware.
  int value = state ? 1 : 0;
  channel_values_[RC_CHANNELS::ARM] = 1000 + value * 1000;
  return true;
}

bool BetaflightPlatform::ownSetOffboardControl(bool offboard)
{
  return true;
}

bool BetaflightPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  // ONLY SUPPORTS ACRO MODE
  if (msg.control_mode != as2_msgs::msg::ControlMode::ACRO) {
    RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
    return false;
  }
  return true;
}

bool BetaflightPlatform::ownSendCommand()
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

  bool out = fcu_.setRc(channel_values_);
  if (!out) {
    RCLCPP_ERROR(this->get_logger(), "Could not send command to flight controller");
    return false;
  }
  return true;
}

void BetaflightPlatform::ownKillSwitch()
{
  // set all channels to 0
  std::fill(channel_values_.begin(), channel_values_.end(), 1000);

  channel_values_[RC_CHANNELS::KILLSWITCH] = 2000;
  while (true) {
    fcu_.setRc(channel_values_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void BetaflightPlatform::ownStopPlatform() {RCLCPP_WARN(this->get_logger(), "NOT IMPLEMENTED");}

void BetaflightPlatform::onStatus(const msp::msg::Status & status)
{
  // Betaflight never sets the GeneralHealth bit (bit 15) in the sensor flags,
  // so status.isHealthy() is always false. Check SENSOR_ACC (bit 0) instead:
  // Betaflight reports this bit whenever the IMU accelerometer is initialised.
  if (!status.hasAccelerometer()) {
    RCLCPP_WARN(this->get_logger(), "Flight controller is not healthy");
  }
}

void BetaflightPlatform::onImu(const msp::msg::RawImu & imu)
{
  std_msgs::msg::Header hdr;
  hdr.stamp = this->get_clock()->now();
  hdr.frame_id = base_link_frame_id_;

  // Publish raw IMU message
  sensor_msgs::msg::Imu imu_raw;
  imu_raw.header = hdr;
  imu_raw.linear_acceleration.x = imu.acc[0];
  imu_raw.linear_acceleration.y = imu.acc[1];
  imu_raw.linear_acceleration.z = imu.acc[2];
  imu_raw.angular_velocity.x = imu.gyro[0] / 180.0 * M_PI;
  imu_raw.angular_velocity.y = imu.gyro[1] / 180.0 * M_PI;
  imu_raw.angular_velocity.z = imu.gyro[2] / 180.0 * M_PI;

  raw_imu_pub_->publish(imu_raw);

  // from Betaflight MPU6000 drivers init: acc_1G = 512.0 * 4
  // const msp::msg::ImuSI imu_si(imu, 512.0 * 4, 1.0 / 4.096, 0.092, 9.80665);
  const double acc_1G = 512.0f;
  const double gyro_scale = 1.0f / 16.0f;
  const msp::msg::ImuSI imu_si(imu, acc_1G, gyro_scale, 0.092, 9.80665);

  // raw imu data without orientation
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = hdr;
  imu_msg.linear_acceleration.x = imu_si.acc[0];
  imu_msg.linear_acceleration.y = imu_si.acc[1];
  imu_msg.linear_acceleration.z = imu_si.acc[2];
  imu_msg.angular_velocity.x = imu_si.gyro[0] / 180.0 * M_PI;
  imu_msg.angular_velocity.y = imu_si.gyro[1] / 180.0 * M_PI;
  imu_msg.angular_velocity.z = imu_si.gyro[2] / 180.0 * M_PI;

  std::array<double, 9> gyro_covariance =
  {imu_gyro_covariance_ / 180.0 * M_PI, 0.0, 0.0,
    0.0, imu_gyro_covariance_ / 180.0 * M_PI, 0.0,
    0.0, 0.0, imu_gyro_covariance_ / 180.0 * M_PI};
  std::array<double, 9> accel_covariance =
  {imu_accel_covariance_, 0.0, 0.0,
    0.0, imu_accel_covariance_, 0.0,
    0.0, 0.0, imu_accel_covariance_};
  std::array<double, 9> ori_covariance =
  {imu_orientation_covariance_, 0.0, 0.0,
    0.0, imu_orientation_covariance_, 0.0,
    0.0, 0.0, imu_orientation_covariance_};
  imu_msg.angular_velocity_covariance = gyro_covariance;
  imu_msg.linear_acceleration_covariance = accel_covariance;
  imu_msg.orientation_covariance = ori_covariance;

  // magnetic field vector
  // sensor_msgs::msg::MagneticField mag_msg;
  // mag_msg.header = hdr;
  // mag_msg.magnetic_field.x = imu_si.mag[0] * 1e-6;
  // mag_msg.magnetic_field.y = imu_si.mag[1] * 1e-6;
  // mag_msg.magnetic_field.z = imu_si.mag[2] * 1e-6;

  imu_sensor_ptr_->updateAndPublish(imu_msg);
}

void BetaflightPlatform::onPiProtocolImu(const pi_IMU_t & imu)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = this->get_clock()->now();
  imu_msg.header.frame_id = base_link_frame_id_;

  // pi-protocol's IMU message already carries SI units (gyro rad/s, accel m/s^2)
  // despite the roll/pitch/yaw field names - see indiflight/src/main/telemetry/pi.c.
  // No scaling needed, unlike MSP RAW_IMU above.
  imu_msg.angular_velocity.x = imu.roll;
  imu_msg.angular_velocity.y = imu.pitch;
  imu_msg.angular_velocity.z = imu.yaw;
  imu_msg.linear_acceleration.x = imu.x;
  imu_msg.linear_acceleration.y = imu.y;
  imu_msg.linear_acceleration.z = imu.z;

  // imu_gyro_covariance_/imu_accel_covariance_ are shared with the MSP raw_imu path
  // (same physical IMU), but unlike that path this one must NOT apply the deg->rad
  // conversion to the gyro covariance: pi-protocol's gyro is already in rad/s.
  imu_msg.angular_velocity_covariance[0] = imu_gyro_covariance_;
  imu_msg.angular_velocity_covariance[4] = imu_gyro_covariance_;
  imu_msg.angular_velocity_covariance[8] = imu_gyro_covariance_;
  imu_msg.linear_acceleration_covariance[0] = imu_accel_covariance_;
  imu_msg.linear_acceleration_covariance[4] = imu_accel_covariance_;
  imu_msg.linear_acceleration_covariance[8] = imu_accel_covariance_;
  // Orientation not provided on this channel.
  imu_msg.orientation_covariance[0] = -1.0;

  imu_high_rate_pub_->publish(imu_msg);
}

void BetaflightPlatform::onAltitude(const msp::msg::Altitude & altitude)
{
  std::cout << "Altitude: " << altitude << std::endl;
}

void BetaflightPlatform::onAttitude(const msp::msg::Attitude & attitude)
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

void BetaflightPlatform::onMotor(const msp::msg::Motor & motor)
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

void BetaflightPlatform::onBattery(const msp::msg::BatteryState & battery)
{
  float voltage_filtered = alpha_voltage_ * voltage_ + (1 - alpha_voltage_) * battery.voltage;
  float max_batt_voltage = max_cell_voltage_ * battery.cell_count;
  float min_batt_voltage = min_cell_voltage_ * battery.cell_count;

  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = this->get_clock()->now();
  battery_msg.voltage = voltage_filtered;
  battery_msg.current = battery.amperage;
  battery_msg.percentage = (voltage_filtered - min_batt_voltage) /
    (max_batt_voltage - min_batt_voltage);
  battery_msg.charge = battery.capacity_mAh;

  battery_sensor_ptr_->updateData(battery_msg);

  voltage_ = voltage_filtered;
}

void BetaflightPlatform::onRc(const msp::msg::Rc & rc)
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

void BetaflightPlatform::publishDebugRc()
{
  // Assign the values from `channel_values_` to the `debug_rc_` message
  debug_rc_command_.data = channel_values_;

  // Publish the message
  debug_rc_command_.stamp = this->now();
  debug_rc_command_pub_->publish(debug_rc_command_);
}

void BetaflightPlatform::rcArm(int channel)
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

void BetaflightPlatform::rcOffboard(int channel)
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
