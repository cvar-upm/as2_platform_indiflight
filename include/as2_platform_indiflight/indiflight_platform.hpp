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
 * @file indiflight_platform.hpp
 *
 * IndiflightPlatform class definition
 *
 * @author Miguel Fernández Cortizas
 */

#ifndef AS2_PLATFORM_INDIFLIGHT__INDIFLIGHT_PLATFORM_HPP_
#define AS2_PLATFORM_INDIFLIGHT__INDIFLIGHT_PLATFORM_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <memory>
#include <cmath>
#include <map>
#include <vector>

#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "as2_msgs/msg/u_int16_multi_array_stamped.hpp"
#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/polynomial_thrust_map.hpp"
#include "as2_core/utils/frame_utils.hpp"

// msp_msg.hpp only, not FlightController.hpp: this node no longer connects
// over MSP (see fcu_ removal), but the dormant MSP telemetry placeholders
// below (onAltitude/onAttitude/onMotor/onRc) still use msp::msg:: types.
#include <msp/msp_msg.hpp>

#include "as2_platform_indiflight/pi_protocol_client.hpp"
#include "as2_platform_indiflight/pi_protocol_clock_sync.hpp"

#define PULSE_RANGE 1000

namespace as2_platform_indiflight
{

/**
 * @brief Enumeration of RC channel indices
 *
 * Defines the mapping between logical control inputs and the 4 channels
 * carried by pi-protocol's RC_OVERRIDE message. ARM/OFFBOARD/killswitch are
 * no longer sent by this node at all - they live permanently on the physical
 * radio underneath indiflight's PI OVERRIDE box mode, which only ever
 * overrides these 4 stick channels.
 */
enum RC_CHANNELS
{
  ROLL = 0,
  PITCH = 1,
  THROTTLE = 2,
  YAW = 3
};

class IndiflightPlatform : public as2::AerialPlatform
{
public:
  explicit IndiflightPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~IndiflightPlatform()
  {
    pi_protocol_client_.disconnect();
  }

public:
  void configureSensors() override;
  void publishSensorData();
  void readParameters();

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  void sendCommand() override
  {
    if (true) {
      ownSendCommand();
    }
  }
  bool ownSendCommand() override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

private:
  // Dormant MSP-based telemetry callbacks. MSP is no longer connected by this
  // node (pi-protocol carries everything at runtime now - see onPiStatus/
  // onPiBattery below), but these stay declared, unwired, as placeholders for
  // pi-protocol equivalents that don't exist yet. onStatus/onBoxNames/onImu
  // and the fcu_ connection itself were dropped outright: box names were
  // never read anywhere, and pi-protocol's EKF_INPUTS already covers IMU.

  /**
   * @brief Callback for altitude data
   */
  void onAltitude(const msp::msg::Altitude & altitude);

  /**
   * @brief Callback for attitude data
   */
  void onAttitude(const msp::msg::Attitude & attitude);

  /**
   * @brief Callback for motor individual throttle commands
   */
  void onMotor(const msp::msg::Motor & motor);

  /**
   * @brief Callback for RC reads from the controller
   */
  void onRc(const msp::msg::Rc & rc);

  // pi-protocol related functions and variables (indiflight's telemetry/command
  // channel - now the only link this node depends on at runtime. A connect
  // failure here is fatal, unlike the old MSP-optional framing this replaced).
  bool pi_protocol_enable_ = true;
  std::string pi_protocol_device_ = "/dev/ttyUSB1";
  int pi_protocol_baudrate_ = 921600;
  PiProtocolClient pi_protocol_client_;
  // Shared by both pi-protocol callbacks below: both messages ride the same FC
  // clock domain, so pooling samples from both converges faster than two
  // independent trackers would. Only ever touched from PiProtocolClient's
  // single dedicated reader thread - see PiProtocolClockSync's own comment.
  PiProtocolClockSync pi_protocol_clock_sync_;

  // TIMESYNC round-trip uplink: sent once on connect and then every second.
  // timesync_seq_ is log-correlation only, not RTT-load-bearing - the reply
  // echoes host_ns verbatim, so RTT/offset are computed from that alone (see
  // onPiTimesync()), with no pending-request state to track.
  rclcpp::TimerBase::SharedPtr timesync_timer_;
  uint32_t timesync_seq_ = 0;
  void sendTimesyncRequest();

  /**
   * @brief Callback for pi-protocol EKF_INPUTS messages: a single synchronized
   * bundle (accel + gyro + all 4 motor speeds, one sample each, one time_us),
   * republished here as sensor_measurements/imu and
   * sensor_measurements/motor_angular_speed - the synchronization only needs
   * to happen on the wire, not in how it's exposed to ROS consumers. Up to 2kHz.
   */
  void onPiProtocolEkfInputs(const pi_EKF_INPUTS_t & msg);

  /**
   * @brief Callback for pi-protocol PI_STATUS messages: armed / PI OVERRIDE
   * active / rx link valid, reported by the FC at TELEMETRY_PI_MAXRATE.
   * Feeds AS2's own arming/offboard state the same way rcArm()/rcOffboard()
   * used to from an MSP RC-channel readback - PI_OVERRIDE_ACTIVE plays the
   * role OFFBOARD used to: it's the FC actually obeying this node's stick
   * commands, not just a channel value this node itself last sent.
   */
  void onPiStatus(const pi_PI_STATUS_t & msg);

  /**
   * @brief Callback for pi-protocol BATTERY messages: pack voltage/current/
   * cell count, replacing the MSP battery message as the source for
   * voltage_ (used by the voltage-aware thrust map).
   */
  void onPiBattery(const pi_BATTERY_t & msg);

  /**
   * @brief Callback for pi-protocol TIMESYNC replies: feeds the round-trip
   * offset estimate (see PiProtocolClockSync::syncRoundTrip()) and logs
   * lock/RTT.
   */
  void onPiTimesync(const pi_TIMESYNC_t & msg);

  void computeControlSlopes()
  {
    roll_slope_ = (max_roll_rate_ - min_roll_rate_) / static_cast<double>(PULSE_RANGE);
    pitch_slope_ = (max_pitch_rate_ - min_pitch_rate_) / static_cast<double>(PULSE_RANGE);
    yaw_slope_ = (max_yaw_rate_ - min_yaw_rate_) / static_cast<double>(PULSE_RANGE);
  }

  /**
   * @brief Rotates IMU gyro/accel samples from indiflight's FRD firmware frame
   * into the user-configured body frame (desired_frame_T.r/p/y), in place.
   * Default desired_frame_T.r = pi implements FRD -> FLU (keep X, negate Y and Z).
   */
  void rotateImuToDesiredFrame(
    Eigen::Vector3d & angular_velocity, Eigen::Vector3d & linear_acceleration) const;

private:
  bool manual_from_operator_ = false;
  bool set_arm_ = false;
  bool set_offboard_ = false;
  geometry_msgs::msg::PoseStamped betaflight_vision_pose_msg_;
  geometry_msgs::msg::TwistStamped betaflight_vision_speed_msg_;

  std::atomic<uint64_t> timestamp_;
  std::vector<uint16_t> channel_values_;
  as2::PolynomialThrustMap thrust_map_;

  void initChannels()
  {
    // channels are : 0 roll, 1 pitch, 2 throttle, 3 yaw - the 4 channels
    // pi-protocol's RC_OVERRIDE carries. ARM/OFFBOARD/killswitch live on the
    // physical radio now, outside this vector entirely.
    channel_values_.clear();
    channel_values_.resize(4, 1000);
    channel_values_[RC_CHANNELS::ROLL] = 1500;
    channel_values_[RC_CHANNELS::PITCH] = 1500;
    channel_values_[RC_CHANNELS::THROTTLE] = 1000;
    channel_values_[RC_CHANNELS::YAW] = 1500;
  }

  double imu_gyro_covariance_ = 0.0;
  double imu_accel_covariance_ = 0.0;
  double imu_orientation_covariance_ = 0.0;

  // Rotation from indiflight's FRD firmware frame to the platform's configured
  // body frame, computed once in readParameters() from desired_frame_T.r/p/y.
  double desired_frame_roll_ = 0.0;
  double desired_frame_pitch_ = 0.0;
  double desired_frame_yaw_ = 0.0;
  Eigen::Matrix3d desired_frame_rotation_ = Eigen::Matrix3d::Identity();

  double battery_hz_ = 0.0;
  double altitude_hz_ = 0.0;
  double attitude_hz_ = 0.0;
  double rc_hz_ = 0.0;
  double motor_hz_ = 0.0;

  double max_thrust_;
  double min_thrust_;
  double min_roll_rate_;
  double max_roll_rate_;
  double min_pitch_rate_;
  double max_pitch_rate_;
  double min_yaw_rate_;
  double max_yaw_rate_;

  double roll_slope_ = 0.0;
  double pitch_slope_ = 0.0;
  double yaw_slope_ = 0.0;

  double voltage_ = 0.0;
  double alpha_voltage_ = 0.0;
  double min_cell_voltage_ = 3.7;
  double max_cell_voltage_ = 4.2;

  bool use_thrust_map_ = false;
  bool limit_output_ = false;
  double limit_roll_percent_ = 100.0;
  double limit_pitch_percent_ = 100.0;
  double limit_yaw_percent_ = 100.0;
  double limit_thrust_percent_ = 100.0;


  bool simulation_mode_ = false;
  bool external_odom_ = true;
  // See readParameters()'s declare_parameter<bool>("use_fcu_stamps", ...)
  // comment for the full tradeoff. Consumed in onPiProtocolEkfInputs().
  bool use_fcu_stamps_ = true;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  // Publishes to sensor_measurements/motor_angular_speed (as2::sensors::Sensor
  // auto-prefixes "sensor_measurements/" the same way imu_sensor_ptr_/
  // battery_sensor_ptr_ do).
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::JointState>> motor_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
  std::unique_ptr<as2::sensors::GPS> gps_sensor_ptr_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr external_odometry_sub_;

// Debug:

private:
  // Debug rc publisher
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_rc_command_pub_;
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_rc_read_pub_;
  // Raw FC time_us behind the imu_sensor_ptr_/motor_sensor_ptr_ publications
  // above - both share the exact same time_us (one EKF_INPUTS bundle feeds
  // both), so a single publisher here covers both.
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr og_timestamp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_motors_pub_;
  // [armed, pi_override_active, rx_link_valid] as 0/1, decoded from PI_STATUS.flags.
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_pi_status_pub_;
  as2_msgs::msg::UInt16MultiArrayStamped debug_rc_command_;

  /**
   * @brief Publish debug RC data
   */
  void publishDebugRc();

  /**
   * @brief Set RC arm channel. Dead code now that MSP is disconnected (no
   * onRc() caller left to feed it) - kept alongside onRc() as a placeholder,
   * see the comment above onAltitude(). Superseded by onPiStatus().
   *
   * @param arm Arm value
   */
  void rcArm(int arm);

  /**
   * @brief Set RC offboard channel. Dead code, same reasoning as rcArm().
   * Superseded by onPiStatus().
   *
   * @param offboard Offboard value
   */
  void rcOffboard(int offboard);
};

}  // namespace as2_platform_indiflight

#endif  // AS2_PLATFORM_INDIFLIGHT__INDIFLIGHT_PLATFORM_HPP_
