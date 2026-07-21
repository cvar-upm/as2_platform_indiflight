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
 * @file betaflight_platform.hpp
 *
 * BetaflightPlatform class definition
 *
 * @author Miguel Fernández Cortizas
 */

#ifndef AS2_PLATFORM_INDIFLIGHT__BETAFLIGHT_PLATFORM_HPP_
#define AS2_PLATFORM_INDIFLIGHT__BETAFLIGHT_PLATFORM_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <memory>
#include <cmath>
#include <map>
#include <vector>

#include <rclcpp/subscription.hpp>

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

#include <msp/FlightController.hpp>
#include <msp/msp_msg.hpp>

#include "as2_platform_indiflight/pi_protocol_client.hpp"
#include "as2_platform_indiflight/pi_protocol_clock_sync.hpp"

#define PULSE_RANGE 1000

namespace as2_platform_indiflight
{

/**
 * @brief Enumeration of RC channel indices
 *
 * Defines the mapping between logical control inputs and RC channel indices.
 */
enum RC_CHANNELS
{
  ROLL = 0,
  PITCH = 1,
  THROTTLE = 2,
  YAW = 3,
  ARM = 4,
  OFFBOARD = 5,
  KILLSWITCH = 6,
  AUX4 = 7,
  AUX5 = 8
};

class BetaflightPlatform : public as2::AerialPlatform
{
public:
  explicit BetaflightPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BetaflightPlatform()
  {
    fcu_.disconnect();
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
  // MSP Related functions and variables
  std::string device_ = "/dev/ttyUSB0";
  int baudrate_ = 115200;

  fcu::FlightController fcu_;
  /**
   * @brief Callback for FCU status messages
   */
  void onStatus(const msp::msg::Status & status);

  /**
   * @brief Callback for FCU box names
   */
  void onBoxNames(const msp::msg::BoxNames & box_names);

  /**
   * @brief Callback for raw IMU data
   */
  void onImu(const msp::msg::RawImu & imu);

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
   * @brief Callback for battery state
   */
  void onBattery(const msp::msg::BatteryState & battery);

  /**
   * @brief Callback for RC reads from the controller
   */
  void onRc(const msp::msg::Rc & rc);

  // pi-protocol related functions and variables (indiflight's high-rate telemetry
  // channel, independent of MSP - a connect failure here must stay non-fatal so a
  // bad second UART can never prevent flight control from starting)
  bool pi_protocol_enable_ = false;
  std::string pi_protocol_device_ = "/dev/ttyUSB1";
  int pi_protocol_baudrate_ = 921600;
  PiProtocolClient pi_protocol_client_;
  // Shared by both pi-protocol callbacks below: both messages ride the same FC
  // clock domain, so pooling samples from both converges faster than two
  // independent trackers would. Only ever touched from PiProtocolClient's
  // single dedicated reader thread - see PiProtocolClockSync's own comment.
  PiProtocolClockSync pi_protocol_clock_sync_;

  /**
   * @brief Callback for pi-protocol IMU messages (up to 2kHz)
   */
  void onPiProtocolImu(const pi_IMU_t & imu);

  /**
   * @brief Callback for pi-protocol MOTOR messages (measured angular speeds, up to 1kHz)
   */
  void onPiProtocolMotor(const pi_MOTOR_t & motor);

  void computeControlSlopes()
  {
    roll_slope_ = (max_roll_rate_ - min_roll_rate_) / static_cast<double>(PULSE_RANGE);
    pitch_slope_ = (max_pitch_rate_ - min_pitch_rate_) / static_cast<double>(PULSE_RANGE);
    yaw_slope_ = (max_yaw_rate_ - min_yaw_rate_) / static_cast<double>(PULSE_RANGE);
  }

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
    // channels are :
    // - 0 roll,
    // - 1 pitch,
    // - 2 throttle,
    // - 3 yaw,
    // - 4 aux1 ( ARM ) ,
    // - 5 aux2 ( OFFBOARD ),
    // - 6 aux3 ( KILLSWITCH ),
    // - 7 aux4
    // - 8 aux5
    // roll, pitch and yaw are set to 1500, throttle to 1000, and the rest to 1000
    channel_values_.clear();
    channel_values_.resize(8, 1000);
    channel_values_[RC_CHANNELS::ROLL] = 1500;
    channel_values_[RC_CHANNELS::PITCH] = 1500;
    channel_values_[RC_CHANNELS::THROTTLE] = 1000;
    channel_values_[RC_CHANNELS::YAW] = 1500;
  }

  double imu_hz_ = 0.0;
  double imu_gyro_covariance_ = 0.0;
  double imu_accel_covariance_ = 0.0;
  double imu_orientation_covariance_ = 0.0;

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
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  std::map<std::string, std::size_t> box_names_;

  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
  std::unique_ptr<as2::sensors::GPS> gps_sensor_ptr_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr external_odometry_sub_;

// Debug:

private:
  // Debug rc publisher
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_rc_command_pub_;
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_rc_read_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_high_rate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr imu_high_rate_time_ref_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_speed_high_rate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr
    motor_speed_high_rate_time_ref_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_motors_pub_;
  as2_msgs::msg::UInt16MultiArrayStamped debug_rc_command_;

  /**
   * @brief Publish debug RC data
   */
  void publishDebugRc();

  /**
   * @brief Set RC arm channel
   *
   * @param arm Arm value
   */
  void rcArm(int arm);

  /**
   * @brief Set RC offboard channel
   *
   * @param offboard Offboard value
   */
  void rcOffboard(int offboard);
};

}  // namespace as2_platform_indiflight

#endif  // AS2_PLATFORM_INDIFLIGHT__BETAFLIGHT_PLATFORM_HPP_
