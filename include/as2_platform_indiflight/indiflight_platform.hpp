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
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#ifndef AS2_PLATFORM_INDIFLIGHT__INDIFLIGHT_PLATFORM_HPP_
#define AS2_PLATFORM_INDIFLIGHT__INDIFLIGHT_PLATFORM_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <memory>
#include <optional>
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
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>

#include "as2_msgs/msg/u_int16_multi_array_stamped.hpp"
#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/polynomial_thrust_map.hpp"
#include "as2_core/utils/frame_utils.hpp"

#include "as2_platform_indiflight/conversions.hpp"
#include "pi_protocol/client.hpp"
#include "pi_protocol/clock_sync.hpp"

#define PULSE_RANGE 1000

// SETPOINT mode field, see indiflight_pkg/pi-protocol/msgs/SETPOINT.yaml
#define SETPOINT_POSITION   0
#define SETPOINT_VELOCITY   1
#define SETPOINT_TRAJECTORY 2
#define SETPOINT_ATTITUDE   3
#define SETPOINT_ACRO       4
#define SETPOINT_YAW_RATE   (1 << 3)

namespace as2_platform_indiflight
{

// Wire positions of the sticks in RC_OVERRIDE, for the default AETR receiver
// mapping. A receiver wired differently needs these changed to match. The
// message carries these four and nothing else: what the pilot's sticks would
// do, in the mode the pilot has selected.
enum RC_CHANNELS
{
  ROLL = 0,
  PITCH = 1,
  THROTTLE = 2,
  YAW = 3
};


// EXTERNAL_POSE mode field, see indiflight_pkg/pi-protocol/msgs/EXTERNAL_POSE.yaml
#define EXT_POSE_USE_POS   (1 << 0)
#define EXT_POSE_USE_QUAT  (1 << 1)
#define EXT_POSE_USE_VEL   (1 << 2)
#define EXT_POSE_TRUST     (1 << 3)

class IndiflightPlatform : public as2::AerialPlatform
{
public:
  explicit IndiflightPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~IndiflightPlatform();

public:
  void configureSensors() override;
  void readParameters();

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  bool ownSendCommand() override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

private:
  /**
   * @brief Read a parameter into param_value, declaring it first if the node
   * has not declared it yet, and log the value that ended up in use.
   *
   * @param param_name Name of the parameter.
   * @param param_value Destination, and the default when use_default is true.
   * @param use_default Declare with param_value as default, making the
   *        parameter optional. When false the parameter is required and the
   *        node throws if it was not passed.
   */
  template<typename T>
  void getParam(const std::string & param_name, T & param_value, bool use_default = false)
  {
    if (!this->has_parameter(param_name)) {
      if (use_default) {
        this->declare_parameter<T>(param_name, param_value);
      } else {
        try {
          this->declare_parameter<T>(param_name);
        } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
          RCLCPP_FATAL(
            this->get_logger(), "Required parameter <%s> was not passed to the node",
            param_name.c_str());
          throw;
        }
      }
    }
    this->get_parameter(param_name, param_value);
    RCLCPP_INFO_STREAM(this->get_logger(), param_name << ": " << param_value);
  }

  /**
   * @brief Compute the rate-to-pulse slopes from the configured rate limits.
   */
  void computeControlSlopes();

  /**
   * @brief Reset the RC_OVERRIDE channels to their idle values: sticks
   * centred, throttle at minimum.
   */
  void initChannels();

  /**
   * @brief Publish an EKF_INPUTS message as sensor_measurements/imu and
   * sensor_measurements/motor_angular_speed.
   *
   * Decodes the fixed-point accel/gyro (EKF_INPUTS.yaml scales) and the motor
   * speeds, rotates the IMU into the body frame and stamps both topics with
   * the same FC instant - the message carries one synchronized sample of
   * everything.
   *
   * @param msg Received EKF_INPUTS message.
   */
  void onPiProtocolEkfInputs(const pi_EKF_INPUTS_t & msg);

  /**
   * @brief Ask the FC to echo a TIMESYNC exchange back, and time it.
   */
  void requestTimesync();

  /**
   * @brief Feed a TIMESYNC reply to the clock estimate.
   *
   * What this buys over the stream the estimate already sees is a measurement
   * of the offset that does not carry the link's floor latency inside it, on a
   * link whose latency nobody has characterised.
   *
   * @param msg Received TIMESYNC reply.
   */
  void onPiTimesync(const pi_TIMESYNC_t & msg);

  /**
   * @brief Publish the 16 receiver channels, in wire order, on debug/rc.
   *
   * @param msg Received RC message.
   */
  void onPiRc(const pi_RC_t & msg);

  /**
   * @brief Update the platform arm/offboard state from a PI_STATUS message and
   * publish its flags on debug/pi_status. The flags are the FC's own state:
   * armed is ARMING_FLAG(ARMED), offboard is the FC actually obeying this
   * node's commands.
   *
   * @param msg Received PI_STATUS message.
   */
  void onPiStatus(const pi_PI_STATUS_t & msg);

  /**
   * @brief Publish a BATTERY message as sensor_measurements/battery and update
   * the filtered pack voltage used by the thrust map.
   *
   * @param msg Received BATTERY message.
   */
  void onPiBattery(const pi_BATTERY_t & msg);

  /**
   * @brief Rotates IMU gyro/accel samples from indiflight's FRD firmware frame
   * into the user-configured body frame (desired_frame_T.r/p/y), in place.
   * Default desired_frame_T.r = pi implements FRD -> FLU (keep X, negate Y and Z).
   *
   * @param angular_velocity Gyro sample [rad/s], rotated in place.
   * @param linear_acceleration Accelerometer sample [m/s^2], rotated in place.
   */
  void rotateImuToDesiredFrame(
    Eigen::Vector3d & angular_velocity, Eigen::Vector3d & linear_acceleration) const;

  /**
   * @brief Stamp for a message whose FC time_us marks when the firmware built
   * it, rather than when it sampled anything.
   *
   * @param fc_time_us Stamp of the message, in the FC micros() domain.
   * @return The instant in host time, or the arrival time when use_fcu_stamps
   *         is false or no offset has been estimated yet.
   */
  rclcpp::Time fcStamp(uint32_t fc_time_us);

  /**
   * @brief Publish sensor_measurements/imu and the raw FC time on
   * debug/platform/og_timestamp.
   *
   * @param stamp Header stamp for both topics.
   * @param fc_time_us Raw FC time_us of the sample.
   * @param angular_velocity Gyro [rad/s], already rotated to the body frame.
   * @param linear_acceleration Accel [m/s^2], already rotated to the body frame.
   */
  void publishImuSample(
    const rclcpp::Time & stamp, uint32_t fc_time_us,
    const Eigen::Vector3d & angular_velocity, const Eigen::Vector3d & linear_acceleration);

  /**
   * @brief Apply an arm/offboard state read from the FC to platform_info_msg_,
   * firing the state machine ARM/DISARM events on change.
   *
   * @param armed Whether the FC reports itself armed.
   * @param offboard Whether the FC reports itself obeying this node.
   */
  void updatePlatformState(bool armed, bool offboard);

  /**
   * @brief Send the BODY_RATES references (command_twist_msg_ body rates and
   * command_thrust_msg_) as RC_OVERRIDE stick pulses, saturated and mapped
   * through the rate limits and the thrust map.
   *
   * @return true if the RC_OVERRIDE message was written to the FC.
   */
  bool sendAcroCommand();

  /**
   * @brief The commanded thrust as a specific force in the FC's FRD body frame.
   *
   * @return Specific force in m/s^2, with upwards thrust along negative z.
   */
  Eigen::Vector3d specificForceFrd() const;

  /**
   * @brief Send the BODY_RATES references as an ACRO SETPOINT in SI units, which reach the
   * INDI without the pilot's rate curve or throttle scaling.
   *
   * @return true if the SETPOINT message was written to the FC.
   */
  bool sendAcroSetpoint();

  /**
   * @brief Send command_pose_msg_ orientation and command_thrust_msg_ as an
   * attitude SETPOINT. Needs no state estimate on the FC.
   *
   * @return true if the SETPOINT message was written to the FC.
   */
  bool sendAttitudeCommand();

  /**
   * @brief Whether the FC can run its onboard position controller, which both
   * POSITION and HOVER are executed with.
   *
   * @param mode Mode name, for the rejection message.
   * @return true if the platform feeds the FC EKF.
   */
  bool acceptFcPositionControl(const char * mode) const;

  /**
   * @brief Whether a HOVER request can be honoured, and latch the reference
   * when it can.
   *
   * @return true if the platform is configured to feed the FC EKF and the mode
   *         being left leaves the FC holding a position setpoint.
   */
  bool acceptHover();

  /**
   * @brief Report the state of the FC link while no message has been decoded,
   * so a silent FC is distinguishable from a stream that does not parse.
   */
  void checkLink();

  /**
   * @brief Latch the current pose as the hover reference, so that a switch to
   * HOVER holds where the vehicle is rather than wherever it was last told to
   * go, and works coming from ACRO too, where the FC has no setpoint at all.
   *
   * @return true if the pose could be read from TF.
   */
  bool latchHoverPose();

  /**
   * @brief Send the latched hover reference as a NED POS_SETPOINT.
   *
   * @return true if the setpoint was written, or if there is nothing latched:
   *         the FC then keeps whatever setpoint it already holds.
   */
  bool sendHoverCommand();

  /**
   * @brief Send an ENU pose, already expressed in earth_frame_id_, as a NED
   * POS_SETPOINT for the FC's onboard position controller.
   *
   * @param pose Pose to command.
   * @return true if the setpoint was written to the FC.
   */
  bool sendPoseSetpoint(
    const geometry_msgs::msg::PoseStamped & pose,
    const Eigen::Vector3d & vel_ned = Eigen::Vector3d::Zero(), uint8_t mode = 0);

  /**
   * @brief Set the yaw-rate bit and its value when the active mode commands yaw
   * as an angular velocity rather than an angle.
   *
   * @param mode Mode byte, updated in place.
   * @return Yaw rate in the FC's NED degrees per second.
   */
  float yawRateForMode(uint8_t & mode);

  /**
   * @brief Send command_twist_msg_ as a NED velocity setpoint.
   *
   * @return true if the setpoint reached the FC.
   */
  bool sendSpeedCommand();

  /**
   * @brief Send the first trajectory setpoint as a position with the commanded
   * velocity as feedforward.
   *
   * @return true if the setpoint reached the FC.
   */
  bool sendTrajectoryCommand();

  /**
   * @brief Send command_pose_msg_ as a NED POS_SETPOINT for the FC's onboard
   * position controller.
   *
   * @return true if the setpoint was written to the FC. false if no pose
   *         reference has arrived, if it cannot be expressed in
   *         earth_frame_id_, or if the write failed.
   */
  bool sendPositionCommand();

  /**
   * @brief Convert a pose to earth_frame_id_ in place, if it is not already
   * there.
   *
   * @param pose Pose to convert, replaced by its converted value on success.
   * @return true if the pose is expressed in earth_frame_id_ on return.
   */
  bool toEarthFrame(geometry_msgs::msg::PoseStamped & pose) const;

  /**
   * @brief Timer callback for the TF pose source: look up the latest
   * earth->base_link transform and forward it with sendExternalPose().
   */
  void sendExternalPoseFromTf();

  /**
   * @brief Subscription callback for the topic pose source: convert the pose
   * to earth_frame_id_ and forward it with sendExternalPose().
   *
   * @param msg Pose received on external_pose.pose_topic.
   */
  void onExternalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Subscription callback for the mocap4r2 topic source: pick the
   * external_pose.rigid_body_name body out of the message, then behave like
   * onExternalPose(). Bodies are matched by name because a mocap publishes
   * every tracked object on the same topic.
   *
   * @param msg Rigid bodies received on external_pose.mocap_topic.
   */
  void onExternalRigidBodies(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);

  /**
   * @brief Send an ENU pose as a NED EXTERNAL_POSE for the FC's onboard EKF,
   * skipping poses already sent (by header stamp).
   *
   * Fed in every control mode, not just POSITION: the FC EKF needs a couple of
   * seconds of continuous measurements to converge, so keeping it fed while
   * flying ACRO is what makes a later switch to POSITION immediate.
   *
   * @param pose Pose already expressed in earth_frame_id_.
   */
  void sendExternalPose(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Publish the last commanded RC channels on debug/rc/command.
   */
  void publishDebugRc();

  // Frames
  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  // ENU frame every pose is expressed in before the ENU->NED conversions.
  std::string earth_frame_id_;
  // Pose held while in HOVER, latched on entering the mode. Empty when it
  // could not be read, which leaves the FC on its previous setpoint.
  std::optional<geometry_msgs::msg::PoseStamped> hover_pose_;

  // Pi-protocol link, the only route to the FC
  std::string pi_protocol_device_ = "/dev/ttyUSB1";
  int pi_protocol_baudrate_ = 921600;
  pi_protocol::Client pi_protocol_client_;
  // Fed by every downlink message: they all ride the same FC clock domain, so
  // pooling their samples converges faster than one tracker per stream would.
  // Only ever touched from pi_protocol::Client's single reader thread.
  pi_protocol::ClockSync pi_protocol_clock_sync_;

  // Inertial telemetry
  bool use_fcu_stamps_ = true;
  int num_rotors_ = 4;
  double imu_gyro_covariance_ = -1.0;
  double imu_accel_covariance_ = -1.0;
  // Rotation from indiflight's FRD firmware frame to the platform's configured
  // body frame, computed once in readParameters() from desired_frame_T.r/p/y.
  double desired_frame_roll_ = 0.0;
  double desired_frame_pitch_ = 0.0;
  double desired_frame_yaw_ = 0.0;
  Eigen::Matrix3d desired_frame_rotation_ = Eigen::Matrix3d::Identity();

  // Airframe mass, kg. Turns the commanded thrust into the specific force that
  // SETPOINT carries, since neither the message nor the FC knows the mass
  double mass_;

  // BODY_RATES command mapping, from rate and thrust references to RC_OVERRIDE pulses
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
  bool use_thrust_map_ = false;
  as2::PolynomialThrustMap thrust_map_;
  bool limit_output_ = false;
  double limit_roll_percent_ = 100.0;
  double limit_pitch_percent_ = 100.0;
  double limit_yaw_percent_ = 100.0;
  double limit_thrust_percent_ = 100.0;
  std::vector<uint16_t> channel_values_;

  // Battery, and the filtered voltage the thrust map corrects with
  double alpha_voltage_ = 0.0;
  double min_cell_voltage_ = 3.7;
  double max_cell_voltage_ = 4.2;
  double voltage_ = 0.0;

  // External pose uplink to the FC's onboard EKF
  bool external_pose_enable_ = false;
  double external_pose_rate_ = 100.0;
  // Whichever of these is non-empty selects the pose source; both empty polls
  // TF. rigid_body_name picks the vehicle out of the mocap message.
  std::string external_pose_pose_topic_;
  std::string external_pose_mocap_topic_;
  std::string external_pose_rigid_body_name_;
  // Stamp of the last pose forwarded to the FC, to skip unchanged ones.
  // Nanoseconds rather than rclcpp::Time, whose comparison throws when the two
  // operands carry different clock types. The optional distinguishes "nothing
  // sent yet" from a source that legitimately stamps 0.
  std::optional<int64_t> last_external_pose_stamp_ns_;
  // Client-side skip count already reported, so the warning only fires while it grows.
  uint64_t ext_pose_skipped_seen_ = 0;
  // FC state from PI_STATUS, which the control modes are gated on.
  bool fc_ekf_converged_ = false;
  bool fc_pos_ctl_active_ = false;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  // Exactly one of these is created, per the source parameters above.
  rclcpp::TimerBase::SharedPtr external_pose_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_pose_sub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr external_rigid_bodies_sub_;

  // Platform state, mirrored from the FC
  bool external_odom_ = true;
  bool set_arm_ = false;
  bool set_offboard_ = false;

  // Sensors
  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  // Publishes to sensor_measurements/motor_angular_speed (as2::sensors::Sensor
  // auto-prefixes "sensor_measurements/" the same way imu_sensor_ptr_/
  // battery_sensor_ptr_ do).
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::JointState>> motor_sensor_ptr_;

  // Watchdog over the FC link. Bytes arriving without frames decoding means a
  // message table or baudrate mismatch; no bytes at all means the FC is not
  // sending, and neither is visible from the topics alone.
  rclcpp::TimerBase::SharedPtr link_check_timer_;

  // Round-trip clock exchange with the FC.
  rclcpp::TimerBase::SharedPtr timesync_timer_;
  bool timesync_locked_ = false;

  // Debug publishers
  std::string debug_rc_command_topic_;
  std::string debug_og_timestamp_topic_;
  std::string debug_pi_status_topic_;
  std::string debug_rc_topic_;
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_rc_command_pub_;
  as2_msgs::msg::UInt16MultiArrayStamped debug_rc_command_;
  // Raw FC time_us behind the imu_sensor_ptr_/motor_sensor_ptr_ publications:
  // both come from the same EKF_INPUTS bundle, so one publisher covers both.
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr og_timestamp_pub_;
  // [armed, pi_override_active, rx_link_valid] as 0/1, decoded from PI_STATUS.flags.
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_pi_status_pub_;
  // The 16 receiver channels, to check stick and switch positions against the radio.
  rclcpp::Publisher<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr debug_rc_pub_;
};

}  // namespace as2_platform_indiflight

#endif  // AS2_PLATFORM_INDIFLIGHT__INDIFLIGHT_PLATFORM_HPP_
