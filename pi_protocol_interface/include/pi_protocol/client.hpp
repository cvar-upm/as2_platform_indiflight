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
* @file client.hpp
*
* pi-protocol Client class definition
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#ifndef PI_PROTOCOL__CLIENT_HPP_
#define PI_PROTOCOL__CLIENT_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <utility>

extern "C" {
#include "pi-messages.h"  // NOLINT(build/include_subdir) - generated, flat include dir
#include "pi-protocol.h"  // NOLINT(build/include_subdir) - generated, flat include dir
}

namespace pi_protocol
{

/**
 * @brief Accepts a downlink message only if its FC time_us stamp is plausible:
 * within [-1 s, +5 s] of the last accepted one, or the second of two
 * self-consistent out-of-window stamps (an FC reboot restarting micros()).
 * Also caches the newest accepted stamp, which is what uplink messages are
 * stamped with.
 *
 * The wire has no length byte and no version handshake, so an FC built against
 * a different message table is read against the wrong entry and only the 8-bit
 * CRC stands in the way, letting ~1/256 of those frames through as garbage.
 *
 * Not thread-safe: only ever touched from the reader thread.
 */
class FcStampGate
{
public:
  /**
   * @brief Feed a received stamp, updating the cached tick on acceptance.
   *
   * @param time_us Stamp of the message, in the FC micros() domain.
   * @return true if the stamp is plausible and the message should be delivered.
   */
  bool accept(uint32_t time_us);

  /**
   * @brief Newest accepted stamp.
   *
   * @return Stamp in the FC micros() domain, which is what uplink messages carry.
   */
  uint32_t last() const {return last_us_;}

  /**
   * @brief Whether any stamp has been accepted yet.
   *
   * @return true once last() holds an FC stamp.
   */
  bool valid() const {return valid_;}

private:
  static constexpr int32_t kMinDeltaUs = -1000000;  // 1 s backwards slack
  static constexpr int32_t kMaxDeltaUs = 5000000;   // 5 s forward gap

  bool valid_ = false;
  uint32_t last_us_ = 0;
  bool pending_valid_ = false;
  uint32_t pending_us_ = 0;
};

/**
 * @brief Host-side client for indiflight's pi-protocol telemetry channel.
 *
 */
/// Link counters, for telling "nothing arrives" from "arrives and does not
/// parse" without a logic analyser. Monotonic since connect().
struct LinkStats
{
  uint64_t bytes_read;      ///< Bytes off the serial port.
  uint64_t frames_parsed;   ///< Frames that passed framing and CRC.
  uint64_t frames_gated;    ///< Parsed but dropped by the FC stamp gate.
};

class Client
{
public:
  using EkfInputsCallback = std::function<void (const pi_EKF_INPUTS_t &)>;
  using AuxCallback = std::function<void (const pi_AUX_t &)>;
  using StatusCallback = std::function<void (const pi_PI_STATUS_t &)>;
  using BatteryCallback = std::function<void (const pi_BATTERY_t &)>;

  Client() = default;
  ~Client();

  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;

  /**
   * @brief Open the serial port and start the reader thread.
   *
   * @param device Serial device the FC is wired to.
   * @param baudrate Line rate, matching the FC's telemetry port.
   * @return true on success, false if the port could not be opened/configured.
   */
  bool connect(const std::string & device, int baudrate);

  /**
   * @brief Stop the reader thread and close the serial port. Safe to call
   * multiple times and safe to call even if connect() was never called.
   */
  void disconnect();

  /**
   * @brief Set the callback invoked for every successfully parsed EKF_INPUTS
   * message - accel + gyro + up to 6 rotor speeds from one sample, under one
   * time_us, for consumers that need every input from the same instant. Must
   * be set before connect() to reliably receive the first messages.
   *
   * @param callback Invoked with each parsed EKF_INPUTS message.
   */
  void setEkfInputsCallback(EkfInputsCallback callback)
  {
    ekf_inputs_callback_ = std::move(callback);
  }

  /**
   * @brief Set the callback invoked for every successfully parsed AUX message
   * (raw rcData of the 14 aux channels, 1000-2000 us): the arm/mode switch
   * readback on builds that do not send PI_STATUS. Must be set before
   * connect() to reliably receive the first messages.
   *
   * @param callback Invoked with each parsed AUX message.
   */
  void setAuxCallback(AuxCallback callback) {aux_callback_ = std::move(callback);}

  /**
   * @brief Set the callback invoked for every successfully parsed PI_STATUS
   * message (armed / PI OVERRIDE active / rx link valid). Not emitted by
   * builds that report switch state via AUX. Must be set before connect() to
   * reliably receive the first messages.
   *
   * @param callback Invoked with each parsed PI_STATUS message.
   */
  void setStatusCallback(StatusCallback callback) {status_callback_ = std::move(callback);}

  /**
   * @brief Set the callback invoked for every successfully parsed BATTERY
   * message. Must be set before connect() to reliably receive the first
   * messages.
   *
   * @param callback Invoked with each parsed BATTERY message.
   */
  void setBatteryCallback(BatteryCallback callback) {battery_callback_ = std::move(callback);}

  /**
   * @brief Send an RC_OVERRIDE message - the roll/pitch/yaw/throttle stick
   * override for indiflight's PI OVERRIDE box mode. Values follow
   * Betaflight's usual 1000-2000 pulse convention. Only takes effect on the
   * FC while PI OVERRIDE is active and the corresponding channel is enabled
   * in pi_override_channels_mask.
   *
   * Stamped with the last FC tick seen on the downlink, as POS_SETPOINT is: a
   * stick command has no sampling instant of its own.
   *
   * @param roll Roll stick pulse [us].
   * @param pitch Pitch stick pulse [us].
   * @param yaw Yaw stick pulse [us].
   * @param throttle Throttle stick pulse [us].
   * @return true on a successful write. false until the first downlink message
   *         has been parsed, since there is no FC tick to stamp with yet, or if
   *         the write fails.
   */
  bool sendRcOverride(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle);

  /**
   * @brief Send a POS_SETPOINT message: position setpoint for the FC's
   * onboard position controller (firmware built with USE_LOCAL_POSITION).
   *
   * Frame/units contract (see io/local_pos.c and telemetry/pi.c firmware-side):
   * position [m] and velocity feed-forward [m/s] in the FC's local NED frame -
   * whose datum is defined by whatever this host feeds via sendExternalPose() -
   * and yaw in DEGREES, NED convention (0 = North, positive clockwise seen
   * from above).
   *
   * Stamped with the last FC tick seen on the downlink: the firmware compares
   * time_us against its own micros() domain and silently discards stale or
   * future setpoints (POS_SETPOINT_OUTDATED_US).
   *
   * @param ned_x North position [m].
   * @param ned_y East position [m].
   * @param ned_z Down position [m].
   * @param ned_xd North velocity feed-forward [m/s].
   * @param ned_yd East velocity feed-forward [m/s].
   * @param ned_zd Down velocity feed-forward [m/s].
   * @param yaw_deg Yaw [deg], NED convention.
   * @return true on a successful write. false until the first downlink message
   *         has been parsed, since there is no FC tick to stamp with yet.
   */
  /**
   * @brief Link counters since connect().
   *
   * @return Bytes read, frames parsed and frames dropped by the stamp gate.
   */
  LinkStats stats() const;

  bool sendPosSetpoint(
    float ned_x, float ned_y, float ned_z,
    float ned_xd, float ned_yd, float ned_zd, float yaw_deg);

  /**
   * @brief Send an EXTERNAL_POSE message: position/attitude measurement for
   * the FC's onboard EKF (firmware built with USE_EKF).
   *
   * Frame/units contract: position [m] and velocity [m/s] in local NED
   * (velocity is carried on the wire but the FC EKF measurement vector fuses
   * position + quaternion only); quaternion is the NED-to-FRD body attitude
   * (w, x, y, z).
   *
   * The firmware's setLocalPosMeas() requires strictly increasing time_us, so
   * this skips the write, returning true, when the stamp has not advanced
   * since the previous send.
   *
   * @param fc_time_us Instant the pose was sampled, in the FC micros() domain.
   * @param ned_x North position [m].
   * @param ned_y East position [m].
   * @param ned_z Down position [m].
   * @param ned_xd North velocity [m/s], carried but not fused by the FC.
   * @param ned_yd East velocity [m/s], carried but not fused by the FC.
   * @param ned_zd Down velocity [m/s], carried but not fused by the FC.
   * @param q_w Scalar part of the NED-to-FRD attitude quaternion.
   * @param q_x Vector part x.
   * @param q_y Vector part y.
   * @param q_z Vector part z.
   * @return true on a successful write, or when the send is skipped because the
   *         stamp has not advanced. false if the write fails.
   */
  bool sendExternalPose(
    uint32_t fc_time_us,
    float ned_x, float ned_y, float ned_z,
    float ned_xd, float ned_yd, float ned_zd,
    float q_w, float q_x, float q_y, float q_z);

private:
  void readLoop();
  bool sendMsg(void * msg_raw);

  int fd_ = -1;
  std::thread read_thread_;
  std::atomic<bool> running_{false};
  EkfInputsCallback ekf_inputs_callback_;
  AuxCallback aux_callback_;
  StatusCallback status_callback_;
  BatteryCallback battery_callback_;
  pi_parse_states_t parse_state_{};

  // Reader-thread-only gate; its accepted tick is mirrored into the atomics
  // below for the TX methods, which run on the ROS executor thread.
  FcStampGate stamp_gate_;
  std::atomic<uint64_t> bytes_read_{0};
  std::atomic<uint64_t> frames_parsed_{0};
  std::atomic<uint64_t> frames_gated_{0};
  std::atomic<uint32_t> last_fc_time_us_{0};
  std::atomic<bool> fc_time_valid_{false};

  // sendExternalPose() bookkeeping for the FC's strictly-increasing time_us
  // requirement. Only touched from the (single) ROS executor thread.
  bool ext_pose_sent_ = false;
  uint32_t last_ext_pose_tick_ = 0;
};

}  // namespace pi_protocol

#endif  // PI_PROTOCOL__CLIENT_HPP_
