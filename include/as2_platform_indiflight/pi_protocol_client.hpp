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
 * @file pi_protocol_client.hpp
 *
 * PiProtocolClient class definition
 */

#ifndef AS2_PLATFORM_INDIFLIGHT__PI_PROTOCOL_CLIENT_HPP_
#define AS2_PLATFORM_INDIFLIGHT__PI_PROTOCOL_CLIENT_HPP_

#include <atomic>
#include <functional>
#include <string>
#include <thread>

extern "C" {
#include "pi-messages.h"
#include "pi-protocol.h"
}

namespace as2_platform_indiflight
{

/**
 * @brief Host-side client for indiflight's pi-protocol telemetry channel.
 *
 * Unlike MSP, pi-protocol is firmware-pushed (not host-polled): the FC streams
 * IMU messages continuously at its own scheduled rate (up to 2kHz). This class
 * owns a dedicated reader thread that parses the byte stream and invokes the
 * IMU callback inline, on the same thread, for every completed frame.
 */
class PiProtocolClient
{
public:
  using ImuCallback = std::function<void (const pi_IMU_t &)>;
  using MotorCallback = std::function<void (const pi_MOTOR_t &)>;

  PiProtocolClient() = default;
  ~PiProtocolClient();

  PiProtocolClient(const PiProtocolClient &) = delete;
  PiProtocolClient & operator=(const PiProtocolClient &) = delete;

  /**
   * @brief Open the serial port and start the reader thread.
   * @return true on success, false if the port could not be opened/configured.
   */
  bool connect(const std::string & device, int baudrate);

  /**
   * @brief Stop the reader thread and close the serial port. Safe to call
   * multiple times and safe to call even if connect() was never called.
   */
  void disconnect();

  /**
   * @brief Set the callback invoked for every successfully parsed IMU message.
   * Must be set before connect() to reliably receive the first messages.
   */
  void setImuCallback(ImuCallback callback) {imu_callback_ = std::move(callback);}

  /**
   * @brief Set the callback invoked for every successfully parsed MOTOR message.
   * Must be set before connect() to reliably receive the first messages.
   */
  void setMotorCallback(MotorCallback callback) {motor_callback_ = std::move(callback);}

private:
  void readLoop();

  int fd_ = -1;
  std::thread read_thread_;
  std::atomic<bool> running_{false};
  ImuCallback imu_callback_;
  MotorCallback motor_callback_;
  pi_parse_states_t parse_state_{};
};

}  // namespace as2_platform_indiflight

#endif  // AS2_PLATFORM_INDIFLIGHT__PI_PROTOCOL_CLIENT_HPP_
