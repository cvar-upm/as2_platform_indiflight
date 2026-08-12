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
* @file client.cpp
*
* pi-protocol Client class implementation
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#include "pi_protocol/client.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace pi_protocol
{

bool FcStampGate::accept(uint32_t time_us)
{
  if (!valid_) {
    valid_ = true;
    last_us_ = time_us;
    return true;
  }
  const int32_t delta = static_cast<int32_t>(time_us - last_us_);
  if (delta >= kMinDeltaUs && delta <= kMaxDeltaUs) {
    if (delta > 0) {
      last_us_ = time_us;  // never regress: TX stamping wants the newest tick
    }
    pending_valid_ = false;
    return true;
  }
  if (pending_valid_) {
    const int32_t pending_delta = static_cast<int32_t>(time_us - pending_us_);
    if (pending_delta >= 0 && pending_delta <= kMaxDeltaUs) {
      // Second stamp consistent with the first outlier: a rebooted FC, not
      // garbage. Adopt the new clock.
      last_us_ = time_us;
      pending_valid_ = false;
      return true;
    }
  }
  pending_us_ = time_us;
  pending_valid_ = true;
  return false;
}

namespace
{
speed_t toTermiosBaud(int baudrate)
{
  switch (baudrate) {
    case 921600: return B921600;
    case 460800: return B460800;
    case 230400: return B230400;
    case 115200: return B115200;
    case 57600: return B57600;
    case 38400: return B38400;
    case 19200: return B19200;
    case 9600: return B9600;
    default: return B0;
  }
}
}  // namespace

Client::~Client()
{
  disconnect();
}

bool Client::connect(const std::string & device, int baudrate)
{
  const speed_t speed = toTermiosBaud(baudrate);
  if (speed == B0) {
    std::cerr << "Client: unsupported baudrate " << baudrate << std::endl;
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    std::cerr << "Client: could not open " << device << ": " <<
      std::strerror(errno) << std::endl;
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "Client: tcgetattr failed: " << std::strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  // VMIN=0, VTIME=1: read() returns after up to 100ms even with no data, so the
  // reader thread can periodically observe running_ and exit cleanly on
  // disconnect() without racing a close() from another thread.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "Client: tcsetattr failed: " << std::strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  tcflush(fd_, TCIOFLUSH);

  // Low-latency tuning: avoids the ~16ms default buffering delay some UART
  // drivers apply, same tuning msp/src/Client.cpp applies to the MSP port.
  struct serial_struct serial_info;
  if (ioctl(fd_, TIOCGSERIAL, &serial_info) == 0) {
    serial_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &serial_info);
  }

  parse_state_ = pi_parse_states_t{};
  running_ = true;
  read_thread_ = std::thread(&Client::readLoop, this);
  return true;
}

void Client::disconnect()
{
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void Client::readLoop()
{
  // Runs the stamp gate and, on acceptance, mirrors the newest FC tick into
  // the atomics consumed by sendPosSetpoint()/sendExternalPose(). Every
  // dispatched message starts with a uint32 time_us in the FC micros() domain.
  const auto gate = [this](uint32_t time_us) {
      if (!stamp_gate_.accept(time_us)) {
        return false;
      }
      last_fc_time_us_.store(stamp_gate_.last(), std::memory_order_relaxed);
      fc_time_valid_.store(true, std::memory_order_release);
      return true;
    };

  // One dispatch shape per message: null-check the double-buffered Rx
  // pointer, run the stamp gate, then invoke the callback if one is set.
  const auto deliver = [&gate](auto * rx_msg, const auto & callback) {
      if (rx_msg == nullptr || !gate(rx_msg->time_us)) {
        return;
      }
      if (callback) {
        callback(*rx_msg);
      }
    };

  uint8_t buf[256];
  while (running_) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;  // real error, e.g. device unplugged
    }
    for (ssize_t i = 0; i < n; i++) {
      switch (piParse(&parse_state_, buf[i])) {
        case PI_MSG_EKF_INPUTS_ID:
          deliver(piMsgEkfInputsRx, ekf_inputs_callback_);
          break;
        case PI_MSG_AUX_ID:
          deliver(piMsgAuxRx, aux_callback_);
          break;
        case PI_MSG_PI_STATUS_ID:
          deliver(piMsgPiStatusRx, status_callback_);
          break;
        case PI_MSG_BATTERY_ID:
          deliver(piMsgBatteryRx, battery_callback_);
          break;
        default:
          break;
      }
    }
  }
}

bool Client::sendMsg(void * msg_raw)
{
  if (fd_ < 0) {
    return false;
  }

  // Same wasting-some-stack sizing piSendMsg() uses firmware-side.
  uint8_t buf[2 * PI_MAX_PACKET_LEN];
  const unsigned int num_bytes = piAccumulateMsg(msg_raw, buf);

  const ssize_t written = ::write(fd_, buf, num_bytes);
  return written == static_cast<ssize_t>(num_bytes);
}

bool Client::sendRcOverride(
  uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle)
{
  if (!fc_time_valid_.load(std::memory_order_acquire)) {
    return false;
  }
  piMsgRcOverrideTx.time_us = last_fc_time_us_.load(std::memory_order_relaxed);
  piMsgRcOverrideTx.roll = roll;
  piMsgRcOverrideTx.pitch = pitch;
  piMsgRcOverrideTx.yaw = yaw;
  piMsgRcOverrideTx.throttle = throttle;

  return sendMsg(&piMsgRcOverrideTx);
}

bool Client::sendPosSetpoint(
  float ned_x, float ned_y, float ned_z,
  float ned_xd, float ned_yd, float ned_zd, float yaw_deg)
{
  if (!fc_time_valid_.load(std::memory_order_acquire)) {
    // No downlink message parsed yet - a host-clock stamp would be silently
    // discarded by the FC's time_us comparisons, so refuse instead.
    return false;
  }
  piMsgPosSetpointTx.time_us = last_fc_time_us_.load(std::memory_order_relaxed);
  piMsgPosSetpointTx.ned_x = ned_x;
  piMsgPosSetpointTx.ned_y = ned_y;
  piMsgPosSetpointTx.ned_z = ned_z;
  piMsgPosSetpointTx.ned_xd = ned_xd;
  piMsgPosSetpointTx.ned_yd = ned_yd;
  piMsgPosSetpointTx.ned_zd = ned_zd;
  piMsgPosSetpointTx.yaw = yaw_deg;

  return sendMsg(&piMsgPosSetpointTx);
}

bool Client::sendExternalPose(
  uint32_t fc_time_us,
  float ned_x, float ned_y, float ned_z,
  float ned_xd, float ned_yd, float ned_zd,
  float q_w, float q_x, float q_y, float q_z)
{
  if (ext_pose_sent_ && static_cast<int32_t>(fc_time_us - last_ext_pose_tick_) <= 0) {
    // setLocalPosMeas() requires strictly increasing time_us and would drop
    // this silently. Skipping is not an error: the offset estimate moves, so a
    // later pose can map just behind the previous one.
    return true;
  }

  piMsgExternalPoseTx.time_us = fc_time_us;
  piMsgExternalPoseTx.ned_x = ned_x;
  piMsgExternalPoseTx.ned_y = ned_y;
  piMsgExternalPoseTx.ned_z = ned_z;
  piMsgExternalPoseTx.ned_xd = ned_xd;
  piMsgExternalPoseTx.ned_yd = ned_yd;
  piMsgExternalPoseTx.ned_zd = ned_zd;
  piMsgExternalPoseTx.body_qi = q_w;
  piMsgExternalPoseTx.body_qx = q_x;
  piMsgExternalPoseTx.body_qy = q_y;
  piMsgExternalPoseTx.body_qz = q_z;

  if (!sendMsg(&piMsgExternalPoseTx)) {
    return false;
  }
  last_ext_pose_tick_ = fc_time_us;
  ext_pose_sent_ = true;
  return true;
}

}  // namespace pi_protocol
