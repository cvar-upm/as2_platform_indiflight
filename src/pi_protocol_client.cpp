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

#include "as2_platform_indiflight/pi_protocol_client.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace as2_platform_indiflight
{

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

PiProtocolClient::~PiProtocolClient()
{
  disconnect();
}

bool PiProtocolClient::connect(const std::string & device, int baudrate)
{
  const speed_t speed = toTermiosBaud(baudrate);
  if (speed == B0) {
    std::cerr << "PiProtocolClient: unsupported baudrate " << baudrate << std::endl;
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    std::cerr << "PiProtocolClient: could not open " << device << ": " <<
      std::strerror(errno) << std::endl;
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "PiProtocolClient: tcgetattr failed: " << std::strerror(errno) << std::endl;
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
    std::cerr << "PiProtocolClient: tcsetattr failed: " << std::strerror(errno) << std::endl;
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
  read_thread_ = std::thread(&PiProtocolClient::readLoop, this);
  return true;
}

void PiProtocolClient::disconnect()
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

void PiProtocolClient::readLoop()
{
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
      const uint8_t msg_id = piParse(&parse_state_, buf[i]);
      if (msg_id == PI_MSG_IMU_ID && piMsgImuRx != nullptr && imu_callback_) {
        imu_callback_(*piMsgImuRx);
      } else if (msg_id == PI_MSG_MOTOR_ID && piMsgMotorRx != nullptr && motor_callback_) {
        motor_callback_(*piMsgMotorRx);
      } else if (msg_id == PI_MSG_EKF_INPUTS_ID && piMsgEkfInputsRx != nullptr &&
        ekf_inputs_callback_)
      {
        ekf_inputs_callback_(*piMsgEkfInputsRx);
      }
    }
  }
}

}  // namespace as2_platform_indiflight
