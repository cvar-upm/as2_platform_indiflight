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
 * @file pi_protocol_node.cpp
 *
 * Publishes sensor_msgs/Imu from indiflight's high-rate pi-protocol telemetry
 * channel, independent of the MSP-based platform node's connection lifecycle.
 */

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "as2_core/utils/tf_utils.hpp"
#include "as2_platform_indiflight/pi_protocol_client.hpp"

namespace as2_platform_indiflight
{

class PiProtocolImuNode : public rclcpp::Node
{
public:
  PiProtocolImuNode()
  : rclcpp::Node("pi_protocol")
  {
    this->declare_parameter<bool>("pi_protocol.enable", true);
    this->declare_parameter<std::string>("pi_protocol.device", "/dev/ttyUSB1");
    this->declare_parameter<int>("pi_protocol.baudrate", 921600);
    // Shared with the MSP platform node's imu.covariance.* params: same physical
    // IMU, same noise characteristics regardless of which channel delivered it.
    // (No orientation.covariance here: this channel never provides orientation.)
    this->declare_parameter<float>("imu.covariance.gyro", 0.0);
    this->declare_parameter<float>("imu.covariance.accel", 0.0);

    const bool enable = this->get_parameter("pi_protocol.enable").as_bool();
    const std::string device = this->get_parameter("pi_protocol.device").as_string();
    const int baudrate = this->get_parameter("pi_protocol.baudrate").as_int();
    gyro_covariance_ = this->get_parameter("imu.covariance.gyro").as_double();
    accel_covariance_ = this->get_parameter("imu.covariance.accel").as_double();

    base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "imu_high_rate", rclcpp::SensorDataQoS());

    if (!enable) {
      RCLCPP_WARN(this->get_logger(), "pi-protocol disabled (pi_protocol.enable=false)");
      return;
    }

    client_.setImuCallback([this](const pi_IMU_t & imu) {onImu(imu);});

    if (!client_.connect(device, baudrate)) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not connect to pi-protocol device %s", device.c_str());
      throw std::runtime_error("Could not connect to pi-protocol device");
    }
    RCLCPP_INFO(
      this->get_logger(), "pi-protocol connected on %s @ %d baud", device.c_str(), baudrate);
  }

  ~PiProtocolImuNode() override
  {
    client_.disconnect();
  }

private:
  void onImu(const pi_IMU_t & imu)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = base_link_frame_id_;

    // pi-protocol's IMU message already carries SI units (gyro rad/s, accel
    // m/s^2) despite the roll/pitch/yaw field names - see
    // indiflight/src/main/telemetry/pi.c. No scaling needed, unlike MSP RAW_IMU.
    msg.angular_velocity.x = imu.roll;
    msg.angular_velocity.y = imu.pitch;
    msg.angular_velocity.z = imu.yaw;
    msg.linear_acceleration.x = imu.x;
    msg.linear_acceleration.y = imu.y;
    msg.linear_acceleration.z = imu.z;

    msg.angular_velocity_covariance[0] = gyro_covariance_;
    msg.angular_velocity_covariance[4] = gyro_covariance_;
    msg.angular_velocity_covariance[8] = gyro_covariance_;
    msg.linear_acceleration_covariance[0] = accel_covariance_;
    msg.linear_acceleration_covariance[4] = accel_covariance_;
    msg.linear_acceleration_covariance[8] = accel_covariance_;
    // Orientation not provided on this channel.
    msg.orientation_covariance[0] = -1.0;

    imu_pub_->publish(msg);
  }

  PiProtocolClient client_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::string base_link_frame_id_;
  double gyro_covariance_ = 0.0;
  double accel_covariance_ = 0.0;
};

}  // namespace as2_platform_indiflight

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<as2_platform_indiflight::PiProtocolImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
