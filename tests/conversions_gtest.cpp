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
* @file conversions_gtest.cpp
*
* AS2 to indiflight frame conversions tests
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#include <gtest/gtest.h>

#include <cmath>

#include "as2_platform_indiflight/conversions.hpp"

namespace as2_platform_indiflight
{

namespace
{
// Quaternions q and -q encode the same rotation.
void expectSameRotation(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  EXPECT_NEAR(std::abs(a.dot(b)), 1.0, 1e-9);
}
}  // namespace

TEST(EnuToNed, AxesMap)
{
  const Eigen::Vector3d ned = enuToNed({1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(ned.x(), 2.0);   // North  = enu_y
  EXPECT_DOUBLE_EQ(ned.y(), 1.0);   // East   = enu_x
  EXPECT_DOUBLE_EQ(ned.z(), -3.0);  // Down   = -enu_z
}

TEST(EnuToNed, IsInvolutive)
{
  // The mapping is its own inverse - applying it twice must round-trip, the
  // invariant that keeps EXTERNAL_POSE and POS_SETPOINT in the same world.
  const Eigen::Vector3d v(0.3, -1.7, 2.9);
  EXPECT_TRUE(enuToNed(enuToNed(v)).isApprox(v));
}

TEST(FluToFrd, AxesMap)
{
  const Eigen::Vector3d frd = fluToFrd({1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(frd.x(), 1.0);   // Forward stays forward, unlike enuToNed
  EXPECT_DOUBLE_EQ(frd.y(), -2.0);  // Right = -left
  EXPECT_DOUBLE_EQ(frd.z(), -3.0);  // Down  = -up
}

TEST(FluToFrd, UpwardsThrustIsNegativeZ)
{
  // The sign the FC reads as thrust: a positive FLU thrust must arrive as a
  // negative specific force, or the INDI drives the craft into the ground.
  const Eigen::Vector3d spf = fluToFrd({0.0, 0.0, 9.81});
  EXPECT_LT(spf.z(), 0.0);
  EXPECT_DOUBLE_EQ(spf.z(), -9.81);
}

TEST(EnuFluToNedFrd, LevelNoseNorthIsIdentity)
{
  // Nose to the North, level, in ENU-FLU: yaw +90 deg about ENU z.
  const Eigen::Quaterniond q_enu_flu(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  expectSameRotation(enuFluToNedFrd(q_enu_flu), Eigen::Quaterniond::Identity());
}

TEST(EnuFluToNedFrd, LevelNoseEastIsYaw90)
{
  // Nose to the East (ENU identity) -> NED yaw +90 deg.
  const Eigen::Quaterniond expected(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  expectSameRotation(enuFluToNedFrd(Eigen::Quaterniond::Identity()), expected);
}

TEST(EnuFluToNedFrd, MapsBodyAxesConsistently)
{
  // For any attitude: the FRD x axis expressed in NED must equal the ENU->NED
  // mapping of the FLU x axis expressed in ENU (both are the nose direction).
  const Eigen::Quaterniond q_enu_flu(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond q_ned_frd = enuFluToNedFrd(q_enu_flu);

  const Eigen::Vector3d nose_enu = q_enu_flu * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d nose_ned = q_ned_frd * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(nose_ned.isApprox(enuToNed(nose_enu), 1e-9));

  // FLU y (left) maps to FRD -y (right is +y in FRD).
  const Eigen::Vector3d left_enu = q_enu_flu * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d left_ned = q_ned_frd * (-Eigen::Vector3d::UnitY());
  EXPECT_TRUE(left_ned.isApprox(enuToNed(left_enu), 1e-9));
}

TEST(YawEnuRadToNedRad, CardinalDirections)
{
  EXPECT_NEAR(yawEnuRadToNedRad(0.0), M_PI_2, 1e-9);         // East
  EXPECT_NEAR(yawEnuRadToNedRad(M_PI / 2.0), 0.0, 1e-9);     // North
  EXPECT_NEAR(yawEnuRadToNedRad(M_PI), -M_PI_2, 1e-9);       // West
  EXPECT_NEAR(yawEnuRadToNedRad(-M_PI / 2.0), -M_PI, 1e-9);  // South -> -pi
}

TEST(YawEnuRadToNedRad, WrapsIntoMinusPiToPi)
{
  for (double yaw = -3.0 * M_PI; yaw <= 3.0 * M_PI; yaw += 0.1) {
    const double rad = yawEnuRadToNedRad(yaw);
    EXPECT_GE(rad, -M_PI);
    EXPECT_LT(rad, M_PI);
  }
}

TEST(YawEnuRadToNedRad, MatchesQuaternionConversion)
{
  // The scalar yaw conversion and the quaternion conversion must agree for a
  // level vehicle - they stamp the same wire message.
  for (double yaw_enu = -3.0; yaw_enu <= 3.0; yaw_enu += 0.25) {
    const Eigen::Quaterniond q_enu_flu(
      Eigen::AngleAxisd(yaw_enu, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q_ned_frd = enuFluToNedFrd(q_enu_flu);
    const double yaw_ned = std::atan2(
      2.0 * (q_ned_frd.w() * q_ned_frd.z() + q_ned_frd.x() * q_ned_frd.y()),
      1.0 - 2.0 * (q_ned_frd.y() * q_ned_frd.y() + q_ned_frd.z() * q_ned_frd.z()));
    const double expected_rad = yawEnuRadToNedRad(yaw_enu);
    const double diff = std::remainder(yaw_ned - expected_rad, 2.0 * M_PI);
    EXPECT_NEAR(diff, 0.0, 1e-6) << "yaw_enu = " << yaw_enu;
  }
}

}  // namespace as2_platform_indiflight
