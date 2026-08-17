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
* @file conversions.cpp
*
* AS2 to indiflight frame conversions implementation
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#include "as2_platform_indiflight/conversions.hpp"

#include <cmath>

namespace as2_platform_indiflight
{

Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu)
{
  return {enu.y(), enu.x(), -enu.z()};
}

Eigen::Vector3d fluToFrd(const Eigen::Vector3d & flu)
{
  return {flu.x(), -flu.y(), -flu.z()};
}

Eigen::Quaterniond enuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu)
{
  // Half-turn about (1,1,0)/sqrt(2) and about x_body. Eigen takes (w, x, y, z).
  static const Eigen::Quaterniond kEnuToNed(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  static const Eigen::Quaterniond kFluToFrd(0.0, 1.0, 0.0, 0.0);
  return (kEnuToNed * q_enu_flu * kFluToFrd).normalized();
}

double yawEnuRadToNedRad(double yaw_enu_rad)
{
  double yaw_ned_rad = M_PI_2 - yaw_enu_rad;
  yaw_ned_rad = std::fmod(yaw_ned_rad + M_PI, 2.0 * M_PI);
  if (yaw_ned_rad < 0.0) {
    yaw_ned_rad += 2.0 * M_PI;
  }
  return yaw_ned_rad - M_PI;
}

}  // namespace as2_platform_indiflight
