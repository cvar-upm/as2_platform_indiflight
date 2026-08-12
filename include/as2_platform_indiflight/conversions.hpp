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
* @file conversions.hpp
*
* AS2 to indiflight frame conversions definition
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#ifndef AS2_PLATFORM_INDIFLIGHT__CONVERSIONS_HPP_
#define AS2_PLATFORM_INDIFLIGHT__CONVERSIONS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace as2_platform_indiflight
{

// EXTERNAL_POSE, the state fed to the FC EKF, and POS_SETPOINT, the position
// commanded to it, must use the same ENU->NED transformation. The FC's NED
// datum is defined by whatever this host feeds through EXTERNAL_POSE, so a
// mismatch between the two flies the vehicle stably to a rotated target.

/**
 * @brief ENU world vector to NED world vector, for positions and velocities.
 *
 * @param enu Vector in ENU.
 * @return The same vector in NED: x (North) = enu.y, y (East) = enu.x,
 *         z (Down) = -enu.z.
 */
Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu);

/**
 * @brief Attitude quaternion ENU<-FLU to NED<-FRD.
 *
 * @param q_enu_flu Quaternion such that v_enu = q * v_flu, the ROS convention
 *        carried by PoseStamped.orientation.
 * @return Quaternion q' such that v_ned = q' * v_frd.
 */
Eigen::Quaterniond enuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu);

/**
 * @brief ENU yaw to NED yaw, in the units POS_SETPOINT carries on the wire.
 *
 * @param yaw_enu_rad Yaw in radians, 0 = East, positive counter-clockwise seen
 *        from above.
 * @return Yaw in degrees, 0 = North, positive clockwise, wrapped to [-180, 180).
 */
double yawEnuRadToNedDeg(double yaw_enu_rad);

}  // namespace as2_platform_indiflight

#endif  // AS2_PLATFORM_INDIFLIGHT__CONVERSIONS_HPP_
