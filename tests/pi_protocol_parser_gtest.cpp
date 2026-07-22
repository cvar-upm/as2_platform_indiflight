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
 * @file pi_protocol_parser_gtest.cpp
 *
 * Exercises pi-protocol's checksum/escape-byte framing without any hardware,
 * using the generated encoder (piAccumulateMsg) to round-trip through the
 * generated/hand-written parser (piParse).
 */

#include <gtest/gtest.h>

extern "C" {
#include "pi-messages.h"
#include "pi-protocol.h"
}

TEST(PiProtocolParser, RoundTripImuMessage)
{
  piMsgImuTx.time_us = 123456;
  piMsgImuTx.roll = 1.5f;
  piMsgImuTx.pitch = -2.5f;
  piMsgImuTx.yaw = 0.25f;
  piMsgImuTx.x = 9.81f;
  piMsgImuTx.y = -0.1f;
  piMsgImuTx.z = 0.2f;

  uint8_t buf[2 * PI_MAX_PACKET_LEN];
  const unsigned int n = piAccumulateMsg(&piMsgImuTx, buf);
  ASSERT_GT(n, 0u);

  pi_parse_states_t state{};
  uint8_t last_id = PI_MSG_NONE_ID;
  for (unsigned int i = 0; i < n; i++) {
    const uint8_t id = piParse(&state, buf[i]);
    if (id != PI_MSG_NONE_ID) {
      last_id = id;
    }
  }

  ASSERT_EQ(last_id, PI_MSG_IMU_ID);
  ASSERT_NE(piMsgImuRx, nullptr);
  EXPECT_EQ(piMsgImuRx->time_us, 123456u);
  EXPECT_FLOAT_EQ(piMsgImuRx->roll, 1.5f);
  EXPECT_FLOAT_EQ(piMsgImuRx->pitch, -2.5f);
  EXPECT_FLOAT_EQ(piMsgImuRx->yaw, 0.25f);
  EXPECT_FLOAT_EQ(piMsgImuRx->x, 9.81f);
  EXPECT_FLOAT_EQ(piMsgImuRx->y, -0.1f);
  EXPECT_FLOAT_EQ(piMsgImuRx->z, 0.2f);
}

TEST(PiProtocolParser, RoundTripMotorMessage)
{
  piMsgMotorTx.time_us = 654321;
  piMsgMotorTx.omega0 = 1133.0f;
  piMsgMotorTx.omega1 = 1140.5f;
  piMsgMotorTx.omega2 = -1125.25f;
  piMsgMotorTx.omega3 = 1150.75f;

  uint8_t buf[2 * PI_MAX_PACKET_LEN];
  const unsigned int n = piAccumulateMsg(&piMsgMotorTx, buf);
  ASSERT_GT(n, 0u);

  pi_parse_states_t state{};
  uint8_t last_id = PI_MSG_NONE_ID;
  for (unsigned int i = 0; i < n; i++) {
    const uint8_t id = piParse(&state, buf[i]);
    if (id != PI_MSG_NONE_ID) {
      last_id = id;
    }
  }

  ASSERT_EQ(last_id, PI_MSG_MOTOR_ID);
  ASSERT_NE(piMsgMotorRx, nullptr);
  EXPECT_EQ(piMsgMotorRx->time_us, 654321u);
  EXPECT_FLOAT_EQ(piMsgMotorRx->omega0, 1133.0f);
  EXPECT_FLOAT_EQ(piMsgMotorRx->omega1, 1140.5f);
  EXPECT_FLOAT_EQ(piMsgMotorRx->omega2, -1125.25f);
  EXPECT_FLOAT_EQ(piMsgMotorRx->omega3, 1150.75f);
}

TEST(PiProtocolParser, RoundTripEkfInputsMessage)
{
  piMsgEkfInputsTx.time_us = 111222;
  piMsgEkfInputsTx.x = 2048;    // 1g on x, per EKF_INPUTS.yaml's int16 * (9.81/2048) scale
  piMsgEkfInputsTx.y = -1024;
  piMsgEkfInputsTx.z = 512;
  piMsgEkfInputsTx.p = 16384;   // per EKF_INPUTS.yaml's int16 * ((2000*pi/180)/32768) scale
  piMsgEkfInputsTx.q = -8192;
  piMsgEkfInputsTx.r = 4096;
  piMsgEkfInputsTx.omega1 = 1133;  // rad/s directly, no scaling
  piMsgEkfInputsTx.omega2 = 1140;
  piMsgEkfInputsTx.omega3 = 1125;
  piMsgEkfInputsTx.omega4 = 1150;

  uint8_t buf[2 * PI_MAX_PACKET_LEN];
  const unsigned int n = piAccumulateMsg(&piMsgEkfInputsTx, buf);
  ASSERT_GT(n, 0u);

  pi_parse_states_t state{};
  uint8_t last_id = PI_MSG_NONE_ID;
  for (unsigned int i = 0; i < n; i++) {
    const uint8_t id = piParse(&state, buf[i]);
    if (id != PI_MSG_NONE_ID) {
      last_id = id;
    }
  }

  ASSERT_EQ(last_id, PI_MSG_EKF_INPUTS_ID);
  ASSERT_NE(piMsgEkfInputsRx, nullptr);
  EXPECT_EQ(piMsgEkfInputsRx->time_us, 111222u);
  EXPECT_EQ(piMsgEkfInputsRx->x, 2048);
  EXPECT_EQ(piMsgEkfInputsRx->y, -1024);
  EXPECT_EQ(piMsgEkfInputsRx->z, 512);
  EXPECT_EQ(piMsgEkfInputsRx->p, 16384);
  EXPECT_EQ(piMsgEkfInputsRx->q, -8192);
  EXPECT_EQ(piMsgEkfInputsRx->r, 4096);
  EXPECT_EQ(piMsgEkfInputsRx->omega1, 1133u);
  EXPECT_EQ(piMsgEkfInputsRx->omega2, 1140u);
  EXPECT_EQ(piMsgEkfInputsRx->omega3, 1125u);
  EXPECT_EQ(piMsgEkfInputsRx->omega4, 1150u);
}

TEST(PiProtocolParser, CorruptedChecksumIsRejected)
{
  piMsgImuTx.time_us = 42;
  piMsgImuTx.roll = 1.0f;
  piMsgImuTx.pitch = 1.0f;
  piMsgImuTx.yaw = 1.0f;
  piMsgImuTx.x = 1.0f;
  piMsgImuTx.y = 1.0f;
  piMsgImuTx.z = 1.0f;

  uint8_t buf[2 * PI_MAX_PACKET_LEN];
  const unsigned int n = piAccumulateMsg(&piMsgImuTx, buf);
  ASSERT_GT(n, 0u);

  buf[n - 1] ^= 0xFF;  // corrupt the trailing checksum byte

  pi_parse_states_t state{};
  uint8_t last_id = PI_MSG_NONE_ID;
  for (unsigned int i = 0; i < n; i++) {
    const uint8_t id = piParse(&state, buf[i]);
    if (id != PI_MSG_NONE_ID) {
      last_id = id;
    }
  }

  EXPECT_EQ(last_id, PI_MSG_NONE_ID);
}
