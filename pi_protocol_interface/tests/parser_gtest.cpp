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
* @file parser_gtest.cpp
*
* pi-protocol framing and stamp gate tests
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#include <gtest/gtest.h>

extern "C" {
#include "pi-messages.h"  // NOLINT(build/include_subdir) - generated, flat include dir
#include "pi-protocol.h"  // NOLINT(build/include_subdir) - generated, flat include dir
}

#include "pi_protocol/client.hpp"

// The wire has no length byte and no version handshake: the compiled table IS
// the protocol (see the pinned checkout's config.yaml). A change here means an FC built
// against the previous table stops parsing, silently.
static_assert(PI_MSG_IMU_ID == 1, "IMU id drifted");
static_assert(PI_MSG_EXTERNAL_POSE_ID == 3, "EXTERNAL_POSE id drifted");
static_assert(PI_MSG_EXTERNAL_POSE_PAYLOAD_LEN == 44, "EXTERNAL_POSE layout drifted");
static_assert(PI_MSG_POS_SETPOINT_ID == 4, "POS_SETPOINT id drifted");
static_assert(PI_MSG_POS_SETPOINT_PAYLOAD_LEN == 32, "POS_SETPOINT layout drifted");
static_assert(PI_MSG_EKF_INPUTS_ID == 9, "EKF_INPUTS id drifted");
static_assert(PI_MSG_EKF_INPUTS_PAYLOAD_LEN == 28, "EKF_INPUTS layout drifted (6 rotors)");
static_assert(PI_MSG_AUX_ID == 12, "AUX id drifted");
static_assert(PI_MSG_AUX_PAYLOAD_LEN == 32, "AUX layout drifted");
static_assert(PI_MSG_RC_OVERRIDE_ID == 13, "RC_OVERRIDE id drifted");
static_assert(PI_MSG_PI_STATUS_ID == 14, "PI_STATUS id drifted");
static_assert(PI_MSG_BATTERY_ID == 15, "BATTERY id drifted");

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

TEST(PiProtocolParser, RoundTripAuxMessage)
{
  piMsgAuxTx.time_us = 654321;
  piMsgAuxTx.aux_1 = 1000;
  piMsgAuxTx.aux_2 = 1500;
  piMsgAuxTx.aux_3 = 2000;
  piMsgAuxTx.aux_4 = 1100;
  piMsgAuxTx.aux_5 = 900;
  piMsgAuxTx.aux_6 = 1800;
  piMsgAuxTx.aux_7 = 1000;
  piMsgAuxTx.aux_8 = 1000;
  piMsgAuxTx.aux_9 = 1000;
  piMsgAuxTx.aux_10 = 1000;
  piMsgAuxTx.aux_11 = 1000;
  piMsgAuxTx.aux_12 = 1000;
  piMsgAuxTx.aux_13 = 1000;
  piMsgAuxTx.aux_14 = 1000;

  uint8_t buf[2 * PI_MAX_PACKET_LEN];
  const unsigned int n = piAccumulateMsg(&piMsgAuxTx, buf);
  ASSERT_GT(n, 0u);

  pi_parse_states_t state{};
  uint8_t last_id = PI_MSG_NONE_ID;
  for (unsigned int i = 0; i < n; i++) {
    const uint8_t id = piParse(&state, buf[i]);
    if (id != PI_MSG_NONE_ID) {
      last_id = id;
    }
  }

  ASSERT_EQ(last_id, PI_MSG_AUX_ID);
  ASSERT_NE(piMsgAuxRx, nullptr);
  EXPECT_EQ(piMsgAuxRx->time_us, 654321u);
  EXPECT_EQ(piMsgAuxRx->aux_1, 1000);
  EXPECT_EQ(piMsgAuxRx->aux_2, 1500);
  EXPECT_EQ(piMsgAuxRx->aux_3, 2000);
  EXPECT_EQ(piMsgAuxRx->aux_4, 1100);
  EXPECT_EQ(piMsgAuxRx->aux_5, 900);
  EXPECT_EQ(piMsgAuxRx->aux_6, 1800);
  EXPECT_EQ(piMsgAuxRx->aux_14, 1000);
}

TEST(FcStampGate, AcceptsMonotonicStream)
{
  pi_protocol::FcStampGate gate;
  EXPECT_TRUE(gate.accept(1000000));
  EXPECT_TRUE(gate.accept(1002000));
  EXPECT_TRUE(gate.accept(1004000));
  EXPECT_EQ(gate.last(), 1004000u);
}

TEST(FcStampGate, AcceptsSmallBackwardsJitter)
{
  // e.g. hex IMU stamped at the gyro EXTI instant, slightly before the AUX
  // sent in the same telemetry tick.
  pi_protocol::FcStampGate gate;
  EXPECT_TRUE(gate.accept(1000000));
  EXPECT_TRUE(gate.accept(999000));
  // The cached tick never regresses (TX stamping wants the newest).
  EXPECT_EQ(gate.last(), 1000000u);
}

TEST(FcStampGate, RejectsGarbageStamp)
{
  pi_protocol::FcStampGate gate;
  EXPECT_TRUE(gate.accept(1000000));
  // A mis-framed packet that slipped the 8-bit CRC carries a random stamp.
  EXPECT_FALSE(gate.accept(0xDEADBEEF));
  // The stream continues unaffected.
  EXPECT_TRUE(gate.accept(1002000));
  EXPECT_EQ(gate.last(), 1002000u);
}

TEST(FcStampGate, ReseedsAfterFcReboot)
{
  pi_protocol::FcStampGate gate;
  EXPECT_TRUE(gate.accept(3600000000u));  // 1h of FC uptime
  // FC reboots: micros() restarts near zero. First message fails the window...
  EXPECT_FALSE(gate.accept(50000));
  // ...but a second consistent stamp re-seeds the clock.
  EXPECT_TRUE(gate.accept(52000));
  EXPECT_EQ(gate.last(), 52000u);
}

TEST(FcStampGate, MicrosWrapIsAccepted)
{
  // micros() wraps every ~71.6 min; the int32 delta comparison is wrap-safe.
  pi_protocol::FcStampGate gate;
  EXPECT_TRUE(gate.accept(0xFFFFFF00u));
  EXPECT_TRUE(gate.accept(0x00000100u));  // 512 us later, across the wrap
  EXPECT_EQ(gate.last(), 0x00000100u);
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
  piMsgEkfInputsTx.omega5 = 1160;
  piMsgEkfInputsTx.omega6 = 1170;

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
  EXPECT_EQ(piMsgEkfInputsRx->omega5, 1160u);
  EXPECT_EQ(piMsgEkfInputsRx->omega6, 1170u);
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
