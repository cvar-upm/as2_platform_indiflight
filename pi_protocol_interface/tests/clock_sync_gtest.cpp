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
* @file clock_sync_gtest.cpp
*
* pi-protocol ClockSync tests
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "pi_protocol/clock_sync.hpp"

using pi_protocol::ClockSync;

TEST(ClockSync, ConvergesToFixedOffset)
{
  ClockSync sync;
  const int64_t true_offset_ns = 5'000'000;  // 5ms constant one-way latency
  const uint32_t step_us = 1000;             // 1kHz cadence
  uint32_t fc_time_us = 1000;

  // Zero jitter: every sample reports exactly the same offset, so the
  // estimate should track it exactly from the very first sample.
  for (int i = 0; i < 100; i++) {
    const int64_t host_now_ns = static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns;
    const int64_t synced_ns = sync.sync(fc_time_us, host_now_ns);
    EXPECT_EQ(synced_ns, host_now_ns);
    fc_time_us += step_us;
  }
}

TEST(ClockSync, RejectsLatencySpike)
{
  ClockSync sync;
  const int64_t true_offset_ns = 5'000'000;
  const uint32_t step_us = 1000;
  uint32_t fc_time_us = 1000;

  // Establish a tight baseline.
  for (int i = 0; i < 50; i++) {
    const int64_t host_now_ns = static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns;
    sync.sync(fc_time_us, host_now_ns);
    fc_time_us += step_us;
  }

  // A single sample arrives 50ms late (e.g. a scheduler hiccup).
  const int64_t spike_host_now_ns =
    static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns + 50'000'000;
  const int64_t spike_synced_ns = sync.sync(fc_time_us, spike_host_now_ns);

  // The estimate must not chase the spike upward - it should stay close to
  // the tight baseline offset, nowhere near the spike's own (inflated) value.
  EXPECT_LT(spike_synced_ns, spike_host_now_ns - 40'000'000);
}

TEST(ClockSync, HandlesTimeUsWraparound)
{
  ClockSync sync;
  const int64_t true_offset_ns = 5'000'000;
  const uint32_t step_us = 1000;

  // Start 3 steps before the uint32_t wraparound point so the sequence
  // crosses it partway through.
  const uint64_t start_unwrapped_us =
    static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) - 3 * step_us;

  int64_t prev_synced_ns = -1;
  for (int i = 0; i < 10; i++) {
    const uint64_t unwrapped_us = start_unwrapped_us + static_cast<uint64_t>(i) * step_us;
    const uint32_t fc_time_us = static_cast<uint32_t>(unwrapped_us);  // wraps via truncation
    const int64_t host_now_ns = static_cast<int64_t>(unwrapped_us) * 1000 + true_offset_ns;

    const int64_t synced_ns = sync.sync(fc_time_us, host_now_ns);
    EXPECT_EQ(synced_ns, host_now_ns);
    if (prev_synced_ns >= 0) {
      // Must advance by exactly one step, never jump by ~71.6 minutes at the
      // wrap boundary (i == 3, where fc_time_us drops from ~UINT32_MAX to ~0).
      EXPECT_EQ(synced_ns - prev_synced_ns, static_cast<int64_t>(step_us) * 1000);
    }
    prev_synced_ns = synced_ns;
  }
}

TEST(ClockSync, RebuildsOffsetAfterFcRestart)
{
  ClockSync sync;
  const int64_t true_offset_ns = 5'000'000;
  const uint32_t step_us = 1000;

  // Session one: the FC has been up for 180 s when the host connects.
  uint32_t fc_time_us = 180'000'000;
  int64_t host_now_ns = static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns;
  for (int i = 0; i < 20; i++) {
    EXPECT_EQ(sync.sync(fc_time_us, host_now_ns), host_now_ns);
    fc_time_us += step_us;
    host_now_ns += static_cast<int64_t>(step_us) * 1000;
  }

  // micros() restarts while the host clock keeps running, so the offset jumps by
  // the whole previous uptime. Too small a backwards step to be a wraparound.
  fc_time_us = 1000;
  host_now_ns += 500'000'000;
  const int64_t restart_offset_ns = host_now_ns - static_cast<int64_t>(fc_time_us) * 1000;
  EXPECT_EQ(sync.sync(fc_time_us, host_now_ns), host_now_ns);

  for (int i = 0; i < 20; i++) {
    fc_time_us += step_us;
    host_now_ns += static_cast<int64_t>(step_us) * 1000;
    EXPECT_EQ(sync.sync(fc_time_us, host_now_ns), host_now_ns);
  }

  // The inverse direction is what stamps uplink messages.
  const auto fc_stamp = sync.toFcTime(host_now_ns);
  ASSERT_TRUE(fc_stamp.has_value());
  EXPECT_EQ(*fc_stamp, fc_time_us);
  EXPECT_EQ(
    host_now_ns - static_cast<int64_t>(fc_time_us) * 1000, restart_offset_ns);
}
