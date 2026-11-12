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

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <random>

#include "pi_protocol/clock_sync.hpp"

using pi_protocol::ClockSync;

namespace
{

// A link to an FC whose clock runs rate_ppm slow against the host's: a sample
// every 500 us arriving 500 us plus an exponential 200 us late (in order, as on
// a serial line), and with round_trips one a second taking 400 us each way. The
// host clock jumps by host_step_ns at step_at_s. Returns the largest stamp error
// from from_s to to_s, against the host instant the FC sampled at plus the
// 500 us floor: where the line sits, round trips or not.
int64_t maxStampError(
  ClockSync & sync, double rate_ppm, bool round_trips, double seconds, double from_s,
  double to_s, int64_t host_step_ns = 0, double step_at_s = -1.0)
{
  std::mt19937 rng(7);
  std::exponential_distribution<double> excess_ns(1.0 / 200'000.0);
  const int64_t start_ns = 10'000'000'000;
  const int64_t step_at_ns = step_at_s < 0.0 ? std::numeric_limits<int64_t>::max() :
    start_ns + static_cast<int64_t>(step_at_s * 1e9);
  auto host = [&](int64_t t_ns) {return t_ns + (t_ns >= step_at_ns ? host_step_ns : 0);};
  auto fc_us = [&](int64_t t_ns) {
      return static_cast<uint32_t>(static_cast<double>(t_ns) * (1.0 - rate_ppm * 1e-6) / 1000.0);
    };

  int64_t worst_ns = 0;
  int64_t last_arrival_ns = 0;
  for (int64_t t_ns = start_ns; t_ns < start_ns + static_cast<int64_t>(seconds * 1e9);
    t_ns += 500'000)
  {
    if (round_trips && (t_ns - start_ns) % 1'000'000'000 == 0) {
      // Stamped at this sample's tick: FC stamps must not go backwards, or
      // the FC looks restarted
      sync.syncRoundTrip(fc_us(t_ns), host(t_ns - 400'000), host(t_ns + 400'000));
    }
    last_arrival_ns = std::max(
      last_arrival_ns, t_ns + 500'000 + static_cast<int64_t>(excess_ns(rng)));
    const int64_t error_ns =
      sync.sync(fc_us(t_ns), host(last_arrival_ns)) - host(t_ns) - 500'000;
    const double at_s = static_cast<double>(t_ns - start_ns) * 1e-9;
    if (at_s >= from_s && at_s < to_s) {
      worst_ns = std::max(worst_ns, std::abs(error_ns));
    }
  }
  return worst_ns;
}

}  // namespace

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

TEST(ClockSync, RoundTripRemovesTheLatencyFloor)
{
  ClockSync sync;
  // Every passive sample arrives 5ms after the FC stamped it, so the passive
  // estimate can only ever place the FC clock 5ms late: the floor is a bias,
  // not noise, and no number of samples removes it.
  const int64_t true_offset_ns = 0;
  const int64_t one_way_ns = 5'000'000;
  const uint32_t step_us = 1000;
  uint32_t fc_time_us = 1000;

  for (int i = 0; i < 50; i++) {
    sync.sync(fc_time_us, static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns + one_way_ns);
    fc_time_us += step_us;
  }
  const int64_t passive_error_ns =
    sync.toHostTime(fc_time_us).value() - (static_cast<int64_t>(fc_time_us) * 1000);
  EXPECT_NEAR(passive_error_ns, one_way_ns, 1000);

  // One symmetric exchange: the FC answers in the middle of a round trip of
  // twice the one-way latency, which is exactly what the midpoint recovers.
  const int64_t send_ns = static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns - one_way_ns;
  const int64_t recv_ns = send_ns + 2 * one_way_ns;
  const auto rtt_ns = sync.syncRoundTrip(fc_time_us, send_ns, recv_ns);
  ASSERT_TRUE(rtt_ns.has_value());
  EXPECT_EQ(*rtt_ns, 2 * one_way_ns);

  const int64_t corrected_error_ns =
    sync.toHostTime(fc_time_us).value() - (static_cast<int64_t>(fc_time_us) * 1000);
  EXPECT_NEAR(corrected_error_ns, true_offset_ns, 1000);
}

TEST(ClockSync, RejectsRoundTripTooSlowToPlace)
{
  ClockSync sync;
  const uint32_t step_us = 1000;
  uint32_t fc_time_us = 1000;

  // A link that answers in 1ms, measured a few times over.
  for (int i = 0; i < 4; i++) {
    const int64_t send_ns = static_cast<int64_t>(fc_time_us) * 1000 - 500'000;
    ASSERT_TRUE(sync.syncRoundTrip(fc_time_us, send_ns, send_ns + 1'000'000).has_value());
    fc_time_us += step_us;
  }
  const int64_t before_ns = sync.toHostTime(fc_time_us).value();

  // A reply that took 100x longer was delayed by an unknown share on each leg,
  // so its midpoint says nothing about where the FC stamp sat.
  const int64_t send_ns = static_cast<int64_t>(fc_time_us) * 1000 - 500'000;
  EXPECT_FALSE(sync.syncRoundTrip(fc_time_us, send_ns, send_ns + 100'000'000).has_value());
  EXPECT_EQ(sync.toHostTime(fc_time_us).value(), before_ns);
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

TEST(ClockSync, RateTrackingFollowsTheFcClockRate)
{
  // Recorded flights: the FC clock 1500 to 2750 ppm slow, 3700 on
  // indi_experiment; -600 is one running fast. Exact round trips 500 us below
  // the passive floor must not move the line: on the drones they scatter.
  for (const bool round_trips : {false, true}) {
    for (const double rate_ppm : {1600.0, -600.0, 3700.0}) {
      ClockSync minimum;
      ClockSync line;
      line.setRateTracking(true, 5'000'000'000);
      // A slow FC clock is what the 2 s minimum gets wrong: it holds the oldest
      if (rate_ppm > 0.0) {
        EXPECT_GT(maxStampError(minimum, rate_ppm, round_trips, 30.0, 5.0, 30.0), 1'000'000)
          << rate_ppm << " ppm, round trips " << round_trips;
      }
      EXPECT_LT(maxStampError(line, rate_ppm, round_trips, 30.0, 5.0, 30.0), 20'000)
        << rate_ppm << " ppm, round trips " << round_trips;
    }
  }
}

TEST(ClockSync, RateTrackingStartsOverWhenTheHostClockSteps)
{
  ClockSync line;
  line.setRateTracking(true, 5'000'000'000);
  // The step itself and the 2 s minimum's lag, while the line and then the minimum let it go by
  EXPECT_LT(maxStampError(line, 1600.0, true, 30.0, 0.0, 30.0, 50'000'000, 15.0), 60'000'000);
  ClockSync after;
  after.setRateTracking(true, 5'000'000'000);
  EXPECT_LT(maxStampError(after, 1600.0, true, 30.0, 20.0, 30.0, 50'000'000, 15.0), 20'000);
}

TEST(ClockSync, RateTrackingToFcTimeInvertsToHostTime)
{
  ClockSync line;
  line.setRateTracking(true, 5'000'000'000);
  maxStampError(line, 1600.0, true, 10.0, 0.0, 0.0);
  for (int64_t host_ns = 19'000'000'000; host_ns < 20'000'000'000; host_ns += 100'000'000) {
    const auto fc_stamp = line.toFcTime(host_ns);
    ASSERT_TRUE(fc_stamp.has_value());
    const auto back_ns = line.toHostTime(*fc_stamp);
    ASSERT_TRUE(back_ns.has_value());
    // toFcTime truncates to the microsecond
    EXPECT_NEAR(static_cast<double>(*back_ns), static_cast<double>(host_ns), 1'100.0);
  }
}

TEST(ClockSync, KeepsTheOffsetWhenARoundTripReplyIsAheadOfASample)
{
  // Drone 3, 2026-11-12: a TIMESYNC reply is stamped when the FC sends it, so the IMU sample
  // queued behind it carries an earlier stamp. That step back is not micros() restarting.
  for (const bool track_rate : {false, true}) {
    ClockSync sync;
    sync.setRateTracking(track_rate, 5'000'000'000);
    const int64_t true_offset_ns = 5'000'000;
    uint32_t fc_time_us = 10'000'000;
    for (int i = 0; i < 6000; i++) {  // 12 s of 500 Hz samples, 500 us late
      sync.sync(fc_time_us, static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns + 500'000);
      fc_time_us += 2000;
    }
    const int64_t host_ns = static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns;
    const int64_t before = sync.toHostTime(fc_time_us).value();
    // The reply, 1 ms ahead of the sample that follows it
    sync.syncRoundTrip(fc_time_us + 1000, host_ns + 600'000, host_ns + 2'400'000);
    const int64_t after = sync.sync(fc_time_us, host_ns + 900'000);
    EXPECT_NEAR(static_cast<double>(after), static_cast<double>(before), 1000.0)
      << "track_rate " << track_rate;
    // And the stream goes on as before
    for (int i = 0; i < 500; i++) {
      fc_time_us += 2000;
      const int64_t sample_host_ns = static_cast<int64_t>(fc_time_us) * 1000 + true_offset_ns;
      EXPECT_NEAR(
        static_cast<double>(sync.sync(fc_time_us, sample_host_ns + 500'000)),
        static_cast<double>(sample_host_ns + 500'000), 1000.0) << "track_rate " << track_rate;
    }
  }
}
