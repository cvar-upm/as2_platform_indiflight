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
 * @file pi_protocol_clock_sync.hpp
 *
 * PiProtocolClockSync class definition
 */

#ifndef AS2_PLATFORM_INDIFLIGHT__PI_PROTOCOL_CLOCK_SYNC_HPP_
#define AS2_PLATFORM_INDIFLIGHT__PI_PROTOCOL_CLOCK_SYNC_HPP_

#include <array>
#include <cstdint>
#include <limits>
#include <optional>

namespace as2_platform_indiflight
{

/**
 * @brief Converts indiflight's FC-boot-relative `time_us` (a wrapping uint32_t
 * microsecond counter with no defined relationship to the host clock) into a
 * host-clock nanosecond timestamp.
 *
 * Uses a sliding-window minimum-offset tracker (the "clock filter" technique
 * NTP/chrony use for asymmetric one-way channels): the host can only ever
 * observe a message after the FC sampled `time_us`, never before, so the
 * minimum observed (host_time - fc_time) candidate is the tightest,
 * least-latency-biased estimate of the true offset. A handful of sub-window
 * buckets let old minimums age out so the estimate can track real clock
 * drift instead of latching permanently onto one lucky low-latency sample.
 *
 * ROS-agnostic (plain int64_t nanoseconds in/out) so it's usable from a
 * hardware-free unit test and doesn't tie transport/sync logic to rclcpp.
 *
 * Not thread-safe: intended to be driven from a single dedicated thread
 * (as PiProtocolClient's reader thread already is) - no internal locking.
 */
class PiProtocolClockSync
{
public:
  /**
   * @brief Feed one (fc_time_us, host_now_ns) sample.
   * @return the current best-estimate host-clock nanosecond timestamp
   * corresponding to fc_time_us.
   */
  int64_t sync(uint32_t fc_time_us, int64_t host_now_ns);

  /**
   * @brief Feed one round-trip TIMESYNC exchange: fc_time_us is the FC's own
   * stamp, echoed back at the moment it handled the request; host_send_ns and
   * host_recv_ns bracket that on the host side. Unlike a passive sample (see
   * sync()), which is always biased high by one-way transit time, placing the
   * FC stamp at the midpoint of the round trip removes that bias - so an
   * accepted round-trip candidate feeds the very same windowed-minimum filter
   * sync() uses, wins the minimum on its own merit, and holds until it ages
   * out, rather than needing a second, separate estimator.
   *
   * A reply slower than kRttGateFactor times the best round trip seen so far
   * is rejected: its delay was probably one-sided, so its midpoint doesn't
   * mean anything. The floor itself creeps towards a link that got
   * permanently worse (see kRttFloorRiseDivisor), so one lucky early exchange
   * doesn't lock every later one out.
   *
   * @return the resulting host-clock timestamp for fc_time_us if the sample
   * was accepted, std::nullopt if it was RTT-gated out.
   */
  std::optional<int64_t> syncRoundTrip(
    uint32_t fc_time_us, int64_t host_send_ns, int64_t host_recv_ns);

private:
  // Wrap-safe unwrap of the FC's free-running microsecond counter, shared by
  // both sync() and syncRoundTrip() so the delicate wrap/restart handling
  // lives in exactly one place.
  int64_t unwrap(uint32_t fc_time_us);

  // Fold one offset candidate into the bucket window and return the
  // resulting host-clock timestamp for unwrapped_fc_time_us. Shared by both
  // sync() and syncRoundTrip().
  int64_t feed(int64_t unwrapped_fc_time_us, int64_t candidate_offset_ns, int64_t host_now_ns);

  static constexpr int64_t kRttGateFactor = 2;
  static constexpr int64_t kRttFloorRiseDivisor = 64;
  // "No floor yet" sentinel - the gate never rejects until a first RTT has
  // actually been observed.
  int64_t best_rtt_ns_ = std::numeric_limits<int64_t>::max();

  static constexpr int kNumBuckets = 8;
  // 250ms -> 2s total window. Was 2s/16s: with pi-protocol running EKF_INPUTS
  // at ~2000Hz over a dedicated 921600-baud link, even 250ms buckets still
  // hold ~500 candidate samples - plenty to keep rejecting genuine latency
  // spikes - while capping the staleness-driven reconstruction error
  // (~skew * staleness, see indi_experiment clock-offset investigation) at
  // ~6ms instead of ~48ms for the ~3700ppm FC clock rate error measured
  // there, pending a real fix on the FC side.
  static constexpr int64_t kBucketDurationNs = 250'000'000;
  static constexpr int64_t kWrapPeriodUs = int64_t{1} << 32;   // ~71.6 minutes

  std::array<int64_t, kNumBuckets> bucket_min_offset_ns_{};
  int current_bucket_ = 0;
  int64_t current_bucket_start_ns_ = 0;
  bool initialized_ = false;

  // Unwrapped (monotonically increasing) FC microsecond counter tracking.
  uint32_t last_fc_time_us_ = 0;
  int64_t fc_time_epoch_us_ = 0;
};

}  // namespace as2_platform_indiflight

#endif  // AS2_PLATFORM_INDIFLIGHT__PI_PROTOCOL_CLOCK_SYNC_HPP_
