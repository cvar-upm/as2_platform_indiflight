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
* @file clock_sync.hpp
*
* pi-protocol ClockSync class definition
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#ifndef PI_PROTOCOL__CLOCK_SYNC_HPP_
#define PI_PROTOCOL__CLOCK_SYNC_HPP_

#include <array>
#include <atomic>
#include <optional>
#include <cstdint>

namespace pi_protocol
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
 * sync() and toHostTime() are not thread-safe and are intended to be driven
 * from a single dedicated thread, as Client's reader thread already is.
 * toFcTime() reads only the published offset and may be called from any
 * thread, which the uplink needs: it runs on the executor.
 */
class ClockSync
{
public:
  /**
   * @brief Feed one (fc_time_us, host_now_ns) sample.
   * @return the current best-estimate host-clock nanosecond timestamp
   * corresponding to fc_time_us.
   *
   * @param fc_time_us Stamp of the message, in the FC micros() domain.
   * @param host_now_ns Host clock reading when the message was received.
   * @return The FC instant expressed in host clock nanoseconds.
   */
  int64_t sync(uint32_t fc_time_us, int64_t host_now_ns);

  /**
   * @brief Convert an FC stamp to host time with the offset estimated so far,
   * without feeding the filter.
   *
   * For streams whose stamp is not a sampling instant: the firmware stamps
   * those when it builds the message, so they sit later than a sample from the
   * same tick and would bias the minimum-offset estimate.
   *
   * @param fc_time_us Stamp in the FC micros() domain.
   * @return The instant in host clock nanoseconds, or nothing until sync() has
   *         been called at least once.
   */
  std::optional<int64_t> toHostTime(uint32_t fc_time_us) const;

  /**
   * @brief Convert a host instant to the FC micros() domain, the inverse of
   * toHostTime().
   *
   * For stamping an uplink measurement with the instant it was actually
   * sampled, so the firmware sees its true age rather than a fresh one.
   *
   * @param host_ns Instant in host clock nanoseconds.
   * @return The FC stamp, truncated to the uint32 the wire carries, or nothing
   *         until sync() has been called at least once.
   */
  std::optional<uint32_t> toFcTime(int64_t host_ns) const;

private:
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
  // Offset in use, republished by sync() so that toFcTime() can read it
  // without touching the filter state the reader thread owns.
  std::atomic<int64_t> published_offset_ns_{0};
  std::atomic<bool> offset_valid_{false};
  int current_bucket_ = 0;
  int64_t current_bucket_start_ns_ = 0;
  bool initialized_ = false;

  // Unwrapped (monotonically increasing) FC microsecond counter tracking.
  uint32_t last_fc_time_us_ = 0;
  int64_t fc_time_epoch_us_ = 0;
};

}  // namespace pi_protocol

#endif  // PI_PROTOCOL__CLOCK_SYNC_HPP_
