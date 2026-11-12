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
#include <deque>
#include <mutex>
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
 * The FC clock does not run at the host's rate (1500 to 2900 ppm slow on the
 * recorded flights, ~3700 ppm on indi_experiment), so a minimum over the last
 * 2 s is up to 2 s stale: stamps run 3 to 5 ms early and step every bucket as
 * old ones age out. setRateTracking() fits the offset as a line in host time
 * instead, through the lower envelope of the last buckets' minima.
 *
 * sync() and toHostTime() are not thread-safe and are intended to be driven
 * from a single dedicated thread, as Client's reader thread already is.
 * toFcTime() reads only the published offset (and rate) and may be called from
 * any thread, which the uplink needs: it runs on the executor.
 */
class ClockSync
{
public:
  /**
   * @brief Also track the rate difference between the FC and host clocks.
   *
   * Off (the default): the offset is the minimum over the last 2 s. On: it is
   * a line in host time through the lower envelope of the passive samples'
   * per-bucket minima of the last window_ns, so a stamp follows the FC clock's
   * actual rate. Until 4 s of buckets have been seen, and again after the FC
   * restarts or the host clock steps, the 2 s minimum is used. Call before the
   * first sync().
   *
   * @param enabled Whether to track the rate.
   * @param window_ns How far back the line is fitted. 5 s: on the recorded
   *        flights the rate wanders by a few hundred ppm within seconds and a
   *        longer line lags it, while the passive floor is crisp enough for a
   *        short one.
   */
  void setRateTracking(bool enabled, int64_t window_ns);

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

  /**
   * @brief Feed one round-trip exchange: the FC's stamp against the midpoint of
   * the two host instants that bracket it.
   *
   * A passive sample only bounds the offset from one side, by however long the
   * message took to arrive, so the estimate carries the link's floor latency as
   * a bias that no amount of averaging removes. A round trip brackets the FC
   * stamp instead: it costs one message per exchange and buys an estimate that
   * depends on the link being symmetric rather than on it being fast, which is
   * what makes it hold on a controller whose clock has not been characterised.
   *
   * @param fc_time_us Stamp the FC put in the reply, micros() domain.
   * @param host_send_ns Host clock when the request went out.
   * @param host_recv_ns Host clock when the reply came back.
   * @return The round trip in nanoseconds, or nothing when it was too slow
   *         against the best the link has shown for its midpoint to mean
   *         anything.
   */
  std::optional<int64_t> syncRoundTrip(
    uint32_t fc_time_us, int64_t host_send_ns, int64_t host_recv_ns);

private:
  /**
   * @brief Extend a wrapping FC stamp into a microsecond count, restarting the
   * estimate when the counter did. A stamp slightly older than the newest keeps
   * the epoch and leaves the newest as it is.
   */
  int64_t unwrap(uint32_t fc_time_us);

  /**
   * @brief Fold one offset candidate into the window and publish the minimum
   * (or, with rate tracking, the line).
   * @param round_trip Whether the candidate is a round trip's: it goes into the
   *        2 s minimum, not into the line.
   * @return The offset now in use, at host_now_ns.
   */
  int64_t feed(int64_t candidate_offset_ns, int64_t host_now_ns, bool round_trip);

  /**
   * @brief With rate tracking: fit the line through the buckets' minima and
   * publish it. A fit that fails keeps the line in use for up to a window.
   * @return Whether a line is in use (else the 2 s minimum is).
   */
  bool fitLine(int64_t host_now_ns);

  /// The offset in use at a host instant: the line's, or the 2 s minimum.
  int64_t offsetAt(int64_t host_ns) const;

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
  // A step back no larger than this is a stamp out of order, not micros() restarting: a restart
  // goes back by the FC's whole uptime
  static constexpr uint32_t kMaxReorderUs = 100'000;

  // A reply that took far longer than this link has shown it can cannot have
  // its midpoint trusted: the delay it met was one-sided by an unknown share.
  static constexpr int64_t kRttGateFactor = 2;
  // The floor rises towards a link that got permanently slower this slowly, so
  // one lucky exchange cannot lock every later one out.
  static constexpr int64_t kRttFloorRiseDivisor = 64;

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

  // Best round trip the link has shown, the yardstick every later one is
  // judged against.
  int64_t min_rtt_ns_ = 0;
  bool rtt_valid_ = false;

  // Rate tracking (setRateTracking). The line needs this many closed buckets
  // (4 s) before it is used, and kMinFitBuckets of them within kLineLateNs of
  // the lower envelope: one later than that is a bucket without a low-latency
  // sample and is left out of the fit. kLineResetBuckets bucket minima in a
  // row further than kLineResetNs from the line mean the host clock stepped,
  // and the fit starts over.
  static constexpr size_t kMinLineBuckets = 16;
  static constexpr double kMinFitBuckets = 4.0;
  static constexpr double kLineLateNs = 100'000.0;
  static constexpr int kLineResetBuckets = 4;
  static constexpr int64_t kLineResetNs = 10'000'000;
  // A rate further than this from the host's is not a clock: the fit is wrong
  static constexpr double kMaxRate = 0.01;

  struct BucketMin
  {
    int64_t host_ns;    // When the bucket's lowest candidate arrived
    int64_t offset_ns;  // That candidate
  };
  bool track_rate_ = false;
  int64_t rate_window_ns_ = 5'000'000'000;
  std::deque<BucketMin> closed_buckets_;  // Oldest first, over rate_window_ns_
  BucketMin open_bucket_{0, 0};
  bool open_bucket_empty_ = true;  // No passive sample in it yet
  // The last line's slope, the open bucket's lowest is picked against; 0 after a restart
  double pick_rate_ = 0.0;
  int off_line_buckets_ = 0;
  // The line in use: offset = line_offset_ns_ + line_rate_ * (host - line_host_ns_).
  // Guarded by line_mutex_ for toFcTime(); line_valid_ false = the 2 s minimum.
  mutable std::mutex line_mutex_;
  bool line_valid_ = false;
  int64_t line_host_ns_ = 0;
  int64_t line_offset_ns_ = 0;
  double line_rate_ = 0.0;
};

}  // namespace pi_protocol

#endif  // PI_PROTOCOL__CLOCK_SYNC_HPP_
