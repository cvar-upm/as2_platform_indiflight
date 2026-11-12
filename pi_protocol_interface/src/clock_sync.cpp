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
* @file clock_sync.cpp
*
* pi-protocol ClockSync class implementation
*
* @authors Rafael Perez-Segui
*          Francisco José Anguita Chamorro
*/

#include "pi_protocol/clock_sync.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

namespace pi_protocol
{

void ClockSync::setRateTracking(bool enabled, int64_t window_ns)
{
  track_rate_ = enabled;
  // At least the buckets the line needs, and one more
  rate_window_ns_ = std::max<int64_t>(
    window_ns, static_cast<int64_t>(kMinLineBuckets + 1) * kBucketDurationNs);
  closed_buckets_.clear();
  off_line_buckets_ = 0;
  pick_rate_ = 0.0;
  std::lock_guard<std::mutex> lock(line_mutex_);
  line_valid_ = false;
}

int64_t ClockSync::unwrap(uint32_t fc_time_us)
{
  constexpr uint32_t kHalfRangeUs = std::numeric_limits<uint32_t>::max() / 2;
  if (!initialized_) {
    last_fc_time_us_ = fc_time_us;
    fc_time_epoch_us_ = 0;
  } else if (fc_time_us < last_fc_time_us_) {
    const uint32_t back_us = last_fc_time_us_ - fc_time_us;
    if (back_us > kHalfRangeUs) {
      // fc_time_us wrapped (uint32_t microsecond counter, ~71.6 minute period).
      fc_time_epoch_us_ += kWrapPeriodUs;
    } else if (back_us <= kMaxReorderUs) {
      // A little older than the newest, not a restart: the FC stamps a TIMESYNC reply when it
      // sends it, so a sample taken before but queued behind it arrives with an earlier stamp.
      return fc_time_epoch_us_ + static_cast<int64_t>(fc_time_us);
    } else {
      // Too small to be a wrap: micros() restarted, so the offset is stale.
      initialized_ = false;
      fc_time_epoch_us_ = 0;
      offset_valid_.store(false, std::memory_order_release);
      // The old link's round trips say nothing about the new one.
      rtt_valid_ = false;
    }
  } else if (fc_time_us - last_fc_time_us_ > kHalfRangeUs) {
    // The same reordering across a wrap: a stamp from before it, after one that crossed it
    return fc_time_epoch_us_ - kWrapPeriodUs + static_cast<int64_t>(fc_time_us);
  }
  last_fc_time_us_ = fc_time_us;

  return fc_time_epoch_us_ + static_cast<int64_t>(fc_time_us);
}

int64_t ClockSync::feed(int64_t candidate_offset_ns, int64_t host_now_ns, bool round_trip)
{
  bool bucket_closed = false;
  BucketMin closed{0, 0};
  if (!initialized_) {
    bucket_min_offset_ns_.fill(candidate_offset_ns);
    current_bucket_start_ns_ = host_now_ns;
    initialized_ = true;
    // A new FC clock (or the first): no line until its buckets come in
    closed_buckets_.clear();
    open_bucket_empty_ = true;
    off_line_buckets_ = 0;
    pick_rate_ = 0.0;
    std::lock_guard<std::mutex> lock(line_mutex_);
    line_valid_ = false;
  } else if (host_now_ns - current_bucket_start_ns_ > kBucketDurationNs) {
    // Rotate into the next bucket, aging out whatever sample sat there a whole
    // window ago.
    current_bucket_ = (current_bucket_ + 1) % kNumBuckets;
    current_bucket_start_ns_ = host_now_ns;
    bucket_min_offset_ns_[current_bucket_] = candidate_offset_ns;
    bucket_closed = !open_bucket_empty_;
    closed = open_bucket_;
    open_bucket_empty_ = true;
  } else {
    bucket_min_offset_ns_[current_bucket_] = std::min(
      bucket_min_offset_ns_[current_bucket_], candidate_offset_ns);
  }

  // The line takes the passive samples only: on the drones the round trips'
  // midpoints scatter by a millisecond, where the passive floor is constant.
  // Against the line's slope: with the clocks drifting apart fast, the lowest
  // raw candidate is one of the bucket's first or last, not its least late
  if (!round_trip) {
    const double drift_ns =
      pick_rate_ * static_cast<double>(host_now_ns - open_bucket_.host_ns);
    if (open_bucket_empty_ ||
      static_cast<double>(candidate_offset_ns - open_bucket_.offset_ns) < drift_ns)
    {
      open_bucket_ = {host_now_ns, candidate_offset_ns};
      open_bucket_empty_ = false;
    }
  }

  const int64_t offset_ns = *std::min_element(
    bucket_min_offset_ns_.begin(), bucket_min_offset_ns_.end());
  published_offset_ns_.store(offset_ns, std::memory_order_relaxed);
  offset_valid_.store(true, std::memory_order_release);
  if (!track_rate_) {
    return offset_ns;
  }
  if (bucket_closed) {
    closed_buckets_.push_back(closed);
    fitLine(host_now_ns);
  }
  return offsetAt(host_now_ns);
}

bool ClockSync::fitLine(int64_t host_now_ns)
{
  while (!closed_buckets_.empty() &&
    closed_buckets_.front().host_ns < host_now_ns - rate_window_ns_)
  {
    closed_buckets_.pop_front();
  }
  if (closed_buckets_.empty()) {
    return false;
  }

  // The host clock stepped (ntpd): the last buckets all far from the line. Keep
  // only those and start over, on the 2 s minimum meanwhile.
  {
    std::lock_guard<std::mutex> lock(line_mutex_);
    if (line_valid_) {
      const BucketMin & last = closed_buckets_.back();
      const double expected = static_cast<double>(line_offset_ns_) +
        line_rate_ * static_cast<double>(last.host_ns - line_host_ns_);
      const bool off_line =
        std::abs(static_cast<double>(last.offset_ns) - expected) > kLineResetNs;
      off_line_buckets_ = off_line ? off_line_buckets_ + 1 : 0;
      if (off_line_buckets_ >= kLineResetBuckets) {
        if (closed_buckets_.size() > static_cast<size_t>(kLineResetBuckets)) {
          closed_buckets_.erase(
            closed_buckets_.begin(), closed_buckets_.end() - kLineResetBuckets);
        }
        off_line_buckets_ = 0;
        pick_rate_ = 0.0;
        line_valid_ = false;
        return false;
      }
    }
  }
  if (closed_buckets_.size() < kMinLineBuckets) {
    std::lock_guard<std::mutex> lock(line_mutex_);
    line_valid_ = false;
    return false;
  }

  // Offset on host time, relative to the newest bucket so the numbers stay small
  const BucketMin & newest = closed_buckets_.back();
  auto x = [&](size_t i) {
      return static_cast<double>(closed_buckets_[i].host_ns - newest.host_ns) * 1e-9;
    };
  auto y = [&](size_t i) {
      return static_cast<double>(closed_buckets_[i].offset_ns - newest.offset_ns);
    };

  // The lower envelope: the lower convex hull's edge across the middle of the
  // window, the highest line under every bucket's minimum there
  std::vector<size_t> hull;
  double x_mean = 0.0;
  for (size_t i = 0; i < closed_buckets_.size(); ++i) {
    while (hull.size() >= 2) {
      const size_t o = hull[hull.size() - 2];
      const size_t m = hull.back();
      if ((x(m) - x(o)) * (y(i) - y(o)) - (y(m) - y(o)) * (x(i) - x(o)) > 0.0) {
        break;
      }
      hull.pop_back();
    }
    hull.push_back(i);
    x_mean += x(i);
  }
  x_mean /= static_cast<double>(closed_buckets_.size());
  double envelope_a = 0.0;  // ns, at the newest bucket
  double envelope_b = 0.0;  // ns per s
  for (size_t k = 0; k + 1 < hull.size(); ++k) {
    const size_t i = hull[k];
    const size_t j = hull[k + 1];
    if (x(j) >= x_mean && x(j) > x(i)) {
      envelope_b = (y(j) - y(i)) / (x(j) - x(i));
      envelope_a = y(i) - envelope_b * x(i);
      break;
    }
  }

  // Least squares through the minima on it: one later than it by more than
  // kLineLateNs still met some latency
  double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0, count = 0.0;
  for (size_t i = 0; i < closed_buckets_.size(); ++i) {
    if (y(i) - (envelope_a + envelope_b * x(i)) > kLineLateNs) {
      continue;
    }
    sx += x(i);
    sy += y(i);
    sxx += x(i) * x(i);
    sxy += x(i) * y(i);
    count += 1.0;
  }
  const double denominator = count * sxx - sx * sx;
  const bool fitted = count >= kMinFitBuckets && denominator > 0.0;
  const double b = fitted ? (count * sxy - sx * sy) / denominator : 0.0;  // ns per s
  const double a = fitted ? (sy - b * sx) / count : 0.0;  // ns, at the newest bucket

  std::lock_guard<std::mutex> lock(line_mutex_);
  if (!fitted || std::abs(b * 1e-9) > kMaxRate) {
    // The line in use stays, for a window: picked against its slope, the next
    // buckets' lowest are the ones that bring the fit back
    line_valid_ = line_valid_ && newest.host_ns - line_host_ns_ < rate_window_ns_;
    return line_valid_;
  }
  pick_rate_ = b * 1e-9;
  line_host_ns_ = newest.host_ns;
  line_offset_ns_ = newest.offset_ns + std::llround(a);
  line_rate_ = b * 1e-9;
  line_valid_ = true;
  return true;
}

int64_t ClockSync::offsetAt(int64_t host_ns) const
{
  std::lock_guard<std::mutex> lock(line_mutex_);
  if (!line_valid_) {
    return published_offset_ns_.load(std::memory_order_relaxed);
  }
  return line_offset_ns_ +
         std::llround(line_rate_ * static_cast<double>(host_ns - line_host_ns_));
}

int64_t ClockSync::sync(uint32_t fc_time_us, int64_t host_now_ns)
{
  const int64_t unwrapped_fc_time_us = unwrap(fc_time_us);
  const int64_t offset_ns =
    feed(host_now_ns - unwrapped_fc_time_us * 1000, host_now_ns, false);
  return unwrapped_fc_time_us * 1000 + offset_ns;
}

std::optional<int64_t> ClockSync::syncRoundTrip(
  uint32_t fc_time_us, int64_t host_send_ns, int64_t host_recv_ns)
{
  const int64_t rtt_ns = host_recv_ns - host_send_ns;
  if (rtt_ns < 0) {
    return std::nullopt;
  }

  if (!rtt_valid_) {
    min_rtt_ns_ = rtt_ns;
    rtt_valid_ = true;
  } else if (rtt_ns < min_rtt_ns_) {
    min_rtt_ns_ = rtt_ns;
  } else if (rtt_ns > kRttGateFactor * min_rtt_ns_) {
    min_rtt_ns_ += (rtt_ns - min_rtt_ns_) / kRttFloorRiseDivisor;
    return std::nullopt;
  }

  // The FC stamped the reply somewhere between the two host instants. On a
  // symmetric link the midpoint is the best guess at where, and unlike a
  // passive sample it is not pushed one way by the time the frame spent on
  // the wire: that is the whole point of paying for the round trip.
  const int64_t host_mid_ns = host_send_ns + rtt_ns / 2;
  const int64_t unwrapped_fc_time_us = unwrap(fc_time_us);
  feed(host_mid_ns - unwrapped_fc_time_us * 1000, host_recv_ns, true);
  return rtt_ns;
}

std::optional<int64_t> ClockSync::toHostTime(uint32_t fc_time_us) const
{
  if (!initialized_) {
    return std::nullopt;
  }
  // The epoch is the one the last sync() left. A stamp arriving across a wrap
  // before sync() sees it lands one period out, for as long as it takes the
  // next synced message to arrive.
  const int64_t unwrapped_fc_time_us = fc_time_epoch_us_ + static_cast<int64_t>(fc_time_us);
  if (track_rate_) {
    // The line at that instant: the 2 s minimum places it within its staleness,
    // and one more step from there to well under a microsecond
    const int64_t fc_ns = unwrapped_fc_time_us * 1000;
    const int64_t guess_ns =
      fc_ns + offsetAt(fc_ns + published_offset_ns_.load(std::memory_order_relaxed));
    return fc_ns + offsetAt(guess_ns);
  }
  const int64_t offset_ns = *std::min_element(
    bucket_min_offset_ns_.begin(), bucket_min_offset_ns_.end());
  return unwrapped_fc_time_us * 1000 + offset_ns;
}

std::optional<uint32_t> ClockSync::toFcTime(int64_t host_ns) const
{
  if (!offset_valid_.load(std::memory_order_acquire)) {
    return std::nullopt;
  }
  const int64_t offset_ns = track_rate_ ? offsetAt(host_ns) :
    published_offset_ns_.load(std::memory_order_relaxed);
  // Truncating to uint32 is what micros() itself does; cmpTimeUs() on the
  // firmware side compares wrap-safely. The epoch is not needed: it only ever
  // adds whole multiples of the wrap period, which the truncation discards.
  return static_cast<uint32_t>((host_ns - offset_ns) / 1000);
}

}  // namespace pi_protocol
