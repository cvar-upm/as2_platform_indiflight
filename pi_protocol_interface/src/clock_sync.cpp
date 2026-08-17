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
#include <cstdint>
#include <limits>

namespace pi_protocol
{

int64_t ClockSync::sync(uint32_t fc_time_us, int64_t host_now_ns)
{
  if (!initialized_) {
    last_fc_time_us_ = fc_time_us;
    fc_time_epoch_us_ = 0;
  } else if (fc_time_us < last_fc_time_us_) {
    constexpr uint32_t kHalfRangeUs = std::numeric_limits<uint32_t>::max() / 2;
    if (last_fc_time_us_ - fc_time_us > kHalfRangeUs) {
      // fc_time_us wrapped (uint32_t microsecond counter, ~71.6 minute period).
      fc_time_epoch_us_ += kWrapPeriodUs;
    } else {
      // Too small to be a wrap: micros() restarted, so the offset is stale.
      initialized_ = false;
      fc_time_epoch_us_ = 0;
      offset_valid_.store(false, std::memory_order_release);
    }
  }
  last_fc_time_us_ = fc_time_us;

  const int64_t unwrapped_fc_time_us = fc_time_epoch_us_ + static_cast<int64_t>(fc_time_us);
  const int64_t candidate_offset_ns = host_now_ns - unwrapped_fc_time_us * 1000;

  if (!initialized_) {
    bucket_min_offset_ns_.fill(candidate_offset_ns);
    current_bucket_start_ns_ = host_now_ns;
    initialized_ = true;
  } else if (host_now_ns - current_bucket_start_ns_ > kBucketDurationNs) {
    // Rotate into the next bucket, aging out whatever sample sat there ~16s ago.
    current_bucket_ = (current_bucket_ + 1) % kNumBuckets;
    current_bucket_start_ns_ = host_now_ns;
    bucket_min_offset_ns_[current_bucket_] = candidate_offset_ns;
  } else {
    bucket_min_offset_ns_[current_bucket_] = std::min(
      bucket_min_offset_ns_[current_bucket_], candidate_offset_ns);
  }

  const int64_t offset_ns = *std::min_element(
    bucket_min_offset_ns_.begin(), bucket_min_offset_ns_.end());
  published_offset_ns_.store(offset_ns, std::memory_order_relaxed);
  offset_valid_.store(true, std::memory_order_release);

  return unwrapped_fc_time_us * 1000 + offset_ns;
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
  const int64_t offset_ns = *std::min_element(
    bucket_min_offset_ns_.begin(), bucket_min_offset_ns_.end());
  return unwrapped_fc_time_us * 1000 + offset_ns;
}

std::optional<uint32_t> ClockSync::toFcTime(int64_t host_ns) const
{
  if (!offset_valid_.load(std::memory_order_acquire)) {
    return std::nullopt;
  }
  const int64_t offset_ns = published_offset_ns_.load(std::memory_order_relaxed);
  // Truncating to uint32 is what micros() itself does; cmpTimeUs() on the
  // firmware side compares wrap-safely. The epoch is not needed: it only ever
  // adds whole multiples of the wrap period, which the truncation discards.
  return static_cast<uint32_t>((host_ns - offset_ns) / 1000);
}

}  // namespace pi_protocol
