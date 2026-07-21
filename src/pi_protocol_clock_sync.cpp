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

#include "as2_platform_indiflight/pi_protocol_clock_sync.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace as2_platform_indiflight
{

int64_t PiProtocolClockSync::sync(uint32_t fc_time_us, int64_t host_now_ns)
{
  if (!initialized_) {
    last_fc_time_us_ = fc_time_us;
    fc_time_epoch_us_ = 0;
  } else if (fc_time_us < last_fc_time_us_ &&
    (last_fc_time_us_ - fc_time_us) > (std::numeric_limits<uint32_t>::max() / 2))
  {
    // fc_time_us wrapped (uint32_t microsecond counter, ~71.6 minute period).
    fc_time_epoch_us_ += kWrapPeriodUs;
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

  return unwrapped_fc_time_us * 1000 + offset_ns;
}

}  // namespace as2_platform_indiflight
