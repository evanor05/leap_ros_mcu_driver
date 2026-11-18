#pragma once
#include <stdint.h>
#include "msg/sample_metadata.h"

// One monotonic-to-epoch mapping per transport session. Publishing a cached
// sample must never manufacture a newer timestamp or revive a previous session.
class SampleClock {
 public:
    void Reset(int64_t session_us) { session_us_ = session_us; epoch_ns_ = 0; }
    void Synchronize(int64_t boot_us, int64_t epoch_ms) {
        if (epoch_ms > 0) { anchor_us_ = boot_us; epoch_ns_ = epoch_ms * 1000000LL; }
    }
    bool Ready() const { return epoch_ns_ > 0; }
    int64_t Stamp(const SampleMetadata& sample, int64_t now_us) const {
        if (!Ready() || !sample.valid || sample.sample_time_us < session_us_ ||
            sample.sample_time_us <= 0 || sample.sample_time_us > now_us ||
            now_us - sample.sample_time_us > kSampleMaxAgeUs) return 0;
        const int64_t stamp = epoch_ns_ + (sample.sample_time_us - anchor_us_) * 1000LL;
        return stamp > 0 ? stamp : 0;
    }
 private:
    int64_t session_us_ = 0, anchor_us_ = 0, epoch_ns_ = 0;
};

class SamplePublishGate {
 public:
    void Reset() { last_stamp_ = 0; }
    bool New(int64_t stamp) const { return stamp > last_stamp_; }
    void Published(int64_t stamp) { last_stamp_ = stamp; }
 private:
    int64_t last_stamp_ = 0;
};
