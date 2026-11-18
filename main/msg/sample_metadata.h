#pragma once
#include <stdint.h>

// Internal queue metadata, not a new ROS wire type. Only successful sampling
// advances sequence and sample_time_us; a failed read invalidates the snapshot.
struct SampleMetadata {
    int64_t sample_time_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

// A valid sample older than this must not be republished as if it were fresh.
static constexpr int64_t kSampleMaxAgeUs = 300000;
