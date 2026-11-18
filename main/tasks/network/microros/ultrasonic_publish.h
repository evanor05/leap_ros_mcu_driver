#pragma once

#include <cmath>
#include <cstdint>

#include "msg/ultrasonic_msg.h"
#include "sample_clock.h"

namespace microros {

struct UltrasonicPublishSample {
    bool publish = false;
    int64_t stamp_ns = 0;
    float range_m = 0.0f;
};

// Select one sample that may be published with its true measurement time.
// Invalid, out-of-range, stale, previous-session, and already-published
// samples are all rejected; the caller advances the gate only after a
// successful rcl_publish().
inline UltrasonicPublishSample SelectUltrasonicPublishSample(
    const UltrasonicStateMsg &state, int64_t now_us,
    float min_range_m, float max_range_m,
    const SampleClock &clock, const SamplePublishGate &gate) {
    UltrasonicPublishSample out;
    if (!state.sample.valid || !std::isfinite(state.reading.distance_cm)) {
        return out;
    }
    const float range_m = state.reading.distance_cm / 100.0f;
    if (!(range_m >= min_range_m && range_m <= max_range_m)) {
        return out;
    }
    const int64_t stamp_ns = clock.Stamp(state.sample, now_us);
    if (!gate.New(stamp_ns)) {
        return out;
    }
    out.publish = true;
    out.stamp_ns = stamp_ns;
    out.range_m = range_m;
    return out;
}

}  // namespace microros
