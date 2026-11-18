#include "ultrasonic_publish.h"

#include <cassert>
#include <cmath>

int main() {
    SampleClock clock;
    SamplePublishGate gate;
    UltrasonicStateMsg state;
    state.sample.valid = true;
    state.sample.sequence = 1;
    state.sample.sample_time_us = 1100000;
    state.reading.distance_cm = 100.0f;

    clock.Reset(1000000);
    assert(!microros::SelectUltrasonicPublishSample(
        state, 1200000, 0.02f, 4.0f, clock, gate).publish);  // No epoch mapping yet.

    clock.Synchronize(1200000, 1700000000200LL);
    const auto first = microros::SelectUltrasonicPublishSample(
        state, 1200000, 0.02f, 4.0f, clock, gate);
    assert(first.publish);
    assert(first.stamp_ns == 1700000000100000000LL);
    assert(std::fabs(first.range_m - 1.0f) < 1e-6f);

    gate.Published(first.stamp_ns);
    assert(!microros::SelectUltrasonicPublishSample(
        state, 1200000, 0.02f, 4.0f, clock, gate).publish);  // Same sample must not be re-stamped.

    state.sample.sample_time_us = 1150000;
    ++state.sample.sequence;
    assert(microros::SelectUltrasonicPublishSample(
        state, 1200000, 0.02f, 4.0f, clock, gate).publish);  // A newer sample may retry.

    state.sample.valid = false;
    assert(!microros::SelectUltrasonicPublishSample(
        state, 1200000, 0.02f, 4.0f, clock, gate).publish);

    state.sample.valid = true;
    state.reading.distance_cm = 500.0f;
    assert(!microros::SelectUltrasonicPublishSample(
        state, 1200000, 0.02f, 4.0f, clock, gate).publish);  // Outside declared Range bounds.

    state.reading.distance_cm = 100.0f;
    state.sample.sample_time_us = 800000;
    SampleClock age_clock;
    SamplePublishGate age_gate;
    age_clock.Reset(0);
    age_clock.Synchronize(1200000, 1700000000200LL);
    assert(!microros::SelectUltrasonicPublishSample(
        state, 1200000, 0.02f, 4.0f, age_clock, age_gate).publish);  // Older than the freshness limit.

    clock.Reset(1300000);
    gate.Reset();
    clock.Synchronize(1400000, 1700000000400LL);
    state.sample.sample_time_us = 1100000;
    assert(!microros::SelectUltrasonicPublishSample(
        state, 1400000, 0.02f, 4.0f, clock, gate).publish);  // Previous-session cache.
}
