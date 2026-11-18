#include "system_globals.h"
#include "board.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

FakeUltrasonic ultrasonic;
Globals globals;
std::vector<UltrasonicStateMsg> samples;
static int64_t fake_now_us = 0;

Globals &system_globals() { return globals; }
int64_t esp_timer_get_time() { fake_now_us += 100000; return fake_now_us; }
void xQueueOverwrite(void *, const UltrasonicStateMsg *msg) { samples.push_back(*msg); }

struct End {};
void vTaskDelay(int) {
    if (samples.size() >= 4) {
        throw End{};
    }
}

void ultrasonic_task(void *p);

int main() {
    static_assert(sizeof(UltrasonicMsg) == 5, "legacy UDP telemetry layout must stay 5 bytes");
    static_assert(offsetof(UltrasonicMsg, distance_cm) == 0, "legacy field order changed");

    globals.queues.ultrasonic_state = reinterpret_cast<void *>(1);
    try {
        ultrasonic_task(nullptr);
    } catch (const End &) {
    }

    assert(samples.size() == 4);
    assert(samples[0].sample.valid);
    assert(samples[0].sample.sequence == 1);
    assert(samples[0].sample.sample_time_us == 100000);
    assert(std::fabs(samples[0].reading.distance_cm - 25.0f) < 1e-6f);
    assert(!samples[0].reading.is_obstacle_detected);

    // Failed reads neither advance the sequence nor pretend to be fresh.
    for (int i = 1; i <= 2; ++i) {
        assert(!samples[i].sample.valid);
        assert(samples[i].sample.sequence == 1);
        assert(samples[i].sample.sample_time_us == 100000);
        assert(samples[i].reading.distance_cm == -1.0f);
    }

    assert(samples[3].sample.valid);
    assert(samples[3].sample.sequence == 2);
    assert(samples[3].sample.sample_time_us == 400000);
    assert(std::fabs(samples[3].reading.distance_cm - 30.0f) < 1e-6f);
}
