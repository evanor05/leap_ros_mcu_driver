#include "system_globals.h"
#include "board.h"
#include "msg/ultrasonic_msg.h"

#include "esp_timer.h"
#include <cmath>

void ultrasonic_task(void *p) {
    (void)p;
    UltrasonicStateMsg state;
    while (1) {
        const float distance_cm = ultrasonic.GetDistanceCm(25.0f);
        const int64_t completed_us = esp_timer_get_time();

        state.sample.valid = std::isfinite(distance_cm) && distance_cm > 0.0f;
        state.reading.distance_cm = state.sample.valid ? distance_cm : -1.0f;
        state.reading.is_obstacle_detected = false;
        if (state.sample.valid) {
            // Completion time of this echo, not the time it is published.
            state.sample.sample_time_us = completed_us;
            ++state.sample.sequence;
        }
        if (system_globals().queues.ultrasonic_state != nullptr) {
            xQueueOverwrite(system_globals().queues.ultrasonic_state, &state);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

