#pragma once

#include "msg/ultrasonic_msg.h"

#define pdMS_TO_TICKS(v) (v)

struct Globals {
    struct {
        void *ultrasonic_state = nullptr;
    } queues;
};

Globals &system_globals();
void xQueueOverwrite(void *, const UltrasonicStateMsg *);
void vTaskDelay(int);
