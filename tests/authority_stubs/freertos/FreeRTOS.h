#pragma once
#include <cstdint>
using TickType_t=uint32_t;
#define pdMS_TO_TICKS(x) (x)
#define portMAX_DELAY 0xffffffff
#define pdTRUE 1
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t*,TickType_t);
